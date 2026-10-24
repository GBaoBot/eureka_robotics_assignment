# Design note: robot vacuum control software

A Python application that drives the simulated robot to named goals, records its RGBD camera
and visualises its behaviour. It communicates with the simulator exclusively through the
ZeroMQ API.

## 1. Architecture

```
 entry point      Qt app (visualizer/)
                            |
 orchestration       MissionExecutor
                            |
 mission     NavigateToGoal(s) + behaviours
                  /         |         \
 core      navigation  localization  drivers  <-- ZeroMQ --> simulator
                  \         |         /
 shared                  common
```

1. **Drivers**
   - Wrap the four sockets exposed by the simulator.
   - Consist of `RpcClient`, `RobotDriver` and `CameraDriver`. `RpcClient` owns the zmq context and serves as the core's gateway to the simulator. `RobotDriver` and `CameraDriver` each hold a reference to `RpcClient` and open their own sockets to receive data and send commands independently.
   - The drivers are the only package that communicates directly with the simulator.
2. **Localization**
   - Depends only on `common` and is designed to be modular and reusable. An external package supplies the measurements, runs the estimation and reads the filtered pose.
   - Measurements are pushed through `submit()` (any object with a pose, wheel ticks and timestamps), and the filtered pose is published through `latest()` / `wait_for()`.
   - The first implementation is an Extended Kalman Filter; other filters can be added behind the same interface.
3. **Navigation** (costmap, planner, controller)
   - Depends only on `common`, is designed to be modular and reusable, and is orchestrated by an external package. The costmap provides the processed map to the planner, which
     computes a path from a start pose to a goal. The controller then follows that path, together with the costmap, to drive the robot point to point.
   - The architecture follows the ROS 2 navigation stack, Nav2.
4. **Mission and orchestration** (mission behaviours and `MissionExecutor`)
   - The mission behaviours combine navigation, localization and the drivers into modular, single-purpose actions. `MissionExecutor` composes these behaviours into the full mission.
   - The main behaviour, `NavigateToGoal`, runs: get state → update map → collision? back up → path valid? else replan → follow path → arrived? `mark_goal`. `NavigateToGoals` visits goals in order with a single costmap.
   - `MissionExecutor` orchestrates the entire sequence, including camera recording. It runs missions and scene loads in a background thread (`start`, `cancel` and `load_scene` return immediately), exposes read-only status, and is the GUI's only entry point into the core.
5. **Visualiser** (`python3 -m vacuum.visualizer.app`)
   - A Qt application that displays the map and robot, and lets the user load scenes, select goals and view logs.
   - Communicates only with `MissionExecutor`.
   - The mission writes a `Telemetry` object (single writer; values are replaced, never mutated), which the GUI polls together with the latest pose through a 30 Hz `QTimer`.

## 2. Core implementation

**Inflated costmap.** The costmap consists of the static map from the simulator plus an obstacle layer, computed as in Nav2. Each cell falls into one of four zones: LETHAL, INSCRIBED, HIGH-COST and FREE. LETHAL cells are obstacles. The INSCRIBED zone extends from each obstacle by the robot radius plus footprint padding; a robot centred there is certain to collide. The robot may therefore only move through HIGH-COST and FREE cells. The HIGH-COST zone surrounds the obstacles, with a cost that decays exponentially with distance, so the planner avoids it whenever a better route exists.

**Localization: Extended Kalman Filter.** The filter fuses wheel odometry with the simulator's pose measurement to produce a smooth estimate of the robot's pose. Without it, the pose fluctuates with large errors, which makes the planned paths uneven and the controller's motion jerky.

**Global planner: cost-aware A\*.** The planner searches an 8-connected grid at the map's 2 cm resolution with an octile heuristic, and diagonal moves cannot cut blocked corners. The grid
path is shortened with line-of-sight checks that never bring it closer to obstacles than the A\* path, then resampled every 5 cm. If the start is in collision (due to pose noise), planning starts from the nearest free cell within 0.3 m.

**Controller: pure pursuit with rotate-in-place.** The controller tracks a point 0.25 m ahead and turns in place when the heading error exceeds 0.6 rad. Speed is limited to 0.30 m/s and 1.5 rad/s, reduced on tight arcs and near the goal, and subject to acceleration limits. The robot stops within 2 cm and 0.03 rad of the goal, and `mark_goal` is called only after the pose has been checked against the simulator's 5 cm / 5° criterion.

**Map mismatch: bumper-driven replanning.** On contact, the robot marks an obstacle of 4 cm radius ahead of (or behind) itself in the costmap, backs up 8 cm and replans. A path is also replanned when a map change blocks it or the robot drifts more than 0.3 m from it. Each goal has limits (180 s, 10 collisions, 5 consecutive planning failures, 3 stops outside tolerance); a failed goal is skipped and the mission continues.

**Recording: ffmpeg, lossless depth, one recording per mission.** Each mission writes `rgb.mkv` (the simulator's JPEGs, stream-copied without re-encoding), `depth.mkv` (16-bit depth in millimetres, FFV1, lossless), `frames.csv` (per frame: seq, simulation time, receive time, robot pose) and `metadata.json` (intrinsics, `depth_scale`, `T_base_camera`, frame counts). Frame *i* of both videos corresponds to row *i* of the CSV. OpenCV's `VideoWriter` was rejected because it only writes 8-bit frames, which would quantise depth to about 2.3 cm. ffmpeg runs in its own session, so a terminal Ctrl-C cannot interrupt it before the files are finalised.

## 3. Known limitations

- **Unmapped obstacles are detected only on contact.** The camera is recorded but not used for navigation, so the robot may bump into the same object several times (scenes 04 and 05).
- **Orchestration is centralised in `MissionExecutor`**, which makes the system harder to extend with new features.
- **Replanning is triggered only by collisions.** This is sufficient for the static scenes in this assignment but not for dynamic obstacles, which would also require real-time obstacle mapping from the depth camera; this was out of scope.
- **No automated test suite in the repository.**

## 4. With more time

1. **Depth-camera obstacle layer:** back-project depth with the intrinsics and `T_base_camera`, mark obstacles 0–9 cm above the floor in a local costmap and replan before contact. `UpdateGlobalMap` is the hook for it.
2. **Mission design from the GUI:** convert the mission behaviours into nodes so missions can be composed as a graph or behaviour tree, replacing the all-in-one `MissionExecutor`.
3. **Automated tests and CI:** a fake simulator, unit tests for the EKF and localization, and end-to-end missions, run headless.
4. **Tighter goal tolerances** (about 1 cm / 1°) now that the pose is accurate, and a controller that predicts collisions (DWB- or MPC-style) instead of pure pursuit.
