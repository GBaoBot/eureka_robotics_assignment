# Design note: robot vacuum control software

Python application that drives the simulated robot to named goals, records its RGBD camera
and visualises what it is doing. It talks to the simulator only over its ZeroMQ API.

## 1. Architecture

```
 front ends     visualizer/app.py (Qt GUI)           cli/app.py (terminal)
                          \                              /
 entry point            MissionExecutor.connect(config)      composition root, mission thread
                          |
 mission        NavigateToGoals -> NavigateToGoal -> behaviours (FollowPath, BackUp, MarkGoal, ...)
                          |                    \
 navigation     Costmap2D, AStarPlanner,         RecordCamera (ffmpeg)
                PurePursuitController
                          ^ poses
 localization   Localization thread (EkfLocalizer | PassThrough)
                          ^ raw states            | commands
 drivers        RpcClient, RobotDriver, CameraDriver  <---------- ZeroMQ ----------> simulator
```

- **Drivers** wrap the four sockets. `RobotDriver` resends the wheel command at 20 Hz (the
  simulator stops the wheels after 250 ms without one) and sends zeros if the mission stops
  updating it for 0.5 s. `RpcClient` recreates its REQ socket after a 5 s timeout ("lazy pirate").
- **Localization** is its own stage, and depends only on `common`: it
  takes measurements pushed to `submit()` (any object with pose, wheel ticks and timestamps)
  and publishes a filtered pose (`latest()` / `wait_for()`). `MissionExecutor` wires
  `RobotDriver.on_state` to it, so neither knows the other. Navigation reads poses from it
  and sends commands to the driver.
- **Mission** (`mission_behaviors/`): `NavigateToGoal` runs once per localized state
  (50 Hz), like a simplified Nav2 behaviour tree: update map → collision? back up →
  path valid? else replan → follow path → arrived? `mark_goal`. `NavigateToGoals` visits goals
  in order with one costmap, so obstacles found on the way to one goal are kept for the next.
- **MissionExecutor** is the only entry point to the core. It builds and wires everything,
  runs missions and scene loads in a background thread (`start`, `cancel`, `load_scene`
  return immediately) and exposes read-only status. Front ends contain no robot logic.
- **Visualiser**: the mission writes a `Telemetry` object (one writer; values are replaced,
  never mutated), and the GUI polls it and the latest pose with a 30 Hz `QTimer`. The views
  are pure renderers. Nothing outside `visualizer/` imports Qt.

## 2. Key decisions and trade-offs

**Motion: own planner and controller with wheel commands, not `goto`.** `goto` drives
straight lines without avoiding obstacles. Wheel velocity commands let the controller follow
a curved path smoothly and slow down for turns and the goal.

**Global planner: cost-aware A\*.** 8-connected grid at the map's 2 cm resolution, octile
heuristic. The costmap is inflated as in Nav2: cells within the robot radius (from
`get_robot_info`, 0.17 m) plus 2 cm padding are blocked, and the cost decays up to 0.45 m.
The step cost is `length × (1 + 3 × cost/252)`, so paths keep away from walls where there is
room but still use narrow gaps. Diagonal moves can't cut blocked corners. The grid path is
shortened by line-of-sight checks that never pass closer to obstacles than the A\* path, then
resampled every 5 cm. If the start is in collision (pose noise), planning starts from the
nearest free cell within 0.3 m. Plans take 15–80 ms in Python, so replanning is cheap.

**Controller: pure pursuit with rotate-in-place.** It tracks a point 0.25 m ahead and turns
in place when the heading error exceeds 0.6 rad. Speed is limited to 0.30 m/s and 1.5 rad/s,
reduced on tight arcs and near the goal, with acceleration limits. It stops within 2 cm and
0.03 rad of the goal, and `mark_goal` is only called after the pose is checked against the
simulator's 5 cm / 5° criterion.

**Map mismatch: bumper-driven replanning.** On contact the robot marks an obstacle of 4 cm radius
ahead of (or behind) it in the costmap, backs up 8 cm and replans. A path is also replanned
if a map change blocks it or the robot is more than 0.3 m off it. Each goal has limits
(180 s, 10 collisions, 5 failed plans in a row, 3 stops outside tolerance); a failed goal is
skipped and the mission continues.

**Localization: EKF on wheel odometry and the simulator's pose.** Measured on the simulator
before designing the filter:

| | Measured |
|---|---|
| Reported pose | white noise, σ ≈ 1.0 cm (x, y) and 0.5° (yaw), moving or still |
| Wheel ticks | quantisation only; odometry drifts slowly, up to 3.7° in yaw over a 32 s drive |
| State stream | 50 Hz, 6.6 % of messages lost while driving; pose and ticks in sync |

State (x, y, yaw); predict with wheel-tick increments (noise grows with distance driven),
update with the reported pose (H = I), Joseph-form covariance update, 99.9 % Mahalanobis
gate, re-initialisation after 0.5 s of rejected measurements. The filter starts from
`load_scene`'s exact start pose. Tuned on a recorded drive: mean NIS 2.89 (ideal 3), pose
jitter 13.9 → 2.1 mm. On a fake robot with the measured noise, turn-rate command changes per
tick dropped by 45 %. A standard EKF was chosen over a square-root form: with 3 states in
float64 the Joseph form is numerically safe. `--no-ekf` runs the same pipeline with the raw
pose.

**Recording: ffmpeg, lossless depth, one recording per mission.** Each mission writes
`rgb.mkv` (the simulator's JPEGs stream-copied, no re-encoding), `depth.mkv` (16-bit depth in
mm, FFV1, lossless), `frames.csv` (per frame: seq, simulation time, receive time, robot pose)
and `metadata.json` (intrinsics, `depth_scale`, `T_base_camera`, frame counts). Frame *i* of
both videos is row *i* of the CSV. OpenCV's `VideoWriter` was rejected because it only writes
8-bit frames, which would quantise depth to about 2.3 cm. ffmpeg runs in its own session so a
terminal Ctrl-C can't cut it off before the files are finalised.

**Robustness.** A stalled state stream stops the robot after 1 s and aborts the mission
after 5 s. Window close, Stop and Ctrl-C cancel the mission within one control tick and
finalise the recording. A mission refuses to start if recording was requested but can't
start, so no run is silently unrecorded. One CPython pitfall is handled explicitly: a Ctrl-C
during `Thread.join()` marks the thread as finished while it still runs, so the executor
tracks completion with its own `threading.Event`.

## 3. Results

Real simulator, seed 7, all goals per scene, 90 s per goal:

| Scene | 01 | 02 | 03 | 04 | 05 | 06 |
|---|---|---|---|---|---|---|
| Goals reached | 4/4 | 4/4 | 5/5 | 3/3 | 4/5 | 6/6 |
| Collisions | 1 | 0 | 1 | 3 | 4 | 0 |

All collisions were with obstacles missing from the map. The failed goal (`bed`, scene 05)
timed out after repeated bumps near it.

## 4. Known limitations

- **Unmapped obstacles are only found by touching them.** The camera is recorded but not used
  for navigation, so the robot can bump the same object several times (scenes 04 and 05).
  Each bump marks only a 4 cm-radius obstacle.
- **Arrival accuracy is limited by the controller's 2 cm stop tolerance**, not the pose
  (EKF σ ≈ 2 mm). One fake-robot run stopped 4.4 cm from a goal.
- **About 4 % of camera frames are missing** from recordings (seq gaps); it is not yet known
  whether the simulator or the recorder drops them.
- **No automated test suite in the repository.** Development tests (EKF unit tests, a fake
  simulator, end-to-end missions) were run outside it.
- A scene load can't be cancelled, and recordings cover missions only, not idle time.

## 5. With more time

1. **Depth-camera obstacle layer**: back-project depth with the intrinsics and
   `T_base_camera`, mark obstacles 0–9 cm above the floor in a local costmap and replan before
   contact. `UpdateGlobalMap` is the hook for it.
2. **Automated tests and CI**: the fake simulator, EKF and localization unit tests, and
   end-to-end missions, run headless.
3. **Tighter goal tolerances** (about 1 cm / 1°) now that the pose is accurate, and a
   controller that predicts collisions (DWB- or MPC-style) instead of pure pursuit.
4. **Find the camera frame loss**, and move hot paths (A\*, costmap inflation) to numpy or C++
   for larger maps.

## Assumptions

- The pose noise is stationary and white, as measured; the filter parameters are tuned for it.
- `load_scene`'s start pose is exact (as the specification states); the filter starts from it.
- Goals are only marked after the robot has stopped inside 5 cm / 5° by its own estimate; the
  simulator's judgement is not visible to the application.
