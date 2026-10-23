# Take-home assignment: Robot Vacuum Control Software

Thank you for your interest in Eureka Robotics. This assignment is close to the work we do day to day: writing software that drives real robots, makes sense of imperfect sensor data, and gives people a clear view of what the robot is doing.

You will build control software for a simulated robot vacuum. The simulator is provided as a Docker image. Treat it like real hardware: you can only talk to it over its network interface.

## What you get

- **Simulator image:** `ghcr.io/eurekarobotics/vacuum-sim:1.0.0`
- **Simulator specification:** It describes the robot, the ZeroMQ API and the data formats.
- **Example clients:** a minimal Python client and a minimal C++ client, attached. They show how to connect, send commands and read sensor data.

## Getting started

```bash
docker pull ghcr.io/eurekarobotics/vacuum-sim:1.0.0
docker run --rm -p 5555-5558:5555-5558 -p 8080:8080 ghcr.io/eurekarobotics/vacuum-sim:1.0.0 --scene 01 --viewer
```

Open http://localhost:8080 to watch the simulated world, then run one of the example clients.

The simulator runs on Linux and macOS (Intel and Apple Silicon) with Docker. Windows with WSL2 should also work. It needs no GPU; please give Docker at least 2 CPUs.

## Your tasks

Build an application, in **Python or C++**, that does the following.

We recommend using Qt for the visualisation in the application.

### 1. Drive the robot to a goal

Use the 2D occupancy map from the simulator to plan a path from the robot's current position to a goal, and drive the robot there.

- Each scene has a few named goals. The user of your application should be able to pick one.
- The map you receive may not match the world exactly. Your software should cope with that.
- The robot can be driven with low-level wheel commands or with a simple "go to pose" command. Either is acceptable.
- When the robot has arrived, call `mark_goal` (see the simulator specification). A goal counts as reached if the robot is within 5 cm of it and, for goals with a `yaw`, facing within 5° of that yaw. Only the first mark of each goal after a load or reset counts.

### 2. Record the camera

Stream the robot's RGBD camera and store it as video for later processing.

- Another engineer should be able to use your recordings offline, so keep timing information and anything else they would need.

### 3. Visualise the robot

Build a visualiser that shows the robot moving through its environment. It can be simple, and a 2D view is sufficient. It should show at least:

- the environment, built from the data available to you;
- the robot's pose over time;
- the goal and your planned path.


Beyond that, show whatever you think makes the robot's behaviour easiest to understand.

## Scenes

Scenes `01`–`03` are simpler; `04`–`06` are more challenging. Your solution should work well on `01`–`03`. How far you get on the others is up to you, and we are just as interested in what you learned from them.

Start poses are random. Pass a seed to get a repeatable run (see the spec).

## Deliverables

Please send us:

1. **Source code**, in a git repository (a link or an archive), with a README covering how to build, run and test it.
2. **A short design note**, 1–2 pages: architecture, key decisions and trade-offs, known limitations, and what you would do with more time.
3. **A screen recording**, 2–5 minutes: your application driving the robot to a few goals with the visualiser running on different scenes.
4. **Example camera recording** produced by your software.

## What we look for

The point of this assignment is the application you build, not how fast the robot reaches its goal.

We will run your application ourselves on several scenes and start poses. The simulator logs every `mark_goal` call and every collision, so we will see which goals were reached, how often the robot hit something, and how it moved. We will then read your code and design note. Overall, we will look at:

- **Motion planning algorithm:** Basic understanding and implementation of robotics algorithm to plan robot motion.
- **Motion quality:** how smoothly and efficiently the robot moves.
- **Software design:** clear structure, sensible separation of concerns, and code that a teammate could extend.
- **Robustness:** how your software behaves when things don't go to plan, as they do on real hardware.
- **The visualiser and user experience:** is it clear what the robot is doing and why?
- **Data recording:** whether the recordings are usable and complete.
- **Testing and documentation:** enough that we can run it and trust it.

Use any libraries and tools you like; please list them in your README. 

AI code assistants are fine, just record what you use and how.


## Logistics

- You have one week to submit.
- Please send your submission to the recruiter.
- If something is unclear, or you think the simulator has a bug, email us directly. Asking questions is welcome and is not held against you. If you have to make an assumption, write it down in your design note.

During the follow-up interview we will ask you to walk us through your solution and explain your decisions. Please have your development environment ready.

Good luck, and have fun!


# Appendix: Simulator specification

The simulator stands in for a robot vacuum and its environment. Treat it like real hardware: you talk to it only over ZeroMQ, and the map you are given may not be complete.

## 1. Quick start

```bash
docker run --rm -p 5555-5558:5555-5558 -p 8080:8080 ghcr.io/eurekarobotics/vacuum-sim:1.0.0 --scene 01 [--viewer] [--seed N]
```

| Port | Socket (sim side) | Your socket | Purpose                   |
|------|-------------------|-------------|---------------------------|
| 5555 | REP               | REQ         | RPC: scenes, map, info    |
| 5556 | PULL              | PUSH        | Robot commands            |
| 5557 | PUB               | SUB         | Robot state, 50 Hz        |
| 5558 | PUB               | SUB         | RGBD camera, 15 Hz        |
| 8080 | HTTP              | browser     | Debug viewer (`--viewer`) |

Like real hardware, commands, state and camera data may be noisy, delayed or lost.

## 2. Conventions

- Units are SI: metres, radians, seconds.
- **World frame:** x/y on the floor, z up. Yaw is counter-clockwise from +x, in (-π, π].
- **Robot frame:** origin at the centre of the disc on the floor, x forward, z up.
- **Camera frame:** OpenCV convention (x right, y down, z forward).
- **Time:** `t` is the simulation time in seconds since the last `load_scene` or `reset`. It runs at wall-clock rate.
- **Messages:** every message is a ZeroMQ multipart message. Frame 0 is a UTF-8 JSON header. Any further frames are binary payloads that the header describes.
- Every stream message carries a `seq` counter.

## 3. Robot

| Property             | Value                                                      |
|----------------------|------------------------------------------------------------|
| Shape                | disc, Ø 0.34 m, h 0.09 m                                   |
| Drive                | differential, kinematic, 100 Hz control loop               |
| Wheel base / radius  | 0.23 m / 0.035 m                                           |
| Max wheel speed      | 10 rad/s                                                   |
| Max linear speed     | 0.35 m/s, acceleration 1.0 m/s²                            |
| Max angular speed    | 2.0 rad/s, acceleration 6.0 rad/s²                         |
| Encoders             | 1024 ticks/rev per wheel, cumulative, signed               |
| Bumper               | single boolean, covers the whole perimeter                 |
| Camera               | RGBD, 640×480, 15 Hz, forward-facing, 8 cm above the floor |

Exact values are available at runtime through `get_robot_info` and `get_camera_info`.

The robot cannot pass through obstacles. On contact it stops at the point of contact (it does not slide along the obstacle), the wheels stall (encoder ticks stop), and `bumper` reads `true` until the robot has moved away from the obstacle. Turning in place is always possible. The robot fits under furniture with more than 9 cm of clearance.

## 4. Scenes

Scenes are numbered `01` … `06`. `01`–`03` are easy, `04`–`06` advanced.

- Each scene has a fixed list of named **goals**. Each goal is a pose `[x, y]` or `[x, y, yaw]`.
- The **start pose** is randomised on every load/reset. Pass a `seed` to make it reproducible.
- Every goal is reachable from every start by the real robot.
- The provided map **may be incomplete**. The real environment can contain obstacles that are not on the map. They are at least 5 cm tall.

## 5. RPC (port 5555)

Request:
```json
{"method": "<name>", "params": {...}}
```

Response (frame 0):
```json
{"ok": true, "result": {...}}
{"ok": false, "error": "message"}
```

| Method            | Params                     | Result                                                                    |
|-------------------|----------------------------|---------------------------------------------------------------------------|
| `ping`            | —                          | `{"t": float}`                                                            |
| `list_scenes`     | —                          | `[{"id": "01", "name": str, "difficulty": "easy"\|"advanced"}]`           |
| `load_scene`      | `id`, `seed?`              | `{"id", "start": {"x", "y", "yaw"}, "goals": [...]}`; `start` is exact    |
| `reset`           | `seed?`                    | same as `load_scene`, current scene                                       |
| `get_goals`       | —                          | `[{"name": str, "pose": [x, y, yaw?]}]`                                   |
| `get_map`         | —                          | header + PNG frame (section 7)                                            |
| `get_robot_info`  | —                          | dimensions, limits, `ticks_per_rev`, `watchdog`, rates                    |
| `get_camera_info` | —                          | `{"width", "height", "fx", "fy", "cx", "cy", "depth_scale", "max_depth", "rate", "T_base_camera": 4×4}` |
| `mark_goal`       | `name`                     | `{"t": float}`: simulation time of the mark                              |

Call `mark_goal` with a goal's name once your robot has arrived at that goal. It only records the arrival; it does not tell you how close you are.

`load_scene` takes about a second. `T_base_camera` maps camera-frame points to the robot frame. Intrinsics follow OpenCV (pixel centres at integer coordinates): `fx = fy ≈ 415.69`, `cx = 319.5`, `cy = 239.5`.

## 6. Commands (port 5556)

Send one JSON frame per command.

### 6.1 Wheel velocities

```json
{"type": "wheels", "left": 3.2, "right": 4.1}
```
- Units are rad/s per wheel, clamped to the wheel speed limit, then to the linear and angular speed limits. Acceleration is limited.
- **Watchdog:** if no `wheels` command arrives for 250 ms, the wheels stop. Send at a steady rate, e.g. 20 Hz.

### 6.2 Go to pose (easy mode)

```json
{"type": "goto", "id": 7, "x": 1.5, "y": -0.4, "yaw": 1.57}
{"type": "cancel", "id": 7}
```
- The robot turns in place, drives in a **straight line**, then turns to `yaw` if one is given. It may stop a few centimetres and degrees off the target.
- It does **not** avoid obstacles: on contact it stops with status `bumped`. Planning and waypoints are up to you.
- Progress is reported in `state.goto`. A goto with an already-seen id is ignored; use a new id for a new target.
- A new `goto` or any `wheels` command pre-empts the current `goto`. The watchdog does not apply to `goto`.

## 7. Map (`get_map`)

Frame 0: `{"ok": true, "result": {"resolution": 0.02, "origin": [x, y], "width": W, "height": H}}`
Frame 1: PNG, 8-bit grey, H rows × W columns: 0 = free, 255 = occupied.

- Row 0 is the top of the image (largest y); rows grow towards −y, columns towards +x.
- `origin` is the world position of the **bottom-left corner** of pixel (row H−1, col 0). The centre of pixel (row r, col c) is
  `x = origin[0] + (c + 0.5)·resolution`, `y = origin[1] + (H − 1 − r + 0.5)·resolution`.
- The map shows everything the robot can collide with at its height (0–9 cm), including obstacles too low for the camera to see, but not furniture the robot fits under (a table shows only its legs).
- Space outside the walls is shown as free; it is not reachable.

## 8. Streams

### 8.1 State (port 5557, 50 Hz)

```json
{
  "t": 12.34, "seq": 617, "scene": "01",
  "pose": {"x": 0.52, "y": -1.10, "yaw": 0.31},
  "wheels": {"left_ticks": 10432, "right_ticks": 10511},
  "bumper": false,
  "goto": {"id": 7, "status": "running"}
}
```
- `goto` describes the most recent goto: `status` is one of `running`, `reached`, `bumped`, `cancelled`, `preempted`. Both fields are `null` if there was no goto since the last load/reset.

### 8.2 Camera (port 5558, 15 Hz)

Frames:
1. Header: `{"t", "seq", "pose": {...}, "width", "height", "rgb": "jpeg", "depth": "u16"}`. `pose` is the robot pose at capture time.
2. RGB as JPEG.
3. Depth as raw little-endian `uint16`, H×W, row-major. Multiply by `depth_scale` (0.001) to get metres. A value of 0 means invalid or out of range (closer than 0.05 m or farther than `max_depth` = 6 m).

Depth is z-depth along the optical axis, not distance along the ray. Use the intrinsics from `get_camera_info` to back-project. Depth and RGB are pixel-aligned.

## 9. Debug viewer

When started with `--viewer`, http://localhost:8080 shows a top-down view of the true world (including obstacles missing from the map), the map, the true and noisy robot pose, the goals and the camera image. It is meant for debugging only: your own software must use the ZMQ API.

## 10. Examples

- `examples/client.py`: Python (pyzmq, numpy, pillow).
- `examples/client.cpp`: C++ (cppzmq + nlohmann/json); `g++ -std=c++17 client.cpp -lzmq` or the CMakeLists.txt next to it.

```python
import json, zmq
ctx = zmq.Context()
rpc = ctx.socket(zmq.REQ); rpc.connect("tcp://localhost:5555")
rpc.send_json({"method": "load_scene", "params": {"id": "01", "seed": 42}})
print(rpc.recv_json())

cmd = ctx.socket(zmq.PUSH); cmd.connect("tcp://localhost:5556")
cmd.send_json({"type": "goto", "id": 1, "x": 1.0, "y": 0.0})

state = ctx.socket(zmq.SUB); state.connect("tcp://localhost:5557")
state.setsockopt(zmq.SUBSCRIBE, b"")
print(json.loads(state.recv()))
```
