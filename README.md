# Robot vacuum control software

Drives the simulated robot vacuum to named goals, records its RGBD camera for offline use and
visualises what it is doing, using only the simulator's ZeroMQ API.

- Path planning (cost-aware A\*) and path following (pure pursuit) on the occupancy map,
  with replanning when hitting obstacles.
- EKF localization fusing wheel odometry with the simulator's noisy pose.
- Lossless camera recording with per-frame timing and pose.
- Qt visualiser to load scenes, start missions and watch the robot.

**Design note** (architecture, decisions, trade-offs, limitations, results): [docs/design.md](docs/design.md)

## Requirements

- Docker with Compose, on Linux, macOS or Windows with WSL2.
- The simulator image `ghcr.io/eurekarobotics/vacuum-sim:1.0.0`.

## Build and run

1. Pull and Start the simulator:
   ```bash
   # Pull the docker image (only run once)
   docker pull ghcr.io/eurekarobotics/vacuum-sim:1.0.0

   # Start simulator container
   docker run --rm -p 5555-5558:5555-5558 -p 8080:8080 ghcr.io/eurekarobotics/vacuum-sim:1.0.0 --scene 01 --viewer
   ```
   Its debug viewer is at http://localhost:8080.

2. Build and run the application container (the repository is mounted at `~/robot_ws`):
   ```bash
   docker compose build core
   docker compose up -d sim core
   docker compose exec core python3 -m vacuum.visualizer.app
   ```

   - Pick a scene and optional seed, then **Load scene**.
   - Select goals and press **Go to selected**, double-click one goal, or press **Go to all**.
     **Stop** halts the robot.
   - Scroll to zoom, drag to pan, double-click to fit the map.
   - Closing the window or pressing Ctrl-C stops the robot and saves the recording.

## Camera recordings

Each mission writes `assets/videos/<time>_scene<id>_<goals>/` (ignored by git):

| File | Contents |
|---|---|
| `rgb.mkv` | RGB as the simulator's JPEGs, stream-copied (MJPEG) |
| `depth.mkv` | depth, 16-bit millimetres, lossless (FFV1 `gray16le`); 0 = invalid |
| `frames.csv` | one row per frame: `frame, seq, t` (simulation s), `received` (unix s), robot `x, y, yaw` |
| `metadata.json` | camera intrinsics, `depth_scale`, `T_base_camera`, frame counts, file formats |

## Project layout

```
vacuum/               the Python package (run with python3 -m vacuum.<front end>.app)
  common/             shared data types (Pose, OccupancyMap); no I/O, NumPy only
  drivers/            ZeroMQ clients: RPC, robot commands/state, camera
  localization/       EKF and the Localization stage (raw state -> pose)
  navigation/         costmap, A* planner, pure-pursuit controller
  mission_behaviors/  NavigateToGoal(s), behaviours, telemetry, camera recorder
  mission_executor/   MissionExecutor: the entry point front ends use
  visualizer/         Qt front end
pyproject.toml        package metadata; `pip install -e .` adds the vacuum-viewer command
docs/design.md        design note
docker/, docker-compose.yaml
examples/             the simulator's example clients
```

## Results

Real simulator, seed 7, all goals per scene, 90 s per goal:

| Scene | 01 | 02 | 03 | 04 | 05 | 06 |
|---|---|---|---|---|---|---|
| Goals reached | 4/4 | 4/4 | 5/5 | 3/3 | 4/5 | 6/6 |
| Collisions | 1 | 0 | 1 | 3 | 4 | 0 |

All collisions were with obstacles missing from the map.

## Reference
This project refers to different sources for implementation:
- `Nav2`: https://docs.nav2.org/rolling/
- `robot_localization`: https://github.com/cra-ros-pkg/robot_localization
- `ZMQ The guide`: https://zguide.zeromq.org/
- `Qt Widget`: https://doc.qt.io/qt-6/widgets-tutorial.html

## Use of AI assistants

Claude Code (Anthropic) was used as a coding assistant: to discuss designs, support to write and refactor code, write test scripts against a fake simulator and the real simulator. Design choices were made and reviewed by the author.
