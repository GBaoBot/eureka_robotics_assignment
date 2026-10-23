# Robot vacuum control software

Drives the simulated robot vacuum to named goals, records its RGBD camera for offline use and
visualises what it is doing, using only the simulator's ZeroMQ API.

- Path planning (cost-aware A\*) and path following (pure pursuit) on the occupancy map,
  with replanning when the map turns out to be wrong.
- EKF localization fusing wheel odometry with the simulator's noisy pose.
- Lossless camera recording with per-frame timing and pose.
- Qt visualiser and a command-line front end.

**Design note** (architecture, decisions, trade-offs, limitations, results): [docs/design.md](docs/design.md)

## Requirements

- Docker with Compose, on Linux, macOS or Windows with WSL2.
- The simulator image `ghcr.io/eurekarobotics/vacuum-sim:1.0.0`.

Everything else runs in the `core` container (`docker/Dockerfile`). The compose file forwards
the display for WSL2 (WSLg); on other hosts, adjust `DISPLAY` and the X11 mount in
`docker-compose.yaml`.

## Build and run

1. Start the simulator (only one simulator at a time; it uses ports 5555–5558 and 8080):
   ```bash
   docker compose up -d sim
   ```
   Its debug viewer is at http://localhost:8080.

2. Build and enter the application container (the repository is mounted at `~/robot_ws`):
   ```bash
   docker compose build core
   docker compose up -d core
   docker compose exec core bash
   ```

3. Run the visualiser:
   ```bash
   python3 scripts/visualizer/app.py --scene 01 --seed 42
   ```
   - Pick a scene and optional seed, then **Load scene**.
   - Select goals and press **Go to selected**, double-click one goal, or press **Go to all**.
     **Stop** halts the robot.
   - Scroll to zoom, drag to pan, double-click to fit the map.
   - Closing the window or pressing Ctrl-C stops the robot and saves the recording.

   Or without a GUI:
   ```bash
   python3 scripts/cli/app.py --list                       # goals of the loaded scene
   python3 scripts/cli/app.py --scene 03 --seed 7 --all    # every goal, in order
   python3 scripts/cli/app.py --scene 01 kitchen living    # chosen goals
   python3 scripts/cli/app.py                              # choose goals interactively
   ```
   Exit code: 0 all goals reached, 1 a goal failed or the mission aborted, 2 simulator
   unreachable or no goals given, 130 interrupted.

Options for both front ends:

| Option | Effect |
|---|---|
| `--scene ID`, `--seed N` | load a scene with a reproducible start pose (default: keep the loaded scene) |
| `--no-ekf` | use the simulator's raw pose instead of the EKF |
| `--no-record`, `--output DIR` | turn camera recording off, or change its folder (default `assets/videos`) |
| `--goal-timeout S` | seconds allowed per goal (default 180) |
| `--host H`, `-v` | simulator host (default `localhost`); debug logging |

## Camera recordings

Each mission writes `assets/videos/<time>_scene<id>_<goals>/` (ignored by git):

| File | Contents |
|---|---|
| `rgb.mkv` | RGB as the simulator's JPEGs, stream-copied (MJPEG) |
| `depth.mkv` | depth, 16-bit millimetres, lossless (FFV1 `gray16le`); 0 = invalid |
| `frames.csv` | one row per frame: `frame, seq, t` (simulation s), `received` (unix s), robot `x, y, yaw` |
| `metadata.json` | camera intrinsics, `depth_scale`, `T_base_camera`, frame counts, file formats |

Frame *i* of both videos is row *i* of `frames.csv`; use the CSV for timing. Reading depth in Python:

```python
import subprocess, numpy as np
raw = subprocess.run(["ffmpeg", "-loglevel", "error", "-i", "depth.mkv", "-f", "rawvideo",
                      "-pix_fmt", "gray16le", "-"], capture_output=True, check=True).stdout
depth_m = np.frombuffer(raw, "<u2").reshape(-1, 480, 640) * 0.001   # 0 = invalid
```

## Testing

There is no automated test suite in the repository yet (see the design note's limitations).
To check a build end to end against the simulator, run every goal of each scene and look at
the exit code and per-goal results:

```bash
for s in 01 02 03 04 05 06; do python3 scripts/cli/app.py --scene $s --seed 7 --all --no-record; done
```

Comparing runs with and without `--no-ekf` shows the effect of the localization filter.

## Project layout

```
scripts/
  drivers/            ZeroMQ clients: RPC, robot commands/state, camera
  localization/       EKF and the Localization stage (raw state -> pose)
  navigation/         costmap, A* planner, pure-pursuit controller
  mission_behaviors/  NavigateToGoal(s), behaviours, telemetry, camera recorder
  mission_executor/   MissionExecutor: the entry point front ends use
  visualizer/         Qt front end
  cli/                command-line front end
docs/design.md        design note
docker/, docker-compose.yaml
examples/             the simulator's example clients
```

## Libraries and tools

- Python 3.12, NumPy, pyzmq, Pillow, PyQt6 (Qt 6), and ffmpeg (camera encoding), installed
  in `docker/Dockerfile`.
- Docker / Docker Compose, Ubuntu 24.04 base image.

## Use of AI assistants

Claude Code (Anthropic) was used as a coding assistant: to discuss designs, support to write and refactor
code, write test scripts against a fake simulator and the real simulator. Design choices were made
and reviewed by the author.
