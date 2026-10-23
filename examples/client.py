"""Minimal Python client: RPC, map, wheels, goto, state and camera streams.

pip install pyzmq numpy pillow; python client.py [--host localhost]
"""
import argparse
import io
import json
import math
import time

import numpy as np
import zmq
from PIL import Image

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="localhost")
ap.add_argument("--port-base", type=int, default=5555)
args = ap.parse_args()


def url(k):
    return f"tcp://{args.host}:{args.port_base + k}"


ctx = zmq.Context()
rpc = ctx.socket(zmq.REQ)
rpc.connect(url(0))
cmd = ctx.socket(zmq.PUSH)
cmd.connect(url(1))
state = ctx.socket(zmq.SUB)
state.connect(url(2))
state.setsockopt(zmq.SUBSCRIBE, b"")
camera = ctx.socket(zmq.SUB)
camera.setsockopt(zmq.RCVHWM, 2)
camera.connect(url(3))
camera.setsockopt(zmq.SUBSCRIBE, b"")


def call(method, **params):
    rpc.send_json({"method": method, "params": params})
    frames = rpc.recv_multipart()
    rep = json.loads(frames[0])
    if not rep["ok"]:
        raise RuntimeError(rep["error"])
    return rep["result"], frames[1:]


def latest(sock):
    """Most recent message: drop anything queued (streams are 'latest wins')."""
    msg = sock.recv_multipart()
    while sock.poll(0):
        msg = sock.recv_multipart()
    return msg


# Load a scene, read the robot and camera info, save the map.
print("scenes:", [s["id"] for s in call("list_scenes")[0]])
scene, _ = call("load_scene", id="01", seed=42)
print("start:", scene["start"], "goals:", [g["name"] for g in scene["goals"]])
robot, _ = call("get_robot_info")
cam, _ = call("get_camera_info")
map_info, (png,) = call("get_map")
Image.open(io.BytesIO(png)).save("map.png")

# Wheels: rad/s per wheel, resend at 20 Hz (250 ms watchdog). Turn in place at 1 rad/s for 1.5 s.
turn_rate = 1.0
wheel_speed = turn_rate * robot["wheel_base"] / 2 / robot["wheel_radius"]
for _ in range(30):
    cmd.send_json({"type": "wheels", "left": -wheel_speed, "right": wheel_speed})
    time.sleep(0.05)
robot_state = json.loads(latest(state)[0])
print("after turning:", robot_state["pose"], robot_state["wheels"])

# goto 0.3 m straight ahead, then follow the state stream for 3 s.
pose = robot_state["pose"]
x = pose["x"] + 0.3 * math.cos(pose["yaw"])
y = pose["y"] + 0.3 * math.sin(pose["yaw"])
cmd.send_json({"type": "goto", "id": 1, "x": x, "y": y})
end = time.monotonic() + 3.0
while time.monotonic() < end:
    robot_state = json.loads(latest(state)[0])
print("goto:", robot_state["goto"], "pose:", robot_state["pose"], "bumper:", robot_state["bumper"])

# Camera: header, JPEG, uint16 depth (mm).
header, jpg, raw = latest(camera)
header = json.loads(header)
rgb = np.array(Image.open(io.BytesIO(jpg)))
depth = np.frombuffer(raw, "<u2").reshape(header["height"], header["width"]) * cam["depth_scale"]

print(header)

# Save one camera frame: RGB, depth and metadata
stem = f"frame_{header['seq']:06d}"

# Back-project the centre pixel: pixel -> camera frame -> robot frame -> world frame.
u, v = header["width"] // 2, header["height"] // 2
z = depth[v, u]
x_camera = (u - cam["cx"]) * z / cam["fx"]
y_camera = (v - cam["cy"]) * z / cam["fy"]
p_camera = np.array([x_camera, y_camera, z, 1.0])
p_robot = np.array(cam["T_base_camera"]) @ p_camera
pose = header["pose"]
cos_yaw, sin_yaw = math.cos(pose["yaw"]), math.sin(pose["yaw"])
x_world = pose["x"] + cos_yaw * p_robot[0] - sin_yaw * p_robot[1]
y_world = pose["y"] + sin_yaw * p_robot[0] + cos_yaw * p_robot[1]
p_world = (x_world, y_world, p_robot[2])
print(f"camera seq {header['seq']}: rgb {rgb.shape}, centre depth {z:.3f} m -> world {np.round(p_world, 3)}")