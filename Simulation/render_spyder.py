"""Render the whole-robot previews: home pose, a joint-range pose and a walking GIF."""
from pathlib import Path
import math

import numpy as np
import pybullet as p
from PIL import Image

import simulate_spyder as sim

PREVIEWS = Path(__file__).resolve().parent / "previews"
WIDTH, HEIGHT = 960, 720
# One leg per joint type, moved well away from home so each motion is visible.
POSED_DEGREES = {"fr_coxa_yaw": 35, "mr_femur_pitch": -45, "rr_tibia_pitch": -60,
                 "fl_coxa_yaw": -35, "fl_femur_pitch": -30, "ml_tibia_pitch": 35, "rl_femur_pitch": 40}


def frame(target, distance=0.9):
    view = p.computeViewMatrixFromYawPitchRoll(list(target), distance, 35, -28, 0, 2)
    projection = p.computeProjectionMatrixFOV(45, WIDTH / HEIGHT, 0.01, 5)
    pixels = p.getCameraImage(WIDTH, HEIGHT, view, projection, renderer=p.ER_TINY_RENDERER, shadow=1)[2]
    return Image.fromarray(np.asarray(pixels, dtype=np.uint8).reshape(HEIGHT, WIDTH, 4)).convert("RGB")


def still(name, angles):
    ground = sim.connect(True, False)
    robot = sim.Spyder(position=(0, 0, 0.25), fixed_base=True, ground=ground)
    robot.pose({joint: math.radians(value) for joint, value in angles.items()})
    frame([0, 0, 0.2]).save(PREVIEWS / name)
    p.disconnect()


def walking(name, seconds=2.4, frames=24):
    ground = sim.connect(True, True)
    robot = sim.Spyder(ground=ground)
    gait = sim.TripodGait(robot)
    home = {joint: 0.0 for joint in robot.joints}
    for _ in range(120):
        robot.command(home)
        p.stepSimulation()
    for _ in range(int(gait.period / sim.TIME_STEP)):  # Skip the stride growing from rest.
        robot.command(gait.targets(0, 0))
        p.stepSimulation()
    images, every, steps = [], max(1, int(seconds / sim.TIME_STEP / frames)), 0
    start = gait.period
    while steps < seconds / sim.TIME_STEP:
        robot.command(gait.targets(start + steps * sim.TIME_STEP, 0.06))
        p.stepSimulation()
        if steps % every == 0:
            x, y, z = robot.body_pose()[0]
            images.append(frame([x, y, z - 0.03]).resize((WIDTH * 2 // 3, HEIGHT * 2 // 3)))
        steps += 1
    images[0].save(PREVIEWS / name, save_all=True, append_images=images[1:], duration=int(seconds * 1000 / len(images)), loop=0)
    p.disconnect()


if __name__ == "__main__":
    still("spyder_home.png", {})
    still("spyder_joints.png", POSED_DEGREES)
    walking("spyder_walk.gif")
