"""Inspect the assembled leg: python simulate.py (or --direct --demo).

The default mode poses joints kinematically. --physics enables approximate
position-control dynamics; its masses/collisions are not hardware validation.
"""
from pathlib import Path
import argparse
import math
import time

import numpy as np
import pybullet as p
import pybullet_data

from kinematics import LegKinematics

HERE = Path(__file__).resolve().parent
START_DEGREES = {"coxa_yaw": 0.0, "femur_pitch": 20.0, "tibia_pitch": 35.0}


def save_screenshot(path, target, distance):
    from PIL import Image

    width, height = 1280, 960
    view = p.computeViewMatrixFromYawPitchRoll(target, distance, 45, -25, 0, 2)
    projection = p.computeProjectionMatrixFOV(45, width / height, 0.005, 3)
    pixels = p.getCameraImage(width, height, view, projection,
                              renderer=p.ER_TINY_RENDERER, shadow=1)[2]
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.asarray(pixels, dtype=np.uint8).reshape(height, width, 4)).convert("RGB").save(path)
    print(f"Saved preview: {path.resolve()}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    model_group = parser.add_mutually_exclusive_group()
    model_group.add_argument("--mesh", action="store_true", help="Use the assembled CAD model (the default)")
    model_group.add_argument("--primitive", action="store_true", help="Use the original illustrative box model")
    parser.add_argument("--direct", action="store_true", help="Run without a GUI and exit after --steps")
    parser.add_argument("--steps", type=int, default=240, help="Headless simulation steps (default: 240)")
    parser.add_argument("--physics", action="store_true", help="Enable approximate dynamics under gravity")
    pose_group = parser.add_mutually_exclusive_group()
    pose_group.add_argument("--demo", action="store_true", help="Animate all three joints inside their limits")
    pose_group.add_argument("--angles", type=float, nargs=3, metavar=("YAW", "HIP", "KNEE"),
                            help="Joint angles in degrees")
    pose_group.add_argument("--target", type=float, nargs=3, metavar=("X", "Y", "Z"),
                            help="Solve foot position in metres relative to base_link")
    parser.add_argument("--screenshot", type=Path, help="Save a PNG of the final pose (implies --direct)")
    parser.add_argument("--base-height", type=float, default=0.28, help="Root link height above the ground, metres")
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")
    if not math.isfinite(args.base_height):
        parser.error("--base-height must be finite")
    direct = args.direct or args.screenshot is not None
    urdf = HERE / ("leg_primitives.urdf" if args.primitive else "leg_assembled.urdf")
    if not urdf.is_file():
        parser.error(f"Missing {urdf.name}; run the assembly builder first")
    try:
        model = LegKinematics(urdf)
        if set(model.names) != set(START_DEGREES):
            raise ValueError(f"Unexpected leg joints: {model.names}")
        initial = np.clip(np.radians([START_DEGREES[n] for n in model.names]), model.lower, model.upper)
        if args.angles is not None:
            initial = model.check_limits(np.radians(args.angles))
        elif args.target is not None:
            initial = model.inverse(args.target, initial)
            print("IK angles (degrees):", tuple(round(float(v), 4) for v in np.degrees(initial)))
    except (ValueError, OSError) as error:
        parser.error(str(error))

    client = p.connect(p.DIRECT if direct else p.GUI)
    if client < 0:
        raise RuntimeError("Could not connect to PyBullet")
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81 if args.physics else 0)
        p.setTimeStep(1 / 240)
        p.loadURDF("plane.urdf")
        base = np.array([0, 0, args.base_height])
        robot = p.loadURDF(str(urdf), basePosition=base.tolist(), useFixedBase=True)
        joint_by_name, link_by_name = {}, {}
        for index in range(p.getNumJoints(robot)):
            info = p.getJointInfo(robot, index)
            name = info[1].decode("utf-8")
            link_by_name[info[12].decode("utf-8")] = index
            if info[2] == p.JOINT_REVOLUTE:
                joint_by_name[name] = index
                print(f"Joint {index}: {name}; limits {math.degrees(info[8]):.1f} .. {math.degrees(info[9]):.1f} deg")
        indices = [joint_by_name[n] for n in model.names]
        tip_index = link_by_name[model.tip_link]
        for index, angle in zip(indices, initial):
            p.resetJointState(robot, index, float(angle))

        tip = model.forward(initial)
        camera_target = (base + tip * 0.45).tolist()
        camera_distance = max(0.38, float(np.linalg.norm(tip)) * 2.5)
        controls = []
        if not direct:
            p.resetDebugVisualizerCamera(camera_distance, 45, -25, camera_target)
            if not args.demo and args.target is None:
                controls = [p.addUserDebugParameter(name + " (degrees)", math.degrees(lo), math.degrees(hi),
                                                    math.degrees(angle))
                            for name, lo, hi, angle in zip(model.names, model.lower, model.upper, initial)]
                print("Move the three sliders to pose the leg; close the window to exit.")
        print("Mode:", "approximate physics" if args.physics else "kinematics")
        steps = 0
        while p.isConnected():
            targets = initial.copy()
            if args.demo:
                # Keep a margin from limits; this is a joint demo, not a gait.
                amplitude = np.minimum(np.radians([25, 18, 20]),
                                       np.minimum(initial - model.lower, model.upper - initial) * 0.8)
                targets += amplitude * np.sin(2 * np.pi * steps / 960 + np.array([0, 0.7, 1.4]))
            elif controls:
                targets = np.radians([p.readUserDebugParameter(control) for control in controls])
            for index, angle, joint in zip(indices, targets, model.joints):
                if args.physics:
                    p.setJointMotorControl2(robot, index, p.POSITION_CONTROL, targetPosition=float(angle),
                                            force=joint.effort, maxVelocity=joint.velocity)
                else:
                    p.resetJointState(robot, index, float(angle), targetVelocity=0)
            if args.physics:
                p.stepSimulation()
            else:
                p.performCollisionDetection()
            steps += 1
            if direct and steps >= args.steps:
                actual = np.array([p.getJointState(robot, index)[0] for index in indices])
                expected = model.forward(actual) + base
                state = p.getLinkState(robot, tip_index, computeForwardKinematics=True)
                position = np.array(state[4])
                error = np.linalg.norm(position - expected)
                print("Foot tip (base metres):", tuple(round(float(v), 6) for v in model.forward(actual)))
                print("Foot tip (world metres):", tuple(round(float(v), 6) for v in position))
                print(f"FK vs PyBullet error: {error * 1000:.6f} mm")
                if error > 1e-5:
                    raise RuntimeError("URDF forward kinematics disagrees with PyBullet")
                if args.screenshot:
                    save_screenshot(args.screenshot, camera_target, camera_distance)
                break
            if not direct:
                time.sleep(1 / 240)
    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
