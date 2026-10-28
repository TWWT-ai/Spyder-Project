r"""Editable starting point for the assembled leg's physics simulation.

Run: .\.venv\Scripts\python.exe my_simulation.py
Test without a window: add --direct --seconds 2

Edit get_target_angles() to try your own movement. Angles are radians.
"""
import argparse
import math
from pathlib import Path
import time

import pybullet as p
import pybullet_data

URDF = Path(__file__).resolve().with_name("leg_assembled.urdf")
TIME_STEP = 1 / 240
MOTOR_FORCE = 0.3  # N m: illustrative simulation limit, not a measured servo rating.
START_ANGLES = {
    "coxa_yaw": 0.0,
    "femur_pitch": math.radians(20),
    "tibia_pitch": math.radians(35),
}


def get_target_angles(simulation_time):
    targets = START_ANGLES.copy()

    targets["coxa_yaw"] += math.radians(15) * math.sin(2 * math.pi * simulation_time / 4)
    targets["femur_pitch"] += math.radians(15) * math.sin(2 * math.pi * simulation_time / 4)
    targets["tibia_pitch"] += math.radians(15) * math.sin(2 * math.pi * simulation_time / 4) # move foot +-15 degrees every 4 seconds

    return targets


def find_joints(robot):
    """Resolve names each time: fixed mesh attachments also have joint indices."""
    joints = {}
    for index in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, index)
        if info[2] == p.JOINT_REVOLUTE:
            joints[info[1].decode("utf-8")] = index
    if set(joints) != set(START_ANGLES):
        raise ValueError(f"Expected the three leg joints, found {list(joints)}")
    return joints


def command_joints(robot, joints, targets):
    """Check commands against URDF limits, then apply motor position control."""
    if set(targets) != set(joints):
        raise ValueError(f"Provide an angle for each joint: {', '.join(joints)}")
    commands = []
    for name, index in joints.items():
        angle = targets[name]
        info = p.getJointInfo(robot, index)
        lower, upper = info[8:10]
        if not math.isfinite(angle) or not lower <= angle <= upper:
            raise ValueError(f"{name}: angle must be between {lower:.3f} and {upper:.3f} radians")
        commands.append((index, angle, info[10], info[11]))
    for index, angle, max_effort, max_speed in commands:
        p.setJointMotorControl2(
            robot, index, p.POSITION_CONTROL,
            targetPosition=angle,
            force=min(MOTOR_FORCE, max_effort),
            maxVelocity=max_speed,
        )


def run(*, direct=False, seconds=None):
    """Run until the window closes, or until the requested simulation duration."""
    if seconds is not None and (not math.isfinite(seconds) or seconds <= 0):
        raise ValueError("seconds must be finite and positive")
    if direct and seconds is None:
        seconds = 2.0
    step_limit = None if seconds is None else math.ceil(seconds / TIME_STEP)
    client = p.connect(p.DIRECT if direct else p.GUI)
    if client < 0:
        raise RuntimeError("Could not open PyBullet")
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(TIME_STEP)
        p.setRealTimeSimulation(0)  # We advance physics explicitly below.
        p.loadURDF("plane.urdf")
        # A single leg is held at its mounting point. A free base would fall.
        robot = p.loadURDF(str(URDF), [0, 0, 0.28], useFixedBase=True)
        joints = find_joints(robot)
        for name, index in joints.items():
            info = p.getJointInfo(robot, index)
            print(f"{name}: index {index}, limits {math.degrees(info[8]):.1f} .. {math.degrees(info[9]):.1f} degrees")
            # Set the initial pose once; subsequent movement uses motor forces.
            p.resetJointState(robot, index, START_ANGLES[name])
        if not direct:
            p.resetDebugVisualizerCamera(0.52, 45, -25, [0.10, 0, 0.20])
            print("Running your movement function. Close the window or press Ctrl+C to stop.")

        step = 0
        while p.isConnected() and (step_limit is None or step < step_limit):
            simulation_time = step * TIME_STEP
            targets = get_target_angles(simulation_time)
            command_joints(robot, joints, targets)  # Commands go INSIDE the loop.
            p.stepSimulation()
            step += 1
            if not direct:
                time.sleep(TIME_STEP)

        if p.isConnected():
            actual = {name: p.getJointState(robot, index)[0] for name, index in joints.items()}
            print("Final angles (degrees):", {name: round(math.degrees(q), 2) for name, q in actual.items()})
            return actual
    except KeyboardInterrupt:
        pass
    except p.error:
        # Closing the GUI can disconnect between the loop's connection check
        # and a physics call. Other Bullet failures should still be reported.
        if p.isConnected():
            raise
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run your editable single-leg simulation.")
    parser.add_argument("--direct", action="store_true", help="Run headlessly; defaults to 2 simulated seconds")
    parser.add_argument("--seconds", type=float, help="Stop after this many simulated seconds")
    args = parser.parse_args()
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")
    run(direct=args.direct, seconds=args.seconds)
