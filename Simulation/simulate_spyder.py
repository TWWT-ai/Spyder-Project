r"""Simulate the whole Spyder: every servo joint moves, by slider or from Python.

Run from this folder:
  python simulate_spyder.py                 sliders for all 18 joints, body held in the air
  python simulate_spyder.py --physics       same sliders, free robot on the ground with gravity
  python simulate_spyder.py --walk          tripod gait with speed and turn sliders
  python simulate_spyder.py --demo          moves each joint in turn
Add --direct to run without a window, and --seconds to stop after a set time.

Angles are zero in the pose exported in main.stl (femurs level, tibias vertical).
Masses and servo strength are estimates, so physics results are not a hardware test.
"""
from pathlib import Path
import argparse
import math
import time

import numpy as np
import pybullet as p
import pybullet_data

from kinematics import SpyderLeg

URDF = Path(__file__).resolve().with_name("spyder.urdf")
TIME_STEP = 1 / 240
LEGS = ("fr", "mr", "rr", "fl", "ml", "rl")
JOINTS = ("coxa_yaw", "femur_pitch", "tibia_pitch")
TRIPODS = (("fr", "ml", "rr"), ("fl", "mr", "rl"))
NEIGHBOURS = (("fr", "mr"), ("mr", "rr"), ("fl", "ml"), ("ml", "rl"), ("fr", "fl"), ("rr", "rl"))
FOOT_FRICTION = 1.0
# Servos give under load: force builds with position error, which also shares the
# weight across all six feet. A rigid motor makes the planted legs fight each other.
SERVO_STIFFNESS = 15.0  # N m per radian of error: illustrative, not a measured servo value.
SERVO_HOLD = 0.02  # N m available at zero error.
START_HEIGHT = 0.1325  # m: body origin above ground, leaving the feet just clear of it.
HELD_HEIGHT = 0.25  # m: body origin when held in the air for posing.


class Spyder:
    """The loaded robot. Joints are addressed by name, e.g. "fr_femur_pitch"."""

    def __init__(self, position=(0, 0, START_HEIGHT), fixed_base=False, ground=None):
        self.id = p.loadURDF(str(URDF), list(position), useFixedBase=fixed_base,
                             flags=p.URDF_USE_INERTIA_FROM_FILE)
        self.joints, self.links, self.limits, self.effort, self.speed = {}, {}, {}, {}, {}
        for index in range(p.getNumJoints(self.id)):
            info = p.getJointInfo(self.id, index)
            self.links[info[12].decode()] = index
            if info[2] == p.JOINT_REVOLUTE:
                name = info[1].decode()
                self.joints[name] = index
                self.limits[name] = (info[8], info[9])
                self.effort[name] = info[10]
                self.speed[name] = info[11]
        expected = {f"{leg}_{joint}" for leg in LEGS for joint in JOINTS}
        if set(self.joints) != expected:
            raise ValueError(f"Expected the 18 Spyder joints, found {sorted(self.joints)}")
        self.legs = {leg: SpyderLeg(URDF, leg) for leg in LEGS}
        for name, index in self.links.items():
            if name.endswith("_foot_tip"):
                p.changeDynamics(self.id, index, lateralFriction=FOOT_FRICTION)
            elif name.endswith("_tibia") and ground is not None:
                # Only the small sphere on each foot tip touches the ground.
                p.setCollisionFilterPair(self.id, ground, index, -1, 0)

    def print_joints(self):
        for name, index in self.joints.items():
            lower, upper = self.limits[name]
            print(f"{index:3d} {name:18s} {math.degrees(lower):7.1f} .. {math.degrees(upper):6.1f} deg")

    def check(self, targets):
        """Reject unknown joints and angles outside the joint limits (radians)."""
        for name, angle in targets.items():
            if name not in self.joints:
                raise ValueError(f"Unknown joint {name!r}; use one of {sorted(self.joints)}")
            lower, upper = self.limits[name]
            if not math.isfinite(angle) or not lower <= angle <= upper:
                raise ValueError(f"{name}: {angle} rad is outside {lower:.3f} .. {upper:.3f}")

    def pose(self, targets):
        """Place joints directly at these angles, without using motors."""
        self.check(targets)
        for name, angle in targets.items():
            p.resetJointState(self.id, self.joints[name], angle)

    def command(self, targets):
        """Drive joints to these angles with servo-limited position control."""
        self.check(targets)
        for name, angle in targets.items():
            error = abs(angle - p.getJointState(self.id, self.joints[name])[0])
            p.setJointMotorControl2(self.id, self.joints[name], p.POSITION_CONTROL, targetPosition=angle,
                                    force=min(self.effort[name], SERVO_HOLD + SERVO_STIFFNESS * error),
                                    maxVelocity=self.speed[name], positionGain=0.25, velocityGain=1.0)

    def angles(self):
        return {name: p.getJointState(self.id, index)[0] for name, index in self.joints.items()}

    def torques(self):
        return {name: p.getJointState(self.id, index)[3] for name, index in self.joints.items()}

    def body_pose(self):
        """Body link origin and orientation; Bullet's base state is its centre of mass."""
        position, orientation = p.getBasePositionAndOrientation(self.id)
        inverse = p.invertTransform(*p.getDynamicsInfo(self.id, -1)[3:5])
        return p.multiplyTransforms(position, orientation, *inverse)

    def set_body_pose(self, position, orientation):
        inertial = p.getDynamicsInfo(self.id, -1)[3:5]
        p.resetBasePositionAndOrientation(self.id, *p.multiplyTransforms(position, orientation, *inertial))

    def foot_world(self, leg):
        return np.array(p.getLinkState(self.id, self.links[f"{leg}_foot_tip"], computeForwardKinematics=True)[4])

    def foot_targets(self, leg, position):
        """Return joint targets that put one foot at a body-frame position (metres)."""
        return {f"{leg}_{joint}": float(angle) for joint, angle in zip(JOINTS, self.legs[leg].inverse(position))}

    def touching_legs(self):
        """Neighbouring legs whose collision hulls overlap in the current pose."""
        touching = []
        for a, b in NEIGHBOURS:
            for link_a in ("coxa", "femur", "tibia"):
                for link_b in ("coxa", "femur", "tibia"):
                    if p.getClosestPoints(self.id, self.id, 0, self.links[f"{a}_{link_a}"],
                                          self.links[f"{b}_{link_b}"]):
                        touching.append(f"{a}_{link_a}/{b}_{link_b}")
        return touching


class TripodGait:
    """Lift, swing and stance foot paths for two alternating groups of three legs."""

    def __init__(self, spyder, period=1.2, lift=0.03):
        self.spyder = spyder
        self.period = period
        self.lift = lift
        self.home = {leg: model.forward(np.zeros(3)) for leg, model in spyder.legs.items()}

    def targets(self, time_s, forward=0.0, turn=0.0):
        """Joint targets at a time, for a speed along +Y (m/s) and a turn rate about +Z (rad/s)."""
        result = {}
        for group, legs in enumerate(TRIPODS):
            phase = (time_s / self.period + group / 2) % 1.0
            for leg in legs:
                home = self.home[leg]
                # A planted foot covers this much ground, relative to the body, in half a cycle.
                stride = (np.array([0.0, forward, 0.0]) + turn * np.array([-home[1], home[0], 0.0])) * self.period / 2
                stride *= min(1.0, time_s / self.period)  # Grow the stride from standing still.
                if phase < 0.5:
                    along, height = 0.5 - 2 * phase, 0.0
                else:
                    swing = (phase - 0.5) * 2
                    along, height = -0.5 * math.cos(math.pi * swing), self.lift * math.sin(math.pi * swing)
                result.update(self.spyder.foot_targets(leg, home + along * stride + np.array([0, 0, height])))
        return result


def body_state(spyder):
    position, orientation = spyder.body_pose()
    roll, pitch, yaw = p.getEulerFromQuaternion(orientation)
    return np.array(position), math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def connect(direct, gravity):
    if p.connect(p.DIRECT if direct else p.GUI) < 0:
        raise RuntimeError("Could not connect to PyBullet")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81 if gravity else 0)
    p.setTimeStep(TIME_STEP)
    plane = p.loadURDF("plane.urdf")
    p.changeDynamics(plane, -1, lateralFriction=FOOT_FRICTION)
    return plane


def add_joint_sliders(spyder, start):
    return {name: p.addUserDebugParameter(f"{name} (deg)", math.degrees(spyder.limits[name][0]),
                                          math.degrees(spyder.limits[name][1]), math.degrees(start.get(name, 0.0)))
            for name in spyder.joints}


def read_sliders(sliders):
    return {name: math.radians(p.readUserDebugParameter(slider)) for name, slider in sliders.items()}


def demo_targets(spyder, time_s, seconds_per_joint=2.5):
    """Swing one joint at a time out to two thirds of each limit; all others stay home."""
    names = list(spyder.joints)
    active = names[int(time_s / seconds_per_joint) % len(names)]
    wave = math.sin(2 * math.pi * (time_s % seconds_per_joint) / seconds_per_joint)
    lower, upper = spyder.limits[active]
    angle = (2 / 3) * (upper * wave if wave > 0 else -lower * wave)
    return {name: (angle if name == active else 0.0) for name in names}, active


def save_screenshot(path, target, distance=0.75):
    from PIL import Image

    width, height = 1280, 960
    view = p.computeViewMatrixFromYawPitchRoll(list(target), distance, 35, -28, 0, 2)
    projection = p.computeProjectionMatrixFOV(45, width / height, 0.01, 5)
    pixels = p.getCameraImage(width, height, view, projection, renderer=p.ER_TINY_RENDERER, shadow=1)[2]
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.asarray(pixels, dtype=np.uint8).reshape(height, width, 4)).convert("RGB").save(path)
    print(f"Saved preview: {path.resolve()}")


def parse_settings(settings):
    """Turn NAME=DEGREES command-line settings into radians."""
    targets = {}
    for setting in settings:
        name, separator, value = setting.partition("=")
        if not separator:
            raise ValueError(f"Use NAME=DEGREES, not {setting!r}")
        targets[name] = math.radians(float(value))
    return targets


def run(*, mode="pose", direct=False, seconds=None, settings=None, screenshot=None, forward=0.04, turn=0.0):
    """Run a mode until the window closes or for a simulated time; return a summary."""
    if seconds is None and direct:
        seconds = 2.0
    held = mode in ("pose", "demo")
    ground = connect(direct, not held)
    summary = {}
    try:
        if not direct:
            for option in (p.COV_ENABLE_RENDERING, p.COV_ENABLE_RGB_BUFFER_PREVIEW, p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                           p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW):
                p.configureDebugVisualizer(option, 0)  # Rendering stays off while the meshes load.
            print("Loading the robot meshes; the window can take a little while to fill in.")
        spyder = Spyder(position=(0, 0, HELD_HEIGHT if held else START_HEIGHT), fixed_base=held, ground=ground)
        if not direct:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        spyder.print_joints()
        start = dict(settings or {})
        spyder.check(start)
        spyder.pose(start)
        home = {**{name: 0.0 for name in spyder.joints}, **start}
        gait = TripodGait(spyder) if mode == "walk" else None
        sliders, body_sliders, walk_sliders = {}, [], {}
        if not direct:
            p.resetDebugVisualizerCamera(0.75, 35, -28, [0, 0, 0.1])
            if mode in ("pose", "physics"):
                sliders = add_joint_sliders(spyder, start)
            if mode == "pose":
                body_sliders = [p.addUserDebugParameter(label, low, high, value) for label, low, high, value in (
                    ("body X (m)", -0.5, 0.5, 0), ("body Y (m)", -0.5, 0.5, 0), ("body Z (m)", 0.0, 0.6, HELD_HEIGHT),
                    ("body roll (deg)", -180, 180, 0), ("body pitch (deg)", -180, 180, 0),
                    ("body yaw (deg)", -180, 180, 0))]
            if mode == "walk":
                walk_sliders = {"forward": p.addUserDebugParameter("speed along +Y (mm/s)", -80, 80, forward * 1000),
                                "turn": p.addUserDebugParameter("turn (deg/s)", -30, 30, math.degrees(turn))}
        if not held:
            for _ in range(int(0.5 / TIME_STEP)):  # Settle onto the feet before moving.
                spyder.command(home)
                p.stepSimulation()
        origin = body_state(spyder)
        steps, peak_torque, peak_joint, warned, label = 0, 0.0, None, None, None
        step_limit = None if seconds is None else math.ceil(seconds / TIME_STEP)
        tilt, lowest = 0.0, origin[0][2]
        while p.isConnected() and (step_limit is None or steps < step_limit):
            now = steps * TIME_STEP
            if mode == "pose":
                spyder.pose(read_sliders(sliders) if sliders else home)
                if body_sliders:
                    values = [p.readUserDebugParameter(slider) for slider in body_sliders]
                    spyder.set_body_pose(values[:3], p.getQuaternionFromEuler(np.radians(values[3:]).tolist()))
            elif mode == "demo":
                targets, active = demo_targets(spyder, now)
                spyder.pose(targets)
                if active != label:
                    label = active
                    print("Moving", active)
            elif mode == "physics":
                spyder.command(read_sliders(sliders) if sliders else home)
            else:
                speed, rate = forward, turn
                if walk_sliders:
                    speed = p.readUserDebugParameter(walk_sliders["forward"]) / 1000
                    rate = math.radians(p.readUserDebugParameter(walk_sliders["turn"]))
                spyder.command(gait.targets(now, speed, rate))
            if held:
                p.performCollisionDetection()
            else:
                p.stepSimulation()
                torques = spyder.torques()
                name = max(torques, key=lambda joint: abs(torques[joint]))
                if abs(torques[name]) > peak_torque:
                    peak_torque, peak_joint = abs(torques[name]), name
                state = body_state(spyder)
                tilt = max(tilt, abs(state[1]), abs(state[2]))
                lowest = min(lowest, state[0][2])
            if not direct and steps % 24 == 0:
                message = ", ".join(spyder.touching_legs()[:3])
                if message != warned:
                    warned = message
                    p.removeAllUserDebugItems()
                    if message:
                        p.addUserDebugText("Legs overlap: " + message, [0, 0, 0.4], [0.9, 0.1, 0.1], 1.2)
            steps += 1
            if not direct:
                time.sleep(TIME_STEP)
        if p.isConnected():
            end = body_state(spyder)
            summary = {"angles": spyder.angles(), "body_start": origin[0], "body_end": end[0],
                       "yaw_change_deg": end[3] - origin[3], "max_tilt_deg": tilt, "lowest_body_z": lowest,
                       "peak_torque_nm": peak_torque, "peak_joint": peak_joint,
                       "touching": spyder.touching_legs()}
            print(f"Body moved {np.round((end[0] - origin[0]) * 1000, 1).tolist()} mm, turned "
                  f"{summary['yaw_change_deg']:.1f} deg; max tilt {tilt:.2f} deg, lowest body height "
                  f"{lowest * 1000:.1f} mm; peak servo torque {peak_torque:.3f} N m ({peak_joint})")
            if summary["touching"]:
                print("Neighbouring legs overlap:", ", ".join(summary["touching"]))
            if screenshot:
                save_screenshot(screenshot, [end[0][0], end[0][1], end[0][2] - 0.03])
        return summary
    except KeyboardInterrupt:
        return summary
    except p.error:
        # Closing the GUI can disconnect between the loop's connection check and a call.
        if p.isConnected():
            raise
        return summary
    finally:
        if p.isConnected():
            p.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--physics", action="store_true", help="Free robot on the ground, motors follow the sliders")
    group.add_argument("--walk", action="store_true", help="Tripod gait under physics")
    group.add_argument("--demo", action="store_true", help="Move each joint in turn")
    parser.add_argument("--direct", action="store_true", help="Run without a window")
    parser.add_argument("--seconds", type=float, help="Stop after this much simulated time")
    parser.add_argument("--set", action="append", default=[], metavar="JOINT=DEG",
                        help="Start a joint at an angle, e.g. --set fr_femur_pitch=20 (repeatable)")
    parser.add_argument("--speed", type=float, default=40.0, help="Walking speed along +Y in mm/s (default 40)")
    parser.add_argument("--turn", type=float, default=0.0, help="Walking turn rate in deg/s (default 0)")
    parser.add_argument("--screenshot", type=Path, help="Save a PNG of the final pose (implies --direct)")
    args = parser.parse_args()
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")
    try:
        settings = parse_settings(args.set)
    except ValueError as error:
        parser.error(str(error))
    mode = "physics" if args.physics else "walk" if args.walk else "demo" if args.demo else "pose"
    direct = args.direct or args.screenshot is not None
    try:
        run(mode=mode, direct=direct, seconds=args.seconds, settings=settings, screenshot=args.screenshot,
            forward=args.speed / 1000, turn=math.radians(args.turn))
    except ValueError as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
