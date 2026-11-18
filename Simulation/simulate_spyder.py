r"""Simulate the whole Spyder: every servo joint moves, by slider or from Python.

Run from this folder:
  python simulate_spyder.py                 sliders for all 18 joints, body held in the air
  python simulate_spyder.py --physics       same sliders, free robot on the ground with gravity
  python simulate_spyder.py --walk          tripod gait with speed and turn sliders
  python simulate_spyder.py --67            stand on four legs and dance with the front two
  python simulate_spyder.py --wiggle        sway and bob with all six feet planted
  python simulate_spyder.py --hello         wave one raised front leg
  python simulate_spyder.py --point         point one raised front leg
  python simulate_spyder.py --v-sign        raise both front legs in a V
  python simulate_spyder.py --foot-tap-wave tap each foot in sequence
  python simulate_spyder.py --crab-walk     walk sideways on alternating tripods
  python simulate_spyder.py --crab-dance    side-step, then wave both front claws
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

    def __init__(self, spyder, period=1.2, lift=0.03, swing_fraction=0.5, lift_exponent=1):
        self.spyder = spyder
        self.period = period
        self.lift = lift
        self.swing_fraction = swing_fraction
        self.lift_exponent = lift_exponent
        self.home = {leg: model.forward(np.zeros(3)) for leg, model in spyder.legs.items()}

    def targets(self, time_s, forward=0.0, turn=0.0, lateral=0.0):
        """Joint targets for +Y forward, +X lateral (m/s), and +Z turn (rad/s)."""
        result = {}
        for group, legs in enumerate(TRIPODS):
            phase = (time_s / self.period + group / 2) % 1.0
            for leg in legs:
                home = self.home[leg]
                # A planted foot covers this much ground, relative to the body, per stride.
                stride = (np.array([lateral, forward, 0.0])
                          + turn * np.array([-home[1], home[0], 0.0])) * self.period / 2
                stride *= min(1.0, time_s / self.period)  # Grow the stride from standing still.
                stance_fraction = 1 - self.swing_fraction
                if phase < stance_fraction:
                    along, height = 0.5 - phase / stance_fraction, 0.0
                else:
                    swing = (phase - stance_fraction) / self.swing_fraction
                    along = -0.5 * math.cos(math.pi * swing)
                    height = self.lift * math.sin(math.pi * swing) ** self.lift_exponent
                result.update(self.spyder.foot_targets(leg, home + along * stride + np.array([0, 0, height])))
        return result


class SixSevenDance:
    """Step the middle feet forward, then raise the front pair for the gesture."""

    STEP_SECONDS = 1.2
    SETTLE_SECONDS = 0.25
    STEP_HEIGHT = 0.04
    FRONT_RAISE_SECONDS = 0.8

    def __init__(self, spyder, front_motion="67"):
        self.spyder = spyder
        if front_motion not in ("67", "v", "claws"):
            raise ValueError(f"Unknown front motion {front_motion!r}")
        self.front_motion = front_motion
        self.middle_home = {}
        self.middle_goal = {}
        for leg in ("mr", "ml"):
            home = spyder.legs[leg].forward(np.zeros(3))
            goal = home.copy()
            goal[1] = 0.15  # Put the middle feet towards the front (+Y).
            self.middle_home[leg], self.middle_goal[leg] = home, goal
        self.step_start = {"mr": 0.0, "ml": self.STEP_SECONDS + self.SETTLE_SECONDS}
        self.front_start = 2 * (self.STEP_SECONDS + self.SETTLE_SECONDS)
        self.gesture_start = self.front_start + self.FRONT_RAISE_SECONDS

    @staticmethod
    def ease(progress):
        progress = max(0.0, min(1.0, progress))
        return progress * progress * (3 - 2 * progress)

    def targets(self, time_s):
        targets = {name: 0.0 for name in self.spyder.joints}
        for leg in ("mr", "ml"):
            phase = (time_s - self.step_start[leg]) / self.STEP_SECONDS
            if phase <= 0:
                continue
            if phase < 0.2:
                along, lift = 0.0, self.ease(phase / 0.2)
            elif phase < 0.8:
                along, lift = self.ease((phase - 0.2) / 0.6), 1.0
            else:
                along, lift = 1.0, 1.0 - self.ease((phase - 0.8) / 0.2)
            foot = (self.middle_home[leg] + along * (self.middle_goal[leg] - self.middle_home[leg])
                    + np.array([0.0, 0.0, self.STEP_HEIGHT * lift]))
            targets.update(self.spyder.foot_targets(leg, foot))

        # Only lift the front pair after both middle feet have landed and settled.
        progress = self.ease((time_s - self.front_start) / self.FRONT_RAISE_SECONDS)
        beat = math.sin(2 * math.pi * (time_s - self.gesture_start) / 1.2)
        for leg in ("rr", "rl"):
            # A small downward bias keeps both rear feet loaded on the ground.
            targets[f"{leg}_femur_pitch"] = math.radians(progress)
        for leg, direction in (("fr", 1), ("fl", -1)):
            if self.front_motion == "67":
                yaw, hip, knee = 45 * direction, -45 + 12 * direction * beat, -82 + 3 * direction * beat
            elif self.front_motion == "v":
                yaw, hip, knee = -15 * direction, -45, -82
            else:
                yaw, hip, knee = direction * (35 + 10 * beat), -45 + 8 * beat, -82 + 3 * beat
            targets[f"{leg}_coxa_yaw"] = math.radians(yaw * progress)
            targets[f"{leg}_femur_pitch"] = math.radians(hip * progress)
            targets[f"{leg}_tibia_pitch"] = math.radians(knee * progress)
        return targets


class WiggleDance:
    """Trace the same low U-shaped arc in both directions with planted feet."""

    ENTRY_SECONDS = 0.8
    PERIOD_SECONDS = 2.6
    SWAY_METRES = 0.022
    DIP_METRES = 0.035
    YAW_DEGREES = 2

    def __init__(self, spyder):
        self.spyder = spyder
        self.home = {leg: model.forward(np.zeros(3)) for leg, model in spyder.legs.items()}

    def targets(self, time_s):
        if time_s < self.ENTRY_SECONDS:
            # Ease from the home pose to one end of the U without lifting the body.
            progress = SixSevenDance.ease(time_s / self.ENTRY_SECONDS)
            sway, dip, yaw = self.SWAY_METRES * progress, 0.0, 0.0
        else:
            phase = 2 * math.pi * (time_s - self.ENTRY_SECONDS) / self.PERIOD_SECONDS
            # sin² makes the centre the bottom on both passes. The return retraces
            # the same arc instead of travelling above it to close a loop.
            sway = self.SWAY_METRES * math.cos(phase)
            dip = -self.DIP_METRES * math.sin(phase) ** 2
            yaw = math.radians(self.YAW_DEGREES) * math.sin(phase) ** 3
        offset = np.array([sway, 0.0, dip])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                             [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        targets = {}
        for leg, home in self.home.items():
            targets.update(self.spyder.foot_targets(leg, rotation.T @ (home - offset)))
        return targets


class FrontEmote:
    """Lift one front leg for a wave or a held point; five feet stay home."""

    def __init__(self, spyder, motion):
        self.spyder = spyder
        if motion not in ("hello", "point"):
            raise ValueError(f"Unknown front emote {motion!r}")
        self.motion = motion

    def targets(self, time_s):
        progress = SixSevenDance.ease(time_s / 0.8)
        beat = math.sin(2 * math.pi * (time_s - 0.8) / 1.2)
        targets = {name: 0.0 for name in self.spyder.joints}
        if self.motion == "hello":
            yaw, hip, knee = 30 + 10 * beat, -45 + 6 * beat, -82
        else:
            yaw, hip, knee = 45, -45, -70
        for joint, angle in zip(JOINTS, (yaw, hip, knee)):
            targets[f"fr_{joint}"] = math.radians(angle * progress)
        return targets


class FootTapWave:
    """Tap one foot at a time from front to rear on alternating sides."""

    ORDER = ("fr", "fl", "mr", "ml", "rr", "rl")
    SECONDS_PER_TAP = 0.75

    def __init__(self, spyder):
        self.spyder = spyder
        self.home = {leg: model.forward(np.zeros(3)) for leg, model in spyder.legs.items()}

    def targets(self, time_s):
        cycle = time_s % (len(self.ORDER) * self.SECONDS_PER_TAP)
        index = int(cycle / self.SECONDS_PER_TAP)
        phase = cycle / self.SECONDS_PER_TAP - index
        foot = self.ORDER[index]
        lift = 0.04 * math.sin(math.pi * phase) ** 2
        targets = {name: 0.0 for name in self.spyder.joints}
        targets.update(self.spyder.foot_targets(foot, self.home[foot] + np.array([0, 0, lift])))
        return targets


class CrabDance:
    """Side-step in both directions, settle, then form a four-foot claw pose."""

    SIDE_SECONDS = 4.8
    SETTLE_SECONDS = 0.3

    def __init__(self, spyder, speed=0.04):
        self.gait = TripodGait(spyder, swing_fraction=0.42, lift_exponent=2)
        self.claws = SixSevenDance(spyder, front_motion="claws")
        self.speed = speed

    def targets(self, time_s):
        if time_s < self.SIDE_SECONDS:
            sideways = self.speed * math.sin(2 * math.pi * time_s / self.SIDE_SECONDS)
            return self.gait.targets(time_s, lateral=sideways)
        if time_s < self.SIDE_SECONDS + self.SETTLE_SECONDS:
            return {name: 0.0 for name in self.claws.spyder.joints}
        return self.claws.targets(time_s - self.SIDE_SECONDS - self.SETTLE_SECONDS)


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


def save_screenshot(path, target, distance=0.75, yaw=35, pitch=-28):
    from PIL import Image

    width, height = 1280, 960
    view = p.computeViewMatrixFromYawPitchRoll(list(target), distance, yaw, pitch, 0, 2)
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
        seconds = {"67": 6.0, "v-sign": 6.0, "foot-tap-wave": 4.5,
                   "crab-dance": 10.0}.get(mode, 2.0)
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
        gait = (TripodGait(spyder) if mode == "walk" else
                TripodGait(spyder, swing_fraction=0.42, lift_exponent=2) if mode == "crab-walk" else None)
        motions = {
            "67": lambda: SixSevenDance(spyder),
            "wiggle": lambda: WiggleDance(spyder),
            "hello": lambda: FrontEmote(spyder, "hello"),
            "point": lambda: FrontEmote(spyder, "point"),
            "v-sign": lambda: SixSevenDance(spyder, front_motion="v"),
            "foot-tap-wave": lambda: FootTapWave(spyder),
            "crab-dance": lambda: CrabDance(spyder, forward),
        }
        dance = motions[mode]() if mode in motions else None
        sliders, body_sliders, walk_sliders = {}, [], {}
        front_view = mode in ("67", "wiggle", "hello", "point", "v-sign", "crab-dance")
        camera_distance = 0.75 if mode == "wiggle" or not front_view else 1.0
        camera_pitch = -15 if mode == "wiggle" else -28
        if not direct:
            p.resetDebugVisualizerCamera(camera_distance, 180 if front_view else 35,
                                         camera_pitch, [0, 0, 0.1])
            if mode in ("pose", "physics"):
                sliders = add_joint_sliders(spyder, start)
            if mode == "pose":
                body_sliders = [p.addUserDebugParameter(label, low, high, value) for label, low, high, value in (
                    ("body X (m)", -0.5, 0.5, 0), ("body Y (m)", -0.5, 0.5, 0), ("body Z (m)", 0.0, 0.6, HELD_HEIGHT),
                    ("body roll (deg)", -180, 180, 0), ("body pitch (deg)", -180, 180, 0),
                    ("body yaw (deg)", -180, 180, 0))]
            if mode in ("walk", "crab-walk"):
                walk_sliders = {"forward": p.addUserDebugParameter(
                                    "speed along +Y (mm/s)" if mode == "walk" else "speed along +X (mm/s)",
                                    -80, 80, forward * 1000),
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
            elif dance is not None:
                spyder.command(dance.targets(now))
            elif gait is not None:
                speed, rate = forward, turn
                if walk_sliders:
                    speed = p.readUserDebugParameter(walk_sliders["forward"]) / 1000
                    rate = math.radians(p.readUserDebugParameter(walk_sliders["turn"]))
                spyder.command(gait.targets(now, forward=speed if mode == "walk" else 0.0,
                                            turn=rate, lateral=speed if mode == "crab-walk" else 0.0))
            else:
                raise ValueError(f"Unknown mode {mode!r}")
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
                       "grounded_legs": [leg for leg in LEGS if p.getContactPoints(spyder.id, ground,
                                                                                     spyder.links[f"{leg}_foot_tip"], -1)],
                       "touching": spyder.touching_legs()}
            print(f"Body moved {np.round((end[0] - origin[0]) * 1000, 1).tolist()} mm, turned "
                  f"{summary['yaw_change_deg']:.1f} deg; max tilt {tilt:.2f} deg, lowest body height "
                  f"{lowest * 1000:.1f} mm; peak servo torque {peak_torque:.3f} N m ({peak_joint})")
            if summary["touching"]:
                print("Neighbouring legs overlap:", ", ".join(summary["touching"]))
            if screenshot:
                save_screenshot(screenshot, [end[0][0], end[0][1], end[0][2] - 0.03],
                                distance=camera_distance,
                                yaw=180 if front_view else 35, pitch=camera_pitch)
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
    group.add_argument("--67", dest="six_seven", action="store_true",
                       help="Balance on four legs while the front two do the 6-7 gesture")
    group.add_argument("--wiggle", action="store_true", help="Sway, bob and yaw with six feet planted")
    group.add_argument("--hello", action="store_true", help="Wave one raised front leg")
    group.add_argument("--point", action="store_true", help="Point one raised front leg")
    group.add_argument("--v-sign", action="store_true", help="Step onto four feet and raise the front pair in a V")
    group.add_argument("--foot-tap-wave", action="store_true", help="Tap each leg from front to rear")
    group.add_argument("--crab-walk", action="store_true", help="Walk sideways on alternating tripods")
    group.add_argument("--crab-dance", action="store_true", help="Side-step, form a four-foot stance, and wave both claws")
    group.add_argument("--demo", action="store_true", help="Move each joint in turn")
    parser.add_argument("--direct", action="store_true", help="Run without a window")
    parser.add_argument("--seconds", type=float, help="Stop after this much simulated time")
    parser.add_argument("--set", action="append", default=[], metavar="JOINT=DEG",
                        help="Start a joint at an angle, e.g. --set fr_femur_pitch=20 (repeatable)")
    parser.add_argument("--speed", type=float, default=40.0,
                        help="Walking speed in mm/s: +Y for --walk, +X for crab modes (default 40)")
    parser.add_argument("--turn", type=float, default=0.0, help="Walking turn rate in deg/s (default 0)")
    parser.add_argument("--screenshot", type=Path, help="Save a PNG of the final pose (implies --direct)")
    args = parser.parse_args()
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")
    try:
        settings = parse_settings(args.set)
    except ValueError as error:
        parser.error(str(error))
    mode = next((name for name, enabled in (
        ("physics", args.physics), ("walk", args.walk), ("67", args.six_seven),
        ("wiggle", args.wiggle), ("hello", args.hello), ("point", args.point),
        ("v-sign", args.v_sign), ("foot-tap-wave", args.foot_tap_wave),
        ("crab-walk", args.crab_walk), ("crab-dance", args.crab_dance),
        ("demo", args.demo)) if enabled), "pose")
    direct = args.direct or args.screenshot is not None
    try:
        run(mode=mode, direct=direct, seconds=args.seconds, settings=settings, screenshot=args.screenshot,
            forward=args.speed / 1000, turn=math.radians(args.turn))
    except ValueError as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
