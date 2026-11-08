"""Kinematics from URDF joint and foot frames; metres and radians.

Inverse kinematics respects joint limits but does not check mesh collisions.
"""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def _vector(text, default):
    values = np.asarray([float(v) for v in (text or default).split()])
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError("URDF vectors must contain three finite values")
    return values


def _rotation(axis, angle):
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def _origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = _vector(element.get("xyz"), "0 0 0")
        roll, pitch, yaw = _vector(element.get("rpy"), "0 0 0")
        result[:3, :3] = (
            _rotation((0, 0, 1), yaw)
            @ _rotation((0, 1, 0), pitch)
            @ _rotation((1, 0, 0), roll)
        )
    return result


@dataclass(frozen=True)
class Joint:
    name: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    movable: bool
    lower: float = 0
    upper: float = 0
    effort: float = 0
    velocity: float = 0


class LegKinematics:
    """A serial URDF chain ending at the explicit ``foot_tip`` link."""

    def __init__(self, urdf, tip_link="foot_tip"):
        self.urdf = Path(urdf)
        root = ET.parse(self.urdf).getroot()
        parents = {}
        for element in root.findall("joint"):
            child = element.find("child").get("link")
            if child in parents:
                raise ValueError(f"Multiple parent joints for {child}")
            parents[child] = element
        if tip_link not in parents:
            raise ValueError(f"{self.urdf.name} needs an explicit {tip_link!r} link and fixed joint")
        chain, seen = [], set()
        child = tip_link
        while child in parents:
            if child in seen:
                raise ValueError("URDF chain contains a cycle")
            seen.add(child)
            element = parents[child]
            kind = element.get("type")
            if kind not in ("fixed", "revolute"):
                raise ValueError(f"Unsupported joint type {kind!r}: {element.get('name')}")
            axis_element = element.find("axis")
            axis = _vector(axis_element.get("xyz") if axis_element is not None else None, "1 0 0")
            if np.linalg.norm(axis) < 1e-12:
                raise ValueError("Joint axis must be nonzero")
            axis /= np.linalg.norm(axis)
            limit = element.find("limit")
            if kind == "revolute" and limit is None:
                raise ValueError(f"Missing limits for {element.get('name')}")
            limits = {} if limit is None else {
                key: float(limit.get(key, "0")) for key in ("lower", "upper", "effort", "velocity")
            }
            if kind == "revolute" and (
                not np.all(np.isfinite(list(limits.values())))
                or limits["lower"] >= limits["upper"]
                or limits["effort"] <= 0 or limits["velocity"] <= 0
            ):
                raise ValueError(f"Invalid revolute limits for {element.get('name')}")
            chain.append(Joint(element.get("name"), child, _origin(element.find("origin")),
                               axis, kind == "revolute", **limits))
            child = element.find("parent").get("link")
        self.base_link = child
        self.tip_link = tip_link
        self.chain = tuple(reversed(chain))
        self.joints = tuple(j for j in self.chain if j.movable)
        self.names = tuple(j.name for j in self.joints)
        self.lower = np.array([j.lower for j in self.joints])
        self.upper = np.array([j.upper for j in self.joints])

    def _angles(self, angles):
        values = np.asarray(angles, dtype=float)
        if values.shape != (len(self.joints),) or not np.all(np.isfinite(values)):
            raise ValueError(f"Expected {len(self.joints)} finite joint angles in radians")
        return values

    def check_limits(self, angles):
        angles = self._angles(angles)
        if np.any(angles < self.lower - 1e-12) or np.any(angles > self.upper + 1e-12):
            raise ValueError("Requested angles exceed the URDF joint limits")
        return angles

    def _evaluate(self, angles):
        angles = iter(self._angles(angles))
        transform = np.eye(4)
        pivots, axes = [], []
        for joint in self.chain:
            transform = transform @ joint.origin
            if joint.movable:
                pivots.append(transform[:3, 3].copy())
                axes.append(transform[:3, :3] @ joint.axis)
                rotation = np.eye(4)
                rotation[:3, :3] = _rotation(joint.axis, next(angles))
                transform = transform @ rotation
        return transform, pivots, axes

    def forward(self, angles):
        """Return the foot contact point in the root link frame (metres)."""
        return self._evaluate(angles)[0][:3, 3]

    def jacobian(self, angles):
        transform, pivots, axes = self._evaluate(angles)
        tip = transform[:3, 3]
        return np.column_stack([np.cross(axis, tip - pivot) for pivot, axis in zip(pivots, axes)])

    def inverse(self, target, initial=None, tolerance=1e-6):
        """Solve a foot position subject to limits, or raise ValueError.

        This bounded numerical search tries several starting poses. Failure means
        no solution was found to the tolerance, not a formal unreachability proof.
        No collision test is applied.
        """
        from scipy.optimize import least_squares

        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("Target must contain three finite coordinates in metres")
        if not np.isfinite(tolerance) or tolerance <= 0:
            raise ValueError("Tolerance must be finite and positive")
        midpoint = (self.lower + self.upper) / 2
        initial = midpoint if initial is None else self.check_limits(initial)
        seeds = [initial, np.clip(np.zeros(len(self.joints)), self.lower, self.upper), midpoint]
        rng = np.random.default_rng(17)
        seeds.extend(rng.uniform(self.lower, self.upper, size=(16, len(self.joints))))
        best_error = float("inf")
        for seed in seeds:
            result = least_squares(
                lambda q: self.forward(q) - target, seed, jac=self.jacobian,
                bounds=(self.lower, self.upper), ftol=1e-12, xtol=1e-12,
                gtol=1e-12, max_nfev=300,
            )
            error = float(np.linalg.norm(result.fun))
            best_error = min(best_error, error)
            if error <= tolerance:
                return self.check_limits(result.x)
        raise ValueError(
            f"No IK solution found within joint limits; closest residual {best_error * 1000:.3f} mm "
            f"(tolerance {tolerance * 1000:.3f} mm)"
        )


class SpyderLeg(LegKinematics):
    """One Spyder leg, with the closed-form coxa-femur-tibia IK.

    Lengths and offsets are read from the URDF, so they follow the CAD. Angles
    are zero in the exported pose; a positive hip angle lowers the foot.
    """

    def __init__(self, urdf, name):
        super().__init__(urdf, f"{name}_foot_tip")
        origins = {joint.name[len(name) + 1:]: joint.origin for joint in self.chain}
        self.mount = origins["mount"]
        self.coxa_length = origins["femur_pitch"][0, 3]
        femur_x, femur_z = origins["tibia_pitch"][[0, 2], 3]
        tibia_x, self.lateral, tibia_z = origins["foot_tip_fixed"][:3, 3]
        self.femur_length = float(np.hypot(femur_x, femur_z))
        self.tibia_length = float(np.hypot(tibia_x, tibia_z))
        self.femur_angle = float(np.arctan2(femur_z, femur_x))
        self.tibia_angle = float(np.arctan2(tibia_z, tibia_x))

    def _solve(self, target, knee_sign, reach_sign):
        # Yaw: atan2 of the target, corrected for the foot's sideways offset.
        x, y, z = (np.linalg.inv(self.mount) @ np.r_[target, 1])[:3]
        reach_sq = x * x + y * y - self.lateral ** 2
        if reach_sq <= 0:
            raise ValueError("Foot target is inside the yaw axis offset")
        reach = reach_sign * float(np.sqrt(reach_sq))
        yaw = np.arctan2(y, x) - np.arctan2(self.lateral, reach)
        # Pitch: two-link triangle in the leg plane, starting from the hip axis.
        planar_x = reach - self.coxa_length
        cosine = (planar_x ** 2 + z ** 2 - self.femur_length ** 2 - self.tibia_length ** 2) / (
            2 * self.femur_length * self.tibia_length)
        if abs(cosine) > 1:
            raise ValueError("Foot target is further than the femur and tibia can reach")
        bend = knee_sign * np.arccos(cosine)
        femur_direction = np.arctan2(z, planar_x) - np.arctan2(
            self.tibia_length * np.sin(bend), self.femur_length + self.tibia_length * np.cos(bend))
        angles = np.array([yaw, self.femur_angle - femur_direction, self.tibia_angle - self.femur_angle - bend])
        return (angles + np.pi) % (2 * np.pi) - np.pi

    def inverse(self, target):
        """Solve a body-frame foot position, or raise ValueError if no solution fits the limits.

        Tries the knee branches with the foot ahead of the yaw axis, then behind it.
        """
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("Target must contain three finite coordinates in metres")
        rejected = []
        for reach_sign, knee_sign in ((1, -1), (1, 1), (-1, -1), (-1, 1)):
            try:
                angles = self._solve(target, knee_sign, reach_sign)
            except ValueError as error:
                rejected.append(str(error))
                continue
            if np.all(angles >= self.lower - 1e-9) and np.all(angles <= self.upper + 1e-9):
                return np.clip(angles, self.lower, self.upper)
            rejected.append(np.round(np.degrees(angles), 1).tolist())
        raise ValueError(f"No solution within the joint limits; rejected (degrees): {rejected}")
