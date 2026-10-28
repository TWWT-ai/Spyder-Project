"""Checks the generated assembly's kinematics against the simulator."""
from pathlib import Path
import sys
import unittest

import numpy as np
import pybullet as p

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kinematics import LegKinematics


class KinematicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = LegKinematics(ROOT / "leg_assembled.urdf")
        cls.client = p.connect(p.DIRECT)
        cls.robot = p.loadURDF(str(cls.model.urdf), useFixedBase=True, physicsClientId=cls.client)
        info = [p.getJointInfo(cls.robot, i, physicsClientId=cls.client)
                for i in range(p.getNumJoints(cls.robot, physicsClientId=cls.client))]
        by_name = {j[1].decode(): j[0] for j in info}
        cls.indices = [by_name[n] for n in cls.model.names]
        cls.tip = next(j[0] for j in info if j[12].decode() == cls.model.tip_link)

    @classmethod
    def tearDownClass(cls):
        p.disconnect(cls.client)

    def test_forward_matches_pybullet_at_neutral_limits_and_random_poses(self):
        rng = np.random.default_rng(12)
        poses = [np.clip(np.zeros(3), self.model.lower, self.model.upper),
                 self.model.lower, self.model.upper]
        poses.extend(rng.uniform(self.model.lower, self.model.upper, size=(20, 3)))
        for angles in poses:
            with self.subTest(angles=angles):
                for index, angle in zip(self.indices, angles):
                    p.resetJointState(self.robot, index, angle, physicsClientId=self.client)
                tip = p.getLinkState(self.robot, self.tip, computeForwardKinematics=True,
                                     physicsClientId=self.client)[4]
                np.testing.assert_allclose(self.model.forward(angles), tip, atol=5e-8, rtol=0)

    def test_inverse_round_trips_and_obeys_limits(self):
        rng = np.random.default_rng(4)
        for angles in rng.uniform(self.model.lower * 0.8, self.model.upper * 0.8, size=(8, 3)):
            target = self.model.forward(angles)
            solution = self.model.inverse(target)
            self.model.check_limits(solution)
            np.testing.assert_allclose(self.model.forward(solution), target, atol=1e-6, rtol=0)

    def test_unreachable_target_and_nonfinite_input_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "No IK solution"):
            self.model.inverse([2, 2, 2])
        for bad in ([float("nan"), 0, 0], [1, 2], [float("inf"), 0, 0]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.model.inverse(bad)
        with self.assertRaisesRegex(ValueError, "exceed"):
            self.model.check_limits(self.model.upper + 0.01)

    def test_jacobian_matches_finite_difference(self):
        angles = (self.model.lower + self.model.upper) / 2
        delta = 1e-6
        numerical = np.column_stack([
            (self.model.forward(angles + np.eye(3)[i] * delta)
             - self.model.forward(angles - np.eye(3)[i] * delta)) / (2 * delta)
            for i in range(3)
        ])
        np.testing.assert_allclose(self.model.jacobian(angles), numerical, atol=1e-9, rtol=0)


if __name__ == "__main__":
    unittest.main()
