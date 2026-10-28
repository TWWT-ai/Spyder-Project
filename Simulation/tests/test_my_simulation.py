"""Exercise motor commands on the actual assembled model under gravity."""
from pathlib import Path
import sys
import unittest

import pybullet as p

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import my_simulation as sim


class SimulationStarterTests(unittest.TestCase):
    def test_name_lookup_and_rejection_of_invalid_commands(self):
        p.connect(p.DIRECT)
        try:
            robot = p.loadURDF(str(sim.URDF), useFixedBase=True)
            joints = sim.find_joints(robot)
            self.assertEqual(set(joints), set(sim.START_ANGLES))
            self.assertEqual(p.getJointInfo(robot, 0)[2], p.JOINT_FIXED)
            for index in joints.values():
                self.assertEqual(p.getJointInfo(robot, index)[2], p.JOINT_REVOLUTE)
            for invalid in (float("nan"), float("inf"), 100):
                target = sim.START_ANGLES.copy()
                target["tibia_pitch"] = invalid
                with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                    sim.command_joints(robot, joints, target)
            with self.assertRaises(ValueError):
                sim.command_joints(robot, joints, {"joint_0": 0.35})
        finally:
            p.disconnect()

    def test_motors_follow_demo_and_hold_other_joints_under_gravity(self):
        actual = sim.run(direct=True, seconds=1)
        expected = sim.get_target_angles(1 - sim.TIME_STEP)
        self.assertGreater(actual["tibia_pitch"] - sim.START_ANGLES["tibia_pitch"], 0.2)
        for name in expected:
            with self.subTest(joint=name):
                self.assertAlmostEqual(actual[name], expected[name], delta=0.02)
        self.assertFalse(p.isConnected())


if __name__ == "__main__":
    unittest.main()
