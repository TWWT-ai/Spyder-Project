"""Check the whole-robot model against main.stl, the kinematics and physics."""
import json
from pathlib import Path
import sys
import unittest

import numpy as np
import pybullet as p
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_spyder as build
from kinematics import SpyderLeg
import simulate_spyder as sim

ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / 'spyder.json').read_text(encoding='utf-8'))
REPORT = json.loads((ROOT / 'spyder_report.json').read_text(encoding='utf-8'))


class ModelTests(unittest.TestCase):
    def setUp(self):
        self.ground = sim.connect(True, False)
        self.robot = sim.Spyder(fixed_base=True, ground=self.ground)
        self.addCleanup(p.disconnect)

    def test_lists_all_18_servo_joints_with_room_to_move_both_ways(self):
        self.assertEqual(len(self.robot.joints), 18)
        kinds = [p.getJointInfo(self.robot.id, i)[2] for i in range(p.getNumJoints(self.robot.id))]
        self.assertEqual(kinds.count(p.JOINT_REVOLUTE), 18)
        self.assertEqual(kinds.count(p.JOINT_FIXED), len(kinds) - 18)
        for name, (lower, upper) in self.robot.limits.items():
            self.assertLess(lower, 0, name)
            self.assertGreater(upper, 0, name)

    def link_frames(self):
        return {name: np.array(p.getLinkState(self.robot.id, index, computeForwardKinematics=True)[4])
                for name, index in self.robot.links.items()
                if name.endswith(('_coxa', '_femur', '_tibia', '_foot_tip'))}

    def test_each_joint_moves_only_the_links_beyond_it(self):
        chain = ('coxa', 'femur', 'tibia', 'foot_tip')
        before = self.link_frames()
        for leg in sim.LEGS:
            for position, joint in enumerate(sim.JOINTS):
                name = f'{leg}_{joint}'
                self.robot.pose({name: 0.5})
                moved = {link for link, frame in self.link_frames().items()
                         if np.linalg.norm(frame - before[link]) > 1e-6}
                # A driven link's origin stays on its axis, so only links after it shift.
                self.assertEqual(moved, {f'{leg}_{link}' for link in chain[position + 1:]}, name)
                self.robot.pose({name: 0.0})

    def test_invalid_commands_are_rejected(self):
        for name, angle in (('fr_femur_pitch', 100), ('fr_femur_pitch', float('nan')), ('joint_0', 0.1)):
            with self.subTest(name=name, angle=angle), self.assertRaises(ValueError):
                self.robot.command({name: angle})

    def test_forward_kinematics_matches_pybullet_for_every_leg(self):
        rng = np.random.default_rng(3)
        base = np.array(self.robot.body_pose()[0])
        for leg, model in self.robot.legs.items():
            for _ in range(5):
                angles = rng.uniform(model.lower, model.upper)
                self.robot.pose({f'{leg}_{joint}': float(a) for joint, a in zip(sim.JOINTS, angles)})
                self.assertLess(np.linalg.norm(self.robot.foot_world(leg) - base - model.forward(angles)), 1e-5, leg)

    def test_home_pose_has_no_overlapping_neighbours(self):
        self.assertEqual(self.robot.touching_legs(), [])


class InverseKinematicsTests(unittest.TestCase):
    def test_round_trip_for_every_leg(self):
        rng = np.random.default_rng(11)
        for leg in sim.LEGS:
            model = SpyderLeg(sim.URDF, leg)
            for _ in range(200):
                target = model.forward(rng.uniform(model.lower, model.upper))
                self.assertLess(np.linalg.norm(model.forward(model.inverse(target)) - target), 1e-9, leg)

    def test_unreachable_and_invalid_targets_are_rejected(self):
        model = SpyderLeg(sim.URDF, 'fr')
        home = model.forward(np.zeros(3))
        for target in (home + [1, 1, 0], [0, 0, float('nan')], [0, 0]):
            with self.subTest(target=target), self.assertRaises(ValueError):
                model.inverse(target)


class SourceTests(unittest.TestCase):
    def test_rebuilt_robot_matches_main_stl_at_zero_angles(self):
        source = build.read_stl(ROOT / CONFIG['source'])
        points = source.reshape(-1, 3)
        offset = np.array([(points[:, 0].min() + points[:, 0].max()) / 2,
                           (points[:, 1].min() + points[:, 1].max()) / 2, 0])
        tree = cKDTree(points - offset)
        body = build.read_stl(ROOT / 'meshes/spyder/body.stl')
        total = len(body)
        rebuilt = [body.reshape(-1, 3)[::20]]
        for leg in sim.LEGS:
            model = SpyderLeg(sim.URDF, leg)
            femur = model.chain[2].origin[:3, 3]
            shift = {'base': 0, 'coxa': 0, 'femur': femur, 'tibia': femur + model.chain[3].origin[:3, 3]}
            for group, settings in CONFIG['groups'].items():
                mesh = build.read_stl(ROOT / f'meshes/spyder/{group}.stl')
                total += len(mesh)
                local = mesh.reshape(-1, 3)[::20] + shift[settings['link']]
                rebuilt.append((model.mount[:3, :3] @ local.T).T + model.mount[:3, 3])
        self.assertEqual(total, len(source))
        self.assertLess(tree.query(np.vstack(rebuilt) * 1000)[0].max(), 0.01)


class PhysicsTests(unittest.TestCase):
    def test_stands_with_the_weight_shared_across_six_feet(self):
        ground = sim.connect(True, True)
        self.addCleanup(p.disconnect)
        robot = sim.Spyder(ground=ground)
        home = {name: 0.0 for name in robot.joints}
        for _ in range(480):
            robot.command(home)
            p.stepSimulation()
        start = np.array(robot.body_pose()[0])
        for _ in range(480):
            robot.command(home)
            p.stepSimulation()
        self.assertLess(np.linalg.norm(np.array(robot.body_pose()[0]) - start), 5e-4)
        forces = {}
        for contact in p.getContactPoints(robot.id):
            forces[contact[3]] = forces.get(contact[3], 0) + contact[9]
        self.assertEqual(set(forces), {robot.links[f'{leg}_foot_tip'] for leg in sim.LEGS})
        weight = REPORT['mass_kg']['total'] * 9.81
        self.assertAlmostEqual(sum(forces.values()), weight, delta=weight * 0.05)
        for force in forces.values():
            self.assertAlmostEqual(force, weight / 6, delta=weight / 6 * 0.25)
        self.assertLess(max(abs(t) for t in robot.torques().values()), 0.3)

    def test_tripod_gait_walks_forward_upright_without_legs_touching(self):
        summary = sim.run(mode='walk', direct=True, seconds=4, forward=0.04)
        travel = summary['body_end'] - summary['body_start']
        self.assertGreater(travel[1], 0.12)
        self.assertLess(abs(travel[0]), 0.02)
        self.assertLess(abs(summary['yaw_change_deg']), 5)
        self.assertLess(summary['max_tilt_deg'], 3)
        self.assertGreater(summary['lowest_body_z'], 0.1)
        self.assertEqual(summary['touching'], [])

    def test_gait_turns_on_the_spot_in_the_commanded_direction(self):
        for rate in (0.3, -0.3):
            summary = sim.run(mode='walk', direct=True, seconds=4, forward=0.0, turn=rate)
            self.assertGreater(summary['yaw_change_deg'] * np.sign(rate), 40, rate)


if __name__ == '__main__':
    unittest.main()
