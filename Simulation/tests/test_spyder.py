"""Check the whole-robot model against main.stl, the kinematics and physics."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

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

    def test_six_seven_targets_alternate_the_front_legs_within_limits(self):
        dance = sim.SixSevenDance(self.robot)
        self.assertTrue(all(angle == 0 for angle in dance.targets(0).values()))
        first_step = dance.targets(dance.STEP_SECONDS / 2)
        second_step = dance.targets(dance.step_start['ml'] + dance.STEP_SECONDS / 2)
        for moving, planted, targets in (('mr', 'ml', first_step), ('ml', 'mr', second_step)):
            angles = [targets[f'{moving}_{joint}'] for joint in sim.JOINTS]
            self.assertGreater(self.robot.legs[moving].forward(angles)[2], dance.middle_home[moving][2] + 0.03)
            if planted == 'ml':
                self.assertTrue(all(targets[f'ml_{joint}'] == 0 for joint in sim.JOINTS))
            else:
                planted_angles = [targets[f'{planted}_{joint}'] for joint in sim.JOINTS]
                self.assertGreater(self.robot.legs[planted].forward(planted_angles)[1], 0.14)
            self.assertTrue(all(targets[f'{leg}_{joint}'] == 0 for leg in ('fr', 'fl') for joint in sim.JOINTS))
        self.assertTrue(all(dance.targets(dance.front_start)[f'{leg}_{joint}'] == 0
                            for leg in ('fr', 'fl') for joint in sim.JOINTS))
        first = dance.targets(dance.gesture_start + 0.3)
        second = dance.targets(dance.gesture_start + 0.9)
        self.robot.check(first)
        self.robot.check(second)
        for leg in ('mr', 'ml'):
            angles = [first[f'{leg}_{joint}'] for joint in sim.JOINTS]
            self.assertGreater(self.robot.legs[leg].forward(angles)[1], 0.14)
        self.assertGreater(first['fr_femur_pitch'], first['fl_femur_pitch'])
        self.assertLess(second['fr_femur_pitch'], second['fl_femur_pitch'])
        for targets in (first, second):
            self.robot.pose(targets)
            self.assertLess(np.linalg.norm(self.robot.foot_world('fr') - self.robot.foot_world('fl')), 0.20)
            self.assertEqual(self.robot.touching_legs(), [])
            for leg in ('fr', 'fl'):
                knee_z = p.getLinkState(self.robot.id, self.robot.links[f'{leg}_tibia'],
                                        computeForwardKinematics=True)[4][2]
                self.assertGreater(self.robot.foot_world(leg)[2] - knee_z, 0.05)

    def test_front_emotes_keep_the_other_five_legs_planted(self):
        for motion in ('hello', 'point'):
            with self.subTest(motion=motion):
                targets = sim.FrontEmote(self.robot, motion).targets(1.2)
                self.robot.check(targets)
                self.assertTrue(all(targets[f'{leg}_{joint}'] == 0
                                    for leg in sim.LEGS if leg != 'fr' for joint in sim.JOINTS))
                self.robot.pose(targets)
                self.assertGreater(self.robot.foot_world('fr')[2],
                                   self.robot.legs['fr'].forward(np.zeros(3))[2] + 0.04)
                self.assertEqual(self.robot.touching_legs(), [])

    def test_foot_taps_lift_one_leg_in_front_to_rear_order(self):
        wave = sim.FootTapWave(self.robot)
        for index, moving in enumerate(wave.ORDER):
            with self.subTest(leg=moving):
                targets = wave.targets((index + 0.5) * wave.SECONDS_PER_TAP)
                self.robot.check(targets)
                self.assertTrue(all(targets[f'{leg}_{joint}'] == 0
                                    for leg in sim.LEGS if leg != moving for joint in sim.JOINTS))
                angles = [targets[f'{moving}_{joint}'] for joint in sim.JOINTS]
                self.assertGreater(self.robot.legs[moving].forward(angles)[2], wave.home[moving][2] + 0.035)

    def test_four_leg_emotes_step_middle_feet_before_lifting_front_pair(self):
        for motion in ('v', 'claws'):
            with self.subTest(motion=motion):
                dance = sim.SixSevenDance(self.robot, front_motion=motion)
                stepping = dance.targets(dance.step_start['ml'] + dance.STEP_SECONDS / 2)
                self.assertTrue(all(stepping[f'{leg}_{joint}'] == 0
                                    for leg in ('fr', 'fl') for joint in sim.JOINTS))
                final = dance.targets(dance.gesture_start + 0.3)
                self.robot.check(final)
                self.robot.pose(final)
                self.assertEqual(self.robot.touching_legs(), [])
                for leg in ('fr', 'fl'):
                    self.assertGreater(self.robot.foot_world(leg)[2], 0.04)

    def test_wiggle_and_crab_motion_start_from_home(self):
        self.assertLess(max(abs(angle) for angle in sim.WiggleDance(self.robot).targets(0).values()), 1e-10)
        self.assertLess(max(abs(angle) for angle in sim.CrabDance(self.robot).targets(0).values()), 1e-10)
        gait = sim.TripodGait(self.robot)
        sideways = gait.targets(0.6, lateral=0.04)
        self.robot.check(sideways)
        self.assertTrue(any(abs(sideways[f'{leg}_coxa_yaw']) > 0.01 for leg in sim.LEGS))


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


class CommandLineTests(unittest.TestCase):
    def test_each_new_flag_selects_its_motion(self):
        for mode in ('wiggle', 'hello', 'point', 'v-sign', 'foot-tap-wave', 'crab-walk', 'crab-dance'):
            with self.subTest(mode=mode), patch.object(sys, 'argv', ['simulate_spyder.py', '--' + mode]), \
                    patch.object(sim, 'run') as run:
                sim.main()
                self.assertEqual(run.call_args.kwargs['mode'], mode)


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
    def test_new_moves_keep_the_expected_support_under_gravity(self):
        motions = (
            ('wiggle', 3.5, 6, sim.WiggleDance),
            ('hello', 2.0, 5, lambda robot: sim.FrontEmote(robot, 'hello')),
            ('point', 2.0, 5, lambda robot: sim.FrontEmote(robot, 'point')),
            ('v-sign', 6.0, 4, lambda robot: sim.SixSevenDance(robot, front_motion='v')),
            ('foot-tap-wave', 4.5, 5, sim.FootTapWave),
            ('crab-walk', 4.0, 3, lambda robot: sim.TripodGait(robot, swing_fraction=0.42, lift_exponent=2)),
            ('crab-dance', 10.0, 3, sim.CrabDance),
        )
        for mode, seconds, support, make_motion in motions:
            with self.subTest(mode=mode):
                ground = sim.connect(True, True)
                try:
                    robot = sim.Spyder(ground=ground)
                    home = {name: 0.0 for name in robot.joints}
                    for _ in range(int(0.5 / sim.TIME_STEP)):
                        robot.command(home)
                        p.stepSimulation()
                    motion = make_motion(robot)
                    minimum = len(sim.LEGS)
                    wiggle_positions, wiggle_yaws = [], []
                    resting_height = sim.body_state(robot)[0][2]
                    foot_links = {robot.links[f'{leg}_foot_tip'] for leg in sim.LEGS}
                    for step in range(int(seconds / sim.TIME_STEP)):
                        time_s = step * sim.TIME_STEP
                        targets = motion.targets(time_s, lateral=0.04) if mode == 'crab-walk' else motion.targets(time_s)
                        robot.command(targets)
                        p.stepSimulation()
                        contacts = {leg for leg in sim.LEGS
                                    if p.getContactPoints(robot.id, ground, robot.links[f'{leg}_foot_tip'], -1)}
                        minimum = min(minimum, len(contacts))
                        if mode == 'wiggle':
                            body_position, _, _, body_yaw = sim.body_state(robot)
                            wiggle_positions.append(body_position)
                            wiggle_yaws.append(body_yaw)
                            self.assertTrue(all(contact[3] in foot_links
                                                for contact in p.getContactPoints(robot.id, ground)))
                    position, roll, pitch, _ = sim.body_state(robot)
                    self.assertGreaterEqual(minimum, support)
                    self.assertGreater(position[2], 0.1)
                    self.assertLess(max(abs(roll), abs(pitch)), 5)
                    self.assertEqual(robot.touching_legs(), [])
                    if mode == 'wiggle':
                        travel = np.ptp(wiggle_positions, axis=0)
                        self.assertGreater(travel[0], 0.040)
                        self.assertGreater(travel[2], 0.032)
                        self.assertGreater(np.ptp(wiggle_yaws), 3)
                        def at(time_s):
                            return wiggle_positions[round(time_s / sim.TIME_STEP)]

                        entry, period = motion.ENTRY_SECONDS, motion.PERIOD_SECONDS
                        right = at(entry)
                        first_bottom = at(entry + period / 4)
                        left = at(entry + period / 2)
                        second_bottom = at(entry + 3 * period / 4)
                        right_again = at(entry + period)
                        centre_x = wiggle_positions[0][0]
                        self.assertGreater(right[0] - centre_x, 0.018)
                        self.assertLess(left[0] - centre_x, -0.018)
                        for edge in (right, left, right_again):
                            self.assertAlmostEqual(edge[2], resting_height, delta=0.003)
                        for bottom in (first_bottom, second_bottom):
                            self.assertAlmostEqual(bottom[0], centre_x, delta=0.005)
                            self.assertLess(bottom[2], resting_height - 0.030)
                        self.assertLess(max(point[2] for point in wiggle_positions), resting_height + 0.002)
                        self.assertAlmostEqual(right_again[0], right[0], delta=0.003)
                    if mode in ('v-sign', 'crab-dance'):
                        self.assertEqual(contacts, {'mr', 'ml', 'rr', 'rl'})
                finally:
                    p.disconnect()

    def test_six_seven_stands_on_four_legs_with_front_legs_raised(self):
        summary = sim.run(mode='67', direct=True, seconds=6)
        self.assertEqual(set(summary['grounded_legs']), {'mr', 'ml', 'rr', 'rl'})
        self.assertLess(summary['max_tilt_deg'], 5)
        self.assertGreater(summary['lowest_body_z'], 0.1)
        self.assertLess(np.linalg.norm((summary['body_end'] - summary['body_start'])[:2]), 0.02)
        self.assertEqual(summary['touching'], [])

    def test_six_seven_steps_middle_feet_forward_without_sliding_body(self):
        ground = sim.connect(True, True)
        self.addCleanup(p.disconnect)
        robot = sim.Spyder(ground=ground)
        dance = sim.SixSevenDance(robot)
        home = {name: 0.0 for name in robot.joints}
        for _ in range(int(0.5 / sim.TIME_STEP)):
            robot.command(home)
            p.stepSimulation()
        start = np.array(robot.body_pose()[0])
        samples = {}
        foot_heights = {}
        min_contacts = len(sim.LEGS)
        for step in range(int(dance.front_start / sim.TIME_STEP)):
            time_s = step * sim.TIME_STEP
            robot.command(dance.targets(time_s))
            p.stepSimulation()
            contacts = {leg for leg in sim.LEGS
                        if p.getContactPoints(robot.id, ground, robot.links[f'{leg}_foot_tip'], -1)}
            min_contacts = min(min_contacts, len(contacts))
            for key, sample_time in (('first', dance.STEP_SECONDS / 2),
                                     ('second', dance.step_start['ml'] + dance.STEP_SECONDS / 2)):
                if abs(time_s - sample_time) < sim.TIME_STEP / 2:
                    samples[key] = contacts
                    foot_heights[key] = robot.foot_world('mr' if key == 'first' else 'ml')[2]
        self.assertEqual(samples['first'], set(sim.LEGS) - {'mr'})
        self.assertEqual(samples['second'], set(sim.LEGS) - {'ml'})
        self.assertGreater(min(foot_heights.values()), 0.025)
        self.assertGreaterEqual(min_contacts, 4)
        self.assertLess(np.linalg.norm((np.array(robot.body_pose()[0]) - start)[:2]), 0.02)

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
