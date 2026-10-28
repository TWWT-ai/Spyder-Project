"""Audit source coverage, fitted axes, units, and the generated assembly."""
from pathlib import Path
import hashlib
import json
import struct
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


class AssemblyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads((ROOT/'assembly.json').read_text())
        cls.report = json.loads((ROOT/'assembly_report.json').read_text())
        cls.parts = {p['name']:p for p in cls.config['parts']}

    def point(self, name, xyz):
        p = self.parts[name]
        return np.array(p['rotation']) @ (np.array(p['scale'])*xyz) + p['translation_mm']

    def test_every_source_used_without_modification(self):
        original = {p.relative_to(ROOT).as_posix() for p in (ROOT/'meshes').rglob('*.stl')
                    if 'assembled' not in p.parts}
        self.assertEqual(original, set(self.report['sources']))
        for source, expected in self.report['sources'].items():
            self.assertEqual(hashlib.sha256((ROOT/source).read_bytes()).hexdigest(), expected)
            source_faces = struct.unpack_from('<I', (ROOT/source).read_bytes(), 80)[0]
            fitted_faces = sum(p['triangles'] for p in self.report['parts'] if p['source']==source)
            self.assertEqual(source_faces, fitted_faces, source)

    def test_thigh_partition_has_no_missing_or_duplicate_faces(self):
        data = (ROOT/'meshes/Thigh Design.stl').read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        triangles = np.array([struct.unpack_from('<9f', data, 84+i*50+12) for i in range(count)]).reshape(-1,3,3)
        centers = triangles.mean(axis=1)
        coverage = np.zeros(count, dtype=int)
        for p in self.config['parts']:
            if p['source']=='meshes/Thigh Design.stl':
                low,high=p['source_region']
                coverage += np.all((centers>=low)&(centers<=high), axis=1)
        np.testing.assert_array_equal(coverage, np.ones(count))

    def test_measured_motor_axes_match_leg_shaft_slots(self):
        length = self.config['joints'][2]['origin_mm'][0]
        for name, expected_x in [('Motor',0),('Motor2',length)]:
            axis = self.point(name,[-5.422895,197.7,66.669016])
            np.testing.assert_allclose(axis,[expected_x,26.8,0],atol=1e-5)
        for x in [754.49007026,831.06631747]:
            shaft=self.point('leg_bottom',[x,1216.97437,-26.0360012])
            self.assertAlmostEqual(shaft[2],0,places=5)
            self.assertLess(min(abs(shaft[0]),abs(shaft[0]-length)),1e-5)

    def test_foot_shaft_fit_and_explicit_contact(self):
        bottom=self.point('foot_bottom',[123.776415,0,-12])
        top=self.point('foot_top',[-282.80487060546875,57.75117492675781,42])
        self.assertLess(np.linalg.norm((bottom-top)[[0,2]]),.14)
        self.assertAlmostEqual((bottom[1]-3)-(top[1]+3),45.91047668457,places=5)
        # Rear plate clears the cradle; output shoulder lies within the driven
        # plate's through-hole instead of forcing the rear plate into the cradle.
        self.assertAlmostEqual(-19.4-(top[1]+3),.2,places=5)
        self.assertLess(bottom[1]-3,26.8)
        self.assertGreater(bottom[1]+3,26.8)
        tip=self.config['foot_contact_mm']
        self.assertTrue(160<tip[0]<175)

    def test_generated_model_is_metric_and_articulated(self):
        robot=ET.parse(ROOT/'leg_assembled.urdf').getroot()
        self.assertEqual(len(robot.findall("joint[@type='revolute']")),3)
        visual_count=len(self.config['parts'])+len(self.config['generated_supports'])
        self.assertEqual(len(robot.findall("joint[@type='fixed']")),visual_count+1)
        self.assertEqual(len(robot.findall('.//visual/geometry/mesh')),visual_count)
        self.assertTrue(all(len(link.findall('visual'))<=1 for link in robot.findall('link')))
        for p in self.report['parts']:
            bounds=np.array(p['bounds_m'])
            self.assertTrue(np.isfinite(bounds).all())
            self.assertLess(np.max(abs(bounds)),.25)
            self.assertGreater(np.max(bounds[1]-bounds[0]),.01)
        for mesh in robot.findall('.//mesh'):
            self.assertTrue((ROOT/mesh.get('filename')).is_file())
            self.assertEqual(mesh.get('scale'),'1 1 1')

    def test_leg_plates_straddle_servo_bodies_as_in_reference(self):
        # Leg design3 has a cradle against the back and a lid above the flange.
        # Checking only coincident screw axes previously missed the wrong side.
        back=self.point('Motor',[-5.422895,155.5,66.669016])
        flange=self.point('Motor',[-5.422895,185.5,66.669016])
        floor=self.point('leg_bottom',[754.49007026,1216.97437,-22.0360012])
        np.testing.assert_allclose(back,floor,atol=.0001)
        for bx,tx in [(764.18583,758.75293),(821.37051,816.50897)]:
            for by,ty in [(1202.97437,170.74926),(1230.97437,198.73806)]:
                seat=self.point('leg_bottom',[bx,by,-14.0360012])
                cover=self.point('leg_top',[tx,ty,21.500044])
                np.testing.assert_allclose(seat[[0,2]],cover[[0,2]],atol=.003)
                self.assertAlmostEqual(cover[1],flange[1],places=5)
                self.assertAlmostEqual(cover[1]-seat[1],22,places=5)
                self.assertGreater(cover[1]-floor[1],29.99)

    def test_thigh_cradle_and_lid_match_reference_and_use_long_fasteners(self):
        back=self.point('thigh_motor',[-5.422894,-333.241791,417.06643677])
        floor=self.point('thigh_bottom',[343.778587,272.5,294])
        np.testing.assert_allclose(back,floor,atol=.0001)
        post=self.point('thigh_bottom',[333.95196533,258.5,300])
        cover=self.point('thigh_top',[139.692504,-927.349,-30.38365])
        np.testing.assert_allclose(post[:2],cover[:2],atol=.02)
        self.assertAlmostEqual(cover[2],14.6,places=5)
        self.assertAlmostEqual(cover[2]-post[2],24,places=4)
        supports={s['name']:s for s in self.config['generated_supports']}
        self.assertEqual(len(supports),6)
        self.assertTrue(all('through_bolt' in name for name in supports))
        for bolt in supports.values():
            shank=bolt['shapes'][0]
            self.assertGreaterEqual(shank['height_mm'],43)
        shank=supports['thigh_through_bolt_1']['shapes'][0]
        np.testing.assert_allclose(shank['center_mm'][:2],post[:2],atol=.002)
        self.assertLess(shank['center_mm'][2]-shank['height_mm']/2,post[2])
        self.assertGreater(shank['center_mm'][2]+shank['height_mm']/2,cover[2])


if __name__=='__main__':
    unittest.main()
