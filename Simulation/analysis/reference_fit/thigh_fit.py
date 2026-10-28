"""Thigh assembly constrained by user's assembled Fusion thumbnail.

The base cradle is under the motor BACK; the cover sits above its mounting
flange. The assembly is held together by long through-fasteners, not by the
two frame plates touching each other.
"""
from pathlib import Path
import json
import numpy as np
import trimesh
import pybullet as p
from PIL import Image

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
sy=28/28.028
identity=np.eye(3).tolist()
parts={
 'thigh_motor':dict(rotation=identity,scale=[1,1,1],translation_mm=[5.422894,333.241791,-432.466418]),
 'thigh_bottom':dict(rotation=identity,scale=[1,1,1],translation_mm=[-343.778587,-272.5,-309.39998123]),
 'thigh_top':dict(rotation=identity,scale=[1,sy,1],translation_mm=[-149.519091,913.335*sy,44.98365]),
}
result=dict(
 parts=parts,
 interpretation='User assembled Thigh Design thumbnail: cradle below motor back; top frame above flange; long vertical fasteners. Supersedes front-side clamp interpretation.',
 measured_planes_mm=dict(motor_back=-15.39998123,cradle_inside_floor=-15.39998123,
                         cradle_outside_floor=-21.39998123,cradle_screw_post_top=-9.39998123,
                         motor_flange_upper=14.6,cover_inner=14.6,cover_outer=20.6),
 through_fasteners=dict(axes_xy_mm=[[-9.8266,-14],[-9.8266,14]],
                       clear_span_between_post_and_cover_mm=23.99998123,
                       shank_extent_mm=[-21.39998123,23.6],
                       illustrative_radius_mm=1.5,
                       note='Two long through-fasteners visible in reference. Diameter and final length unconfirmed; extent permits a head below the cradle and nut above cover. No Screw STL provided.'),
 remove_previous_generated_parts=['yaw_rear_pivot_adapter','thigh_clamp_spacer_1','thigh_clamp_spacer_2'],
 evidence=[
   'Reference visibly separates lower cradle and top cover across full motor body; previously proposed short collar is contradicted.',
   'Servo source back face Z417.06643677 maps to -15.39998123; cradle source inner floor Z294 maps to same plane.',
   'Servo flange upper source Z447.0664 maps to +14.6; cover source underside Z-30.38365 maps to same plane.',
   'Lower through-hole axes source X333.951965,Y258.5/286.5 map to X-9.826622,Y±14.',
   'Upper hole axes source X139.692504,Y-927.349/-899.321 align after 0.999000999 Y fit; raw mismatch was only0.014mm per side.',
   'No change to the motor geometry or output axis is required.'
 ],
 limitations=[
   'The lower blue coupling inner face at Z-26.8 remains5.400019mm below the cradle outer floor; the thumbnail does not show joint coupling hardware and does not justify inventing a backingplate/hub.',
   'Thigh guide tops at source Z320 lie4mm below cover underside; this is not evidence that the entire cover should be placed on the output side alongside the lower holder.',
   'Reference is a small thumbnail; fastening specs and real clearances still require confirmation.'
 ])
(OUT/'thigh_reference_transforms.json').write_text(json.dumps(result,indent=2)+'\n')

p.connect(p.DIRECT)
try:
 for name,spec in parts.items():
  mesh=trimesh.load_mesh(ROOT/'analysis/joints'/f'{name}_source.stl',process=False)
  mesh.vertices=(mesh.vertices*spec['scale'])@np.array(spec['rotation']).T+spec['translation_mm']
  mesh.apply_scale(.001)
  path=OUT/f'{name}_reference.stl';mesh.export(path)
  color={'thigh_motor':[.16,.18,.21,1],'thigh_bottom':[.04,.45,.47,1],'thigh_top':[.12,.78,.83,1]}[name]
  vis=p.createVisualShape(p.GEOM_MESH,fileName=str(path),rgbaColor=color)
  p.createMultiBody(baseMass=0,baseVisualShapeIndex=vis)
 # Illustrative long screws show the actual connection pattern from reference.
 for y in [-.014,.014]:
  vis=p.createVisualShape(p.GEOM_CYLINDER,radius=.0015,length=.045,rgbaColor=[.6,.62,.65,1])
  p.createMultiBody(baseMass=0,baseVisualShapeIndex=vis,basePosition=[-.0098266,y,.0011])
 width,height=1000,800
 for name,eye in [('front',[.11,-.16,.12]),('rear',[-.13,.16,.1])]:
  view=p.computeViewMatrix(eye,[-.01,0,.002],[0,0,1])
  proj=p.computeProjectionMatrixFOV(30,width/height,.001,1)
  pix=p.getCameraImage(width,height,view,proj,renderer=p.ER_TINY_RENDERER,lightDirection=[-1,-2,4])[2]
  Image.fromarray(np.asarray(pix,dtype=np.uint8).reshape(height,width,4)).convert('RGB').save(OUT/f'thigh_{name}.png')
finally:
 p.disconnect()
print('Wrote thigh_reference_transforms.json and standalone previews')
