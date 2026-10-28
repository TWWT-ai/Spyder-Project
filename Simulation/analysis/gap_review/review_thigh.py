"""Diagnostic only: compare holder orientation using measured shaft/flange planes."""
from pathlib import Path
import json
import numpy as np
import trimesh
import pybullet as p
from PIL import Image

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).parent
config=json.loads((ROOT/'assembly.json').read_text())
selected=[q for q in config['parts'] if q['link'] in ['base_link','coxa_link']]
corrections={
 'thigh_bottom':dict(rotation=np.diag([1,-1,-1]).tolist(),translation_mm=[-343.778587,272.5,314.6]),
 'thigh_top':dict(rotation=np.diag([1,-1,-1]).tolist(),translation_mm=[-149.519091,-913.334956,-18.78365]),
}
diagnostic=dict(corrections=corrections,planes_mm=dict(shaft_shoulder=26.8,
 lower_front_face=26.6,lower_post_face=14.6,servo_flange=[11.6,14.6],
 upper_cover_inner_face=11.6,servo_back_face=-15.4,blue_rear_inner_face=-26.8),
 generated_support_proposal=dict(backplate=dict(center_mm=[-10.75,0,-16.9],size_mm=[40.6,20,3],
   hole_centres_xy_mm=[[-28.75,-7.7509],[-28.75,7.7509],[7.25,-7.7509],[7.25,7.7509]],
   hole_diameter_mm=2.2),
 hub=dict(center_mm=[0,0,-22.6],height_mm=8.4,outer_radius_mm=8,inner_radius_mm=3.2),
 clamp_spacers=dict(centres_mm=[[-9.8266,-14,13.1],[-9.8266,14,13.1]],height_mm=3,outer_radius_mm=3.5,inner_radius_mm=2)),
 measured_features=dict(
   lower_keyhole_source_centre_mm=[343.77997832,272.49999978,288],
   lower_keyhole_semiaxes_mm=[3.91300736,3.76249506],
   lower_screw_post_centres_source_mm=[[333.95196533,258.5,300],[333.95196533,286.5,300]],
   upper_matching_hole_centres_source_mm=[[139.692504,-927.349, -30.38365],[139.692504,-899.321,-30.38365]],
   yaw_plate_hex_circumradius_mm=6.62392,yaw_plate_hex_across_flats_mm=11.544,
   motor_rear_screw_centres_assembled_xy_mm=[[-28.75,-7.7509],[-28.75,7.7509],[7.25,-7.7509],[7.25,7.7509]]),
 caveat='Support plate/hub are new inferred hardware, not supplied CAD; attachment pattern and bearing specification remain unknown.')
(OUT/'thigh_correction.json').write_text(json.dumps(diagnostic,indent=2))
p.connect(p.DIRECT)
for q in selected:
 q=dict(q);q.update(corrections.get(q['name'],{}))
 m=trimesh.load_mesh(ROOT/q['source'],process=False)
 if 'source_region' in q:
  lo,hi=q['source_region'];mask=np.all((m.triangles_center>=lo)&(m.triangles_center<=hi),axis=1)
  m=trimesh.Trimesh(vertices=m.vertices.copy(),faces=m.faces[mask],process=False)
 m.vertices=(m.vertices*np.array(q['scale']))@np.array(q['rotation']).T+q['translation_mm']
 filename=OUT/(q['name']+'.stl');m.export(filename)
 color=q['color']
 visual=p.createVisualShape(p.GEOM_MESH,fileName=str(filename),meshScale=[.001]*3,rgbaColor=color)
 p.createMultiBody(baseMass=0,baseVisualShapeIndex=visual)
for name,mesh in [('rear_backplate',trimesh.creation.box([40.6,20,3])),('rear_hub',trimesh.creation.annulus(3.2,8,8.4,sections=64))]:
 center=[-10.75,0,-16.9] if name=='rear_backplate' else [0,0,-22.6]
 mesh.apply_translation(center);file=OUT/(name+'.stl');mesh.export(file)
 visual=p.createVisualShape(p.GEOM_MESH,fileName=str(file),meshScale=[.001]*3,rgbaColor=[.72,.73,.76,1])
 p.createMultiBody(baseMass=0,baseVisualShapeIndex=visual)
for yaw in [30,120,210]:
 width,height=1040,800
 view=p.computeViewMatrixFromYawPitchRoll([.008,0,0],.19,yaw,-20,0,2)
 proj=p.computeProjectionMatrixFOV(40,width/height,.01,1)
 pixels=p.getCameraImage(width,height,view,proj,renderer=p.ER_TINY_RENDERER,lightDirection=[1,-1,2])[2]
 Image.fromarray(np.asarray(pixels,dtype=np.uint8).reshape(height,width,4)).save(OUT/f'corrected_{yaw}.png')
p.disconnect()
