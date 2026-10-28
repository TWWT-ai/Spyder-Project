"""Inspect existing foot in the CAD reference's horizontal-plate orientation."""
from pathlib import Path
import json
import numpy as np
import trimesh
import pybullet as p
from PIL import Image,ImageDraw

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
config=json.loads((ROOT/'assembly.json').read_text())
foot=[part for part in config['parts'] if part['link']=='tibia_link']
# Whole-assembly rotation only: servo axis Y becomes vertical Z. This does not
# change any relative fitting transform and is for matching the CAD thumbnail.
rot=np.array([[1,0,0],[0,0,-1],[0,1,0]])
p.connect(p.DIRECT)
measure={}
for part in foot:
    mesh=trimesh.load_mesh(ROOT/part['source'])
    mesh.vertices=(mesh.vertices*np.array(part['scale']))@np.array(part['rotation']).T+part['translation_mm']
    measure[part['name']]={'bounds_link_mm':mesh.bounds.tolist()}
    mesh.vertices=mesh.vertices@rot.T
    path=OUT/(part['name']+'_reference_view.stl')
    mesh.export(path)
    color=[.59,.61,.59,1] if part['name']!='foot_tip' else [.18,.21,.24,1]
    shape=p.createVisualShape(p.GEOM_MESH,fileName=str(path),meshScale=[.001]*3,rgbaColor=color)
    p.createMultiBody(0,baseVisualShapeIndex=shape)
for name,yaw,pitch in [('foot_reference_orientation',45,-25),('foot_reference_reverse',-45,-25)]:
    view=p.computeViewMatrixFromYawPitchRoll([.075,0,0],.29,yaw,pitch,0,2)
    projection=p.computeProjectionMatrixFOV(45,1.6,.001,2)
    pixels=p.getCameraImage(1120,700,view,projection,renderer=p.ER_TINY_RENDERER,shadow=0)[2]
    image=Image.fromarray(np.asarray(pixels,dtype=np.uint8).reshape(700,1120,4)).convert('RGB')
    image.save(OUT/(name+'.png'))
p.disconnect()
(OUT/'foot_review_measurements.json').write_text(json.dumps(measure,indent=2)+'\n')
print(json.dumps(measure,indent=2))
