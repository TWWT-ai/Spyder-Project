from pathlib import Path
import json
import numpy as np
from PIL import Image
import pybullet as p

out=Path(__file__).parent
data=json.loads((out/'assembly_transforms.json').read_text())
p.connect(p.DIRECT)
for i,part in enumerate(data['parts']):
    color=([.2,.5,.7,1] if 'joint_' in part['name'] else [.8,.5,.2,1] if 'yaw_' in part['name'] else [.2,.7,.5,1] if 'pitch_' in part['name'] else [.2,.2,.25,1] if 'motor' in part['name'] else [.6,.3,.7,1])
    shape=p.createVisualShape(p.GEOM_MESH,fileName=str((out/(part['name']+'_assembled.stl')).resolve()),meshScale=[.001]*3,rgbaColor=color)
    p.createMultiBody(baseMass=0,baseVisualShapeIndex=shape)
for yaw in [30,120,210]:
    view=p.computeViewMatrixFromYawPitchRoll([.01,0,0],.19,yaw,-25,0,2)
    proj=p.computeProjectionMatrixFOV(40,1.3,.01,1)
    pixels=p.getCameraImage(1040,800,view,proj,renderer=p.ER_TINY_RENDERER,lightDirection=[1,-1,2])[2]
    Image.fromarray(np.asarray(pixels,dtype=np.uint8).reshape(800,1040,4)).save(out/f'assembly_view_{yaw}.png')
p.disconnect()
