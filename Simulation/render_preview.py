"""Render repeatable assembly views and a short kinematic animation."""
from pathlib import Path
import math
import json
import numpy as np
from PIL import Image, ImageDraw
import pybullet as p

ROOT=Path(__file__).resolve().parent


def component_views(output):
    """Compare each rigid assembly in the upright orientation of the CAD tiles."""
    config=json.loads((ROOT/'assembly.json').read_text())
    tiles=[]
    for group,label,target,distance in [
        ('base_link','Thigh Design',[0,0,0],.15),
        ('femur_link','Leg design 3',[.038,0,0],.23),
        ('tibia_link','Foot Design',[.076,0,0],.32),
    ]:
        p.resetSimulation()
        rotation=[0,0,0,1] if group=='base_link' else p.getQuaternionFromEuler([math.pi/2,0,0])
        for part in config['parts']+config.get('generated_supports',[]):
            if part['link'] != group: continue
            shape=p.createVisualShape(p.GEOM_MESH,fileName=str(ROOT/'meshes/assembled'/(part['name']+'.stl')),
                                      rgbaColor=part['color'])
            p.createMultiBody(baseMass=0,baseVisualShapeIndex=shape,baseOrientation=rotation)
        width,height=800,650
        view=p.computeViewMatrixFromYawPitchRoll(target,distance,35,-25,0,2)
        projection=p.computeProjectionMatrixFOV(42,width/height,.001,2)
        pixels=p.getCameraImage(width,height,view,projection,renderer=p.ER_TINY_RENDERER,
                                lightDirection=[-1,-2,4],shadow=0)[2]
        tile=Image.fromarray(np.asarray(pixels,dtype=np.uint8).reshape(height,width,4)).convert('RGB')
        ImageDraw.Draw(tile).text((24,24),label,fill=(30,40,50))
        tile.save(output/(group+'_reference.png'))
        tiles.append(tile)
    panel=Image.new('RGB',(2400,650),'white')
    for i,tile in enumerate(tiles): panel.paste(tile,(800*i,0))
    panel.save(output/'reference_components.png')


def main():
    output=ROOT/'previews'
    output.mkdir(exist_ok=True)
    p.connect(p.DIRECT)
    try:
        body=p.loadURDF(str(ROOT/'leg_assembled.urdf'),useFixedBase=True)
        movable = [i for i in range(p.getNumJoints(body)) if p.getJointInfo(body,i)[2] == p.JOINT_REVOLUTE]
        def render(angles, yaw=38, pitch=-25, target=(.12,0,-.07), distance=.43, width=1000, height=750):
            for i,angle in zip(movable,angles): p.resetJointState(body,i,math.radians(angle))
            view=p.computeViewMatrixFromYawPitchRoll(target,distance,yaw,pitch,0,2)
            projection=p.computeProjectionMatrixFOV(45,width/height,.001,2)
            pixels=p.getCameraImage(width,height,view,projection,renderer=p.ER_TINY_RENDERER,
                                    lightDirection=[-1,-2,4],shadow=0)[2]
            return Image.fromarray(np.asarray(pixels,dtype=np.uint8).reshape(height,width,4)).convert('RGB')
        render([0,20,35]).save(output/'assembled_leg.png')
        render([0,0,0],yaw=35,pitch=-65,target=(.12,0,0),distance=.47).save(output/'neutral.png')
        render([0,0,0],yaw=-30,pitch=-25,target=(.075,0,0),distance=.24).save(output/'joint_detail.png')
        frames=[]
        for frame in range(64):
            t=2*math.pi*frame/64
            angles=[22*math.sin(t),20+15*math.sin(t+.7),35+18*math.sin(t+1.4)]
            view=render(angles,width=640,height=480)
            draw=ImageDraw.Draw(view)
            draw.text((16,16),'Single-leg joint motion | inferred CAD assembly',fill=(30,40,50))
            draw.text((16,450),'Yaw / hip / knee: '+ ' / '.join(f'{a:.0f} deg' for a in angles),fill=(30,40,50))
            frames.append(view)
        frames[0].save(output/'joint_motion.gif',save_all=True,append_images=frames[1:],duration=65,loop=0)
        component_views(output)
        print('Saved assembly, reference component views and joint_motion.gif in previews/')
    finally:
        p.disconnect()


if __name__=='__main__':
    main()
