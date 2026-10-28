import pybullet as p,pathlib,numpy as np
from PIL import Image
p.connect(p.DIRECT)
for name,color in [('foot_top',[.2,.6,.9,1]),('foot_bottom',[.2,.6,.9,1]),('foot_ground',[.9,.5,.15,1]),('foot_tip',[.2,.25,.3,1])]:
 shape=p.createVisualShape(p.GEOM_MESH,fileName=str(pathlib.Path('analysis/foot/'+name+'_assembled.stl').resolve()),meshScale=[.001]*3,rgbaColor=color);p.createMultiBody(0,baseVisualShapeIndex=shape)
for name,yaw,pitch in [('assembled',45,-35),('assembled_side',0,0),('assembled_top',0,-89.9)]:
 vm=p.computeViewMatrixFromYawPitchRoll([.075,0,0],.30,yaw,pitch,0,2);pm=p.computeProjectionMatrixFOV(48,1.6,.01,1)
 pix=p.getCameraImage(1200,750,vm,pm,renderer=p.ER_TINY_RENDERER)[2];Image.fromarray(np.asarray(pix,dtype=np.uint8).reshape(750,1200,4)).save('analysis/foot/'+name+'.png')
p.disconnect()
