import pybullet as p, struct, pathlib, json, math
p.connect(p.DIRECT)
w,h=800,600
for name in ('foot_top','foot_bottom','foot_ground','foot_tip'):
 data=json.loads(pathlib.Path('analysis/foot/'+name+'.json').read_text()); vs=data['vertices']; lo=[min(v[a] for v in vs) for a in range(3)];hi=[max(v[a] for v in vs) for a in range(3)]; center=[(a+b)/2 for a,b in zip(lo,hi)];size=max(b-a for a,b in zip(lo,hi))
 shape=p.createVisualShape(p.GEOM_MESH,fileName=str(pathlib.Path('meshes/'+name+'.stl').resolve()),meshScale=[.001]*3,rgbaColor=[.25,.58,.9,1]);body=p.createMultiBody(0,baseVisualShapeIndex=shape,basePosition=[-c*.001 for c in center]);
 for view,(yaw,pitch) in {'top':(0,-89.9),'oblique':(35,-45)}.items():
  vm=p.computeViewMatrixFromYawPitchRoll([0,0,0],size*.0018,yaw,pitch,0,2);pm=p.computeProjectionMatrixFOV(50,w/h,.001,2)
  rgba=p.getCameraImage(w,h,vm,pm,renderer=p.ER_TINY_RENDERER)[2];rgba=list(rgba)
  if len(rgba)==h: rgba=[c for row in rgba for pixel in row for c in pixel]
  stride=(w*3+3)//4*4;buf=bytearray()
  for y in range(h-1,-1,-1):
   for x in range(w):
    i=(y*w+x)*4;buf+=bytes((rgba[i+2],rgba[i+1],rgba[i]))
   buf+=b'\x00'*(stride-w*3)
  header=struct.pack('<2sIHHI',b'BM',54+len(buf),0,0,54)+struct.pack('<IiiHHIIiiII',40,w,h,1,24,0,len(buf),0,0,0,0)
  pathlib.Path('analysis/foot/'+name+'_'+view+'.bmp').write_bytes(header+buf)
 p.removeBody(body)
p.disconnect()
