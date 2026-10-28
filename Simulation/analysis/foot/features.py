import trimesh,numpy as np
for name in ['foot_top','foot_bottom','foot_ground','foot_tip']:
 m=trimesh.load('meshes/'+name+'.stl'); v=m.vertices
 print('\n'+name)
 if name=='foot_bottom': sel=v[(v[:,0]<135)&(abs(v[:,1])<4.1)];print(np.round(sel,3))
 if name=='foot_ground':
  for low,high in [(47,66),(108,121),(160,174)]:
   sel=v[(v[:,0]>low)&(v[:,0]<high)&(v[:,2]>46)&(v[:,2]<47)]; print(low, np.round(sel,3))
