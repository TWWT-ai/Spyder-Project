import trimesh,numpy as np
m=trimesh.load('meshes/foot_top.stl'); v=m.vertices
print(np.round(v[(v[:,0]>-292)&(v[:,0]<-273)&(v[:,1]>49)&(v[:,1]<66)&(v[:,2]>44.9)],4))
