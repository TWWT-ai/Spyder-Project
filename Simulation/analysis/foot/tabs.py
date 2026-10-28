import trimesh,numpy as np
m=trimesh.load('meshes/foot_ground.stl'); v=m.vertices
print(v[np.abs(v[:,2]-48.631397)<.001])
print('mids',np.unique(np.round(v[:,1],5))[0:10])
