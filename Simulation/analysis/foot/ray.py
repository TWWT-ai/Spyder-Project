import trimesh,numpy as np
for name,x,y in [('foot_bottom',224,0),('foot_top',224-406.7141,57.7512)]:
 m=trimesh.load('meshes/'+name+'.stl');tri=m.triangles;p=np.array([x,y]);z=[]
 for a,b,c in tri:
  M=np.column_stack([b[:2]-a[:2],c[:2]-a[:2]])
  if abs(np.linalg.det(M))<1e-8:continue
  uv=np.linalg.solve(M,p-a[:2]);u,v=uv
  if min(u,v)>=-1e-7 and u+v<=1+1e-7:z.append(a[2]+u*(b[2]-a[2])+v*(c[2]-a[2]))
 print(name,sorted(set(np.round(z,5))))
