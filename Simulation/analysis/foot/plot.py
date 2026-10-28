import trimesh,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(4,1,figsize=(14,12))
for a,name in zip(ax,['foot_top','foot_bottom','foot_ground','foot_tip']):
 m=trimesh.load('meshes/'+name+'.stl');v=m.vertices; a.triplot(v[:,0],v[:,1],m.faces,lw=.2,color='black');a.set_aspect('equal');a.set_title(name);a.grid();a.set_xticks(np.arange(np.floor(v[:,0].min()/10)*10,np.ceil(v[:,0].max()/10)*10+1,10));a.set_yticks(np.arange(np.floor(v[:,1].min()/5)*5,np.ceil(v[:,1].max()/5)*5+1,5))
fig.tight_layout();fig.savefig('analysis/foot/projections.png',dpi=150)
