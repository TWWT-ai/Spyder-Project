import trimesh,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
m=trimesh.load('meshes/foot_bottom.stl');v=m.vertices
fig,axes=plt.subplots(2,1,figsize=(14,7))
for ax,(lo,hi) in zip(axes,[(160,180),(215,234)]):
 for z in [-15,-12]:
  s=m.section([0,0,1],[0,0,z+.001]);
  for a in s.discrete: ax.plot(a[:,0],a[:,1],lw=.8,label=str(z))
 ax.set_xlim(lo,hi);ax.set_ylim(-8,8);ax.set_xticks(np.arange(lo,hi,.5));ax.set_yticks(np.arange(-8,9,1));ax.tick_params(axis='x',rotation=90);ax.grid();ax.set_aspect('equal')
fig.tight_layout();fig.savefig('analysis/foot/slots.png',dpi=150)
