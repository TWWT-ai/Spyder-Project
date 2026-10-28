exec(open('analysis/inspect_leg.py').read().split("names =")[0])
fig,axs=plt.subplots(3,2,figsize=(16,12))
for row,name in enumerate(['leg_bottom','leg_top','Motor']):
 m=trimesh.load_mesh(ROOT/'meshes'/'Leg'/(name+'.stl'))
 for col,normal in enumerate([[0,0,1],[0,1,0]]):
  ax=axs[row,col]
  if name=='leg_bottom': levels=[-25,-23,-21,-15,-10,0] if col==0 else [1203,1207,1217,1230]
  elif name=='leg_top': levels=[22,25,28,30] if col==0 else [170,175,185,195]
  else: levels=[58,66.669,75] if col==0 else [156,183,186,194,197,200]
  a,b=(0,1) if col==0 else (0,2)
  for i,level in enumerate(levels):
   origin=[0.,0.,0.];origin[2 if col==0 else 1]=level
   sec=m.section(origin,normal) if False else m.section(plane_origin=origin,plane_normal=normal)
   if sec:
    for n,p in enumerate(sec.discrete): ax.plot(p[:,a],p[:,b],linewidth=.7,color=f'C{i}',label=str(level) if n==0 else None)
  ax.axis('equal');ax.legend();ax.grid();ax.set_title(name+' '+('XY' if col==0 else 'XZ'))
fig.tight_layout();fig.savefig(ROOT/'analysis'/'leg_sections.png',dpi=180)
