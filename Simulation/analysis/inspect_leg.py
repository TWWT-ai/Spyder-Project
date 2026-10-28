from pathlib import Path
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path(__file__).resolve().parents[1]
names = ['leg_bottom', 'leg_top', 'Motor']
fig = plt.figure(figsize=(16, 12))
for row, name in enumerate(names):
    mesh = trimesh.load_mesh(ROOT / 'meshes' / 'Leg' / (name + '.stl'))
    print(name, 'bounds', mesh.bounds, 'components', [(len(m.faces), m.bounds.tolist()) for m in mesh.split(only_watertight=False) if len(m.faces)>50])
    print('planes', [(k, sorted(zip(*np.unique(np.round(mesh.vertices[:,k],3),return_counts=True)),key=lambda a:-a[1])[:15]) for k in range(3)])
    for col,(a,b) in enumerate([(0,1),(0,2),(1,2)]):
        ax=fig.add_subplot(3,4,row*4+col+1)
        v=mesh.vertices
        ax.scatter(v[:,a],v[:,b],s=0.5)
        ax.set_aspect('equal'); ax.set_xlabel('XYZ'[a]); ax.set_ylabel('XYZ'[b]); ax.set_title(name)
    ax=fig.add_subplot(3,4,row*4+4, projection='3d')
    ax.add_collection3d(Poly3DCollection(mesh.triangles, facecolor='lightsteelblue',edgecolor='gray',linewidth=.05))
    ax.auto_scale_xyz(*mesh.vertices.T)
    ax.set_box_aspect(mesh.extents.copy())
fig.tight_layout()
fig.savefig(ROOT/'analysis'/'leg_parts.png',dpi=160)
