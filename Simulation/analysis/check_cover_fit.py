"""Compare measured cover/seat faces and the servo flange pockets."""
from pathlib import Path
import json
import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parents[1]
config=json.loads((ROOT/'assembly.json').read_text())
parts={p['name']:p for p in config['parts']}

def fitted(name):
    p=parts[name]
    m=trimesh.load_mesh(ROOT/p['source'])
    m.vertices=(m.vertices*np.array(p['scale']))@np.array(p['rotation']).T+p['translation_mm']
    return m

bottom=fitted('leg_bottom')
top=fitted('leg_top')
for name,m in [('bottom',bottom),('top',top)]:
    print(name,'axial planes',sorted(zip(*np.unique(np.round(m.vertices[:,1],3),return_counts=True)),key=lambda x:-x[1])[:8])
for source_z in [21.6,24.4,24.6,27.4]:
    m=trimesh.load_mesh(ROOT/'meshes/Leg/leg_top.stl')
    sec=m.section(plane_origin=[0,0,source_z],plane_normal=[0,0,1])
    print('cover depth',source_z,'loop XY bounds',[(p.min(0)[:2].round(4).tolist(),p.max(0)[:2].round(4).tolist()) for p in sec.discrete if np.ptp(p[:,0])>20])
