from pathlib import Path
import struct
from collections import Counter, defaultdict
import json

ROOT = Path(__file__).resolve().parents[2]

def read(path):
    b=path.read_bytes()
    n=struct.unpack_from('<I',b,80)[0]
    return [tuple(tuple(struct.unpack_from('<fff',b,84+i*50+12+j*12)) for j in range(3)) for i in range(n)]

def info(tris):
    vs=set(v for t in tris for v in t)
    return dict(triangles=len(tris),vertices=len(vs),bounds=[[min(v[k] for v in vs) for k in range(3)],[max(v[k] for v in vs) for k in range(3)]], common_coords=[Counter(round(v[k],3) for v in vs).most_common(16) for k in range(3)])

def split(tris):
    byv=defaultdict(list)
    for i,t in enumerate(tris):
        for v in t: byv[tuple(round(x,4) for x in v)].append(i)
    seen=set(); comps=[]
    for i in range(len(tris)):
        if i in seen: continue
        seen.add(i); todo=[i]; comp=[]
        while todo:
            a=todo.pop();comp.append(tris[a])
            for v in tris[a]:
                for j in byv[tuple(round(x,4) for x in v)]:
                    if j not in seen: seen.add(j);todo.append(j)
        comps.append(comp)
    return sorted(comps,key=len,reverse=True)

if __name__=='__main__':
    for p in list((ROOT/'meshes/Joints').glob('*.stl'))+[ROOT/'meshes/Thigh Design.stl']:
        ts=read(p); print(p.name,json.dumps(info(ts)))
        cs=split(ts)
        print('COMPONENTS',len(cs))
        for i,c in enumerate(cs): print(i,json.dumps(info(c)))
