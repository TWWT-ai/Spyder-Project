import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))
from inspect_meshes import *
import numpy as np

def loops(tris,k,level,tol=.01):
    ts=np.asarray(tris)
    ts=ts[np.max(abs(ts[:,:,k]-level),axis=1)<tol]
    edges=Counter()
    for t in ts:
        t=[tuple(np.round(v,4)) for v in t]
        for a,b in [(0,1),(1,2),(2,0)]:edges[tuple(sorted((t[a],t[b])))]+=1
    adj=defaultdict(set)
    for (a,b),n in edges.items():
        if n==1:adj[a].add(b);adj[b].add(a)
    seen=set();components=[]
    for a in adj:
        if a in seen:continue
        todo=[a];comp=[];seen.add(a)
        while todo:
            p=todo.pop();comp.append(p)
            for q in adj[p]:
                if q not in seen:seen.add(q);todo.append(q)
        ps=np.asarray(comp); dims=[i for i in range(3) if i!=k];ps2=ps[:,dims]
        # least-squares circle fit
        A=np.column_stack([2*ps2[:,0],2*ps2[:,1],np.ones(len(ps2))]);b=(ps2**2).sum(axis=1)
        v=np.linalg.lstsq(A,b,rcond=None)[0];r=np.linalg.norm(ps2-v[:2],axis=1)
        components.append(dict(n=len(comp),bounds=[ps.min(0).tolist(),ps.max(0).tolist()],circle_center=v[:2].tolist(),radius=float(r.mean()),error=float(r.std())))
    return sorted(components,key=lambda c:c['n'],reverse=True)

if __name__=='__main__':
    for stem,z in [('joint_top',-130),('joint_bottom',-30)]:
        print(stem,json.dumps(loops(read(ROOT/'meshes/Joints'/f'{stem}.stl'),2,z),indent=2))
    cs=split(read(ROOT/'meshes/Thigh Design.stl'))
    for idx,z in [(5,288),(5,294),(5,300),(6,-30.38367),(6,-24.38367)]:
        print('thigh',idx,z,json.dumps(loops(cs[idx],2,z),indent=2))
