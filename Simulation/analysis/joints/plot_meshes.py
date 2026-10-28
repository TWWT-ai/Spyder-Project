from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).parent))
from inspect_meshes import read,split,ROOT
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np

def plot(ts,name):
    ts=np.array(ts);fig,axs=plt.subplots(1,3,figsize=(18,6))
    for ax,(a,b) in zip(axs,[(0,1),(0,2),(1,2)]):
        # render edges in orthographic projections
        lines=np.concatenate([ts[:,[0,1]],ts[:,[1,2]],ts[:,[2,0]]])[:,:,[a,b]]
        ax.add_collection(LineCollection(lines,colors='#173d62',linewidths=.25,alpha=.6))
        ax.autoscale(); ax.set_aspect('equal');ax.grid();ax.set_xlabel('xyz'[a]);ax.set_ylabel('xyz'[b])
    fig.suptitle(name);fig.tight_layout();fig.savefig(Path(__file__).parent/(name+'.png'),dpi=120);plt.close(fig)

for p in (ROOT/'meshes/Joints').glob('*.stl'):
    if '2' not in p.stem:plot(read(p),p.stem)
cs=split(read(ROOT/'meshes/Thigh Design.stl'))
for i in [0,5,6]:plot(cs[i],'thigh_'+str(i))
