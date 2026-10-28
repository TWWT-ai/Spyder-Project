"""Measured / inferred rigid fitting study; output units are millimetres."""
from pathlib import Path
import sys,json
sys.path.insert(0,str(Path(__file__).parent))
from inspect_meshes import ROOT,read
import numpy as np
import trimesh

OUT=Path(__file__).parent
parts=[]

def add(name,source,triangles,rotation,scale,anchor,target,notes):
    r=np.array(rotation,float);s=np.broadcast_to(scale,3)
    matrix=r@np.diag(s);translation=np.array(target)-matrix@np.array(anchor)
    mesh=trimesh.Trimesh(vertices=np.asarray(triangles).reshape(-1,3),faces=np.arange(len(triangles)*3).reshape(-1,3),process=False)
    mesh.export(OUT/(name+'_source.stl'))
    mesh.vertices=mesh.vertices@matrix.T+translation
    mesh.export(OUT/(name+'_assembled.stl'))
    parts.append(dict(name=name,source=source,source_split_stl=str((OUT/(name+'_source.stl')).relative_to(ROOT)),rotation=r.tolist(),scale=s.tolist(),anchor_source_mm=anchor,anchor_assembled_mm=target,linear=matrix.tolist(),translation_mm=translation.tolist(),notes=notes))
    return mesh

I=np.eye(3)
bottom_anchor=[1583.946533,-1170.746885,-30.0]
top_anchor=[983.586414,-1189.856956,-130.0]
shaft_distance=(1801.002559-1583.946533)*.1
top_sx=shaft_distance/(1206.354416-983.586414)
ring_center=[shaft_distance,0,0]
meshes=[]
meshes.append(add('joint_cross','meshes/Joints/joint_cross.stl',read(ROOT/'meshes/Joints/joint_cross.stl'),I,.1,[1582.296,-240.007,225.699],ring_center,'Scale 0.1 inferred from matching 5.933mm ring slab, plate fork slots, 4.8mm recess depth; not a unit conversion.'))
for name,source,r,anchor,target,scale in [
    ('yaw_bottom','joint_bottom.stl',[[-1,0,0],[0,-1,0],[0,0,1]],bottom_anchor,[shaft_distance,0,26.8],.1),
    ('yaw_top','joint_top.stl',[[-1,0,0],[0,1,0],[0,0,-1]],top_anchor,[shaft_distance,0,-26.8],[top_sx,.1,.1]),
    ('pitch_bottom','joint_bottom2.stl',[[1,0,0],[0,0,1],[0,-1,0]],bottom_anchor,[shaft_distance,26.8,0],.1),
    ('pitch_top','joint_top2.stl',[[1,0,0],[0,0,-1],[0,1,0]],top_anchor,[shaft_distance,-26.8,0],[top_sx,.1,.1]),
]:
    meshes.append(add(name,'meshes/Joints/'+source,read(ROOT/'meshes/Joints'/source),r,scale,anchor,target,'Four plates all rigid with ring. Top X scale additionally corrected 2.564% so hex bore and opposing servo bore share an axis; slot fit retains approximately 0.15mm clearance.'))

ts=np.asarray(read(ROOT/'meshes/Thigh Design.stl'));centers=ts.mean(1)
servo=ts[centers[:,0]<50]
base=ts[centers[:,1]>0]
cover=ts[centers[:,1]<-800]
assert len(servo)+len(base)+len(cover)==len(ts)
motor_axis=[-5.422894,-333.241791,459.266418]
meshes.append(add('thigh_motor','meshes/Thigh Design.stl',servo,I,1,motor_axis,[0,0,26.8],'Servo cluster includes screws/spline details. Axial output mounting shoulder z459.2664; spline tip is +4mm. Output-axis location is measured; shoulder-plane contact placement inferred.'))
meshes.append(add('thigh_bottom','meshes/Thigh Design.stl',base,I,1,[343.778587,272.5,294],[0,0,-15.4],'Motor base sits on 294mm bracket floor. Its access bore approximately matches the servo axis; raw bracket anisotropic dimensions differ from servo by about 0.9mm along X.'))
meshes.append(add('thigh_top','meshes/Thigh Design.stl',cover,I,1,[139.692504,-913.334956,-30.38365],[-9.826587,0,14.6],'Frame sits on servo mounting-flange upper surface. Aligns matching bracket screw holes within 0.03mm. Leaves 4mm gap above bottom-bracket uprights: actual spacer/fastener arrangement is absent and cannot be uniquely inferred.'))

manifest=dict(units='mm',convention='assembled = linear @ source_vertex + translation_mm; rotation is proper orthonormal; scale is separate from unit conversion',frame='yaw axis midpoint at (0,0,0); pitch axis is Y through (43.4112052,0,0); ring normal X',yaw_axis=[0,0,1],pitch_axis=[0,1,0],pitch_axis_point_mm=[2*shaft_distance,0,0],parts=parts,limitations=['No hardware, bearing or servo horn STL supplied. Small mounting gaps need real hardware validation.','Uniform 0.1 correction for joint meshes and 0.974359 additional top-X correction are inferred CAD repairs, not verified real dimensions.','Thigh upper cover requires 4mm stand-off from lower uprights to avoid intersecting motor flanges; the source parts do not form an exact rigid snap-fit.','Lower yaw bearing inner face is 5.4mm below thigh bottom bracket; missing bearing/spacer arrangement remains unverified.'])
(OUT/'assembly_transforms.json').write_text(json.dumps(manifest,indent=2))
trimesh.util.concatenate(meshes).export(OUT/'joint_thigh_assembly.stl')
print('wrote assembly_transforms.json; pitch offset',2*shaft_distance,'top x scale',top_sx)
