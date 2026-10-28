"""Record the measured assembly from the independent fitting studies."""
from pathlib import Path
import json
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
joint = json.loads((ROOT/'analysis/joints/assembly_transforms.json').read_text())
foot = json.loads((ROOT/'analysis/foot/assembly_transforms.json').read_text())
parts = []
teal = [.08,.55,.58,1]
blue = [.18,.42,.72,1]
orange = [.92,.48,.12,1]
dark = [.19,.22,.26,1]
for p in joint['parts']:
    name = p['name']
    part = {k:p[k] for k in ('name','source','rotation','scale','translation_mm')}
    part['link'] = 'base_link' if name.startswith('thigh_') else 'coxa_link'
    part['color'] = dark if name=='thigh_motor' else (teal if name.startswith('thigh_') else blue)
    part['fit_note'] = p['notes']
    if name=='thigh_motor': part['source_region']=[[-10000,-10000,-10000],[50,10000,10000]]
    elif name=='thigh_bottom': part['source_region']=[[-10000,0,-10000],[10000,10000,10000]]
    elif name=='thigh_top': part['source_region']=[[-10000,-10000,-10000],[10000,-800,10000]]
    parts.append(part)

left_axis=754.49007026
right_axis=831.06631747
length=right_axis-left_axis
center=(left_axis+right_axis)/2
shell_r=[[1,0,0],[0,0,1],[0,-1,0]]
sx=(821.37051-764.18583)/(816.50897-758.75293)
sy=28.0/(198.73806-170.74926)
for name,r,s,t,note in [
    ('leg_bottom',shell_r,[1,1,1],[-left_axis,6.6360012,1216.97437],
     'Leg design3 reference: cradle at motor BACK. Inner floor source Z-22.0360012 maps to axial-15.4mm, supporting motor backs. Passive-pivot slots align with output axes in X/Z.'),
    ('leg_top',shell_r,[sx,sy,1],[center-left_axis-sx*787.63095,-6.900044,sy*184.74366],
     'Leg design3 reference: top frame at OUTPUT end, source underside Z21.500044 maps to flange top axial14.6mm. Four hole axes align to lower cradle; long fasteners span22mm between posts and lid.'),
    ('Motor',np.diag([-1,1,-1]).tolist(),[1,1,1],[-5.422895,-170.9,66.669016],
     'Proximal servo output shaft points+Y and is held by coxa bracket; servo casing moves with femur. The imported spline detail remains a fixed visual part of casing.'),
    ('Motor2',np.eye(3).tolist(),[1,1,1],[length+5.422895,-170.9,-66.669016],
     'Distal servo casing belongs to femur; output drives tibia. Both motors retain original size. Approximately1mm mismatch remains between case centering and shell mounting openings.')
]:
    parts.append(dict(name=name,source=f'meshes/Leg/{name}.stl',rotation=r,scale=s,translation_mm=t,
                      link='femur_link',color=dark if name.startswith('Motor') else teal,fit_note=note))

flip=np.diag([1,-1,-1])
# Clear the corrected lower cradle, rather than letting the foot's rear plate
# intersect its 4mm floor. The four foot pieces retain their relative transforms.
cradle_back=-19.4
foot_axial_shift=cradle_back-.2+foot['inside_plate_gap']/2
for name,p in foot['parts'].items():
    parts.append(dict(name=name,source=f'meshes/{name}.stl',
                      rotation=(flip@np.array(p['rotation'])).tolist(),scale=p['scale'],
                      translation_mm=(flip@np.array(p['translation'])+[0,foot_axial_shift,0]).tolist(),
                      link='tibia_link',color=dark if name=='foot_tip' else orange,
                      fit_note='Measured foot assembly; see analysis/foot/assembly_transforms.json for mating evidence.'))

def link(mass,com,size,boxes):
    return dict(mass_kg=mass,com_mm=com,inertia_box_mm=size,
                collision_boxes=[dict(center_mm=c,size_mm=s) for c,s in boxes])

config=dict(
    units='millimetres for source and fitting coordinates; kilograms for mass',
    transform='link_vertex_m=(rotation @ (scale * source_vertex_mm) + translation_mm)/1000',
    assumptions=[
        'Assembly follows user-supplied Foot Design, Leg design3 and Thigh Design reference thumbnails. Leg Designs1/2 are excluded. Original source STLs are preserved.',
        'User identified servo as probably MG996R or similar. Original servo geometry is retained; model identity remains unconfirmed.',
        'Joint hardware scaled0.1 physically; top joint X additionally scaled0.974359. These are fitting corrections, separate from mm-to-m conversion.',
        'Leg cover X/Y fitted to lower-shell screw holes; approximately1mm case/opening mismatch remains from source distortions.',
        'Foot ground X scaled0.990517; foot opposed shaft holes retain0.132810mm X mismatch when distal tabs align.',
        'Leg and thigh use a rear cradle and output-side top frame, connected by long fasteners as shown in the reference. Six smooth-shank fasteners are illustrative; exact screw specifications were not supplied.',
        'Previous rear adapter and short spacers were removed because the reference does not show them. The blue coupling still has5.4mm rear-pivot clearance; its actual bearing/fastener arrangement is unresolved.',
        'Entire foot moved axially to give0.2mm nominal clearance to leg cradle rear face; no internal foot-piece transforms changed.',
        'All masses, inertias, torque settings, joint limits and collision boxes are illustrative. Default mode is kinematic and does not validate collisions, load capacity or walking.',
        'Detailed servo spline geometry stays in the casing visual; separate horns/fasteners are absent. Joint axes, not spline teeth, define motion.'
    ],
    links={
        'base_link':link(.10,[0,0,0],[60,35,55],[([-10,0,5],[40,20,39])]),
        'coxa_link':link(.045,[21.7,0,0],[50,66,66],[([21.7,0,29],[6,42,5]),([21.7,0,-29],[6,42,5])]),
        'femur_link':link(.145,[length/2,0,0],[113,48,35],[([10.75,4,0],[40,38,20]),([length-10.75,4,0],[40,38,20])]),
        'tibia_link':link(.045,[85,foot_axial_shift,0],[182,58,32],[([85,foot_axial_shift,0],[150,6,6]),([164,foot_axial_shift,0],[6,12,16])])
    },
    joints=[
        dict(name='coxa_yaw',parent='base_link',child='coxa_link',origin_mm=[0,0,0],axis=[0,0,1],limits_deg=[-60,60]),
        dict(name='femur_pitch',parent='coxa_link',child='femur_link',origin_mm=joint['pitch_axis_point_mm'],axis=[0,1,0],limits_deg=[-50,70]),
        dict(name='tibia_pitch',parent='femur_link',child='tibia_link',origin_mm=[length,0,0],axis=[0,1,0],limits_deg=[-70,85])
    ],
    foot_contact_mm=[foot['contact_point'][0],foot_axial_shift,0],
    parts=parts)

# The supplied assembled reference supersedes the earlier short-clamp inference.
thigh_reference=json.loads((ROOT/'analysis/reference_fit/thigh_reference_transforms.json').read_text())
for p in parts:
    if p['name']=='thigh_bottom':
        p.update(thigh_reference['parts'][p['name']])
        p['fit_note']='Thigh Design reference: upright cradle supports servo back on source floor Z294. Long fasteners link lower posts to output-side top frame.'
    if p['name']=='thigh_top':
        p.update(thigh_reference['parts'][p['name']])
        p['fit_note']='Thigh Design reference: output-side top frame underside at Z14.6, above flange. Hole pitch corrected28.028→28mm to align with cradle; long fasteners span24mm above lower posts.'
    if p['name']=='leg_top': p['color']=[.12,.78,.83,1]
    if p['name']=='thigh_top': p['color']=[.12,.78,.83,1]
    if p['name'] in ('leg_bottom','thigh_bottom'): p['color']=[.05,.43,.46,1]

config['generated_supports']=[]
def fastener(name,link,axis,center,start,end):
    axis=np.asarray(axis)
    center=np.asarray(center)
    config['generated_supports'].append(dict(
        name=name,link=link,color=[.48,.51,.55,1],
        note='Simplified long through-fastener shown in assembled reference. Hole axis is measured;3mm smooth shank and hex end sizes are illustrative. Threads are not modelled.',
        shapes=[dict(type='cylinder',axis=axis.tolist(),radius_mm=1.5,
                     height_mm=end-start,center_mm=(center+axis*(start+end)/2).tolist()),
                dict(type='cylinder',axis=axis.tolist(),radius_mm=3.2,sections=6,
                     height_mm=3,center_mm=(center+axis*(end-1.5)).tolist()),
                dict(type='cylinder',axis=axis.tolist(),radius_mm=3.2,sections=6,
                     height_mm=2.4,center_mm=(center+axis*(start+1.2)).tolist())]))

for index,y in enumerate([-14,14]):
    fastener(f'thigh_through_bolt_{index+1}','base_link',[0,0,1],[-9.8266,y,0],-21.4,23.6)
index=0
for x in [764.18583-left_axis,821.37051-left_axis]:
    for z in [-14,14]:
        index+=1
        fastener(f'leg_through_bolt_{index}','femur_link',[0,1,0],[x,0,z],-19.4,23.6)
(ROOT/'assembly.json').write_text(json.dumps(config,indent=2)+'\n',encoding='utf-8')
print('Lc',joint['pitch_axis_point_mm'][0],'Lf',length,'Lt',foot['contact_point'][0],'foot axial offset',foot_axial_shift)
