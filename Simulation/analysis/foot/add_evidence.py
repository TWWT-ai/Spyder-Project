import json,pathlib,numpy as np,trimesh
p=pathlib.Path('analysis/foot/assembly_transforms.json');d=json.loads(p.read_text())
d['shaft_centres_source_mm']={'foot_bottom':[123.776415,0,-12],'foot_top':[-282.80487060546875,57.75117492675781,42]}
d['shaft_axes_source']={'foot_bottom':[0,0,1],'foot_top':[0,0,1]}
d['shaft_centres_assembled_mm']={}
for name,a in d['shaft_centres_source_mm'].items():
 t=d['parts'][name];d['shaft_centres_assembled_mm'][name]=(np.array(t['rotation'])@np.array(a)+np.array(t['translation'])).tolist()
d['mating_evidence']=[
'Top and bottom distal forks overlay with top XY translation(+406.714096,-57.751175)mm; proximal shaft centres then differ0.132810mm in X. Cannot align both shafts and both distal forks exactly by rigid translation.',
'Ground distal crossbar spans source X109.372391..114.824089mm. Sideplate bar spans source X221.109222..226.509222mm. Fitted ground Xscale0.990517 aligns these widths and proximal bar near165.309..172.509mm simultaneously.',
'Ground notch floors source Y134.543564 and180.454041mm define sideplate INNER faces,45.910477mm apart. Both sideplates6mm thick; midplane separation51.910477mm.',
'Ground plane thickness6.643990mm fits side plate central openings of8mm with0.678005mm clearance on either side.',
'Tip fingers point back toward knee. Tip inner finger gap in source X is about6.798mm, compatible with ground plane6.644mm. Ground distal fork5.4mm deep fits6mm long fingers with0.6mm offset/clearance.',
'Foot contact point is outer centre of foot_tip planar pad, not its centre of mass.']
d['assumptions']=['Dimensions interpreted as millimetres as exported by user.','Ground X scale adjustment is a virtual fitting correction; source STLs are unchanged.','This is a mechanically plausible inferred assembly, not a manufacturing clearance certification.','No separate passive ankle joint: all four pieces are one rigid tibia link.']
p.write_text(json.dumps(d,indent=2))
