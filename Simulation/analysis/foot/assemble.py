import trimesh,numpy as np,json,pathlib
bot=trimesh.load('meshes/foot_bottom.stl');ground=trimesh.load('meshes/foot_ground.stl');tip=trimesh.load('meshes/foot_tip.stl');top=trimesh.load('meshes/foot_top.stl')
shaft=123.776415;cy=(130.43222045898438+184.5653839111328)/2;cz=(39.49591064453125+46.13990020751953)/2
sx=5.4/(114.82408905029297-109.37239074707031);tx=221.10922241210938-sx*109.37239074707031-shaft
notch=(180.45404052734375-134.54356384277344)/2
parts={
'foot_bottom':{'rotation':[[1,0,0],[0,0,-1],[0,1,0]],'scale':[1,1,1],'translation':[-shaft,-notch-15,0]},
'foot_top':{'rotation':[[1,0,0],[0,0,1],[0,-1,0]],'scale':[1,1,1],'translation':[406.71409606933594-shaft,notch-39,57.75117492675781]},
'foot_ground':{'rotation':[[1,0,0],[0,1,0],[0,0,1]],'scale':[sx,1,1],'translation':[tx,-cy,-cz]},
'foot_tip':{'rotation':[[0,0,-1],[0,1,0],[1,0,0]],'scale':[1,1,1],'translation':[sx*167.5237579345703+tx+36.499996185302734,-53.71949005126953,-78.70100784301758]}}
for name,p in parts.items():
 m=trimesh.load('meshes/'+name+'.stl');r=np.array(p['rotation']);m.vertices=(m.vertices*np.array(p['scale']))@r.T+p['translation'];m.export('analysis/foot/'+name+'_assembled.stl');print(name,m.bounds)
result={'units':'mm','formula':'assembled_vertex = R @ (scale * source_vertex) + translation','frame':'origin approximately bottom shaft centre, X outward, Y shaft axis, Z transverse to plate; shaft hole mismatch 0.13mm','parts':parts,'contact_point':[sx*167.5237579345703+tx+12,0,0],'inside_plate_gap':2*notch,'ground_scale_note':'X fitted to5.4mm distal sideplate bar; native ground dimension5.451698mm,1% correction. Y/Z preserved. Slots remain loose due independently resized parts.'}
pathlib.Path('analysis/foot/assembly_transforms.json').write_text(json.dumps(result,indent=2));print(result)
