import struct, pathlib, collections, json
for p in pathlib.Path('meshes').glob('foot*.stl'):
 b=p.read_bytes(); n=struct.unpack_from('<I',b,80)[0]; faces=[struct.unpack_from('<12fH',b,84+i*50)[3:12] for i in range(n)]; vs=sorted(set(tuple(f[j:j+3]) for f in faces for j in (0,3,6)))
 print('\n',p,n,len(vs));print('bounds',[[min(v[a] for v in vs),max(v[a] for v in vs)] for a in range(3)])
 for a in range(3): print('axis',a,collections.Counter(round(v[a],4) for v in vs).most_common(20))
 pathlib.Path('analysis/foot/'+p.stem+'.json').write_text(json.dumps({'vertices':vs,'faces':faces}))
