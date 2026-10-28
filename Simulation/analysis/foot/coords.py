import json, collections
for name in ['foot_top','foot_bottom','foot_ground']:
 vs=json.load(open('analysis/foot/'+name+'.json'))['vertices']; print('\n',name)
 for a in range(3):
  c=collections.Counter(round(v[a],3) for v in vs); print('axis',a,sorted((v,n) for v,n in c.items() if n>=10))
