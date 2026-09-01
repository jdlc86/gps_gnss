import csv, math, statistics, json
from collections import Counter,defaultdict
P='gnss_log.txt'
counts=Counter(); fixes=[]; raws=[]; imu=defaultdict(list); status=[]
with open(P,encoding='utf-8') as f:
 for line in f:
  if not line or line[0]=='#': continue
  r=next(csv.reader([line])); typ=r[0]; counts[typ]+=1
  try:
   if typ=='Fix' and len(r)>=14:
    fixes.append(dict(lat=float(r[2]),lon=float(r[3]),alt=float(r[4]),speed=float(r[5]),acc=float(r[6]),bearing=None if r[7]=='' else float(r[7]),t=int(r[8]),sacc=None if r[9]=='' else float(r[9]),bacc=None if r[10]=='' else float(r[10]),ert=int(r[11]),vacc=None if r[12]=='' else float(r[12])))
   elif typ=='Raw' and len(r)>=37:
    raws.append(dict(t=int(r[1]),svid=int(r[11]),state=int(r[13]),cn0=float(r[16]),prr=float(r[17]),prru=float(r[18]),adrstate=int(r[19]),adr=float(r[20]),adru=float(r[21]),freq=float(r[22]) if r[22] else None,const=int(r[28]),agc=float(r[29]) if r[29] else None,bb=float(r[30]) if r[30] else None,code=r[35] if len(r)>35 else ''))
   elif typ=='Status' and len(r)>=14:
    status.append(dict(const=int(r[4]),svid=int(r[5]),freq=float(r[6]) if r[6] else None,cn0=float(r[7]),used=int(r[10])))
   elif typ in ('UncalAccel','UncalGyro','UncalMag'):
    imu[typ].append(tuple(float(x) for x in r[3:6]))
  except (ValueError,IndexError): pass

def pct(a,p):
 a=sorted(a); return a[min(len(a)-1,max(0,round((len(a)-1)*p)))] if a else None
def mean(a): return statistics.fmean(a) if a else None
def sd(a): return statistics.pstdev(a) if len(a)>1 else 0
out={'counts':dict(counts)}
if fixes:
 lat0=mean([x['lat'] for x in fixes]); lon0=mean([x['lon'] for x in fixes]); R=6378137.; cl=math.cos(math.radians(lat0))
 east=[math.radians(x['lon']-lon0)*R*cl for x in fixes]; north=[math.radians(x['lat']-lat0)*R for x in fixes]; rad=[math.hypot(e,n) for e,n in zip(east,north)]
 out['fix']={'n':len(fixes),'duration_s':(fixes[-1]['t']-fixes[0]['t'])/1000,'mean_lat':lat0,'mean_lon':lon0,'mean_accuracy_m':mean([x['acc'] for x in fixes]),'accuracy_p95_m':pct([x['acc'] for x in fixes],.95),'east_sd_m':sd(east),'north_sd_m':sd(north),'radial_p50_m':pct(rad,.5),'radial_p95_m':pct(rad,.95),'radial_max_m':max(rad),'speed_mean_mps':mean([x['speed'] for x in fixes]),'speed_p95_mps':pct([x['speed'] for x in fixes],.95),'stationary_fraction_speed_lt_0_1':sum(x['speed']<.1 for x in fixes)/len(fixes)}
if raws:
 bands=Counter(); cons=Counter(); cn=[]; prru=[]; adru=[]; adrvalid=0
 for x in raws:
  cons[x['const']]+=1; cn.append(x['cn0']); prru.append(x['prru']); adru.append(x['adru']); adrvalid += int((x['adrstate'] & 1)!=0)
  f=x['freq'] or 0
  bands['L1/E1/B1-like' if 1.50e9<=f<=1.61e9 else 'L5/E5/B2-like' if 1.15e9<=f<=1.22e9 else 'other']+=1
 out['raw']={'n':len(raws),'constellation_counts':dict(cons),'band_counts':dict(bands),'cn0_mean_dbhz':mean(cn),'cn0_p50_dbhz':pct(cn,.5),'cn0_p95_dbhz':pct(cn,.95),'prr_uncertainty_p50_mps':pct(prru,.5),'prr_uncertainty_p95_mps':pct(prru,.95),'adr_valid_fraction':adrvalid/len(raws),'adr_uncertainty_p50_m':pct(adru,.5),'adr_uncertainty_p95_m':pct(adru,.95),'unique_satellites':len(set((x['const'],x['svid']) for x in raws))}
if status:
 out['status']={'n':len(status),'used_fraction':sum(x['used'] for x in status)/len(status),'unique_satellites':len(set((x['const'],x['svid']) for x in status))}
for k,v in imu.items():
 if v:
  out[k]={'n':len(v),'mean':[mean([x[i] for x in v]) for i in range(3)],'sd':[sd([x[i] for x in v]) for i in range(3)]}
print(json.dumps(out,indent=2))
open('analysis_report.json','w').write(json.dumps(out,indent=2))
