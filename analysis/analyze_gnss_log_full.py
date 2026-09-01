#!/usr/bin/env python3
import csv, json, math, statistics, sys
from collections import Counter, defaultdict
from pathlib import Path

LOG = Path(sys.argv[1] if len(sys.argv) > 1 else 'gnss_log.txt')
OUT_JSON = Path(sys.argv[2] if len(sys.argv) > 2 else 'analysis/gnss_analysis.json')
OUT_MD = OUT_JSON.with_suffix('.md')

CONST = {0:'UNKNOWN',1:'GPS',2:'SBAS',3:'GLONASS',4:'QZSS',5:'BEIDOU',6:'GALILEO',7:'IRNSS'}

def f(x):
    try:
        return float(x) if x not in ('', None, 'null', 'NaN') else None
    except Exception:
        return None

def i(x):
    try:
        return int(float(x)) if x not in ('', None, 'null', 'NaN') else None
    except Exception:
        return None

def q(vals, p):
    vals = sorted(v for v in vals if v is not None and math.isfinite(v))
    if not vals: return None
    if len(vals)==1: return vals[0]
    x=(len(vals)-1)*p
    lo=int(math.floor(x)); hi=int(math.ceil(x))
    if lo==hi: return vals[lo]
    return vals[lo]*(hi-x)+vals[hi]*(x-lo)

def stats(vals):
    vals=[v for v in vals if v is not None and math.isfinite(v)]
    if not vals: return {'n':0}
    return {
        'n':len(vals),'mean':statistics.fmean(vals),'median':q(vals,.5),
        'std':statistics.stdev(vals) if len(vals)>1 else 0.0,
        'p05':q(vals,.05),'p50':q(vals,.5),'p95':q(vals,.95),'p99':q(vals,.99),
        'min':min(vals),'max':max(vals)
    }

def band(freq):
    if freq is None: return 'UNKNOWN'
    if 1.50e9 <= freq <= 1.62e9: return 'L1/E1/B1-like'
    if 1.15e9 <= freq <= 1.23e9: return 'L5/E5/B2-like'
    return 'OTHER'

def circ_stats_deg(vals):
    vals=[v for v in vals if v is not None]
    if not vals: return {'n':0}
    rads=[math.radians(v) for v in vals]
    s=statistics.fmean(math.sin(x) for x in rads); c=statistics.fmean(math.cos(x) for x in rads)
    mean=(math.degrees(math.atan2(s,c))+360)%360
    R=math.hypot(s,c)
    std=math.degrees(math.sqrt(max(0,-2*math.log(max(R,1e-15)))))
    return {'n':len(vals),'circular_mean_deg':mean,'circular_std_deg':std,'resultant_R':R}

headers={}
counts=Counter(); fixes=[]; raws=[]
acc=[]; gyro=[]; mag=[]; orient=[]; status=[]
all_times=[]
with LOG.open('r', encoding='utf-8', errors='replace', newline='') as fh:
    for line in fh:
        line=line.strip()
        if not line: continue
        if line.startswith('# '):
            try:
                parts=next(csv.reader([line[2:]]))
                if len(parts)>1: headers[parts[0]]=parts[1:]
            except Exception: pass
            continue
        if line.startswith('#'): continue
        try: row=next(csv.reader([line]))
        except Exception: continue
        if not row: continue
        typ=row[0]; counts[typ]+=1
        h=headers.get(typ)
        if not h: continue
        vals=row[1:]
        if len(vals)<len(h): vals += ['']*(len(h)-len(vals))
        d=dict(zip(h,vals))
        # collect a usable UTC timestamp where available
        for key in ('utcTimeMillis','UnixTimeMillis'):
            t=i(d.get(key))
            if t is not None: all_times.append(t); break
        if typ=='Fix': fixes.append(d)
        elif typ=='Raw': raws.append(d)
        elif typ in ('UncalAccel','Accel'): acc.append((typ,d))
        elif typ in ('UncalGyro','Gyro'): gyro.append((typ,d))
        elif typ in ('UncalMag','Mag'): mag.append((typ,d))
        elif typ=='OrientationDeg': orient.append(d)
        elif typ=='Status': status.append(d)

result={'file':str(LOG),'row_counts':dict(counts)}
if all_times:
    result['recording']={'start_ms':min(all_times),'end_ms':max(all_times),'duration_s':(max(all_times)-min(all_times))/1000}

# FIX / static repeatability
lat=[f(d.get('LatitudeDegrees')) for d in fixes]; lon=[f(d.get('LongitudeDegrees')) for d in fixes]
valid=[(a,b,d) for a,b,d in zip(lat,lon,fixes) if a is not None and b is not None]
if valid:
    lat0=q([x[0] for x in valid],.5); lon0=q([x[1] for x in valid],.5)
    R=6378137.0; cl=math.cos(math.radians(lat0))
    E=[]; N=[]; radial=[]
    for la,lo,_ in valid:
        e=R*math.radians(lo-lon0)*cl; n=R*math.radians(la-lat0)
        E.append(e); N.append(n); radial.append(math.hypot(e,n))
    times=[i(d.get('UnixTimeMillis')) for _,_,d in valid]
    ordered=[x for x in zip(times,valid) if x[0] is not None]
    ordered.sort(key=lambda z:z[0])
    drift=None
    if len(ordered)>=4:
        half=len(ordered)//2
        def center(part):
            las=[x[1][0] for x in part]; los=[x[1][1] for x in part]
            return q(las,.5),q(los,.5)
        a1,o1=center(ordered[:half]); a2,o2=center(ordered[half:])
        de=R*math.radians(o2-o1)*cl; dn=R*math.radians(a2-a1)
        drift={'east_m':de,'north_m':dn,'distance_m':math.hypot(de,dn)}
    accuracy=[f(d.get('AccuracyMeters')) for _,_,d in valid]
    speed=[f(d.get('SpeedMps')) for _,_,d in valid]
    result['fix']={
        'count':len(valid),'median_lat_deg':lat0,'median_lon_deg':lon0,
        'east_m':stats(E),'north_m':stats(N),'radial_from_median_m':stats(radial),
        'android_accuracy_m':stats(accuracy),'speed_mps':stats(speed),
        'speed_fraction_gt_0_1':sum(v is not None and v>0.1 for v in speed)/len(speed),
        'speed_fraction_gt_0_5':sum(v is not None and v>0.5 for v in speed)/len(speed),
        'first_half_to_second_half_median_drift':drift,
        'providers':dict(Counter(d.get('Provider','') for _,_,d in valid))
    }

# RAW GNSS
if raws:
    constellation=Counter(); sig_counts=Counter(); freq_counts=Counter(); sv_by_const=defaultdict(set)
    cn0_by_band=defaultdict(list); cn0_by_const=defaultdict(list); bb_by_band=defaultdict(list)
    pr_unc=[]; adr_unc=[]; adr_states=Counter(); code_types=Counter(); epochs=set(); raw_t=[]
    for d in raws:
        c=i(d.get('ConstellationType')); name=CONST.get(c,str(c)); constellation[name]+=1
        s=i(d.get('Svid')); 
        if s is not None: sv_by_const[name].add(s)
        fr=f(d.get('CarrierFrequencyHz')); b=band(fr); sig_counts[b]+=1
        if fr is not None: freq_counts[round(fr/1e6,3)]+=1
        cn=f(d.get('Cn0DbHz')); 
        if cn is not None: cn0_by_band[b].append(cn); cn0_by_const[name].append(cn)
        bb=f(d.get('BasebandCn0DbHz'))
        if bb is not None: bb_by_band[b].append(bb)
        pu=f(d.get('PseudorangeRateUncertaintyMetersPerSecond')); 
        if pu is not None: pr_unc.append(pu)
        au=f(d.get('AccumulatedDeltaRangeUncertaintyMeters')); 
        if au is not None: adr_unc.append(au)
        st=i(d.get('AccumulatedDeltaRangeState')); 
        if st is not None: adr_states[st]+=1
        ct=d.get('CodeType');
        if ct: code_types[ct]+=1
        ep=i(d.get('utcTimeMillis')); 
        if ep is not None: epochs.add(ep); raw_t.append(ep)
    n=len(raws)
    valid=sum(cnt for st,cnt in adr_states.items() if st & 1)
    reset=sum(cnt for st,cnt in adr_states.items() if st & 2)
    slip=sum(cnt for st,cnt in adr_states.items() if st & 4)
    result['raw']={
        'measurement_rows':n,'epochs':len(epochs),
        'epoch_rate_hz': (len(epochs)-1)/((max(raw_t)-min(raw_t))/1000) if len(epochs)>1 and max(raw_t)>min(raw_t) else None,
        'measurements_by_constellation':dict(constellation),
        'unique_svids_by_constellation':{k:len(v) for k,v in sv_by_const.items()},
        'measurements_by_band':dict(sig_counts),
        'carrier_frequency_mhz_counts':{str(k):v for k,v in sorted(freq_counts.items())},
        'cn0_dbhz_by_band':{k:stats(v) for k,v in cn0_by_band.items()},
        'cn0_dbhz_by_constellation':{k:stats(v) for k,v in cn0_by_const.items()},
        'baseband_cn0_dbhz_by_band':{k:stats(v) for k,v in bb_by_band.items()},
        'pseudorange_rate_uncertainty_mps':stats(pr_unc),
        'adr_uncertainty_m':stats(adr_unc),
        'adr_state_counts':{str(k):v for k,v in adr_states.items()},
        'adr_valid_fraction':valid/n if n else None,
        'adr_reset_fraction':reset/n if n else None,
        'adr_cycle_slip_fraction':slip/n if n else None,
        'code_types':dict(code_types)
    }

# IMU static statistics
if acc:
    preferred=[d for typ,d in acc if typ=='UncalAccel'] or [d for _,d in acc]
    def av(d, axis): return f(d.get(f'UncalAccel{axis}Mps2') or d.get(f'Accel{axis}Mps2'))
    xs=[av(d,'X') for d in preferred]; ys=[av(d,'Y') for d in preferred]; zs=[av(d,'Z') for d in preferred]
    norms=[math.sqrt(x*x+y*y+z*z) for x,y,z in zip(xs,ys,zs) if None not in (x,y,z)]
    result['accelerometer']={'count':len(preferred),'x_mps2':stats(xs),'y_mps2':stats(ys),'z_mps2':stats(zs),'norm_mps2':stats(norms)}
if gyro:
    preferred=[d for typ,d in gyro if typ=='UncalGyro'] or [d for _,d in gyro]
    def gv(d,axis): return f(d.get(f'UncalGyro{axis}RadPerSec') or d.get(f'Gyro{axis}RadPerSec'))
    xs=[gv(d,'X') for d in preferred]; ys=[gv(d,'Y') for d in preferred]; zs=[gv(d,'Z') for d in preferred]
    dx=[f(d.get('DriftXRadPerSec')) for d in preferred]; dy=[f(d.get('DriftYRadPerSec')) for d in preferred]; dz=[f(d.get('DriftZRadPerSec')) for d in preferred]
    result['gyroscope']={'count':len(preferred),'x_radps':stats(xs),'y_radps':stats(ys),'z_radps':stats(zs),'reported_bias_x_radps':stats(dx),'reported_bias_y_radps':stats(dy),'reported_bias_z_radps':stats(dz)}
if mag:
    preferred=[d for typ,d in mag if typ=='UncalMag'] or [d for _,d in mag]
    def mv(d,axis): return f(d.get(f'UncalMag{axis}MicroT') or d.get(f'Mag{axis}MicroT'))
    xs=[mv(d,'X') for d in preferred]; ys=[mv(d,'Y') for d in preferred]; zs=[mv(d,'Z') for d in preferred]
    norms=[math.sqrt(x*x+y*y+z*z) for x,y,z in zip(xs,ys,zs) if None not in (x,y,z)]
    result['magnetometer']={'count':len(preferred),'x_uT':stats(xs),'y_uT':stats(ys),'z_uT':stats(zs),'norm_uT':stats(norms)}
if orient:
    result['orientation']={
        'count':len(orient),'yaw':circ_stats_deg([f(d.get('yawDeg')) for d in orient]),
        'roll_deg':stats([f(d.get('rollDeg')) for d in orient]),'pitch_deg':stats([f(d.get('pitchDeg')) for d in orient])
    }

OUT_JSON.parent.mkdir(parents=True,exist_ok=True)
OUT_JSON.write_text(json.dumps(result,indent=2,sort_keys=True),encoding='utf-8')

def fmt(v, nd=3):
    return 'n/a' if v is None else f'{v:.{nd}f}'
fx=result.get('fix',{}); rw=result.get('raw',{}); ac=result.get('accelerometer',{}); gy=result.get('gyroscope',{})
lines=['# GNSS static log analysis','',f"- Rows: **{sum(counts.values())}**",f"- Duration: **{fmt(result.get('recording',{}).get('duration_s'),1)} s**",'']
if fx:
    rr=fx['radial_from_median_m']; aa=fx['android_accuracy_m']; sp=fx['speed_mps']; dr=fx.get('first_half_to_second_half_median_drift') or {}
    lines += ['## Android Fix / static repeatability',
        f"- Fixes: **{fx['count']}**",
        f"- Median coordinate: **{fx['median_lat_deg']:.8f}, {fx['median_lon_deg']:.8f}**",
        f"- Radial P50 / P95 / P99: **{fmt(rr.get('p50'))} / {fmt(rr.get('p95'))} / {fmt(rr.get('p99'))} m**",
        f"- Radial max: **{fmt(rr.get('max'))} m**",
        f"- East/North std: **{fmt(fx['east_m'].get('std'))} / {fmt(fx['north_m'].get('std'))} m**",
        f"- Android reported Accuracy median / P95: **{fmt(aa.get('median'))} / {fmt(aa.get('p95'))} m**",
        f"- Speed median / P95 / max at rest: **{fmt(sp.get('median'))} / {fmt(sp.get('p95'))} / {fmt(sp.get('max'))} m/s**",
        f"- Drift median first-half -> second-half: **{fmt(dr.get('distance_m'))} m**",'']
if rw:
    lines += ['## Raw GNSS',f"- Raw measurement rows / epochs: **{rw['measurement_rows']} / {rw['epochs']}**",f"- Epoch rate: **{fmt(rw.get('epoch_rate_hz'),2)} Hz**",f"- Bands: **{rw['measurements_by_band']}**",f"- Unique SVs: **{rw['unique_svids_by_constellation']}**",f"- ADR valid/reset/cycle-slip fractions: **{fmt(rw.get('adr_valid_fraction'))} / {fmt(rw.get('adr_reset_fraction'))} / {fmt(rw.get('adr_cycle_slip_fraction'))}**",'']
    for b,s in rw.get('cn0_dbhz_by_band',{}).items(): lines.append(f"- C/N0 {b}: median **{fmt(s.get('median'))} dB-Hz**, P95 **{fmt(s.get('p95'))} dB-Hz**")
    lines.append('')
if ac:
    lines += ['## IMU static',f"- Accelerometer norm mean/std: **{fmt(ac['norm_mps2'].get('mean'))} / {fmt(ac['norm_mps2'].get('std'))} m/s²**"]
if gy:
    lines += [f"- Gyro mean XYZ: **{fmt(gy['x_radps'].get('mean'),6)}, {fmt(gy['y_radps'].get('mean'),6)}, {fmt(gy['z_radps'].get('mean'),6)} rad/s**",f"- Gyro std XYZ: **{fmt(gy['x_radps'].get('std'),6)}, {fmt(gy['y_radps'].get('std'),6)}, {fmt(gy['z_radps'].get('std'),6)} rad/s**"]
lines += ['', '> Note: radial dispersion around the median measures repeatability, not absolute accuracy. Without an independent surveyed ground-truth point, absolute position error cannot be determined.']
OUT_MD.write_text('\n'.join(lines)+'\n',encoding='utf-8')
print(OUT_MD.read_text(encoding='utf-8'))
print('\n---JSON---')
print(OUT_JSON.read_text(encoding='utf-8'))
