#!/usr/bin/env python3
"""Android position + Doppler + IMU-assisted hysteretic ZUPT.

The IMU is used only to classify stopped/moving; it is not integrated for
position yet. This isolates whether a stable stop detector improves the
position filter before attempting a full INS/EKF.
"""
from __future__ import annotations
import argparse,csv,json,math
from pathlib import Path
import numpy as np, pandas as pd
from filterpy.kalman import KalmanFilter
from run_position_doppler_kf import (GPS_UNIX_EPOCH_OFFSET_MS,GPS_UTC_LEAP_MS,read_fix,lla_to_local,ecef_velocity_to_enu,radial_stats,half_drift,process_noise_cv)

def read_imu(path):
    hdr={}; acc=[]; gyr=[]
    with Path(path).open(encoding='utf-8',errors='replace') as fh:
        for line in fh:
            line=line.strip()
            if line.startswith('# '):
                p=next(csv.reader([line[2:]]));
                if len(p)>1: hdr[p[0]]=p[1:]
                continue
            if not line or line.startswith('#'): continue
            r=next(csv.reader([line])); typ=r[0]
            if typ not in ('UncalAccel','Accel','UncalGyro','Gyro') or typ not in hdr: continue
            vals=r[1:]+['']*max(0,len(hdr[typ])-(len(r)-1)); d=dict(zip(hdr[typ],vals))
            def num(*names):
                for n in names:
                    try:
                        if d.get(n,'')!='': return float(d[n])
                    except: pass
                return np.nan
            t=num('utcTimeMillis','UnixTimeMillis')
            if not np.isfinite(t): continue
            if typ in ('UncalAccel','Accel'):
                x=num('UncalAccelXMps2','AccelXMps2'); y=num('UncalAccelYMps2','AccelYMps2'); z=num('UncalAccelZMps2','AccelZMps2')
                if np.all(np.isfinite([x,y,z])): acc.append((t,math.sqrt(x*x+y*y+z*z)))
            else:
                x=num('UncalGyroXRadPerSec','GyroXRadPerSec'); y=num('UncalGyroYRadPerSec','GyroYRadPerSec'); z=num('UncalGyroZRadPerSec','GyroZRadPerSec')
                if np.all(np.isfinite([x,y,z])): gyr.append((t,math.sqrt(x*x+y*y+z*z)))
    def secmed(rows,name):
        if not rows:return pd.DataFrame(columns=['unix_ms',name])
        d=pd.DataFrame(rows,columns=['unix_ms',name]); d['bin']=(d.unix_ms/1000).round().astype('int64')
        o=d.groupby('bin')[name].median().reset_index(); o['unix_ms']=o.bin*1000.; return o[['unix_ms',name]].sort_values('unix_ms')
    return secmed(acc,'acc_norm'),secmed(gyr,'gyro_norm')

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('log',nargs='?',default='gnss_log.txt'); ap.add_argument('--doppler',default='analysis/validated_doppler/doppler_velocity_plain.csv'); ap.add_argument('--out-dir',default='analysis/imu_zupt_output'); a=ap.parse_args()
    out=Path(a.out_dir); out.mkdir(parents=True,exist_ok=True)
    fix=read_fix(Path(a.log)); dop=pd.read_csv(a.doppler)
    for c in ('gps_millis','vx_ecef_mps','vy_ecef_mps','vz_ecef_mps'): dop[c]=pd.to_numeric(dop[c],errors='coerce')
    dop=dop.dropna(subset=['gps_millis','vx_ecef_mps','vy_ecef_mps','vz_ecef_mps']).copy()
    lat0=float(np.median(fix.LatitudeDegrees)); lon0=float(np.median(fix.LongitudeDegrees)); fix['east_m'],fix['north_m']=lla_to_local(fix.LatitudeDegrees,fix.LongitudeDegrees,lat0,lon0)
    enu=[ecef_velocity_to_enu(r.vx_ecef_mps,r.vy_ecef_mps,r.vz_ecef_mps,lat0,lon0) for r in dop.itertuples()]; dop['ve']=[x[0] for x in enu]; dop['vn']=[x[1] for x in enu]; dop['unix_ms']=dop.gps_millis+GPS_UNIX_EPOCH_OFFSET_MS-GPS_UTC_LEAP_MS
    acc,gyr=read_imu(a.log); dop=dop.sort_values('unix_ms'); dop=pd.merge_asof(dop,acc,on='unix_ms',direction='nearest',tolerance=1200); dop=pd.merge_asof(dop,gyr,on='unix_ms',direction='nearest',tolerance=1200)
    # nearest Android speed at Doppler epoch
    fs=fix[['UnixTimeMillis','SpeedMps']].copy().rename(columns={'UnixTimeMillis':'unix_ms','SpeedMps':'android_speed'}).sort_values('unix_ms'); dop=pd.merge_asof(dop,fs,on='unix_ms',direction='nearest',tolerance=1500)
    events=[]
    for r in fix.itertuples():
        sig=float(r.AccuracyMeters) if np.isfinite(getattr(r,'AccuracyMeters',np.nan)) and r.AccuracyMeters>0 else 3.; events.append((float(r.UnixTimeMillis),'pos',np.array([r.east_m,r.north_m]),sig,None))
    for r in dop.itertuples(): events.append((float(r.unix_ms),'vel',np.array([r.ve,r.vn]),0.30,r))
    events.sort(key=lambda x:x[0]); k=KalmanFilter(dim_x=4,dim_z=2); k.x=np.array([fix.iloc[0].east_m,fix.iloc[0].north_m,0.,0.]); k.P=np.diag([25.,25.,4.,4.]); Hp=np.array([[1.,0.,0.,0.],[0.,1.,0.,0.]]); Hv=np.array([[0.,0.,1.,0.],[0.,0.,0.,1.]])
    stopped=False; enter_n=0; exit_n=0; entries=0; exits=0; zupt=0; vacc=0; vrej=0; rows=[]; prev=events[0][0]
    for t,kind,z,sig,meta in events:
        dt=max(0.,min((t-prev)/1000.,5.));
        if dt>0:
            k.F=np.array([[1.,0.,dt,0.],[0.,1.,0.,dt],[0.,0.,1.,0.],[0.,0.,0.,1.]]); k.Q=process_noise_cv(dt,0.02 if stopped else 0.75); k.predict()
        prev=t; action=kind
        if kind=='pos': k.H=Hp; k.R=np.eye(2)*sig*sig; k.update(z)
        else:
            sp=float(np.linalg.norm(z)); an=float(meta.acc_norm) if np.isfinite(meta.acc_norm) else np.nan; gn=float(meta.gyro_norm) if np.isfinite(meta.gyro_norm) else np.nan; asp=float(meta.android_speed) if np.isfinite(meta.android_speed) else np.nan
            quiet=(sp<0.35 and (not np.isfinite(asp) or asp<0.30) and (not np.isfinite(gn) or gn<0.03) and (not np.isfinite(an) or abs(an-9.80665)<0.15))
            moving=(sp>0.80 or (np.isfinite(asp) and asp>0.60) or (np.isfinite(gn) and gn>0.08) or (np.isfinite(an) and abs(an-9.80665)>0.50))
            if not stopped:
                enter_n=enter_n+1 if quiet else 0
                if enter_n>=3: stopped=True; entries+=1; exit_n=0
            else:
                exit_n=exit_n+1 if moving else 0
                if exit_n>=2: stopped=False; exits+=1; enter_n=0
            if stopped:
                k.H=Hv; k.R=np.eye(2)*0.05**2; k.update(np.zeros(2)); zupt+=1; action='imu_zupt'
            else:
                R=np.eye(2)*sig*sig; y=z-Hv@k.x; S=Hv@k.P@Hv.T+R
                try:d2=float(y.T@np.linalg.solve(S,y))
                except:d2=1e9
                if d2<=9.21: k.H=Hv;k.R=R;k.update(z);vacc+=1;action='vel_accept'
                else: vrej+=1;action='vel_reject'
        rows.append({'unix_ms':t,'event':kind,'action':action,'stopped':stopped,'east_kf_m':float(k.x[0]),'north_kf_m':float(k.x[1]),'ve_kf_mps':float(k.x[2]),'vn_kf_mps':float(k.x[3])})
    fu=pd.DataFrame(rows); fp=fu[fu.event=='pos'].reset_index(drop=True); A=radial_stats(fix.east_m,fix.north_m); D=half_drift(fix.east_m,fix.north_m); K=radial_stats(fp.east_kf_m,fp.north_kf_m); KD=half_drift(fp.east_kf_m,fp.north_kf_m)
    S={'method':'Android position + Raw Doppler + IMU stop classifier + hysteretic ZUPT','android_position':{**A,'half_drift':D},'imu_zupt_kf':{**K,'half_drift':KD},'detector':{'entries':entries,'exits':exits,'zupt_updates':zupt,'velocity_accepted':vacc,'velocity_rejected':vrej},'imu_match':{'acc_epochs':int(dop.acc_norm.notna().sum()),'gyro_epochs':int(dop.gyro_norm.notna().sum())},'warning':'Static repeatability only; moving log required.'}; (out/'summary.json').write_text(json.dumps(S,indent=2)); fu.to_csv(out/'fused_events.csv',index=False)
    f=lambda x:'n/a' if x is None else f'{x:.3f}'; lines=['# IMU-assisted hysteretic ZUPT','', '| Static repeatability | Android | IMU + Doppler ZUPT |','|---|---:|---:|',f"| P50 radial (m) | {f(A.get('p50_m'))} | {f(K.get('p50_m'))} |",f"| P95 radial (m) | {f(A.get('p95_m'))} | {f(K.get('p95_m'))} |",f"| P99 radial (m) | {f(A.get('p99_m'))} | {f(K.get('p99_m'))} |",f"| Half-to-half drift (m) | {f((D or {}).get('distance_m'))} | {f((KD or {}).get('distance_m'))} |",'',f'Entries/exits/zupt: **{entries}/{exits}/{zupt}**',f'Velocity accepted/rejected: **{vacc}/{vrej}**']; (out/'summary.md').write_text('\n'.join(lines)+'\n'); print('\n'.join(lines))
if __name__=='__main__':main()
