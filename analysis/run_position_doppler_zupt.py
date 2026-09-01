#!/usr/bin/env python3
"""Android position + Raw-GNSS Doppler with innovation gating and ZUPT.

Controlled extension of run_position_doppler_kf.py. It does not assume the log
is static. STOP is entered only after consecutive low-speed Doppler samples and
a low Android-reported speed. While stopped, velocity is constrained to zero
and process acceleration noise is reduced so repeated GNSS fixes estimate one
stationary position instead of a moving trajectory.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
from filterpy.kalman import KalmanFilter

from run_position_doppler_kf import (
    GPS_UNIX_EPOCH_OFFSET_MS, GPS_UTC_LEAP_MS, read_fix, lla_to_local,
    ecef_velocity_to_enu, radial_stats, half_drift, process_noise_cv,
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", default="gnss_log.txt")
    ap.add_argument("--doppler", default="analysis/validated_doppler/doppler_velocity_plain.csv")
    ap.add_argument("--out-dir", default="analysis/zupt_output")
    ap.add_argument("--position-sigma-m", type=float, default=3.0)
    ap.add_argument("--doppler-sigma-mps", type=float, default=0.30)
    ap.add_argument("--moving-accel-sigma", type=float, default=0.75)
    ap.add_argument("--stopped-accel-sigma", type=float, default=0.03)
    ap.add_argument("--stop-doppler-mps", type=float, default=0.15)
    ap.add_argument("--stop-android-mps", type=float, default=0.30)
    ap.add_argument("--stop-count", type=int, default=3)
    ap.add_argument("--zupt-sigma-mps", type=float, default=0.05)
    ap.add_argument("--mahalanobis2", type=float, default=9.21,
                    help="2D velocity innovation gate; 9.21 ~= chi-square 99%%")
    args = ap.parse_args()

    out = Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
    fix = read_fix(Path(args.log))
    if fix.empty: raise RuntimeError("No Fix rows found")
    dop = pd.read_csv(args.doppler)
    req = ["gps_millis", "vx_ecef_mps", "vy_ecef_mps", "vz_ecef_mps"]
    for c in req: dop[c] = pd.to_numeric(dop[c], errors="coerce")
    dop = dop.dropna(subset=req).copy()

    lat0 = float(np.median(fix.LatitudeDegrees)); lon0 = float(np.median(fix.LongitudeDegrees))
    fix["east_m"], fix["north_m"] = lla_to_local(fix.LatitudeDegrees, fix.LongitudeDegrees, lat0, lon0)
    enu = [ecef_velocity_to_enu(r.vx_ecef_mps, r.vy_ecef_mps, r.vz_ecef_mps, lat0, lon0)
           for r in dop.itertuples()]
    dop["ve_mps"] = [v[0] for v in enu]; dop["vn_mps"] = [v[1] for v in enu]
    dop["unix_ms"] = dop.gps_millis + GPS_UNIX_EPOCH_OFFSET_MS - GPS_UTC_LEAP_MS

    events=[]
    for r in fix.itertuples():
        acc=getattr(r,"AccuracyMeters",np.nan); sig=float(acc) if np.isfinite(acc) and acc>0 else args.position_sigma_m
        sp=getattr(r,"SpeedMps",np.nan)
        events.append((float(r.UnixTimeMillis),"pos",np.array([r.east_m,r.north_m]),sig,float(sp) if np.isfinite(sp) else np.nan))
    for r in dop.itertuples():
        events.append((float(r.unix_ms),"vel",np.array([r.ve_mps,r.vn_mps]),args.doppler_sigma_mps,np.nan))
    events.sort(key=lambda x:x[0])

    kf=KalmanFilter(dim_x=4,dim_z=2)
    kf.x=np.array([fix.iloc[0].east_m,fix.iloc[0].north_m,0.,0.])
    kf.P=np.diag([25.,25.,4.,4.])
    Hpos=np.array([[1.,0.,0.,0.],[0.,1.,0.,0.]])
    Hvel=np.array([[0.,0.,1.,0.],[0.,0.,0.,1.]])
    latest_android_speed=np.nan; low_count=0; stopped=False
    accepted_vel=0; rejected_vel=0; zupt_updates=0; stop_entries=0
    rows=[]; prev=events[0][0]

    for t,kind,z,sigma,android_speed in events:
        dt=max(0.,min((t-prev)/1000.,5.))
        if dt>0:
            kf.F=np.array([[1.,0.,dt,0.],[0.,1.,0.,dt],[0.,0.,1.,0.],[0.,0.,0.,1.]])
            accel_sigma=args.stopped_accel_sigma if stopped else args.moving_accel_sigma
            kf.Q=process_noise_cv(dt,accel_sigma); kf.predict()
        prev=t

        action=kind
        if kind=="pos":
            if np.isfinite(android_speed): latest_android_speed=android_speed
            kf.H=Hpos; kf.R=np.eye(2)*sigma*sigma; kf.update(z)
        else:
            speed=float(np.linalg.norm(z))
            android_low=(not np.isfinite(latest_android_speed)) or latest_android_speed <= args.stop_android_mps
            if speed <= args.stop_doppler_mps and android_low: low_count+=1
            else: low_count=0
            was_stopped=stopped
            stopped=low_count>=args.stop_count
            if stopped and not was_stopped: stop_entries+=1

            if stopped:
                kf.H=Hvel; kf.R=np.eye(2)*args.zupt_sigma_mps**2; kf.update(np.zeros(2))
                zupt_updates+=1; action="zupt"
            else:
                R=np.eye(2)*sigma*sigma
                innov=z-Hvel@kf.x; S=Hvel@kf.P@Hvel.T+R
                try: d2=float(innov.T@np.linalg.solve(S,innov))
                except np.linalg.LinAlgError: d2=float("inf")
                if d2 <= args.mahalanobis2:
                    kf.H=Hvel; kf.R=R; kf.update(z); accepted_vel+=1; action="vel_accept"
                else:
                    rejected_vel+=1; action="vel_reject"

        rows.append({"unix_ms":t,"event":kind,"action":action,"stopped":stopped,
                     "east_kf_m":float(kf.x[0]),"north_kf_m":float(kf.x[1]),
                     "ve_kf_mps":float(kf.x[2]),"vn_kf_mps":float(kf.x[3])})

    fused=pd.DataFrame(rows); fused_pos=fused[fused.event=="pos"].reset_index(drop=True)
    if len(fused_pos)!=len(fix): raise RuntimeError("position-event count mismatch")
    a_stats=radial_stats(fix.east_m,fix.north_m); a_drift=half_drift(fix.east_m,fix.north_m)
    z_stats=radial_stats(fused_pos.east_kf_m,fused_pos.north_kf_m); z_drift=half_drift(fused_pos.east_kf_m,fused_pos.north_kf_m)
    ds=np.hypot(dop.ve_mps,dop.vn_mps)
    summary={
      "method":"FilterPy CV KF + Doppler innovation gate + automatic zero-velocity updates",
      "thresholds":{"stop_doppler_mps":args.stop_doppler_mps,"stop_android_mps":args.stop_android_mps,
                    "stop_count":args.stop_count,"mahalanobis2":args.mahalanobis2,"zupt_sigma_mps":args.zupt_sigma_mps},
      "android_position":{**a_stats,"half_drift":a_drift},
      "gated_zupt_kf":{**z_stats,"half_drift":z_drift},
      "detector":{"stop_entries":stop_entries,"zupt_updates":zupt_updates,"accepted_velocity_updates":accepted_vel,
                  "rejected_velocity_updates":rejected_vel,"position_epochs_marked_stopped":int(fused_pos.stopped.sum())},
      "doppler_horizontal_speed":{"p50_mps":float(np.percentile(ds,50)),"p95_mps":float(np.percentile(ds,95)),"max_mps":float(np.max(ds))},
      "warning":"Static repeatability only; moving-vehicle validation is required before navigation use."
    }
    (out/"summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
    fused.to_csv(out/"fused_events.csv",index=False)
    pd.DataFrame({"unix_ms":fix.UnixTimeMillis.to_numpy(),"android_east_m":fix.east_m.to_numpy(),
                  "android_north_m":fix.north_m.to_numpy(),"kf_east_m":fused_pos.east_kf_m.to_numpy(),
                  "kf_north_m":fused_pos.north_kf_m.to_numpy(),"stopped":fused_pos.stopped.to_numpy()}).to_csv(out/"position_comparison.csv",index=False)
    f=lambda x:"n/a" if x is None else f"{x:.3f}"
    lines=["# Doppler gating + automatic ZUPT","",
      "| Static repeatability | Android | Gated + ZUPT KF |","|---|---:|---:|",
      f"| P50 radial (m) | {f(a_stats.get('p50_m'))} | {f(z_stats.get('p50_m'))} |",
      f"| P95 radial (m) | {f(a_stats.get('p95_m'))} | {f(z_stats.get('p95_m'))} |",
      f"| P99 radial (m) | {f(a_stats.get('p99_m'))} | {f(z_stats.get('p99_m'))} |",
      f"| Half-to-half drift (m) | {f((a_drift or {}).get('distance_m'))} | {f((z_drift or {}).get('distance_m'))} |","",
      f"STOP entries / ZUPT updates: **{stop_entries} / {zupt_updates}**",
      f"Velocity accepted / rejected: **{accepted_vel} / {rejected_vel}**",
      f"Doppler static P50/P95/max: **{f(np.percentile(ds,50))} / {f(np.percentile(ds,95))} / {f(np.max(ds))} m/s**","",
      "STOP is inferred from consecutive Doppler + Android speed observations; the script is not told that the log is static."]
    (out/"summary.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
    print("\n".join(lines))

if __name__=="__main__": main()
