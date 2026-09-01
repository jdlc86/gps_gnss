#!/usr/bin/env python3
"""Robust Raw-GNSS experiment: residual FDE + Doppler receiver velocity.

Reuses gnss_lib_py for Android Raw parsing, broadcast ephemerides, WLS and
fault detection/exclusion. The only custom estimator here is the small linear
least-squares velocity solve from pseudorange-rate after satellite states are
known.

This script is deliberately GPS L1-only so the comparison with the previous
baseline is controlled and does not yet mix inter-constellation/frequency
biases.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path

import gnss_lib_py as glp
import numpy as np
import pandas as pd


def q(vals, p):
    a = sorted(float(v) for v in vals if np.isfinite(v))
    if not a:
        return None
    if len(a) == 1:
        return a[0]
    x = (len(a)-1)*p
    lo, hi = int(math.floor(x)), int(math.ceil(x))
    if lo == hi:
        return a[lo]
    return a[lo]*(hi-x) + a[hi]*(x-lo)


def stats(vals):
    a = [float(v) for v in vals if np.isfinite(v)]
    if not a:
        return {"n": 0}
    return {
        "n": len(a), "mean": statistics.fmean(a),
        "std": statistics.stdev(a) if len(a) > 1 else 0.0,
        "p50": q(a, .50), "p95": q(a, .95), "p99": q(a, .99),
        "min": min(a), "max": max(a),
    }


def local_dispersion(lat, lon):
    lat = np.asarray(lat, float); lon = np.asarray(lon, float)
    good = np.isfinite(lat) & np.isfinite(lon)
    lat = lat[good]; lon = lon[good]
    if not len(lat):
        return {}
    lat0 = float(np.median(lat)); lon0 = float(np.median(lon))
    R = 6378137.0
    e = R*np.deg2rad(lon-lon0)*math.cos(math.radians(lat0))
    n = R*np.deg2rad(lat-lat0)
    r = np.hypot(e,n)
    return {
        "epochs": int(len(lat)), "median_lat_deg": lat0, "median_lon_deg": lon0,
        "east_m": stats(e), "north_m": stats(n), "radial_from_median_m": stats(r),
    }


def half_drift(lat, lon):
    lat = np.asarray(lat,float); lon=np.asarray(lon,float)
    good=np.isfinite(lat)&np.isfinite(lon); lat=lat[good]; lon=lon[good]
    h=len(lat)//2
    if h<2: return None
    la1,lo1=float(np.median(lat[:h])),float(np.median(lon[:h]))
    la2,lo2=float(np.median(lat[h:])),float(np.median(lon[h:]))
    R=6378137.0
    de=R*math.radians(lo2-lo1)*math.cos(math.radians((la1+la2)/2))
    dn=R*math.radians(la2-la1)
    return {"east_m":de,"north_m":dn,"distance_m":math.hypot(de,dn)}


def read_fix(log_path):
    hdr=None; rows=[]
    with Path(log_path).open(encoding="utf-8",errors="replace") as fh:
        for line in fh:
            if line.startswith("# Fix,"):
                hdr=next(csv.reader([line[2:].strip()]))[1:]
            elif hdr and line.startswith("Fix,"):
                vals=next(csv.reader([line.strip()]))[1:]
                vals += [""]*max(0,len(hdr)-len(vals))
                rows.append(dict(zip(hdr,vals)))
    df=pd.DataFrame(rows)
    for c in ("LatitudeDegrees","LongitudeDegrees","UnixTimeMillis","SpeedMps"):
        if c in df: df[c]=pd.to_numeric(df[c],errors="coerce")
    return df


def prepare(log_path, min_cn0, max_pr_sigma):
    raw=glp.AndroidRawGnss(log_path,filter_measurements=False,verbose=True)
    df=raw.pandas_df(); before=len(df)
    num=["gps_millis","raw_pr_m","raw_pr_sigma_m","cn0_dbhz","CarrierFrequencyHz",
         "PseudorangeRateMetersPerSecond","PseudorangeRateUncertaintyMetersPerSecond"]
    for c in num:
        if c in df: df[c]=pd.to_numeric(df[c],errors="coerce")
    required=["gps_millis","raw_pr_m","raw_pr_sigma_m","cn0_dbhz","gnss_id","CarrierFrequencyHz"]
    missing=[c for c in required if c not in df]
    if missing: raise RuntimeError(f"Missing parser rows: {missing}")
    mask=(
        (df["gnss_id"]=="gps") & df["CarrierFrequencyHz"].between(1.55e9,1.60e9)
        & df["gps_millis"].notna() & df["raw_pr_m"].between(1e6,6e7)
        & df["raw_pr_sigma_m"].between(0,max_pr_sigma) & (df["cn0_dbhz"]>=min_cn0)
    )
    df=df.loc[mask].copy()
    meas=glp.NavData(pandas_df=df)
    full=glp.add_sv_states_rinex(meas)
    full["corr_pr_m"]=full["raw_pr_m"]+full["b_sv_m"]
    full["weights"]=1.0/np.maximum(full["raw_pr_sigma_m"],1e-3)**2
    return full,before,len(df)


def position_solution(nav):
    sol=glp.solve_wls(nav,weight_type="weights")
    df=sol.pandas_df()
    xyz=df[["x_rx_wls_m","y_rx_wls_m","z_rx_wls_m"]].to_numpy(float).T
    lla=glp.ecef_to_geodetic(xyz)
    df["lat_deg"]=lla[0]; df["lon_deg"]=lla[1]; df["alt_m"]=lla[2]
    good=np.isfinite(df["lat_deg"])&np.isfinite(df["lon_deg"])
    return df.loc[good].copy()


def doppler_velocity(full, pos_df):
    """Estimate receiver ECEF velocity + clock drift per epoch.

    rho_dot = u.(v_sv-v_rx) + bdot_rx - bdot_sv
    -> (rho_dot + bdot_sv - u.v_sv) = [-u, 1] [v_rx, bdot_rx]
    """
    fdf=full.pandas_df().copy()
    req=["gps_millis","x_sv_m","y_sv_m","z_sv_m","vx_sv_mps","vy_sv_mps","vz_sv_mps",
         "b_dot_sv_mps","PseudorangeRateMetersPerSecond","PseudorangeRateUncertaintyMetersPerSecond"]
    missing=[c for c in req if c not in fdf.columns]
    if missing: return pd.DataFrame(), {"error":f"missing {missing}"}
    for c in req: fdf[c]=pd.to_numeric(fdf[c],errors="coerce")
    pmap={int(round(r.gps_millis)):(r.x_rx_wls_m,r.y_rx_wls_m,r.z_rx_wls_m)
          for r in pos_df.itertuples() if np.isfinite(r.gps_millis)}
    rows=[]
    for t,g in fdf.groupby("gps_millis"):
        key=int(round(float(t)))
        if key not in pmap: continue
        rx=np.asarray(pmap[key],float)
        A=[]; y=[]; w=[]
        for r in g.itertuples():
            vals=[r.x_sv_m,r.y_sv_m,r.z_sv_m,r.vx_sv_mps,r.vy_sv_mps,r.vz_sv_mps,
                  r.b_dot_sv_mps,r.PseudorangeRateMetersPerSecond,r.PseudorangeRateUncertaintyMetersPerSecond]
            if not np.all(np.isfinite(vals)): continue
            sv=np.asarray(vals[:3]); vv=np.asarray(vals[3:6]); d=sv-rx; norm=np.linalg.norm(d)
            if norm<=0: continue
            u=d/norm
            z=float(vals[7])+float(vals[6])-float(np.dot(u,vv))
            A.append([-u[0],-u[1],-u[2],1.0]); y.append(z)
            sig=max(float(vals[8]),0.02); w.append(1.0/(sig*sig))
        if len(A)<4: continue
        A=np.asarray(A); y=np.asarray(y); sw=np.sqrt(np.asarray(w))
        try:
            x,*_=np.linalg.lstsq(A*sw[:,None],y*sw,rcond=None)
        except np.linalg.LinAlgError:
            continue
        speed=float(np.linalg.norm(x[:3]))
        residual=y-A@x
        rows.append({"gps_millis":float(t),"vx_ecef_mps":x[0],"vy_ecef_mps":x[1],"vz_ecef_mps":x[2],
                     "clock_drift_mps":x[3],"speed_3d_mps":speed,"doppler_residual_rms_mps":float(np.sqrt(np.mean(residual**2))),
                     "satellites":len(A)})
    out=pd.DataFrame(rows)
    summary={}
    if not out.empty:
        summary={"epochs":len(out),"speed_3d_mps":stats(out.speed_3d_mps),
                 "residual_rms_mps":stats(out.doppler_residual_rms_mps),
                 "satellites_per_epoch":stats(out.satellites)}
    return out,summary


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("log",nargs="?",default="gnss_log.txt")
    ap.add_argument("--out-dir",default="analysis/robust_output")
    ap.add_argument("--min-cn0",type=float,default=20.0)
    ap.add_argument("--max-pr-sigma",type=float,default=150.0)
    ap.add_argument("--max-faults",type=int,default=2)
    args=ap.parse_args()
    out=Path(args.out_dir); out.mkdir(parents=True,exist_ok=True)

    full,before,gated=prepare(Path(args.log),args.min_cn0,args.max_pr_sigma)
    base=position_solution(full)
    base_disp=local_dispersion(base.lat_deg,base.lon_deg)
    base_disp["first_half_to_second_half_median_drift"]=half_drift(base.lat_deg,base.lon_deg)

    # gnss_lib_py residual-FDE is reused rather than writing custom outlier logic.
    fde=glp.solve_fde(full,method="residual",remove_outliers=True,max_faults=args.max_faults)
    robust=position_solution(fde)
    robust_disp=local_dispersion(robust.lat_deg,robust.lon_deg)
    robust_disp["first_half_to_second_half_median_drift"]=half_drift(robust.lat_deg,robust.lon_deg)

    vel,vel_summary=doppler_velocity(fde,robust)
    fix=read_fix(args.log)
    android={}
    if not fix.empty:
        android=local_dispersion(fix.LatitudeDegrees,fix.LongitudeDegrees)
        android["first_half_to_second_half_median_drift"]=half_drift(fix.LatitudeDegrees,fix.LongitudeDegrees)
        if "SpeedMps" in fix: android["speed_mps"]=stats(fix.SpeedMps)

    result={
        "method":"GPS L1 broadcast ephemeris; sigma-weighted WLS; gnss_lib_py residual FDE; Doppler velocity WLS",
        "raw_measurements_before":before,"gps_l1_after_gate":gated,
        "fde":{"method":"residual","max_faults":args.max_faults,"measurements_after_fde":len(fde)},
        "android_fix":android,"plain_wls":base_disp,"residual_fde_wls":robust_disp,"doppler_velocity":vel_summary,
    }
    (out/"summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    base.to_csv(out/"plain_wls.csv",index=False); robust.to_csv(out/"robust_wls.csv",index=False)
    vel.to_csv(out/"doppler_velocity.csv",index=False)

    def m(d,k): return d.get("radial_from_median_m",{}).get(k)
    def fmt(x): return "n/a" if x is None else f"{x:.3f}"
    lines=["# Robust WLS + Doppler experiment","",
           f"- Raw rows / GPS-L1 gated / after FDE: **{before} / {gated} / {len(fde)}**","",
           "| Static repeatability | Android | Plain WLS | Residual-FDE WLS |",
           "|---|---:|---:|---:|",
           f"| P50 radial (m) | {fmt(m(android,'p50'))} | {fmt(m(base_disp,'p50'))} | {fmt(m(robust_disp,'p50'))} |",
           f"| P95 radial (m) | {fmt(m(android,'p95'))} | {fmt(m(base_disp,'p95'))} | {fmt(m(robust_disp,'p95'))} |",
           f"| P99 radial (m) | {fmt(m(android,'p99'))} | {fmt(m(base_disp,'p99'))} | {fmt(m(robust_disp,'p99'))} |",
           f"| Half-to-half drift (m) | {fmt((android.get('first_half_to_second_half_median_drift') or {}).get('distance_m'))} | {fmt((base_disp.get('first_half_to_second_half_median_drift') or {}).get('distance_m'))} | {fmt((robust_disp.get('first_half_to_second_half_median_drift') or {}).get('distance_m'))} |","",
           "## Doppler velocity (phone was static)",
           f"- Epochs: **{vel_summary.get('epochs','n/a')}**",
           f"- 3D speed P50 / P95 / max: **{fmt(vel_summary.get('speed_3d_mps',{}).get('p50'))} / {fmt(vel_summary.get('speed_3d_mps',{}).get('p95'))} / {fmt(vel_summary.get('speed_3d_mps',{}).get('max'))} m/s**",
           f"- Doppler residual RMS P50 / P95: **{fmt(vel_summary.get('residual_rms_mps',{}).get('p50'))} / {fmt(vel_summary.get('residual_rms_mps',{}).get('p95'))} m/s**","",
           "Static dispersion is repeatability around each solution's median, not absolute accuracy."]
    (out/"summary.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
    print("\n".join(lines))

if __name__=="__main__": main()
