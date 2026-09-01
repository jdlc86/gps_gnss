#!/usr/bin/env python3
"""Baseline fusion: Android position + Raw-GNSS Doppler velocity using FilterPy.

This is intentionally *not* an INS/EKF yet. It is a controlled 2D constant-
velocity Kalman baseline to answer one question: does the Doppler velocity we
observed in the static log help suppress Android position wandering?

Inputs:
- GnssLogger-compatible gnss_log.txt (Fix rows)
- doppler_velocity_plain.csv produced by run_robust_wls_doppler.py

State: [east_m, north_m, east_velocity_mps, north_velocity_mps]
Measurements:
- Android Fix -> east/north position
- Raw GNSS pseudorange-rate solution -> east/north velocity

The default noises are conservative baselines, not device-specific final
calibration. Results on a static log measure repeatability only, not absolute
accuracy.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
from filterpy.kalman import KalmanFilter

GPS_UNIX_EPOCH_OFFSET_MS = 315964800000
GPS_UTC_LEAP_MS = 18000  # GPS-UTC offset valid for this 2026 data set.


def read_fix(log_path: Path) -> pd.DataFrame:
    header = None
    rows = []
    with log_path.open(encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if line.startswith("# Fix,"):
                header = next(csv.reader([line[2:].strip()]))[1:]
            elif header and line.startswith("Fix,"):
                values = next(csv.reader([line.strip()]))[1:]
                values += [""] * max(0, len(header) - len(values))
                rows.append(dict(zip(header, values)))
    df = pd.DataFrame(rows)
    for c in ("LatitudeDegrees", "LongitudeDegrees", "UnixTimeMillis", "AccuracyMeters", "SpeedMps"):
        if c in df:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    return df.dropna(subset=["LatitudeDegrees", "LongitudeDegrees", "UnixTimeMillis"]).copy()


def lla_to_local(lat_deg, lon_deg, lat0_deg, lon0_deg):
    R = 6378137.0
    lat = np.asarray(lat_deg, dtype=float)
    lon = np.asarray(lon_deg, dtype=float)
    east = R * np.deg2rad(lon - lon0_deg) * math.cos(math.radians(lat0_deg))
    north = R * np.deg2rad(lat - lat0_deg)
    return east, north


def ecef_velocity_to_enu(vx, vy, vz, lat_deg, lon_deg):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    slat, clat = math.sin(lat), math.cos(lat)
    slon, clon = math.sin(lon), math.cos(lon)
    east = -slon * vx + clon * vy
    north = -slat * clon * vx - slat * slon * vy + clat * vz
    up = clat * clon * vx + clat * slon * vy + slat * vz
    return east, north, up


def radial_stats(east, north):
    e = np.asarray(east, dtype=float)
    n = np.asarray(north, dtype=float)
    good = np.isfinite(e) & np.isfinite(n)
    e, n = e[good], n[good]
    if len(e) == 0:
        return {}
    em, nm = float(np.median(e)), float(np.median(n))
    r = np.hypot(e - em, n - nm)
    return {
        "epochs": int(len(r)),
        "p50_m": float(np.percentile(r, 50)),
        "p95_m": float(np.percentile(r, 95)),
        "p99_m": float(np.percentile(r, 99)),
        "max_m": float(np.max(r)),
        "std_east_m": float(np.std(e, ddof=1)) if len(e) > 1 else 0.0,
        "std_north_m": float(np.std(n, ddof=1)) if len(n) > 1 else 0.0,
    }


def half_drift(east, north):
    e = np.asarray(east, dtype=float)
    n = np.asarray(north, dtype=float)
    good = np.isfinite(e) & np.isfinite(n)
    e, n = e[good], n[good]
    h = len(e) // 2
    if h < 2:
        return None
    de = float(np.median(e[h:]) - np.median(e[:h]))
    dn = float(np.median(n[h:]) - np.median(n[:h]))
    return {"east_m": de, "north_m": dn, "distance_m": math.hypot(de, dn)}


def process_noise_cv(dt: float, sigma_a: float) -> np.ndarray:
    q = sigma_a * sigma_a
    return q * np.array([
        [dt**4 / 4, 0, dt**3 / 2, 0],
        [0, dt**4 / 4, 0, dt**3 / 2],
        [dt**3 / 2, 0, dt**2, 0],
        [0, dt**3 / 2, 0, dt**2],
    ], dtype=float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", default="gnss_log.txt")
    ap.add_argument("--doppler", default="analysis/robust_output/doppler_velocity_plain.csv")
    ap.add_argument("--out-dir", default="analysis/fusion_output")
    ap.add_argument("--position-sigma-m", type=float, default=3.0,
                    help="Fallback 1-sigma position noise when AccuracyMeters is unavailable")
    ap.add_argument("--doppler-sigma-mps", type=float, default=0.30,
                    help="Conservative horizontal Doppler velocity sigma baseline")
    ap.add_argument("--process-accel-sigma-mps2", type=float, default=0.75)
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    fix = read_fix(Path(args.log))
    if fix.empty:
        raise RuntimeError("No Fix rows found")
    dop = pd.read_csv(args.doppler)
    for c in ("gps_millis", "vx_ecef_mps", "vy_ecef_mps", "vz_ecef_mps"):
        dop[c] = pd.to_numeric(dop[c], errors="coerce")
    dop = dop.dropna(subset=["gps_millis", "vx_ecef_mps", "vy_ecef_mps", "vz_ecef_mps"]).copy()

    lat0 = float(np.median(fix["LatitudeDegrees"]))
    lon0 = float(np.median(fix["LongitudeDegrees"]))
    fix["east_m"], fix["north_m"] = lla_to_local(
        fix["LatitudeDegrees"], fix["LongitudeDegrees"], lat0, lon0
    )

    enu = [ecef_velocity_to_enu(r.vx_ecef_mps, r.vy_ecef_mps, r.vz_ecef_mps, lat0, lon0)
           for r in dop.itertuples()]
    dop["ve_mps"] = [v[0] for v in enu]
    dop["vn_mps"] = [v[1] for v in enu]
    dop["vu_mps"] = [v[2] for v in enu]
    dop["unix_ms"] = dop["gps_millis"] + GPS_UNIX_EPOCH_OFFSET_MS - GPS_UTC_LEAP_MS

    # Position event and velocity event streams are fused chronologically.
    events = []
    for r in fix.itertuples():
        acc = getattr(r, "AccuracyMeters", np.nan)
        sigma = float(acc) if np.isfinite(acc) and acc > 0 else args.position_sigma_m
        events.append((float(r.UnixTimeMillis), "pos", np.array([r.east_m, r.north_m]), sigma))
    for r in dop.itertuples():
        events.append((float(r.unix_ms), "vel", np.array([r.ve_mps, r.vn_mps]), args.doppler_sigma_mps))
    events.sort(key=lambda x: x[0])

    kf = KalmanFilter(dim_x=4, dim_z=2)
    kf.x = np.array([fix.iloc[0].east_m, fix.iloc[0].north_m, 0.0, 0.0], dtype=float)
    kf.P = np.diag([25.0, 25.0, 4.0, 4.0])
    H_pos = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.]])
    H_vel = np.array([[0., 0., 1., 0.], [0., 0., 0., 1.]])

    rows = []
    prev_ms = events[0][0]
    for t_ms, kind, z, sigma in events:
        dt = max(0.0, min((t_ms - prev_ms) / 1000.0, 5.0))
        if dt > 0:
            kf.F = np.array([[1., 0., dt, 0.], [0., 1., 0., dt],
                             [0., 0., 1., 0.], [0., 0., 0., 1.]])
            kf.Q = process_noise_cv(dt, args.process_accel_sigma_mps2)
            kf.predict()
        prev_ms = t_ms

        if kind == "pos":
            kf.H = H_pos
            kf.R = np.eye(2) * sigma * sigma
        else:
            kf.H = H_vel
            kf.R = np.eye(2) * sigma * sigma
        kf.update(z)

        rows.append({
            "unix_ms": t_ms, "event": kind,
            "east_kf_m": float(kf.x[0]), "north_kf_m": float(kf.x[1]),
            "ve_kf_mps": float(kf.x[2]), "vn_kf_mps": float(kf.x[3]),
        })

    fused = pd.DataFrame(rows)
    # Compare only at Android position-update times to avoid biasing metrics by
    # extra Doppler events.
    fused_pos = fused[fused["event"] == "pos"].reset_index(drop=True)
    if len(fused_pos) != len(fix):
        raise RuntimeError("Unexpected position-event count mismatch")

    android_stats = radial_stats(fix["east_m"], fix["north_m"])
    android_drift = half_drift(fix["east_m"], fix["north_m"])
    kf_stats = radial_stats(fused_pos["east_kf_m"], fused_pos["north_kf_m"])
    kf_drift = half_drift(fused_pos["east_kf_m"], fused_pos["north_kf_m"])
    dop_speed = np.hypot(dop["ve_mps"], dop["vn_mps"])

    summary = {
        "method": "FilterPy 2D constant-velocity KF: Android position + Raw-GNSS Doppler horizontal velocity",
        "reference_lat_deg": lat0,
        "reference_lon_deg": lon0,
        "noise": {
            "position": "Android AccuracyMeters per fix, fallback from CLI",
            "doppler_sigma_mps": args.doppler_sigma_mps,
            "process_accel_sigma_mps2": args.process_accel_sigma_mps2,
        },
        "android_position": {**android_stats, "half_drift": android_drift},
        "position_doppler_kf": {**kf_stats, "half_drift": kf_drift},
        "doppler_horizontal_speed_static": {
            "epochs": int(len(dop_speed)),
            "p50_mps": float(np.percentile(dop_speed, 50)),
            "p95_mps": float(np.percentile(dop_speed, 95)),
            "max_mps": float(np.max(dop_speed)),
        },
        "warning": "Static-log repeatability is not absolute accuracy and does not validate moving-vehicle dynamics.",
    }

    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    fused.to_csv(out_dir / "fused_events.csv", index=False)
    comparison = pd.DataFrame({
        "unix_ms": fix["UnixTimeMillis"].to_numpy(),
        "android_east_m": fix["east_m"].to_numpy(),
        "android_north_m": fix["north_m"].to_numpy(),
        "kf_east_m": fused_pos["east_kf_m"].to_numpy(),
        "kf_north_m": fused_pos["north_kf_m"].to_numpy(),
    })
    comparison.to_csv(out_dir / "position_comparison.csv", index=False)

    def f(v):
        return "n/a" if v is None else f"{v:.3f}"
    lines = [
        "# Android position + Raw Doppler Kalman baseline", "",
        "| Static repeatability | Android | Position + Doppler KF |",
        "|---|---:|---:|",
        f"| P50 radial (m) | {f(android_stats.get('p50_m'))} | {f(kf_stats.get('p50_m'))} |",
        f"| P95 radial (m) | {f(android_stats.get('p95_m'))} | {f(kf_stats.get('p95_m'))} |",
        f"| P99 radial (m) | {f(android_stats.get('p99_m'))} | {f(kf_stats.get('p99_m'))} |",
        f"| Half-to-half drift (m) | {f((android_drift or {}).get('distance_m'))} | {f((kf_drift or {}).get('distance_m'))} |",
        "",
        f"Doppler horizontal speed, static P50/P95/max: **{f(summary['doppler_horizontal_speed_static']['p50_mps'])} / {f(summary['doppler_horizontal_speed_static']['p95_mps'])} / {f(summary['doppler_horizontal_speed_static']['max_mps'])} m/s**",
        "",
        "This static test measures repeatability, not absolute accuracy. A moving vehicle log is required before accepting this filter for parking navigation.",
    ]
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
