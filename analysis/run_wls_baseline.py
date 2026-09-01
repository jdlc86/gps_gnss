#!/usr/bin/env python3
"""Compute a reproducible WLS baseline from Android Raw GNSS measurements.

This intentionally reuses Stanford NAV Lab's gnss_lib_py rather than
implementing satellite orbit propagation or WLS from scratch.

Pipeline:
  GnssLogger-compatible Raw rows
    -> AndroidRawGnss parser / pseudoranges
    -> conservative GPS L1 quality gating
    -> broadcast RINEX satellite states
    -> satellite clock correction
    -> weighted least squares
    -> static repeatability metrics + comparison with Android Fix

Why GPS L1 for the first baseline?
- Broadcast ephemeris is available for current-day/realtime processing.
- gnss_lib_py's RINEX broadcast path currently supports GPS.
- Restricting to L1 avoids using L1 and L5 from the same SV as independent
  observations before explicitly modelling inter-frequency code biases.

The current phone reports ADR state 16 for all observations, so ADR validity is
NOT used as a gate here. This baseline is code/pseudorange based, not carrier-
phase positioning.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path

import numpy as np
import pandas as pd
import gnss_lib_py as glp


def quantile(values, p):
    values = sorted(float(v) for v in values if np.isfinite(v))
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    x = (len(values) - 1) * p
    lo = int(math.floor(x))
    hi = int(math.ceil(x))
    if lo == hi:
        return values[lo]
    return values[lo] * (hi - x) + values[hi] * (x - lo)


def stats(values):
    vals = [float(v) for v in values if np.isfinite(v)]
    if not vals:
        return {"n": 0}
    return {
        "n": len(vals),
        "mean": statistics.fmean(vals),
        "std": statistics.stdev(vals) if len(vals) > 1 else 0.0,
        "p50": quantile(vals, 0.50),
        "p95": quantile(vals, 0.95),
        "p99": quantile(vals, 0.99),
        "min": min(vals),
        "max": max(vals),
    }


def local_dispersion(lat, lon):
    lat = np.asarray(lat, dtype=float)
    lon = np.asarray(lon, dtype=float)
    good = np.isfinite(lat) & np.isfinite(lon)
    lat = lat[good]
    lon = lon[good]
    if len(lat) == 0:
        return {}, np.array([]), np.array([]), np.array([])
    lat0 = float(np.median(lat))
    lon0 = float(np.median(lon))
    r = 6378137.0
    east = r * np.deg2rad(lon - lon0) * math.cos(math.radians(lat0))
    north = r * np.deg2rad(lat - lat0)
    radial = np.hypot(east, north)
    summary = {
        "median_lat_deg": lat0,
        "median_lon_deg": lon0,
        "east_m": stats(east),
        "north_m": stats(north),
        "radial_from_median_m": stats(radial),
    }
    return summary, east, north, radial


def read_android_fixes(log_path: Path):
    header = None
    rows = []
    with log_path.open("r", encoding="utf-8", errors="replace", newline="") as fh:
        for line in fh:
            if line.startswith("# Fix,"):
                header = next(csv.reader([line[2:].strip()]))[1:]
            elif line.startswith("Fix,") and header:
                vals = next(csv.reader([line.strip()]))[1:]
                if len(vals) < len(header):
                    vals += [""] * (len(header) - len(vals))
                rows.append(dict(zip(header, vals)))
    if not rows:
        return pd.DataFrame()
    df = pd.DataFrame(rows)
    for col in ("LatitudeDegrees", "LongitudeDegrees", "UnixTimeMillis"):
        if col in df:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", default="gnss_log.txt")
    ap.add_argument("--out-dir", default="analysis/wls_output")
    ap.add_argument("--min-cn0", type=float, default=20.0)
    ap.add_argument("--max-pr-sigma", type=float, default=150.0)
    args = ap.parse_args()

    log_path = Path(args.log)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # The logger currently exposes ADR_STATE=16, which gnss_lib_py's default
    # ADR gate would reject. For a pseudorange WLS baseline we intentionally
    # disable the package's full default filter and apply code-measurement gates
    # below. We are NOT claiming a carrier-phase solution.
    raw = glp.AndroidRawGnss(log_path, filter_measurements=False, verbose=True)
    rdf = raw.pandas_df()
    before = len(rdf)

    required = [
        "gps_millis", "raw_pr_m", "raw_pr_sigma_m", "cn0_dbhz",
        "gnss_id", "CarrierFrequencyHz"
    ]
    missing = [c for c in required if c not in rdf.columns]
    if missing:
        raise RuntimeError(
            f"AndroidRawGnss parser missing expected rows: {missing}; "
            f"available={list(rdf.columns)}"
        )

    for col in (
        "gps_millis", "raw_pr_m", "raw_pr_sigma_m", "cn0_dbhz",
        "CarrierFrequencyHz"
    ):
        rdf[col] = pd.to_numeric(rdf[col], errors="coerce")

    # First, deliberately simple and auditable baseline:
    #   GPS + L1 C/A-like observations only.
    # This prevents duplicate L1/L5 observations from the same SV from being
    # treated as independent before modelling differential code biases.
    is_gps_l1 = (
        (rdf["gnss_id"] == "gps")
        & rdf["CarrierFrequencyHz"].between(1.55e9, 1.60e9)
    )
    mask = (
        is_gps_l1
        & rdf["gps_millis"].notna()
        & rdf["raw_pr_m"].between(1.0e6, 6.0e7)
        & rdf["raw_pr_sigma_m"].between(0.0, args.max_pr_sigma)
        & (rdf["cn0_dbhz"] >= args.min_cn0)
    )
    rdf = rdf.loc[mask].copy()
    after_gate = len(rdf)
    if after_gate < 4:
        raise RuntimeError(f"Too few measurements after quality gate: {after_gate}/{before}")

    meas = glp.NavData(pandas_df=rdf)

    # Use broadcast RINEX ephemerides because this is a current-day experiment.
    # Precise SP3 products can lag and were unavailable for the latest UTC day
    # during the first CI attempt.
    full = glp.add_sv_states_rinex(meas)
    full["corr_pr_m"] = full["raw_pr_m"] + full["b_sv_m"]
    full["weights"] = 1.0 / np.maximum(full["raw_pr_sigma_m"], 1e-3) ** 2

    wls = glp.solve_wls(full, weight_type="weights")
    wdf = wls.pandas_df()

    xyz_cols = ["x_rx_wls_m", "y_rx_wls_m", "z_rx_wls_m"]
    for c in xyz_cols:
        if c not in wdf.columns:
            raise RuntimeError(f"WLS output missing {c}; available={list(wdf.columns)}")

    xyz = wdf[xyz_cols].to_numpy(dtype=float).T
    lla = glp.ecef_to_geodetic(xyz)
    wdf["lat_rx_wls_deg"] = lla[0, :]
    wdf["lon_rx_wls_deg"] = lla[1, :]
    wdf["alt_rx_wls_m"] = lla[2, :]

    # Remove epochs for which the solver did not converge to a finite position.
    good = np.isfinite(wdf["lat_rx_wls_deg"]) & np.isfinite(wdf["lon_rx_wls_deg"])
    wdf = wdf.loc[good].copy()
    if len(wdf) == 0:
        raise RuntimeError("No finite WLS epochs were produced")

    wls_summary, _, _, _ = local_dispersion(
        wdf["lat_rx_wls_deg"], wdf["lon_rx_wls_deg"]
    )

    # Half-to-half drift in the WLS series.
    n = len(wdf)
    half = n // 2
    drift = None
    if half >= 2:
        lat1 = float(np.median(wdf["lat_rx_wls_deg"].iloc[:half]))
        lon1 = float(np.median(wdf["lon_rx_wls_deg"].iloc[:half]))
        lat2 = float(np.median(wdf["lat_rx_wls_deg"].iloc[half:]))
        lon2 = float(np.median(wdf["lon_rx_wls_deg"].iloc[half:]))
        r = 6378137.0
        de = r * math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2.0))
        dn = r * math.radians(lat2 - lat1)
        drift = {"east_m": de, "north_m": dn, "distance_m": math.hypot(de, dn)}
    wls_summary["first_half_to_second_half_median_drift"] = drift
    wls_summary["epochs"] = len(wdf)

    # Android baseline from the same file, for a direct static repeatability
    # comparison. This is still repeatability, not absolute accuracy.
    fix_df = read_android_fixes(log_path)
    android_summary = {}
    if not fix_df.empty:
        android_summary, *_ = local_dispersion(
            fix_df["LatitudeDegrees"], fix_df["LongitudeDegrees"]
        )
        android_summary["epochs"] = int(len(fix_df))

    result = {
        "library": "gnss-lib-py",
        "method": "code-pseudorange WLS, GPS L1, broadcast RINEX ephemeris",
        "carrier_phase_used": False,
        "quality_gate": {
            "min_cn0_dbhz": args.min_cn0,
            "max_raw_pr_sigma_m": args.max_pr_sigma,
            "constellations": ["gps"],
            "band": "L1",
            "frequency_hz": [1.55e9, 1.60e9],
            "measurements_before": before,
            "measurements_after": after_gate,
        },
        "android_fix": android_summary,
        "wls": wls_summary,
    }

    wdf.to_csv(out_dir / "wls_solution.csv", index=False)
    (out_dir / "wls_summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")

    ar = android_summary.get("radial_from_median_m", {})
    wr = wls_summary.get("radial_from_median_m", {})
    lines = [
        "# Raw GNSS WLS baseline",
        "",
        "This is a **code/pseudorange** WLS baseline. ADR/carrier phase is not used.",
        "",
        f"- Raw measurements: **{before}**",
        f"- After GPS L1 quality gate: **{after_gate}**",
        f"- Finite WLS epochs: **{len(wdf)}**",
        "",
        "## Static repeatability",
        "",
        "| Metric | Android Fix | Raw WLS |",
        "|---|---:|---:|",
        f"| Radial P50 | {ar.get('p50', float('nan')):.3f} m | {wr.get('p50', float('nan')):.3f} m |",
        f"| Radial P95 | {ar.get('p95', float('nan')):.3f} m | {wr.get('p95', float('nan')):.3f} m |",
        f"| Radial P99 | {ar.get('p99', float('nan')):.3f} m | {wr.get('p99', float('nan')):.3f} m |",
        f"| Radial max | {ar.get('max', float('nan')):.3f} m | {wr.get('max', float('nan')):.3f} m |",
        "",
        f"WLS half-to-half median drift: **{(drift or {}).get('distance_m', float('nan')):.3f} m**",
        "",
        "> Dispersion about each solution's own median measures repeatability, not absolute accuracy. A surveyed ground-truth point is still required for absolute error.",
        "",
    ]
    (out_dir / "wls_summary.md").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))
    print("\n---JSON---")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
