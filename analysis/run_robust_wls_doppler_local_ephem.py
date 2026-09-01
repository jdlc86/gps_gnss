#!/usr/bin/env python3
"""Same robust WLS/Doppler experiment, but forces local RINEX ephemeris.

This wrapper exists only to make CI deterministic and avoid gnss_lib_py's FTP
fallback. Estimator logic is reused from run_robust_wls_doppler.py.
"""
from pathlib import Path
import argparse
import json

import gnss_lib_py as glp
import numpy as np
import pandas as pd

from run_robust_wls_doppler import (
    position_solution, local_dispersion, half_drift, doppler_velocity, read_fix
)


def prepare(log_path, min_cn0, max_pr_sigma, ephemeris_path):
    raw = glp.AndroidRawGnss(log_path, filter_measurements=False, verbose=True)
    df = raw.pandas_df()
    before = len(df)
    num = ["gps_millis", "raw_pr_m", "raw_pr_sigma_m", "cn0_dbhz", "CarrierFrequencyHz",
           "PseudorangeRateMetersPerSecond", "PseudorangeRateUncertaintyMetersPerSecond"]
    for c in num:
        if c in df:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    required = ["gps_millis", "raw_pr_m", "raw_pr_sigma_m", "cn0_dbhz", "gnss_id", "CarrierFrequencyHz"]
    missing = [c for c in required if c not in df]
    if missing:
        raise RuntimeError(f"Missing parser rows: {missing}")
    mask = (
        (df["gnss_id"] == "gps")
        & df["CarrierFrequencyHz"].between(1.55e9, 1.60e9)
        & df["gps_millis"].notna()
        & df["raw_pr_m"].between(1e6, 6e7)
        & df["raw_pr_sigma_m"].between(0, max_pr_sigma)
        & (df["cn0_dbhz"] >= min_cn0)
    )
    df = df.loc[mask].copy()
    meas = glp.NavData(pandas_df=df)
    full = glp.add_sv_states_rinex(meas, ephemeris_path=str(ephemeris_path))
    full["corr_pr_m"] = full["raw_pr_m"] + full["b_sv_m"]
    full["weights"] = 1.0 / np.maximum(full["raw_pr_sigma_m"], 1e-3) ** 2
    return full, before, len(df)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", default="gnss_log.txt")
    ap.add_argument("--ephemeris-path", default="data/ephemeris")
    ap.add_argument("--out-dir", default="analysis/robust_output")
    ap.add_argument("--min-cn0", type=float, default=20.0)
    ap.add_argument("--max-pr-sigma", type=float, default=150.0)
    ap.add_argument("--max-faults", type=int, default=2)
    args = ap.parse_args()

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    full, before, gated = prepare(Path(args.log), args.min_cn0, args.max_pr_sigma, Path(args.ephemeris_path))

    base = position_solution(full)
    base_disp = local_dispersion(base.lat_deg, base.lon_deg)
    base_disp["first_half_to_second_half_median_drift"] = half_drift(base.lat_deg, base.lon_deg)

    fde = glp.solve_fde(full, method="residual", remove_outliers=True, max_faults=args.max_faults)
    after_fde = fde.num_cols
    robust = position_solution(fde)
    robust_disp = local_dispersion(robust.lat_deg, robust.lon_deg)
    robust_disp["first_half_to_second_half_median_drift"] = half_drift(robust.lat_deg, robust.lon_deg)

    vel_plain, vel_plain_summary = doppler_velocity(full, base)
    vel_fde, vel_fde_summary = doppler_velocity(fde, robust)

    fix = read_fix(args.log)
    android = {}
    if not fix.empty:
        android = local_dispersion(fix.LatitudeDegrees, fix.LongitudeDegrees)
        android["first_half_to_second_half_median_drift"] = half_drift(fix.LatitudeDegrees, fix.LongitudeDegrees)

    result = {
        "method": "GPS L1 local broadcast ephemeris; sigma-weighted WLS; residual FDE; Doppler velocity WLS",
        "raw_measurements_before": before,
        "gps_l1_after_gate": gated,
        "fde": {
            "method": "residual", "max_faults": args.max_faults,
            "measurements_after_fde": after_fde,
            "removed": gated - after_fde,
            "removed_fraction": (gated - after_fde) / gated if gated else None,
        },
        "android_fix": android,
        "plain_wls": base_disp,
        "residual_fde_wls": robust_disp,
        "doppler_velocity_plain": vel_plain_summary,
        "doppler_velocity_after_fde": vel_fde_summary,
    }
    (out / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    base.to_csv(out / "plain_wls.csv", index=False)
    robust.to_csv(out / "robust_wls.csv", index=False)
    vel_plain.to_csv(out / "doppler_velocity_plain.csv", index=False)
    vel_fde.to_csv(out / "doppler_velocity_fde.csv", index=False)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
