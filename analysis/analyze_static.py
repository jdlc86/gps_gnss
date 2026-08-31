#!/usr/bin/env python3
"""Static GNSS baseline analysis for a GnssLogger-compatible log.

Usage:
    python analysis/analyze_static.py path/to/gnss_log.txt
    python analysis/analyze_static.py path/to/gnss_log.txt --truth-lat 40.0 --truth-lon -3.0

Without a surveyed/reference coordinate, the script reports dispersion around the
sample median. That is repeatability/precision, not absolute positioning error.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import pandas as pd

EARTH_RADIUS_M = 6_378_137.0
FIX_COLUMNS = [
    "record_type", "Provider", "LatitudeDegrees", "LongitudeDegrees", "AltitudeMeters",
    "SpeedMps", "AccuracyMeters", "BearingDegrees", "UnixTimeMillis", "SpeedAccuracyMps",
    "BearingAccuracyDegrees", "elapsedRealtimeNanos", "VerticalAccuracyMeters", "MockLocation",
]


def read_fixes(path: Path) -> pd.DataFrame:
    rows: list[list[str]] = []
    with path.open("r", encoding="utf-8") as handle:
        for row in csv.reader(handle):
            if row and row[0] == "Fix":
                rows.append(row[: len(FIX_COLUMNS)] + [""] * max(0, len(FIX_COLUMNS) - len(row)))
    if not rows:
        raise SystemExit("No GnssLogger Fix records found")
    df = pd.DataFrame(rows, columns=FIX_COLUMNS)
    for name in ("LatitudeDegrees", "LongitudeDegrees", "AccuracyMeters"):
        df[name] = pd.to_numeric(df[name], errors="coerce")
    return df.dropna(subset=["LatitudeDegrees", "LongitudeDegrees"]).copy()


def local_xy(lat_deg: np.ndarray, lon_deg: np.ndarray, lat0_deg: float, lon0_deg: float) -> tuple[np.ndarray, np.ndarray]:
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    x = EARTH_RADIUS_M * (lon - lon0) * np.cos((lat + lat0) / 2.0)
    y = EARTH_RADIUS_M * (lat - lat0)
    return x, y


def summarize(errors_m: np.ndarray, east_m: np.ndarray, north_m: np.ndarray) -> dict[str, float]:
    return {
        "samples": float(len(errors_m)),
        "mean_error_m": float(np.mean(errors_m)),
        "rmse_m": float(np.sqrt(np.mean(errors_m**2))),
        "p50_m": float(np.percentile(errors_m, 50)),
        "p95_m": float(np.percentile(errors_m, 95)),
        "max_m": float(np.max(errors_m)),
        "east_bias_m": float(np.mean(east_m)),
        "north_bias_m": float(np.mean(north_m)),
        "east_std_m": float(np.std(east_m, ddof=1)) if len(east_m) > 1 else 0.0,
        "north_std_m": float(np.std(north_m, ddof=1)) if len(north_m) > 1 else 0.0,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("gnss_log", type=Path)
    parser.add_argument("--truth-lat", type=float)
    parser.add_argument("--truth-lon", type=float)
    args = parser.parse_args()

    if (args.truth_lat is None) != (args.truth_lon is None):
        parser.error("--truth-lat and --truth-lon must be supplied together")

    df = read_fixes(args.gnss_log)
    if df.empty:
        raise SystemExit("No valid GNSS fixes found")

    if args.truth_lat is not None:
        ref_lat, ref_lon = args.truth_lat, args.truth_lon
        mode = "ABSOLUTE ERROR AGAINST PROVIDED REFERENCE"
    else:
        ref_lat = float(df["LatitudeDegrees"].median())
        ref_lon = float(df["LongitudeDegrees"].median())
        mode = "DISPERSION AROUND SAMPLE MEDIAN (NOT ABSOLUTE ACCURACY)"

    east, north = local_xy(
        df["LatitudeDegrees"].to_numpy(float),
        df["LongitudeDegrees"].to_numpy(float),
        ref_lat,
        ref_lon,
    )
    horizontal = np.hypot(east, north)
    stats = summarize(horizontal, east, north)

    print(mode)
    print(f"Reference: lat={ref_lat:.10f}, lon={ref_lon:.10f}")
    for key, value in stats.items():
        if key == "samples":
            print(f"{key:18s}: {int(value)}")
        else:
            print(f"{key:18s}: {value:.3f}")

    reported = df["AccuracyMeters"].dropna().to_numpy(float)
    if len(reported):
        print("\nAndroid reported horizontal accuracy (metadata, not ground truth):")
        print(f"median_accuracy_m : {np.median(reported):.3f}")
        print(f"p95_accuracy_m    : {np.percentile(reported, 95):.3f}")


if __name__ == "__main__":
    main()
