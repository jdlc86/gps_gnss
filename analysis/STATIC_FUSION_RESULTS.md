# Static GNSS/Doppler/IMU fusion results

Dataset: `gnss_log.txt`, phone stationary for about 584 s.

These metrics measure repeatability around each solution's own median. They are **not absolute positioning accuracy** because this test has no surveyed ground-truth coordinate.

## Results

| Static repeatability | Android Fix | Position + Doppler KF | Doppler-only ZUPT | IMU + Doppler ZUPT |
|---|---:|---:|---:|---:|
| P50 radial (m) | 0.436 | **0.397** | 0.608 | 0.816 |
| P95 radial (m) | **2.866** | 3.130 | 3.147 | 2.913 |
| P99 radial (m) | 3.500 | 3.572 | 3.351 | **3.124** |
| Half-to-half drift (m) | 1.394 | **1.295** | 1.593 | 1.935 |

Validated Raw-GNSS Doppler horizontal speed while stationary:

- P50: 0.041 m/s
- P95: 0.247 m/s
- max: 1.170 m/s

Detector behavior:

- Doppler-only STOP: 9 stop entries, 220 ZUPT updates. It oscillates too much for a truly stationary phone.
- IMU-assisted hysteretic STOP: 1 entry, 0 exits, 280 ZUPT updates. Classification is much more stable.

## Interpretation

1. Raw Doppler is a useful observable: it is substantially cleaner than the GPS-L1 pseudorange WLS position baseline.
2. A simple constant-velocity Kalman filter improves P50 and half-to-half drift slightly, but worsens P95/P99. It is therefore not an overall positioning improvement.
3. Doppler-only ZUPT is rejected: its stop detector is unstable and the position metrics worsen.
4. IMU-assisted hysteresis correctly recognizes the stationary interval much more consistently and improves P99 (3.500 -> 3.124 m), but worsens P50 and drift and does not beat Android at P95 (2.913 vs 2.866 m).
5. There is no justification for further tuning on this one static log. Doing so would risk overfitting thresholds and covariance parameters to this phone/session.

## Decision gate

The next required dataset is a **moving-vehicle log** containing, in one continuous recording:

1. 30-60 s stationary start.
2. Straight motion.
3. At least one left and one right turn.
4. Low-speed maneuvering representative of a parking aisle.
5. A complete parking maneuver.
6. 30-60 s stationary end.

Preferably repeat exactly the same route at least 3 times. A few physically identifiable points or a known route geometry should be available for comparison even if centimeter-level ground truth is not.

The next algorithmic evaluation should compare Android Fix, Android+Raw Doppler, and GNSS+Doppler+gyro/accel using lateral/longitudinal trajectory error, stop detection, heading consistency and repeatability across runs. Only after that test should a full vehicle-state EKF be promoted as the main approach.
