# Experimental Crosstrek model calibration proposal

**Draft for review, not validated for vehicle deployment.** The physical steering ratio,
normalized tire stiffness, slip factor, and resulting acceleration are unverified.
Passing synthetic tests establishes software consistency, not road safety or correct
vehicle response. This proposal does not fix the separately investigated engagement
state/synchronization issue.

## Reproduce the model and clipping diagnostic

`opendbc.car.subaru.calibration_audit` examines local logs without changing the
controller, firmware, calibration, or safety limits. Run it in the matching
openpilot Python environment, with this opendbc checkout installed or on
`PYTHONPATH`:

```bash
python -m opendbc.car.subaru.calibration_audit \
  /path/to/route/63/rlog.zst /path/to/route/66/rlog.zst > calibration-report.json
```

Supply files in chronological order. The CLI does not download logs. The full
openpilot log reader is required because opendbc's minimal CAN schema does not
expose `carState`, `carControl`, `carOutput`, and `vehicleParameters`. The reported
times are seconds from the earliest event in the first supplied file; when the
file contains the route's original `initData`, these are route-relative times.
Recorded CarParams must select the Crosstrek angle model and match this checkout's
ratio, geometry, and normalized tire stiffness. Use the matching revisions when
auditing an older or newer drive.

The report separates two observations:

- **Model disagreement:** compare the learned and fixed model's steering-angle
  gain at the same speed, zero roll and zero angle offset. A difference above 10%
  lasting at least 0.5 seconds is a diagnostic trigger. The learned ratio is an
  estimator result, not independent physical ground truth.
- **Fixed-limit clipping:** the requested angle is more than 0.25 degrees outside
  the fixed model's angle bound while the controller output is within 0.25 degrees
  of that bound, with the same sign, for at least 0.5 seconds. Report the
  desired-to-output gap separately from output-to-measured tracking error. A large
  requested angle alone, or an output still far from the bound, does not establish
  fixed-angle clipping.

These thresholds select evidence for review; they are not physical acceptance
criteria. Only active lateral control without driver override, valid CAN and
learned parameters, speed of at least 10 m/s, and dynamic signals no older than 150 ms are
eligible. Slowly learned `vehicleParameters` may be up to 350 ms old to accommodate
their 4 Hz qlog sampling; dynamic signals and output gaps retain the 150 ms limit.
The tool joins preceding signals at each output timestamp. Missing,
invalid, stale, or override samples break persistent intervals; absent segments
are never counted as continuous evidence. At least one continuous eligible second
is required; adding many short, disconnected fragments cannot pass the coverage
check. The low-speed filter is out of scope.
Tracking error is a contemporaneous comparison, not an identified actuator delay.

The CLI exits **1** for `needs_review`, **2** for insufficient eligible data or
invalid input, and **0** for `no_diagnostic_trigger`. No result certifies a
physical calibration or road safety. Qlogs can be used for this sampled diagnostic;
full-rate rlogs are needed to replay CAN/safety and inspect transition timing.

On the audited 0.11.2/`510e9a6d8` drive with opendbc `df8411ee`, full-rate segments
63 and 66 supply 111.83 eligible seconds. The learned-to-fixed gain median is
1.37185. Persistent fixed-limit clipping appears at 3790.515–3791.928 seconds and
3977.606–3979.486 seconds. At the largest clipped gap, the desired-to-output
difference is 5.120 degrees while output-to-measured error is 0.023 degrees.
This supports investigating the model/limit interaction; it does not establish
that increasing the fixed ratio or safety envelope would be physically correct.
Segments 33–35 provide an additional comparison around the earlier bookmark.
The same two qlog segments also trigger review and retain both clipping windows
(1.20 and 1.80 seconds at the lower sample rate), with 111.30 eligible seconds.

The committed numerical fixture contains rounded observations from the two
clipping windows, repeated at synthetic timestamps in tests. It includes no raw
route, location, VIN, video, or CAN capture. It checks that the diagnostic detects
this combination of clipping and close actuator tracking, while rejecting stale,
invalid, brief, or driver-override evidence. It is not a replacement for a retained
full-rate route fixture or controller-to-safety replay.

Before changing calibration, collect independent steering-angle/unit/ratio
evidence for this trim and EPS, validate the learned model's offset, stiffness and
roll assumptions, and reproduce tracking across speeds and both turn directions.
Any accepted model change must update Python and embedded safety together and
pass full-rate replay. One drive's estimated ratio is insufficient justification
to expand steering authority.

## Provenance and scope

Candidate adapted from [jacobwaller/opendbc at 753fdd3](https://github.com/jacobwaller/opendbc/tree/753fdd383a18c3f9e86af801f1e0e9db2a168e72),
including [the ratio change in b81a9b7](https://github.com/jacobwaller/opendbc/commit/b81a9b7c38866a4935c0e24e8805078033b07a91).
Those source changes propose ratio 13.0 instead of 17.0. Their presence is not evidence
that the ratio is physically correct for every 2025 Crosstrek trim or EPS version.

Only `SUBARU_CROSSTREK_2025` selects the new model. Unlike the source's global
replacement, all other angle ports retain the Ascent limiting model. Torque ports,
CAN permissions, debug gates, fingerprints, and numeric acceleration/jerk constants
are unchanged. The new safety parameter flag value 16 (1 << 4) is active only with GEN2 and LKAS_ANGLE.
Without those prerequisite bits, it cannot enable angle control or select this model.

The Python and C limiters use the same candidate model:

| Parameter | Existing limiter (Ascent) | Candidate (Crosstrek) |
| --- | ---: | ---: |
| Steering ratio | 13.5 | 13.0 |
| Wheelbase (m) | 2.89 | 2.67 |
| Slip factor | -0.000580374471400815 | -0.0006281955863217758 |

The candidate slip factor is recomputed from this repository's mass-normalized
Python model. Its tiny precision difference from the source branch reflects the
current schema representation; boundary tests compare Python results to freshly
compiled C hooks, including CAN speed and angle quantization.

## Material behavior changes

1. CarParams steering ratio changes from 17 to 13. With the remaining Crosstrek
   parameters fixed, converting a fixed curvature to steering angle yields 13/17
   of the former angle, a **23.53% reduction**. This can materially change lateral
   tracking and is not justified by a passing model test.
2. The limiter's nominal angle and angle-rate envelope tightens relative to the
   Ascent limiter. This does **not** demonstrate the real vehicle remains within
   physical acceleration or jerk limits; that requires correct physical parameters.

The unchanged limits in this branch use 3.0 + 9.81*0.06 = 3.5886 for lateral
acceleration (m/s²) and jerk (m/s³). At a common model speed, raw nominal bounds are:

| Model speed (m/s) | Old max angle (deg) | New max angle (deg) | Old delta/20 ms (deg) | New delta/20 ms (deg) |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 8026.5939 | 7141.2633 | 160.5319 | 142.8253 |
| 5 | 325.5333 | 289.9545 | 6.5107 | 5.7991 |
| 10 | 84.8751 | 75.8511 | 1.6975 | 1.5170 |
| 20 | 24.7106 | 22.3252 | 0.4942 | 0.4465 |
| 30 | 13.5690 | 12.4130 | 0.2714 | 0.2483 |
| 40 | 9.6694 | 8.9438 | 0.1934 | 0.1789 |

These are pre-cap model values, not necessarily transmittable commands. Python uses
max(raw speed, 1 m/s), then its existing 190-degree absolute cap and 5-degree
per-message comfort cap. Safety uses max(minimum sampled speed - 1 m/s, 1 m/s),
its existing limits and integer rounding (+1 CAN unit); its low-speed behavior is
not identical to Python's. The table compares like-for-like model inputs, not a
single raw vehicle speed across the two implementations.

For nonnegative speed v, the zero-roll angle coefficient is
ratio * wheelbase * (1 - slip * v²). The old coefficients are approximately
39.0150014 + 0.02264331*v²; the new ones are 34.7100010 + 0.02180467*v².
Both decrease, proving the ideal nominal envelope cannot expand at any speed.
The tests also sweep 0–130 m/s in 0.01 m/s increments through Python's existing
caps and CAN-unit rounding. C-hook boundary tests cover both angle signs and
quantized speed with the new flag and retain the existing-model suites.

## Review and integration gates

- Obtain independent ratio/calibration evidence for the exact vehicle/trim/EPS;
  check steering-angle signal units, wheelbase, stiffness and model assumptions.
- Review tracking consequences separately from limiter envelope changes.
- Review the new safety parameter allocation and C/Python handshake together.
- Rebuild and validate the actual Panda/safety firmware against this header.
  Python-only installation does not update embedded safety. Older firmware ignores
  flag value 16 and keeps the Ascent envelope, so it does not enforce the candidate bounds.
- Review any parent openpilot/opendbc submodule pin separately; this PR does not
  update a deployment pin, build/flash a vehicle, merge, or enable auto-merge.
- Retain the draft until software review and physical calibration validation are
  complete. Bench/synthetic success alone is insufficient for road deployment.
- Coordinate review/merge ordering with the separate transition-test and engagement
  drafts: overlapping Subaru tests/controller lines may need conflict resolution.
  Re-run combined suites on the resolved tree rather than assuming independent
  branch results establish integration correctness.
