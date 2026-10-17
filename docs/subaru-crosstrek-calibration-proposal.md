# Experimental Crosstrek model calibration proposal

**Draft for review, not validated for vehicle deployment.** The physical steering ratio,
normalized tire stiffness, slip factor, and resulting acceleration are unverified.
Passing synthetic tests establishes software consistency, not road safety or correct
vehicle response. This proposal does not fix the separately investigated engagement
state/synchronization issue.

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
