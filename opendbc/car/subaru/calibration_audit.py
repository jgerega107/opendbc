"""Offline diagnostics for Crosstrek angle limiting; never a physical calibration approval."""
import argparse
import json
import math
from collections import Counter
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np

from opendbc.car.lateral import get_max_angle_vm
from opendbc.car.subaru.carcontroller import get_safety_CP
from opendbc.car.subaru.values import CAR, CarControllerParams, SubaruSafetyFlags
from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor


@dataclass(frozen=True)
class Criteria:
  # Diagnostic tolerances, not vehicle acceptance or physical safety limits.
  min_speed: float = 10.0  # the low-speed command filter is inactive above this speed
  max_age: float = 0.15  # dynamic signals/output gaps; never bridge a missing segment
  max_parameter_age: float = 0.35  # slowly learned parameters are sampled at 4 Hz in qlogs
  angle_tolerance: float = 0.25  # degrees; accommodate sampling and CAN quantization
  model_difference: float = 0.10  # fractional difference in zero-roll angle/curvature gain
  persistence: float = 0.5  # seconds, excludes isolated/transient observations
  min_duration: float = 1.0  # seconds in at least one continuous eligible interval


DEFAULT_CRITERIA = Criteria()


@dataclass(frozen=True)
class Sample:
  time: float
  speed: float
  desired_angle: float
  output_angle: float
  measured_angle: float
  steer_ratio: float
  stiffness_factor: float
  eligible: bool = True
  exclusion: str = ""


def samples_from_events(events, criteria=DEFAULT_CRITERIA):
  """Join preceding signals at carOutput time, rejecting stale/invalid data.

  Each event is (time_seconds, service_name, valid, payload_dict), ordered by time.
  No interpolation or future samples are used. Invalid output samples also break runs.
  """
  latest = {}
  previous = -math.inf
  for time, service, valid, payload in events:
    if time < previous:
      raise ValueError("Logs must be supplied in chronological order without overlapping segments")
    previous = time
    latest[service] = (time, valid, payload)
    if service != "carOutput":
      continue

    reason = ""
    required = ("carState", "carControl", "vehicleParameters", "carOutput")
    if not all(name in latest for name in required):
      reason = "missing_signals"
    elif any(time - latest[name][0] > (criteria.max_parameter_age if name == "vehicleParameters" else criteria.max_age)
             for name in required):
      reason = "stale_signals"
    elif not all(latest[name][1] for name in required):
      reason = "invalid_messages"
    if reason:
      yield Sample(time, 0, 0, 0, 0, 0, 0, False, reason)
      continue

    cs, cc, vp = (latest[name][2] for name in required[:3])
    try:
      sample = Sample(time, cs["vEgoRaw"], cc["actuators"]["steeringAngleDeg"],
                      payload["actuatorsOutput"]["steeringAngleDeg"], cs["steeringAngleDeg"],
                      vp["steerRatio"], vp["stiffnessFactor"])
      if not cc["latActive"] or cs["steeringPressed"]:
        reason = "inactive_or_driver_override"
      elif not cs["canValid"] or cs["steerFaultTemporary"] or cs["steerFaultPermanent"]:
        reason = "car_fault"
      elif not all(vp[name] for name in ("valid", "sensorValid", "steerRatioValid", "stiffnessFactorValid")):
        reason = "invalid_parameters"
      elif not all(math.isfinite(value) for value in asdict(sample).values() if isinstance(value, (int, float))):
        reason = "nonfinite_values"
      elif sample.steer_ratio <= 0 or sample.stiffness_factor <= 0:
        reason = "invalid_parameters"
      elif sample.speed < criteria.min_speed:
        reason = "low_speed"
    except (KeyError, TypeError):
      sample = Sample(time, 0, 0, 0, 0, 0, 0)
      reason = "missing_fields"
    yield Sample(**(asdict(sample) | {"eligible": not reason, "exclusion": reason}))


def intervals(rows, flag, criteria):
  runs = []
  start = end = None
  for row in rows:
    time = row["time"]
    if not row.get(flag, False) or (end is not None and time - end > criteria.max_age):
      if start is not None and end - start >= criteria.persistence:
        runs.append({"start": start, "end": end, "duration": end - start})
      start = end = None
    if row.get(flag, False):
      start = time if start is None else start
      end = time
  if start is not None and end - start >= criteria.persistence:
    runs.append({"start": start, "end": end, "duration": end - start})
  return runs


def summary(values):
  if not values:
    return None
  return {"min": min(values), "median": float(np.median(values)), "p95": float(np.percentile(values, 95)), "max": max(values)}


def audit_samples(samples, criteria=DEFAULT_CRITERIA):
  fixed = VehicleModel(get_safety_CP(CAR.SUBARU_CROSSTREK_2025))
  learned = VehicleModel(get_safety_CP(CAR.SUBARU_CROSSTREK_2025))
  rows, excluded = [], Counter()
  eligible_duration = 0.0
  previous = None
  for sample in samples:
    row = {"time": sample.time, "eligible": sample.eligible}
    if sample.eligible:
      if previous is not None and previous.eligible and 0 <= sample.time - previous.time <= criteria.max_age:
        eligible_duration += sample.time - previous.time
      learned.update_params(sample.stiffness_factor, sample.steer_ratio)
      # Compare the two models at the SAME speed, zero roll and arbitrary unit curvature.
      # This is model disagreement, not a measurement of the physical steering ratio.
      gain_ratio = learned.get_steer_from_curvature(1.0, sample.speed, 0) / fixed.get_steer_from_curvature(1.0, sample.speed, 0)
      bound = min(get_max_angle_vm(sample.speed, fixed, CarControllerParams), CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX)
      row |= asdict(sample) | {
        "fixed_angle_bound": bound,
        "learned_to_fixed_gain": gain_ratio,
        "desired_output_gap": abs(sample.desired_angle - sample.output_angle),
        "output_measured_error": abs(sample.output_angle - sample.measured_angle),
        "model_disagreement": abs(gain_ratio - 1) > criteria.model_difference,
        "fixed_limit_clipping": (abs(sample.desired_angle) > bound + criteria.angle_tolerance and
                                 abs(sample.output_angle - math.copysign(bound, sample.desired_angle)) <= criteria.angle_tolerance),
      }
    else:
      excluded[sample.exclusion] += 1
    rows.append(row)
    previous = sample
  clipping = intervals(rows, "fixed_limit_clipping", criteria)
  disagreement = intervals(rows, "model_disagreement", criteria)
  eligible_runs = intervals(rows, "eligible", replace(criteria, persistence=criteria.min_duration))
  eligible = [row for row in rows if row["eligible"]]
  status = "insufficient_data" if not eligible_runs else (
    "needs_review" if clipping or disagreement else "no_diagnostic_trigger")
  return {
    "status": status,
    "interpretation": "Diagnostic evidence only; no physical calibration or road-safety approval, and no proposed limit increase.",
    "criteria": asdict(criteria),
    "fixed_model": {"steer_ratio": fixed.sR, "wheelbase": fixed.l, "slip_factor": calc_slip_factor(fixed)},
    "samples": len(rows), "eligible_samples": len(eligible), "eligible_duration": eligible_duration, "excluded": dict(excluded),
    "eligible_intervals_at_least_min_duration": eligible_runs,
    "statistics": {key: summary([row[key] for row in eligible]) for key in (
      "steer_ratio", "stiffness_factor", "learned_to_fixed_gain", "desired_output_gap", "output_measured_error")},
    "persistent_fixed_limit_clipping": clipping,
    "persistent_model_disagreement": disagreement,
    "largest_clipped_gaps": sorted((row for row in eligible if row["fixed_limit_clipping"]),
                                   key=lambda row: row["desired_output_gap"], reverse=True)[:5],
  }


def validate_car_params(cp):
  expected = get_safety_CP(CAR.SUBARU_CROSSTREK_2025)
  fields = ("steerRatio", "wheelbase", "centerToFront", "tireStiffnessFrontPerMass", "tireStiffnessRearPerMass")
  model_flags = SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LKAS_ANGLE | SubaruSafetyFlags.CROSSTREK_ANGLE_MODEL
  if cp.carFingerprint != CAR.SUBARU_CROSSTREK_2025 or not any(
    cfg.safetyModel == expected.safetyConfigs[0].safetyModel and cfg.safetyParam & model_flags == model_flags for cfg in cp.safetyConfigs
  ) or not all(math.isclose(getattr(cp, field), getattr(expected, field), rel_tol=1e-5) for field in fields):
    raise ValueError("Recorded CarParams do not select the audited Crosstrek model; use the matching opendbc revision")


def local_log_events(paths):
  # opendbc.car.logreader's CAN-only schema omits these services. Use the matching
  # openpilot schema, available in an openpilot environment/PYTHONPATH, without
  # copying a changing cereal schema into opendbc or modifying import paths here.
  try:
    from openpilot.tools.lib.logreader import LogReader
  except ModuleNotFoundError as exc:
    raise RuntimeError("Run in the matching openpilot Python environment; its full log schema is required") from exc

  origin = None
  for path in paths:
    reader = LogReader(str(path), sort_by_time=True)
    car_params = list(reader.filter("carParams"))
    if not car_params:
      raise ValueError(f"No CarParams in {path}")
    # CarParams may be published after the segment starts. Validate its recorded
    # configuration before analyzing any telemetry, without using future signals.
    for cp in car_params:
      validate_car_params(cp)
    for msg in reader:
      time = msg.logMonoTime / 1e9
      origin = time if origin is None else origin
      service = msg.which()
      if service in ("carState", "carControl", "carOutput", "vehicleParameters"):
        yield time - origin, service, msg.valid, getattr(msg, service).to_dict()


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("logs", nargs="+", type=Path, help="Local rlog/qlog files, in chronological order (no downloads)")
  args = parser.parse_args()
  if any(not path.is_file() for path in args.logs):
    parser.error("Every input must be an existing local log file")
  try:
    report = audit_samples(samples_from_events(local_log_events(args.logs)))
  except (RuntimeError, ValueError) as exc:
    parser.error(str(exc))
  print(json.dumps(report, indent=2, allow_nan=False))
  return {"no_diagnostic_trigger": 0, "needs_review": 1, "insufficient_data": 2}[report["status"]]


if __name__ == "__main__":
  raise SystemExit(main())
