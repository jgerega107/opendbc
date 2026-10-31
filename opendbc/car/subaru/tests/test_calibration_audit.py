import json
import unittest
from dataclasses import replace
from pathlib import Path

from opendbc.car.subaru.calibration_audit import Sample, audit_samples, samples_from_events, validate_car_params
from opendbc.car.subaru.carcontroller import get_safety_CP
from opendbc.car.subaru.values import CAR, SubaruSafetyFlags


def events_for_sample(sample):
  cs = {"vEgoRaw": sample.speed, "steeringAngleDeg": sample.measured_angle, "steeringPressed": False,
        "canValid": True, "steerFaultTemporary": False, "steerFaultPermanent": False}
  cc = {"latActive": True, "actuators": {"steeringAngleDeg": sample.desired_angle}}
  vp = {"steerRatio": sample.steer_ratio, "stiffnessFactor": sample.stiffness_factor,
        "valid": True, "sensorValid": True, "steerRatioValid": True, "stiffnessFactorValid": True}
  co = {"actuatorsOutput": {"steeringAngleDeg": sample.output_angle}}
  return [(sample.time, name, True, data) for name, data in
          (("carState", cs), ("carControl", cc), ("vehicleParameters", vp), ("carOutput", co))]


class TestCalibrationAudit(unittest.TestCase):
  def setUp(self):
    self.cases = json.loads(Path(__file__).with_name("crosstrek_calibration_cases.json").read_text())["cases"]

  @staticmethod
  def plateau(sample, length=21):
    return [replace(sample, time=i / 10) for i in range(length)]

  def test_observed_clipping_and_small_tracking_error(self):
    for case in self.cases:
      for direction in (-1, 1):
        with self.subTest(case=case, direction=direction):
          sample = Sample(time=0, **case)
          sample = replace(sample, desired_angle=sample.desired_angle * direction,
                           output_angle=sample.output_angle * direction, measured_angle=sample.measured_angle * direction)
          report = audit_samples(self.plateau(sample))
          self.assertEqual(report["status"], "needs_review")
          self.assertEqual(len(report["persistent_fixed_limit_clipping"]), 1)
          self.assertAlmostEqual(report["persistent_fixed_limit_clipping"][0]["duration"], 2.0)
          self.assertGreater(report["statistics"]["desired_output_gap"]["median"], 3.7)
          self.assertLess(report["statistics"]["output_measured_error"]["max"], 0.05)
          self.assertGreater(report["statistics"]["learned_to_fixed_gain"]["median"], 1.35)

  def test_consistent_model_and_unlimited_tracking(self):
    sample = Sample(0, 30, 2, 2, 2, 13, 1)
    report = audit_samples(self.plateau(sample))
    self.assertEqual(report["status"], "no_diagnostic_trigger")
    self.assertEqual(report["statistics"]["learned_to_fixed_gain"]["median"], 1)
    self.assertFalse(report["persistent_fixed_limit_clipping"])
    self.assertFalse(report["persistent_model_disagreement"])

  def test_model_and_clipping_are_separate_diagnostics(self):
    sample = Sample(0, **self.cases[0])
    report = audit_samples(self.plateau(replace(sample, steer_ratio=13, stiffness_factor=1)))
    self.assertTrue(report["persistent_fixed_limit_clipping"])
    self.assertFalse(report["persistent_model_disagreement"])
    report = audit_samples(self.plateau(replace(sample, desired_angle=2, output_angle=2, measured_angle=2)))
    self.assertFalse(report["persistent_fixed_limit_clipping"])
    self.assertTrue(report["persistent_model_disagreement"])

  def test_output_far_from_bound_is_not_called_clipping(self):
    # A tracking fault or a jerk-limited transient cannot establish angle-cap clipping.
    sample = replace(Sample(0, **self.cases[0]), output_angle=0)
    self.assertFalse(audit_samples(self.plateau(sample))["persistent_fixed_limit_clipping"])

  def test_short_or_gapped_observations_do_not_make_a_persistent_run(self):
    sample = Sample(0, **self.cases[0])
    short = self.plateau(sample, length=4)
    report = audit_samples(short + [replace(s, time=s.time + 10) for s in short])
    self.assertEqual(report["status"], "insufficient_data")
    self.assertAlmostEqual(report["eligible_duration"], 0.6)
    self.assertFalse(report["persistent_fixed_limit_clipping"])
    self.assertFalse(report["persistent_model_disagreement"])

  def test_invalid_observation_breaks_persistence(self):
    samples = self.plateau(Sample(0, **self.cases[0]), length=9)
    samples[4] = replace(samples[4], eligible=False, exclusion="invalid_messages")
    report = audit_samples(samples)
    self.assertFalse(report["persistent_fixed_limit_clipping"])
    self.assertEqual(report["excluded"], {"invalid_messages": 1})

  def test_fragmented_coverage_is_insufficient_even_if_total_exceeds_one_second(self):
    samples = self.plateau(Sample(0, 30, 2, 2, 2, 13, 1), length=30)
    samples = [replace(s, eligible=False, exclusion="stale_signals") if i % 4 == 3 else s for i, s in enumerate(samples)]
    report = audit_samples(samples)
    self.assertGreater(report["eligible_duration"], 1)
    self.assertEqual(report["status"], "insufficient_data")

  def test_qlog_parameter_frequency_and_missing_parameter_gap(self):
    sample = Sample(0, **self.cases[0])
    # Qlogs retain dynamic signals at 10 Hz but vehicleParameters at only 4 Hz.
    dynamic = [event for s in self.plateau(sample, length=41) for event in events_for_sample(s) if event[1] != "vehicleParameters"]
    parameters = [events_for_sample(replace(sample, time=i / 4))[2] for i in range(17)]
    events = sorted(dynamic + parameters, key=lambda event: event[0])
    report = audit_samples(samples_from_events(events))
    self.assertEqual(report["status"], "needs_review")
    self.assertNotIn("stale_signals", report["excluded"])
    self.assertEqual(len(report["persistent_fixed_limit_clipping"]), 1)
    self.assertGreater(report["persistent_fixed_limit_clipping"][0]["duration"], 3.5)

    # The slower freshness allowance must still expire if parameter updates stop.
    parameters = [event for event in parameters if not 1 <= event[0] < 2]
    report = audit_samples(samples_from_events(sorted(dynamic + parameters, key=lambda event: event[0])))
    self.assertGreater(report["excluded"]["stale_signals"], 0)
    self.assertEqual(len(report["persistent_fixed_limit_clipping"]), 2)
    self.assertLess(report["persistent_fixed_limit_clipping"][0]["end"], 1.2)
    self.assertGreaterEqual(report["persistent_fixed_limit_clipping"][1]["start"], 2)

  def test_join_uses_preceding_fresh_valid_messages(self):
    sample = Sample(0, **self.cases[0])
    events = events_for_sample(sample)
    self.assertEqual(list(samples_from_events(events)), [sample])
    missing = list(samples_from_events(events[:2] + events[3:]))[0]
    self.assertEqual(missing.exclusion, "missing_signals")
    # The next output cannot reuse old parameters or measurements across a gap.
    stale = events + [(1, *events[-1][1:])]
    self.assertEqual(list(samples_from_events(stale))[-1].exclusion, "stale_signals")
    events[2] = (0, "vehicleParameters", False, events[2][3])
    self.assertEqual(list(samples_from_events(events))[0].exclusion, "invalid_messages")
    with self.assertRaisesRegex(ValueError, "chronological"):
      list(samples_from_events([(1, *events[0][1:]), *events[1:]]))

  def test_exclude_override_faults_invalid_parameters_and_low_speed(self):
    for service, key, value, reason in (
      ("carState", "steeringPressed", True, "inactive_or_driver_override"),
      ("carControl", "latActive", False, "inactive_or_driver_override"),
      ("carState", "canValid", False, "car_fault"),
      ("carState", "steerFaultTemporary", True, "car_fault"),
      ("vehicleParameters", "steerRatioValid", False, "invalid_parameters"),
      ("vehicleParameters", "steerRatio", 0, "invalid_parameters"),
      ("vehicleParameters", "stiffnessFactor", float("nan"), "nonfinite_values"),
      ("carState", "vEgoRaw", 5, "low_speed"),
    ):
      with self.subTest(service=service, key=key):
        events = events_for_sample(Sample(0, **self.cases[0]))
        next(event[3] for event in events if event[1] == service)[key] = value
        sample = list(samples_from_events(events))[0]
        self.assertFalse(sample.eligible)
        self.assertEqual(sample.exclusion, reason)

  def test_empty_data_does_not_pass(self):
    report = audit_samples([])
    self.assertEqual(report["status"], "insufficient_data")
    self.assertIsNone(report["statistics"]["steer_ratio"])

  def test_reject_wrong_platform_or_recorded_model(self):
    validate_car_params(get_safety_CP(CAR.SUBARU_CROSSTREK_2025))
    with self.assertRaisesRegex(ValueError, "matching opendbc revision"):
      validate_car_params(get_safety_CP())
    for field in ("steerRatio", "wheelbase", "centerToFront", "tireStiffnessFrontPerMass", "tireStiffnessRearPerMass"):
      with self.subTest(field=field):
        cp = get_safety_CP(CAR.SUBARU_CROSSTREK_2025)
        setattr(cp, field, getattr(cp, field) * 1.1)
        with self.assertRaises(ValueError):
          validate_car_params(cp)
    cp = get_safety_CP(CAR.SUBARU_CROSSTREK_2025)
    cp.safetyConfigs[0].safetyParam = SubaruSafetyFlags.CROSSTREK_ANGLE_MODEL.value
    with self.assertRaises(ValueError):
      validate_car_params(cp)


if __name__ == "__main__":
  unittest.main()
