"""Synthetic model checks only: these do not validate physical vehicle calibration."""
import math
import unittest

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.subaru.carcontroller import get_safety_CP
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR, CarControllerParams, SubaruFlags, SubaruSafetyFlags
from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor


class TestCrosstrekCalibration(unittest.TestCase):
  def test_model_selection(self):
    self.assertEqual(get_safety_CP().carFingerprint, CAR.SUBARU_ASCENT)
    for car in CAR:
      with self.subTest(car=car):
        cp = CarInterface.get_non_essential_params(car)
        selected = car == CAR.SUBARU_CROSSTREK_2025
        self.assertEqual(bool(cp.safetyConfigs[0].safetyParam & SubaruSafetyFlags.CROSSTREK_ANGLE_MODEL), selected)
        expected = CAR.SUBARU_CROSSTREK_2025 if selected else CAR.SUBARU_ASCENT
        self.assertEqual(get_safety_CP(car).carFingerprint, expected)
        if cp.flags & SubaruFlags.LKAS_ANGLE:
          controller = CarInterface(cp).CC
          expected_vm = VehicleModel(get_safety_CP(car))
          self.assertEqual(controller.VM.sR, expected_vm.sR)
          self.assertEqual(controller.VM.l, expected_vm.l)
          self.assertEqual(calc_slip_factor(controller.VM), calc_slip_factor(expected_vm))

  def test_experimental_model_parameters(self):
    vm = VehicleModel(get_safety_CP(CAR.SUBARU_CROSSTREK_2025))
    self.assertEqual(vm.sR, 13.0)
    self.assertAlmostEqual(vm.l, 2.67, places=6)
    self.assertAlmostEqual(calc_slip_factor(vm), -0.0006281955863217758, places=12)

  def test_envelope_does_not_expand(self):
    old = VehicleModel(get_safety_CP())
    new = VehicleModel(get_safety_CP(CAR.SUBARU_CROSSTREK_2025))
    # The zero-roll angle coefficient is ratio*wheelbase*(1 - slip*v^2).
    # Both coefficients decrease, proving the nominal inequality for all v >= 0.
    self.assertLess(new.sR * new.l, old.sR * old.l)
    self.assertLess(-calc_slip_factor(new) * new.sR * new.l, -calc_slip_factor(old) * old.sR * old.l)
    for speed_centi in range(13001):
      speed = max(speed_centi / 100, 1.0)
      for limit in (get_max_angle_vm, get_max_angle_delta_vm):
        old_bound, new_bound = (limit(speed, vm, CarControllerParams) for vm in (old, new))
        self.assertLessEqual(new_bound, old_bound)
        # Same CAN quantization, absolute cap, and per-message comfort cap.
        self.assertLessEqual(math.floor(new_bound * 100) + 1, math.floor(old_bound * 100) + 1)
        cap = (CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX if limit is get_max_angle_vm
               else CarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE)
        self.assertLessEqual(min(new_bound, cap), min(old_bound, cap))

  def test_ratio_change_is_material(self):
    cp = CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)
    new = VehicleModel(cp)
    cp.steerRatio = 17.0
    old = VehicleModel(cp)
    for speed in (1., 5., 10., 20., 30., 40.):
      self.assertAlmostEqual(new.get_steer_from_curvature(0.01, speed, 0.) / old.get_steer_from_curvature(0.01, speed, 0.), 13. / 17.)


if __name__ == "__main__":
  unittest.main()
