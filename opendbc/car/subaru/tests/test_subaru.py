import unittest

from opendbc.car import structs
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR, SubaruFlags, SubaruSafetyFlags
from opendbc.car.structs import CarParams


class TestSubaruFingerprint(unittest.TestCase):
  def test_fw_version_format(self):
    for platform, fws_per_ecu in FW_VERSIONS.items():
      for (ecu, _, _), fws in fws_per_ecu.items():
        fw_size = len(fws[0])
        for fw in fws:
          assert len(fw) == fw_size, f"{platform} {ecu}: {len(fw)} {fw_size}"


class TestSubaruCrosstrek2025(unittest.TestCase):
  @staticmethod
  def _controller_angle(speed, target_angle, lat_active=True, steering_angle=0):
    CP = CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)
    CI = CarInterface(CP)
    CI.update([])
    CI.CS.out.vEgoRaw = speed
    CI.CS.out.steeringAngleDeg = steering_angle

    CC = structs.CarControl()
    CC.latActive = lat_active
    CC.actuators.steeringAngleDeg = target_angle
    actuators, _ = CI.apply(CC.as_reader(), 0)
    return actuators.steeringAngleDeg

  def test_interface_configuration(self):
    CP = CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)

    self.assertFalse(CP.dashcamOnly)
    self.assertEqual(CP.steerControlType, CarParams.SteerControlType.angle)
    self.assertTrue(CP.flags & SubaruFlags.GLOBAL_GEN2)
    self.assertTrue(CP.flags & SubaruFlags.LKAS_ANGLE)
    self.assertTrue(CP.safetyConfigs[0].safetyParam & SubaruSafetyFlags.GEN2)
    self.assertTrue(CP.safetyConfigs[0].safetyParam & SubaruSafetyFlags.LKAS_ANGLE)
    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertAlmostEqual(CP.wheelbase, 2.67, places=6)
    self.assertAlmostEqual(CP.steerRatio, 17, places=6)

  def test_low_speed_angle_deadzone(self):
    # Below 2 m/s np.interp clamps the deadzone to 6 degrees.
    self.assertAlmostEqual(self._controller_angle(0, 5.99), 0)
    self.assertAlmostEqual(self._controller_angle(2, 6.01), 5, places=6)

    # The deadzone linearly decreases to 3 degrees just below 10 m/s.
    self.assertAlmostEqual(self._controller_angle(9, 3.37), 0)
    self.assertGreater(self._controller_angle(9, 3.38), 0)

    # It is disabled at 10 m/s, and inactive control follows measured angle.
    self.assertAlmostEqual(self._controller_angle(10, 1), 1, places=6)
    self.assertAlmostEqual(self._controller_angle(5, 190, lat_active=False, steering_angle=37), 37, places=6)
