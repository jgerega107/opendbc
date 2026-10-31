import unittest

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.can_definitions import CanData
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
  def _new_controller():
    CP = CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)
    CI = CarInterface(CP)
    CI.update([])
    return CI

  @staticmethod
  def _apply_controller_angle(CI, speed, target_angle, lat_active=True, steering_angle=0, steering_pressed=False):
    CI.CS.out.vEgoRaw = speed
    CI.CS.out.steeringAngleDeg = steering_angle
    CI.CS.out.steeringPressed = steering_pressed

    CC = structs.CarControl()
    CC.latActive = lat_active
    CC.actuators.steeringAngleDeg = target_angle
    actuators, _ = CI.apply(CC.as_reader(), 0)
    return actuators.steeringAngleDeg

  @classmethod
  def _controller_angle(cls, speed, target_angle, lat_active=True, steering_angle=0, steering_pressed=False):
    CI = cls._new_controller()
    # Synchronization frame, intervening non-steering tick, then active output.
    for _ in range(2):
      cls._apply_controller_angle(CI, speed, target_angle, lat_active, steering_angle, steering_pressed)
    return cls._apply_controller_angle(CI, speed, target_angle, lat_active, steering_angle, steering_pressed)

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
    self.assertAlmostEqual(CP.steerRatio, 13.0, places=6)

  def test_low_speed_angle_filter_is_continuous(self):
    # The old deadzone jumped from 0 to 5 degrees when this target crossed 6 degrees.
    below_old_deadzone = self._controller_angle(2, 5.99)
    above_old_deadzone = self._controller_angle(2, 6.01)
    self.assertGreater(below_old_deadzone, 0)
    self.assertLess(above_old_deadzone, 1)
    self.assertLess(above_old_deadzone - below_old_deadzone, 0.01)

  def test_low_speed_angle_filter_fades_with_speed(self):
    low_speed_angle = self._controller_angle(2, 1)
    medium_speed_angle = self._controller_angle(9, 1)

    self.assertGreater(medium_speed_angle, low_speed_angle)
    self.assertLess(medium_speed_angle, 1)
    # Filtering is disabled at 10 m/s.
    self.assertAlmostEqual(self._controller_angle(10, 1), 1, places=6)

  def test_low_speed_angle_filter_converges(self):
    CI = self._new_controller()
    outputs = [self._apply_controller_angle(CI, 2, 5) for _ in range(40)]

    self.assertTrue(all(outputs[i] >= outputs[i - 1] for i in range(1, len(outputs))))
    self.assertGreater(outputs[-1], 4)
    self.assertLess(outputs[-1], 5)

  def test_angle_filter_resets_on_driver_override(self):
    CI = self._new_controller()
    self._apply_controller_angle(CI, 2, 50)
    self._apply_controller_angle(CI, 2, 50)

    # The steering command is updated every other control frame.
    override_angle = self._apply_controller_angle(CI, 2, 50, steering_angle=37, steering_pressed=True)
    self._apply_controller_angle(CI, 2, 37, steering_angle=37)
    resumed_angle = self._apply_controller_angle(CI, 2, 37, steering_angle=37)

    self.assertAlmostEqual(override_angle, 37, places=6)
    self.assertAlmostEqual(resumed_angle, 37, places=6)

  def test_inactive_angle_control_follows_measured_angle(self):
    self.assertAlmostEqual(self._controller_angle(5, 190, lat_active=False, steering_angle=37), 37, places=6)

  def test_engagement_synchronizes_filter_and_angle(self):
    CI = self._new_controller()
    angle = self._apply_controller_angle(CI, 2, 50, steering_angle=12)
    self.assertAlmostEqual(angle, 12)
    self.assertAlmostEqual(CI.CC.angle_filter.x, 12)
    self._apply_controller_angle(CI, 2, 50, steering_angle=12)
    angle = self._apply_controller_angle(CI, 2, 50, steering_angle=12)
    self.assertGreater(angle, 12)
    self.assertLess(angle, 17)

  def test_non_steering_tick_disengagement_requires_resynchronization(self):
    for override in (False, True):
      with self.subTest(override=override):
        CI = self._new_controller()
        self._apply_controller_angle(CI, 20, 0)
        # Frame 1 does not transmit steering, but must invalidate readiness.
        self._apply_controller_angle(CI, 20, 0, lat_active=override, steering_pressed=override)
        angle = self._apply_controller_angle(CI, 20, 10, steering_angle=3)
        self.assertAlmostEqual(angle, 3)

  def test_recovery_reports_temporary_unavailability(self):
    CI = self._new_controller()
    packer = CANPacker("subaru_global_2017_generated")
    frame = 0

    def update(measured, eps_warning=False):
      nonlocal frame
      frame += 1
      msgs = [
        packer.make_can_msg("Wheel_Speeds", 1, {k: 72 for k in ("FL", "FR", "RL", "RR")}),
        packer.make_can_msg("Steering_2", 0, {"Steering_Angle": measured}),
        packer.make_can_msg("Steering_Torque", 0, {"Steer_Warning": eps_warning}),
      ]
      return CI.update([(frame * 10_000_000, [CanData(*msg) for msg in msgs])])

    # Neither a driver override nor the ordinary synchronization frame should
    # generate a controller-unavailable warning.
    for _ in range(2):
      self._apply_controller_angle(CI, 20, 0, steering_angle=25, steering_pressed=True)
    self.assertFalse(update(25).steerFaultTemporary)
    for _ in range(2):
      self._apply_controller_angle(CI, 20, 0, steering_angle=20)
    self.assertFalse(update(20).steerFaultTemporary)

    self._apply_controller_angle(CI, 20, 0, lat_active=False, steering_angle=25)
    for _ in range(2):
      self._apply_controller_angle(CI, 20, 0, steering_angle=25)
    self.assertTrue(CI.CC.angle_control_recovery)
    self.assertTrue(update(25).steerFaultTemporary)
    # controlsd disables latActive in response to temporary unavailability.
    # That must not hide the warning or prevent recovery when the wheel returns.
    for _ in range(10):
      self._apply_controller_angle(CI, 20, 0, lat_active=False, steering_angle=25)
      self.assertTrue(update(25).steerFaultTemporary)
    self.assertFalse(update(22).steerFaultTemporary)
    self.assertFalse(CI.CC.angle_control_recovery)
    self.assertTrue(update(22, eps_warning=True).steerFaultTemporary)
    self.assertFalse(update(22).steerFaultTemporary)

  def test_torque_port_ignores_angle_recovery_state(self):
    CI = CarInterface(CarInterface.get_non_essential_params(CAR.SUBARU_IMPREZA))
    CI.CC.angle_control_recovery = True
    self.assertFalse(CI.update([]).steerFaultTemporary)
