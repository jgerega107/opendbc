#!/usr/bin/env python3
import enum
import unittest
import numpy as np

from functools import partial

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR, CarControllerParams, SubaruSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, MAX_WRONG_COUNTERS, round_speed, away_round


class SubaruMsg(enum.IntEnum):
  Brake_Status      = 0x13c
  CruiseControl     = 0x240
  Throttle          = 0x40
  Steering_Torque   = 0x119
  Steering_2        = 0x11a
  Wheel_Speeds      = 0x13a
  ES_LKAS           = 0x122
  ES_LKAS_ANGLE     = 0x124
  ES_Distance       = 0x221
  ES_Status         = 0x222
  ES_DashStatus     = 0x321
  ES_LKAS_State     = 0x322
  ES_Infotainment   = 0x323


SUBARU_MAIN_BUS = 0
SUBARU_ALT_BUS  = 1
SUBARU_CAM_BUS  = 2


def lkas_tx_msgs(alt_bus, lkas_msg=SubaruMsg.ES_LKAS):
  return [[lkas_msg,                    SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Distance,       alt_bus],
          [SubaruMsg.ES_DashStatus,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_LKAS_State,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Infotainment,   SUBARU_MAIN_BUS]]


def fwd_blacklisted_addr(lkas_msg=SubaruMsg.ES_LKAS):
  return {SUBARU_CAM_BUS: [lkas_msg, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State, SubaruMsg.ES_Infotainment]}


class TestSubaruSafetyBase(common.CarSafetyTest):
  DBC = "subaru_global_2017_generated"
  SAFETY_MODEL = CarParams.SafetyModel.subaru

  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr()

  MAX_RT_DELTA = 940

  DRIVER_TORQUE_ALLOWANCE = 60
  DRIVER_TORQUE_FACTOR = 50

  ALT_MAIN_BUS = SUBARU_MAIN_BUS
  ALT_CAM_BUS = SUBARU_CAM_BUS

  DEG_TO_CAN = 100

  INACTIVE_GAS = 1818

  def _set_prev_torque(self, t):
    self.safety.set_desired_torque_last(t)
    self.safety.set_rt_torque_last(t)

  def _torque_driver_msg(self, torque):
    values = {"Steer_Torque_Sensor": torque}
    return self.packer.make_can_msg_safety("Steering_Torque", 0, values)

  def _speed_msg(self, speed):
    values = {s: speed for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("Wheel_Speeds", self.ALT_MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"Brake": brake}
    return self.packer.make_can_msg_safety("Brake_Status", self.ALT_MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"Throttle_Pedal": gas}
    return self.packer.make_can_msg_safety("Throttle", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("CruiseControl", self.ALT_MAIN_BUS, values)


class TestSubaruStockLongitudinalSafetyBase(TestSubaruSafetyBase):
  def _cancel_msg(self, cancel, cruise_throttle=0):
    values = {"Cruise_Cancel": cancel, "Cruise_Throttle": cruise_throttle}
    return self.packer.make_can_msg_safety("ES_Distance", self.ALT_MAIN_BUS, values)

  def test_cancel_message(self):
    # test that we can only send the cancel message (ES_Distance) with inactive throttle (1818) and Cruise_Cancel=1
    for cancel in [True, False]:
      self._generic_limit_safety_check(partial(self._cancel_msg, cancel), self.INACTIVE_GAS, self.INACTIVE_GAS, 0, 2**12, 1, self.INACTIVE_GAS, cancel)


class TestSubaruTorqueSafetyBase(TestSubaruSafetyBase, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):
  MAX_RATE_UP = 50
  MAX_RATE_DOWN = 70
  MAX_TORQUE_LOOKUP = [0], [2047]

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 7
  MAX_INVALID_STEERING_FRAMES = 1
  STEER_STEP = 2

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_Request": steer_req}
    return self.packer.make_can_msg_safety("ES_LKAS", SUBARU_MAIN_BUS, values)


class TestSubaruGen1TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruTorqueSafetyBase):
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS)


class TestSubaruGen2TorqueSafetyBase(TestSubaruTorqueSafetyBase):
  ALT_MAIN_BUS = SUBARU_ALT_BUS
  ALT_CAM_BUS = SUBARU_ALT_BUS

  MAX_RATE_UP = 40
  MAX_RATE_DOWN = 40
  MAX_TORQUE_LOOKUP = [0], [1000]


class TestSubaruGen2TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruGen2TorqueSafetyBase):
  SAFETY_PARAM = SubaruSafetyFlags.GEN2

  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS)


class TestSubaruAngleSafetyBase(TestSubaruSafetyBase, common.AngleSteeringSafetyTest):
  STEER_ANGLE_MAX = 190
  DEG_TO_CAN = 100

  # VM-based limits, not breakpoint-based
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  LATERAL_FREQUENCY = 50

  cnt_angle_cmd = 0

  def setUp(self):
    self.__class__.cnt_angle_cmd = 0
    super().setUp()
    from opendbc.car.subaru.carcontroller import get_safety_CP
    self.VM = VehicleModel(get_safety_CP())

  def _speed_msg(self, speed):
    # speed is in m/s for angle tests, convert to kph for DBC
    speed_kph = speed * 3.6
    values = {s: speed_kph for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("Wheel_Speeds", self.ALT_MAIN_BUS, values)

  def _angle_cmd_msg(self, angle, enabled, increment_timer=True):
    values = {"LKAS_Output": angle, "LKAS_Request": enabled, "SET_3": 3}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("ES_LKAS_ANGLE", SUBARU_MAIN_BUS, values)

  def _angle_meas_msg(self, angle):
    values = {"Steering_Angle": angle}
    return self.packer.make_can_msg_safety("Steering_2", SUBARU_MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("ES_Status", self.ALT_MAIN_BUS, values)

  def test_angle_cmd_when_enabled(self):
    # VM-based limits are tested below
    pass

  def _reset_angle_safety(self):
    self.__class__.cnt_angle_cmd = 0
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.SAFETY_PARAM)
    self.safety.init_tests()

  def _rx_check_msgs(self):
    return {
      "Throttle": self._user_gas_msg(0),
      "Steering_Torque": self._torque_driver_msg(0),
      "Wheel_Speeds": self._speed_msg(0),
      "Brake_Status": self._user_brake_msg(False),
      "ES_Status": self._pcm_status_msg(False),
      "Steering_2": self._angle_meas_msg(0),
    }

  def test_rx_checksums(self):
    for name in self._rx_check_msgs():
      with self.subTest(name=name):
        self._reset_angle_safety()
        msg = self._rx_check_msgs()[name]
        self.assertTrue(self._rx(msg))

        msg = self._rx_check_msgs()[name]
        msg[0].data[0] ^= 0xFF
        self.safety.set_controls_allowed(True)
        self.assertFalse(self._rx(msg))
        self.assertFalse(self.safety.get_controls_allowed())

  def test_rx_counters(self):
    for name in self._rx_check_msgs():
      with self.subTest(name=name):
        self._reset_angle_safety()
        msg = self._rx_check_msgs()[name]
        self.assertTrue(self._rx(msg))

        # A stuck counter must eventually invalidate RX and disengage controls.
        for _ in range(MAX_WRONG_COUNTERS + 1):
          self.safety.set_controls_allowed(True)
          ret = self._rx(msg)
        self.assertFalse(ret)
        self.assertFalse(self.safety.get_controls_allowed())

  # Adapted from jacobwaller/opendbc hackathon-crosstrek, commit 753fdd38.
  def test_rx_wrong_bus(self):
    for make_msg, get_value in ((self._angle_meas_msg, self.safety.get_angle_meas_max),
                                (self._pcm_status_msg, self.safety.get_controls_allowed)):
      for bus in range(3):
        with self.subTest(message=make_msg.__name__, bus=bus):
          self._reset_angle_safety()
          msg = make_msg(1)
          expected_bus = msg[0].bus
          msg[0].bus = bus
          self._rx(msg)
          # Include the expected bus as a positive control.
          expected = self.DEG_TO_CAN if make_msg == self._angle_meas_msg else 1
          self.assertEqual(get_value(), expected if bus == expected_bus else 0)

  def test_stale_es_brake_cannot_engage(self):
    self.assertTrue(self._rx(self._speed_msg(0)))
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._user_brake_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._pcm_status_msg(False)))
    self.assertTrue(self._rx(self._user_brake_msg(False)))
    # ES_Brake may remain high after ACC cancels at a stop. Only ES_Status
    # is authoritative for cruise activation in angle-control safety.
    self._rx(self.packer.make_can_msg_safety("ES_Brake", self.ALT_MAIN_BUS, {"Cruise_Activated": 1}))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_controller_angle_safety_transitions(self):
    platform = CAR.SUBARU_CROSSTREK_2025 if self.SAFETY_PARAM & SubaruSafetyFlags.GEN2 else CAR.SUBARU_FORESTER_2022
    # Synthetic engagement above the nominal bound, and a speed increase
    # that lowers the bound. Existing exact accel/jerk limit tests remain.
    for start_speed, end_speed, measured, desired in ((13.24, 13.24, 51.59, 46.21), (13.0, 13.5, 46.75, 46.75)):
      for sign in (-1, 1):
        with self.subTest(sign=sign, start_speed=start_speed, end_speed=end_speed):
          self._reset_angle_safety()
          CI = CarInterface(CarInterface.get_non_essential_params(platform))
          CI.update([])
          self._reset_angle_measurement(sign * measured)
          CC = structs.CarControl()
          CC.actuators.steeringAngleDeg = sign * desired
          angle_messages = 0
          previous_tx_active = False
          for frame in range(100):
            speed = start_speed if frame < 20 else end_speed
            CI.CS.out = structs.CarState(vEgo=speed, vEgoRaw=speed, steeringAngleDeg=sign * measured)
            self._reset_speed_measurement(speed)
            CC.latActive = frame not in (0, 50)
            self.safety.set_controls_allowed(CC.latActive)
            self.safety.set_timer(frame * 10_000)
            _, messages = CI.apply(CC.as_reader(), frame * 10_000_000)
            for addr, data, bus in messages:
              if addr == SubaruMsg.ES_LKAS_ANGLE:
                angle_messages += 1
                # Permit one inactive synchronization TX at an engagement edge,
                # so this coverage also composes with engagement-sync fixes.
                if not CC.latActive or previous_tx_active:
                  self.assertEqual(bool(data[1] & 0x10), CC.latActive)
                previous_tx_active = CC.latActive
                self.assertTrue(self._tx(libsafety_py.make_CANPacket(addr, bus, data)), f"frame {frame}")
          self.assertEqual(angle_messages, 50)

  def test_inactive_angle_moving_measurement_order(self):
    # Exercise the rolling measurement window without requiring an
    # engagement transition: an inactive command follows a moving wheel.
    for sign in (-1, 1):
      for measurement_before_tx in (False, True):
        with self.subTest(sign=sign, measurement_before_tx=measurement_before_tx):
          self._reset_angle_safety()
          self._reset_angle_measurement(sign * 5)
          for frame in range(20):
            angle = sign * (5 + frame * 0.01)
            msg = self._angle_meas_msg(angle)
            if measurement_before_tx:
              self.assertTrue(self._rx(msg))
            # The other ordering is one CAN unit newer than the latest sample,
            # within the existing inactive-angle tolerance in either direction.
            self.assertTrue(self._tx(self._angle_cmd_msg(angle, False)))
            self.assertFalse(self.safety.get_controls_allowed())
            if not measurement_before_tx:
              self.assertTrue(self._rx(msg))

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match Wheel_Speeds rounding on CAN (factor 0.057 kph)
      speed = round_speed(away_round(speed * 3.6 / 0.057) * 0.057 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)
        self._tx(self._angle_cmd_msg(0, True))

        # Use the exact quantized wheel speed received by safety, including its
        # one m/s model tolerance, as the input to the independent Python VM.
        safety_speed = max(self.safety.get_vehicle_speed_min() - 1, 1)
        # Match the C safety calculation: convert the independent Python VM result
        # to 0.01 degree CAN units and include safety's one-unit tolerance.
        max_angle_can = int(get_max_angle_vm(safety_speed, self.VM, CarControllerParams) * self.DEG_TO_CAN) + 1
        apply_angle_can = min(max_angle_can, self.STEER_ANGLE_MAX * self.DEG_TO_CAN)

        self.safety.set_desired_angle_last(apply_angle_can * sign)
        self.assertTrue(self._tx(self._angle_cmd_msg(apply_angle_can / self.DEG_TO_CAN * sign, True)))

        above_limit_can = min(max_angle_can + 1, self.STEER_ANGLE_MAX * self.DEG_TO_CAN)
        self.safety.set_desired_angle_last(above_limit_can * sign)

        should_tx = max_angle_can >= self.STEER_ANGLE_MAX * self.DEG_TO_CAN
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(above_limit_can / self.DEG_TO_CAN * sign, True)))

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      speed = round_speed(away_round(speed * 3.6 / 0.057) * 0.057 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)
        self._tx(self._angle_cmd_msg(0, True))

        safety_speed = max(self.safety.get_vehicle_speed_min() - 1, 1)
        max_delta_can = int(get_max_angle_delta_vm(safety_speed, self.VM, CarControllerParams) * self.DEG_TO_CAN) + 1

        self.safety.set_desired_angle_last(0)
        self.assertTrue(self._tx(self._angle_cmd_msg(max_delta_can / self.DEG_TO_CAN * sign, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_delta_can / self.DEG_TO_CAN * sign, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        above_delta_can = max_delta_can + 1
        self.assertFalse(self._tx(self._angle_cmd_msg(above_delta_can / self.DEG_TO_CAN * sign, True)))
        self.safety.set_desired_angle_last(round(above_delta_can * sign))
        self.assertTrue(self._tx(self._angle_cmd_msg(above_delta_can / self.DEG_TO_CAN * sign, True)))
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))


class TestSubaruGen1AngleStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruAngleSafetyBase):
  SAFETY_PARAM = SubaruSafetyFlags.LKAS_ANGLE
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS, SubaruMsg.ES_LKAS_ANGLE)
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS_ANGLE, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr(SubaruMsg.ES_LKAS_ANGLE)


class TestSubaruGen2AngleStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruAngleSafetyBase):
  ALT_MAIN_BUS = SUBARU_ALT_BUS
  SAFETY_PARAM = SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LKAS_ANGLE
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS, SubaruMsg.ES_LKAS_ANGLE)
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS_ANGLE, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr(SubaruMsg.ES_LKAS_ANGLE)

  @staticmethod
  def _decode_angle_cmd(dat):
    raw_angle = int.from_bytes(dat[5:8], byteorder="little") & 0x1FFFF
    if raw_angle & 0x10000:
      raw_angle -= 0x20000
    return -raw_angle / 100, bool(dat[1] & 0x10)

  def test_crosstrek_controller_driver_override(self):
    self._reset_angle_safety()
    self._reset_speed_measurement(15)
    self._reset_angle_measurement(0)

    CP = CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)
    CI = CarInterface(CP)
    CI.update([])
    carstate_packer = CANPackerSafety("subaru_global_2017_generated")

    # The global Subaru driver-torque threshold is strict: exactly +/-80 does
    # not count as an override, while the next representable value does.
    # CarStateBase.update_steering_pressed filters brief spikes around the
    # threshold, so the flag only flips after several consistent frames.
    STEER_THRESHOLD = 80
    MIN_COUNT = 5
    DEBOUNCE_COUNT = MIN_COUNT + 1
    pressed_count = 0

    def track_expected_pressed(steering_torque):
      nonlocal pressed_count
      pressed_count += 1 if abs(steering_torque) > STEER_THRESHOLD else -1
      pressed_count = max(0, min(pressed_count, MIN_COUNT * 2 + 1))
      return pressed_count > MIN_COUNT

    frame = 0

    def update_carstate(steering_torque, steering_angle):
      nonlocal frame
      can_sends = [
        CanData(*carstate_packer.make_can_msg("Steering_Torque", SUBARU_MAIN_BUS,
                                              {"Steer_Torque_Sensor": steering_torque})),
        CanData(*carstate_packer.make_can_msg("Steering_2", SUBARU_MAIN_BUS,
                                              {"Steering_Angle": steering_angle})),
      ]
      frame += 1
      return CI.update([(frame * 10_000_000, can_sends)])

    # sustained sub-threshold torque never registers, in either direction
    for steering_torque in (0, 80, -80):
      for _ in range(DEBOUNCE_COUNT + 2):
        CS = update_carstate(steering_torque, 0)
        self.assertFalse(CS.steeringPressed)
        self.assertFalse(track_expected_pressed(steering_torque))

    # isolated spikes above the threshold are filtered out entirely
    for i in range((MIN_COUNT + 1) * 2 + 1):
      steering_torque = 500 if i % 4 == 0 else 30
      CS = update_carstate(steering_torque, 0)
      self.assertFalse(CS.steeringPressed)
      self.assertFalse(track_expected_pressed(steering_torque))

    # sustained overrides register once the debounce window elapses
    for steering_torque in (81, -81):
      # Drain any state left by the previous case so each direction starts
      # from the bottom of the counter and proves the activation boundary.
      for _ in range(MIN_COUNT * 2 + 1):
        CS = update_carstate(0, 0)
        self.assertEqual(CS.steeringPressed, track_expected_pressed(0))
      self.assertFalse(CS.steeringPressed)

      for i in range(DEBOUNCE_COUNT + 3):
        CS = update_carstate(steering_torque, 0)
        expected_pressed = track_expected_pressed(steering_torque)
        self.assertEqual(CS.steeringPressed, expected_pressed)
        self.assertEqual(expected_pressed, i >= MIN_COUNT)

    # Exercise normal control, a driver override, resumption after the driver
    # releases the wheel, another override, and finally a full disengagement.
    # Each phase runs past the debounce window so states settle within it.
    sequence = (
      [(True, 0, 0)] * (DEBOUNCE_COUNT + 2) +
      [(True, 81, 5)] * (DEBOUNCE_COUNT + 2) +
      [(True, 0, 10)] * (DEBOUNCE_COUNT + 2) +
      [(True, -81, 15)] * (DEBOUNCE_COUNT + 2) +
      [(False, -81, 20)] * (DEBOUNCE_COUNT + 2) +
      [(False, 0, 20)] * (DEBOUNCE_COUNT + 2)
    )

    angle_msgs = 0
    for offset, (lat_active, steering_torque, steering_angle) in enumerate(sequence):
      self.assertTrue(self._rx(self._angle_meas_msg(steering_angle)))

      CS = update_carstate(steering_torque, steering_angle)
      steering_pressed = track_expected_pressed(steering_torque)
      self.assertEqual(CS.steeringPressed, steering_pressed)
      self.assertAlmostEqual(CS.steeringAngleDeg, steering_angle, places=2)

      CI.CS.out.vEgoRaw = 15

      CC = structs.CarControl()
      CC.latActive = lat_active
      CC.actuators.steeringAngleDeg = 20

      self.safety.set_controls_allowed(lat_active)
      self.safety.set_timer(frame * 10_000)
      _, can_sends = CI.apply(CC.as_reader(), frame * 10_000_000)

      for addr, dat, bus in can_sends:
        if addr == SubaruMsg.ES_LKAS_ANGLE:
          angle_msgs += 1
          apply_angle, lkas_request = self._decode_angle_cmd(dat)
          expected_request = lat_active and not steering_pressed
          self.assertEqual(lkas_request, expected_request)
          if not expected_request:
            self.assertAlmostEqual(apply_angle, steering_angle, places=2)

          packet = libsafety_py.make_CANPacket(addr, bus, dat)
          self.assertTrue(self._tx(packet), f"rejected driver override angle TX at {offset=} {steering_angle=}")

    self.assertEqual(angle_msgs, (len(sequence) + 1) // 2)

if __name__ == "__main__":
  unittest.main()
