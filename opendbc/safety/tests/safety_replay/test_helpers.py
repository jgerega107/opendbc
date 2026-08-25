import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car.structs import CarParams
from opendbc.car.subaru.values import SubaruSafetyFlags
from opendbc.safety.tests.safety_replay.helpers import get_steer_value, is_steering_msg


class TestSubaruSafetyReplayHelpers(unittest.TestCase):
  MODE = CarParams.SafetyModel.subaru

  @classmethod
  def setUpClass(cls):
    cls.packer = CANPacker("subaru_global_2017_generated")

  def test_steering_message_for_safety_mode(self):
    self.assertTrue(is_steering_msg(self.MODE, 0, 0x122))
    self.assertFalse(is_steering_msg(self.MODE, 0, 0x124))

    param = SubaruSafetyFlags.LKAS_ANGLE
    self.assertFalse(is_steering_msg(self.MODE, param, 0x122))
    self.assertTrue(is_steering_msg(self.MODE, param, 0x124))

  def test_angle_command_decode(self):
    param = SubaruSafetyFlags.LKAS_ANGLE
    for angle in (-190, -37, 0, 37, 190):
      with self.subTest(angle=angle):
        _, dat, _ = self.packer.make_can_msg("ES_LKAS_ANGLE", 0, {"LKAS_Output": angle})
        torque, angle_can = get_steer_value(self.MODE, param, SimpleNamespace(data=dat))
        self.assertEqual(torque, 0)
        self.assertEqual(angle_can, angle * 100)


if __name__ == "__main__":
  unittest.main()
