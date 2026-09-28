"""Sharp filtering checks without hardware."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from robomaster_round2.Slam.sharp_sensor import sharp_sensor


class TestSharpFilter(unittest.TestCase):
    def setUp(self):
        self.sensor = sharp_sensor(SimpleNamespace(sensor_adaptor=Mock()))

    def test_median_rejects_one_spike_and_keeps_ports_separate(self):
        with patch.object(self.sensor, 'read_sample', side_effect=[
                (0, 0, 7.5), (0, 0, 7.6), (0, 0, 20.0),
                (0, 0, 12.0)]):
            front = [self.sensor.get_distance(1, 1) for _ in range(3)]
            back = self.sensor.get_distance(2, 1)
        self.assertEqual(front, [7.5, 7.55, 7.6])
        self.assertEqual(back, 12.0)

    def test_invalid_reading_does_not_reuse_old_value(self):
        with patch.object(self.sensor, 'read_sample', side_effect=[
                (0, 0, 7.5), (0, 0, None), (0, 0, 9.0)]):
            self.assertEqual(self.sensor.get_distance(1, 1), 7.5)
            self.assertIsNone(self.sensor.get_distance(1, 1))
            self.assertEqual(self.sensor.get_distance(1, 1), 9.0)

    def test_reset_discards_previous_cell(self):
        with patch.object(self.sensor, 'read_sample', side_effect=[
                (0, 0, 7.5), (0, 0, 9.0)]):
            self.assertEqual(self.sensor.get_distance(1, 1), 7.5)
            self.sensor.reset_filter()
            self.assertEqual(self.sensor.get_distance(1, 1), 9.0)


if __name__ == '__main__':
    unittest.main()
