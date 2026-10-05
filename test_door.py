"""Hardware-free regression tests. Run: python3 -m unittest discover -s door -v."""

from types import SimpleNamespace
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import Mock, call, patch

import numpy as np

import door


class StopLoop(BaseException):
    """End an infinite controller loop without invoking its error handler."""


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.module = door
        self.now = 1000.0
        self.clock = SimpleNamespace(
            time=lambda: self.now, sleep=Mock(side_effect=self.advance))
        self.exit_process = Mock(side_effect=StopLoop)
        self.gpio = SimpleNamespace(OUT=0, setup=Mock(), output=Mock())
        self.module.type_to_logtime.clear()
        log_patch = patch.object(door, "log")
        log_patch.start()
        self.addCleanup(log_patch.stop)

    def advance(self, seconds):
        self.now += seconds

    def test_import_has_no_hardware_or_logging_side_effects(self):
        result = subprocess.run(
            [sys.executable, "-B", "-c", """
import signal
import sys
previous_handler = signal.getsignal(signal.SIGHUP)
import door
assert door.log_file is None
assert signal.getsignal(signal.SIGHUP) == previous_handler
assert not {'board', 'adafruit_mlx90393', 'RPi.GPIO'} & sys.modules.keys()
"""], cwd=Path(__file__).parent, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_sensor_reads_from_supplied_reader(self):
        reader = Mock(return_value=(12, 34, 56))
        sensor = self.module.Sensor(callback=Mock(), read_magnetic=reader)
        sensor.read_sensor()
        reader.assert_called_once_with()
        sample = sensor.value_lookback[-1]
        self.assertEqual((sample.x, sample.y, sample.z), (12, 34, 56))

    def test_sensor_callback_opens_its_door(self):
        controller = self.make_door()
        controller.sensor.callback()
        self.assertTrue(controller.open)
        self.assertEqual(controller.motor.trig.call_args_list,
                         [call(True), call(False)])

    def make_door(self):
        sensor = self.make_sensor()
        sensor.force_calibration = Mock()
        return self.module.Door(sensor=sensor, motor=Mock(), direction=Mock(),
                                clock=self.clock, exit_process=self.exit_process)

    def check_closing_once(self, door):
        # The controller's final sleep ends this iteration; movement is mocked.
        with patch.object(self.clock, "sleep", side_effect=StopLoop):
            with self.assertRaises(StopLoop):
                door.read_sensor_thread()

    def test_minimum_open_time(self):
        for elapsed, should_close in ((0, False), (89.9, False),
                                      (90, False), (90.1, True)):
            with self.subTest(elapsed=elapsed):
                door = self.make_door()
                door.open = True
                door.open_time = self.now - elapsed
                door.close_door = Mock()
                self.check_closing_once(door)
                self.assertEqual(door.close_door.called, should_close)
                self.exit_process.assert_not_called()

    def test_active_and_recent_activity_prevents_closing(self):
        for source in ("magnet", "beam"):
            for active, age in ((True, 100), (False, 7.9), (False, 8)):
                with self.subTest(source=source, active=active, age=age):
                    door = self.make_door()
                    door.open = True
                    door.open_time = self.now - 100
                    door.close_door = Mock()
                    if source == "magnet":
                        door.sensor.magnet = active
                        door.sensor.magnet_time = self.now - age
                    else:
                        door.beam.broken = active
                        door.beam.broken_time = self.now - age
                    self.check_closing_once(door)
                    self.assertEqual(door.close_door.called,
                                     not active and age >= 8)

    def test_maximum_open_timeout_exits_only_after_500_seconds(self):
        for elapsed in (499, 500, 500.1):
            with self.subTest(elapsed=elapsed):
                self.exit_process.reset_mock()
                door = self.make_door()
                door.open = True
                door.open_time = self.now - elapsed
                door.sensor.magnet = True
                door.close_door = Mock()
                self.check_closing_once(door)
                door.close_door.assert_not_called()
                if elapsed > 500:
                    self.exit_process.assert_called_once_with(1)
                else:
                    self.exit_process.assert_not_called()

    def test_closed_or_locked_door_is_not_automatically_closed(self):
        for opened, locked in ((False, False), (True, True)):
            with self.subTest(opened=opened, locked=locked):
                door = self.make_door()
                door.open, door.lock = opened, locked
                door.open_time = self.now - 100
                door.close_door = Mock()
                self.check_closing_once(door)
                door.close_door.assert_not_called()

    def test_open_motor_sequence_and_timer_start(self):
        door = self.make_door()
        events = Mock()
        events.attach_mock(door.direction, "direction")
        events.attach_mock(door.motor, "motor")
        events.attach_mock(self.clock.sleep, "sleep")
        door.open_door()
        self.assertEqual(events.mock_calls, [
            call.direction.trig(True), call.motor.trig(True), call.sleep(13),
            call.motor.trig(False), call.sleep(3)])
        self.assertTrue(door.open)
        self.assertFalse(door.lock)
        self.assertEqual(door.open_time, 1016)
        door.sensor.force_calibration.assert_called_once_with()

    def test_close_motor_sequence(self):
        door = self.make_door()
        door.open = True
        events = Mock()
        events.attach_mock(door.direction, "direction")
        events.attach_mock(door.motor, "motor")
        events.attach_mock(self.clock.sleep, "sleep")
        door.close_door()
        self.assertEqual(events.mock_calls, [
            call.direction.trig(False), call.motor.trig(True), call.sleep(16),
            call.motor.trig(False), call.direction.trig(True), call.sleep(2)])
        self.assertFalse(door.open)
        self.assertFalse(door.lock)
        self.assertEqual(door.closed_time, 1016)
        door.sensor.force_calibration.assert_called_once_with()

    def test_redundant_or_locked_movement_does_not_run_motor(self):
        for action, opened, locked in (("open_door", True, False),
                                       ("open_door", False, True),
                                       ("close_door", False, False),
                                       ("close_door", True, True)):
            with self.subTest(action=action, opened=opened, locked=locked):
                door = self.make_door()
                door.open, door.lock = opened, locked
                getattr(door, action)()
                door.motor.trig.assert_not_called()
                door.direction.trig.assert_not_called()

    def test_relay_polarity_and_initial_off_state(self):
        for inverted, off_level in ((False, True), (True, False)):
            with self.subTest(inverted=inverted):
                self.gpio.output.reset_mock()
                relay = self.module.Relay(27, inverted=inverted, gpio=self.gpio)
                relay.trig(True)
                relay.trig(False)
                self.assertEqual(self.gpio.output.call_args_list,
                                 [call(27, off_level), call(27, not off_level),
                                  call(27, off_level)])

    def make_sensor(self):
        sensor = self.module.Sensor(callback=Mock(), read_magnetic=Mock(return_value=(100, 0, 0)))
        sensor.avg = np.array([100.0, 0.0, 0.0])
        sensor.avg_norm = 100.0
        return sensor

    def test_calibration_requires_full_stable_window(self):
        sensor = self.make_sensor()
        for _ in range(9):
            sensor.value_lookback.append(self.module.SensorData(100, 0, 0))
        self.assertIsNone(sensor.compute_avg(.9))
        sensor.value_lookback.append(self.module.SensorData(100, 0, 0))
        np.testing.assert_allclose(sensor.compute_avg(.9), (100, 0, 0))
        sensor.value_lookback.append(self.module.SensorData(100, 20, 0))
        self.assertIsNone(sensor.compute_avg(.9))

    def test_magnet_magnitude_and_direction_thresholds(self):
        sensor = self.make_sensor()
        for vector, detected in (((100, 0, 0), False), ((105, 0, 0), False),
                                 ((106, 0, 0), True), ((94, 0, 0), True)):
            with self.subTest(vector=vector):
                self.assertEqual(sensor.magnet_detected(
                    self.module.SensorData(*vector)), detected)
        for degrees, detected in ((8, False), (10, True)):
            with self.subTest(degrees=degrees):
                radians = np.deg2rad(degrees)
                data = self.module.SensorData(100 * np.cos(radians),
                                              100 * np.sin(radians), 0)
                self.assertEqual(sensor.magnet_detected(data), detected)

    def test_single_magnetic_spike_is_ignored(self):
        sensor = self.make_sensor()
        for values, detected in (((100, 100, 120, 100), False),
                                 ((100, 120, 100, 120), True)):
            with self.subTest(values=values):
                samples = [self.module.SensorData(x, 0, 0) for x in values]
                self.assertEqual(sensor.magnet_detect_lookback(samples), detected)


if __name__ == "__main__":
    unittest.main()
