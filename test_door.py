"""Hardware-free regression tests. Run: python3 -m unittest discover -s door -v."""

from types import SimpleNamespace
from pathlib import Path
import subprocess
import sys
import time
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
            time=lambda: self.now, sleep=Mock(side_effect=self.advance),
            localtime=time.gmtime)
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
        sensor = self.module.Sensor(on_magnet_detected=Mock(), read_magnetic=reader)
        sensor.read_sensor()
        reader.assert_called_once_with()
        sample = sensor.value_lookback[-1]
        self.assertEqual((sample.x, sample.y, sample.z), (12, 34, 56))

    def test_on_magnet_detected_opens_its_door(self):
        controller = self.make_door()
        controller.sensor.on_magnet_detected()
        self.assertTrue(controller.open)
        self.assertEqual(controller.motor.trig.call_args_list,
                         [call(True), call(False)])

    def make_door(self, keep_open_start="00:00", keep_open_end="00:00"):
        """Returns:
            Door: A controller with fake hardware and the given daily keep-open range.
        """
        sensor = self.make_sensor()
        sensor.force_calibration = Mock()
        return self.module.Door(sensor=sensor, motor=Mock(), direction=Mock(),
                                clock=self.clock, exit_process=self.exit_process,
                                keep_open_start=keep_open_start,
                                keep_open_end=keep_open_end,
                                close_check=Mock(return_value=True))

    def test_close_checks_photo_before_motor_including_website_and_force(self):
        """Every closing path must receive a clear result before running the motor."""
        for source, force in (("automatic", False), ("website", False),
                              ("automatic", True)):
            with self.subTest(source=source, force=force):
                controller = self.make_door()
                controller.open = True
                events = Mock()
                events.attach_mock(controller.close_check, "check")
                events.attach_mock(controller.motor, "motor")
                controller.close_door(source=source, force=force)
                self.assertEqual(events.mock_calls,
                                 [call.check(), call.motor.trig(True), call.motor.trig(False)])

    def test_startup_lowers_without_camera_check(self):
        """Homing establishes closed state even when the camera is unavailable."""
        controller = self.make_door()
        controller.close_check.side_effect = TimeoutError("phone offline")
        controller.home_door()
        controller.close_check.assert_not_called()
        self.assertFalse(controller.open)
        self.assertEqual(controller.motor.trig.call_args_list,
                         [call(True), call(False)])

    def test_failed_check_keeps_open_and_retries_with_another_photo(self):
        """Blocked, unavailable, and invalid results never energize the motor."""
        for result in (False, None, TimeoutError("phone offline"), RuntimeError("model failed")):
            with self.subTest(result=result):
                controller = self.make_door()
                controller.open = True
                controller.close_check.side_effect = [result, True]
                controller.close_door()
                controller.close_door()
                controller.motor.trig.assert_not_called()
                self.assertTrue(controller.open)
                self.assertFalse(controller.movement_lock.locked())
                self.assertEqual(controller.close_check.call_count, 1)
                self.advance(5)
                controller.close_door()
                self.assertFalse(controller.open)
                self.assertEqual(controller.close_check.call_count, 2)

    def test_sensor_activity_during_capture_prevents_automatic_close(self):
        """Recheck sensors after waiting for the phone."""
        controller = self.make_door()
        controller.open = True

        def capture():
            """Returns:
                bool: A clear image after a new magnet event.
            """
            controller.sensor.magnet_time = self.now
            return True

        controller.close_check.side_effect = capture
        controller.close_door()
        controller.motor.trig.assert_not_called()

    def test_website_open_cancels_close_while_waiting_for_photo(self):
        """An Open command received during capture cancels the pending closing."""
        controller = self.make_door()
        controller.open = True

        def capture():
            """Returns:
                bool: A clear image after the user cancels closing.
            """
            controller.open_door(source="website")
            return True

        controller.close_check.side_effect = capture
        controller.close_door(source="website")
        controller.motor.trig.assert_not_called()
        self.assertFalse(controller.website_override == "closed")

    def test_schedule_start_during_capture_prevents_close(self):
        """Honor a schedule boundary crossed while waiting for a photo."""
        self.now = 8 * 3600 - 5
        controller = self.make_door(keep_open_start="08:00", keep_open_end="18:00")
        controller.open = True

        def capture():
            """Returns:
                bool: A clear image received after the schedule starts.
            """
            self.advance(10)
            return True

        controller.close_check.side_effect = capture
        controller.close_door()
        controller.motor.trig.assert_not_called()

    def test_daily_schedule_boundaries(self):
        controller = self.make_door(keep_open_start="08:00", keep_open_end="18:00")
        for seconds, active in ((8*3600-1, False), (8*3600, True),
                                (18*3600-1, True), (18*3600, False),
                                (86400+8*3600, True)):
            with self.subTest(seconds=seconds):
                self.now = seconds
                self.assertEqual(controller.is_scheduled_open(), active)
        self.assertEqual(controller.schedule_description(), "08:00–18:00")

    def test_nonpositive_schedule_interval_preserves_normal_closing(self):
        for start, end in (("08:00", "08:00"), ("18:00", "08:00")):
            with self.subTest(start=start, end=end):
                self.now = 12*3600
                controller = self.make_door(keep_open_start=start, keep_open_end=end)
                controller.open = True
                controller.open_time = self.now - 100
                controller.close_door = Mock()
                self.assertFalse(controller.is_scheduled_open())
                controller.update_door_state()
                controller.close_door.assert_called_once_with()

    def test_schedule_opens_closed_door_without_magnet(self):
        self.now = 8*3600
        controller = self.make_door(keep_open_start="08:00", keep_open_end="18:00")
        controller.open = False
        controller.open_door = Mock()
        controller.close_door = Mock()
        controller.update_door_state()
        controller.open_door.assert_called_once_with(source="schedule")
        controller.close_door.assert_not_called()

    def test_schedule_prevents_closing_and_maximum_timeout(self):
        self.now = 12*3600
        controller = self.make_door(keep_open_start="08:00", keep_open_end="18:00")
        controller.open = True
        controller.open_time = 8*3600
        controller.sensor.magnet = True
        controller.close_door()  # Automatic closing must respect the schedule.
        controller.update_door_state()
        controller.motor.trig.assert_not_called()
        self.exit_process.assert_not_called()

    def test_schedule_end_restarts_normal_timers(self):
        self.now = 18*3600-1
        controller = self.make_door(keep_open_start="08:00", keep_open_end="18:00")
        controller.open = True
        controller.open_time = 8*3600
        controller.close_door = Mock()
        controller.update_door_state()
        self.now += 1
        controller.update_door_state()
        self.assertEqual(controller.open_time, self.now)
        controller.close_door.assert_not_called()
        self.now += 90.1
        controller.update_door_state()
        controller.close_door.assert_called_once_with()
        self.exit_process.assert_not_called()

    def test_website_close_overrides_schedule_and_sensor_triggers(self):
        self.now = 12*3600
        controller = self.make_door(keep_open_start="08:00", keep_open_end="18:00")
        controller.open = True
        controller.close_door(source="website")
        self.assertFalse(controller.open)
        self.assertTrue(controller.website_override == "closed")
        controller.motor.reset_mock()
        controller.open_door_from_magnet()
        controller.open_door_from_beam()
        controller.update_door_state()
        self.now += 60
        controller.update_door_state()
        controller.motor.trig.assert_not_called()
        self.exit_process.assert_not_called()

    def test_website_open_holds_past_minimum_time(self):
        """Manual Open prevents automatic closing until the hold is replaced."""
        controller = self.make_door()
        controller.open_door(source="website")
        self.now += 3600
        controller.update_door_state()
        controller.close_door()
        controller.close_check.assert_not_called()
        self.assertTrue(controller.open)
        self.assertEqual(controller.website_override, "open")

    def test_website_open_clears_close_override(self):
        controller = self.make_door()
        controller.close_door(source="website")
        controller.open_door(source="website")
        self.assertFalse(controller.website_override == "closed")
        self.assertTrue(controller.open)
        self.assertEqual(controller.motor.trig.call_args_list, [call(True), call(False)])

    def test_next_schedule_start_clears_manual_close_and_opens(self):
        self.now = 7*3600
        controller = self.make_door(keep_open_start="08:00", keep_open_end="18:00")
        controller.close_door(source="website")
        self.now = 8*3600
        controller.update_door_state()
        self.assertFalse(controller.website_override == "closed")
        self.assertTrue(controller.open)

    def test_next_schedule_end_clears_manual_close_and_restores_sensors(self):
        self.now = 12*3600
        controller = self.make_door(keep_open_start="08:00", keep_open_end="18:00")
        controller.close_door(source="website")
        self.now = 18*3600
        controller.update_door_state()
        self.assertFalse(controller.website_override == "closed")
        controller.motor.trig.assert_not_called()
        controller.open_door_from_magnet()
        self.assertTrue(controller.open)

    def test_midnight_does_not_expire_manual_close(self):
        self.now = 23*3600
        controller = self.make_door(keep_open_start="08:00", keep_open_end="18:00")
        controller.close_door(source="website")
        self.now = 86400
        controller.update_door_state()
        self.assertTrue(controller.website_override == "closed")
        controller.motor.trig.assert_not_called()

    def test_skipped_schedule_boundaries_still_expire_manual_close(self):
        self.now = 12*3600
        controller = self.make_door(keep_open_start="08:00", keep_open_end="18:00")
        controller.close_door(source="website")
        self.now += 86400
        controller.update_door_state()
        self.assertFalse(controller.website_override == "closed")
        self.assertTrue(controller.open)

    def test_website_close_at_boundary_overrides_that_schedule_period(self):
        self.now = 7*3600
        controller = self.make_door(keep_open_start="08:00", keep_open_end="18:00")
        self.now = 8*3600
        controller.close_door(source="website")
        controller.update_door_state()
        self.assertTrue(controller.website_override == "closed")
        controller.motor.trig.assert_not_called()

    def test_disabled_schedule_keeps_manual_close_across_days(self):
        controller = self.make_door()
        controller.close_door(source="website")
        self.now += 2*86400
        controller.update_door_state()
        self.assertTrue(controller.website_override == "closed")

    def test_control_loop_updates_then_waits(self):
        controller = self.make_door()
        events = Mock()
        controller.update_door_state = events.update
        events.attach_mock(self.clock.sleep, "sleep")
        self.clock.sleep.side_effect = StopLoop
        with self.assertRaises(StopLoop):
            controller.run_control_loop()
        self.assertEqual(events.mock_calls, [call.update(), call.sleep(1.5)])

    def test_website_close_is_deferred_during_opening(self):
        controller = self.make_door(keep_open_start="08:00", keep_open_end="18:00")
        self.now = 12*3600

        def request_close_during_sleep(seconds):
            if seconds == 13:
                controller.close_door(source="website")
            self.advance(seconds)

        with patch.object(self.clock, "sleep", side_effect=request_close_during_sleep):
            controller.open_door()
        self.assertTrue(controller.website_override == "closed")
        self.assertTrue(controller.open)
        self.assertEqual(controller.motor.trig.call_args_list, [call(True), call(False)])

        controller.update_door_state()
        self.assertFalse(controller.open)
        self.assertEqual(controller.motor.trig.call_args_list,
                         [call(True), call(False), call(True), call(False)])

    def test_new_controller_has_no_website_override(self):
        controller = self.make_door()
        controller.close_door(source="website")
        self.assertFalse(self.make_door().website_override == "closed")

    def test_forced_close_can_override_schedule(self):
        self.now = 12*3600
        controller = self.make_door(keep_open_start="08:00", keep_open_end="18:00")
        controller.open = True
        controller.close_door(force=True)
        self.assertFalse(controller.open)
        self.assertEqual(controller.motor.trig.call_args_list,
                         [call(True), call(False)])

    def test_schedule_rejects_invalid_times(self):
        for value in ("8:00", "24:00", "12:60", "-1:00", "08:00pm"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    self.make_door(keep_open_start=value)

    def test_minimum_open_time(self):
        for elapsed, should_close in ((0, False), (89.9, False),
                                      (90, False), (90.1, True)):
            with self.subTest(elapsed=elapsed):
                door = self.make_door()
                door.open = True
                door.open_time = self.now - elapsed
                door.close_door = Mock()
                door.update_door_state()
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
                    door.update_door_state()
                    self.assertEqual(door.close_door.called,
                                     not active and age >= 8)

    def test_activity_keeps_door_open_without_timeout_restart(self):
        """Long-running activity must not restart the controller or close the door."""
        for elapsed in (499, 500, 501, 3600):
            with self.subTest(elapsed=elapsed):
                controller = self.make_door()
                controller.open = True
                controller.open_time = self.now - elapsed
                controller.sensor.magnet = True
                controller.close_door = Mock()
                controller.update_door_state()
                controller.close_door.assert_not_called()
                self.exit_process.assert_not_called()

    def test_closed_or_locked_door_is_not_automatically_closed(self):
        for opened, locked in ((False, False), (True, True)):
            with self.subTest(opened=opened, locked=locked):
                door = self.make_door()
                door.open, door.lock = opened, locked
                door.open_time = self.now - 100
                door.close_door = Mock()
                door.update_door_state()
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
        door.close_check.assert_not_called()
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
        """Returns:
            Sensor: A sensor with a fake reader and a fixed magnetic baseline.
        """
        sensor = self.module.Sensor(on_magnet_detected=Mock(), read_magnetic=Mock(return_value=(100, 0, 0)))
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
