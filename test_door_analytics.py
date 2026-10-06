"""Tests for transit counting and reports, without a web server."""

from datetime import datetime
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

from door_analytics import Analytics
from door import Door


class AnalyticsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.analytics = Analytics(Path(self.temp.name) / "events.sqlite3")
        self.now = datetime(2023, 8, 1, 12).timestamp()

    def total(self, dataset="live"):
        """Returns:
            int: Transit count in the fixture's six-month window.
        """
        return self.analytics.report(dataset=dataset)["total"]

    def test_bursts_require_quiet_not_just_elapsed_time(self):
        for offset in (0, 1, 7, 13, 20):
            self.analytics.magnet(self.now + offset)
        self.assertEqual(self.total(), 1)
        self.analytics.magnet(self.now + 28)
        self.assertEqual(self.total(), 2)

    def test_motor_events_and_settling_do_not_double_count(self):
        self.analytics.magnet(self.now, blocked=True)
        self.analytics.movement_finished(self.now + 16, "OPEN", "magnet")
        self.analytics.magnet(self.now + 17)
        self.analytics.magnet(self.now + 20, blocked=True)
        self.analytics.movement_finished(self.now + 38, "CLOSE", "automatic")
        self.assertEqual(self.total(), 1)
        report = self.analytics.report(dataset="live")
        self.assertEqual((report["opens"], report["closes"]), (1, 1))

    def test_schedule_open_still_records_magnet_visits(self):
        clock = Mock()
        clock.time.return_value = self.now
        controller = Door(sensor=Mock(), motor=Mock(), direction=Mock(),
                          clock=clock, analytics=self.analytics)
        controller.open = True
        controller.open_door = Mock()
        controller.open_door_from_magnet()
        clock.time.return_value += 10
        controller.open_door_from_magnet()
        self.assertEqual(self.total(), 2)
        controller.lock = True
        clock.time.return_value += 10
        controller.open_door_from_magnet()
        self.assertEqual(self.total(), 2)

    def test_six_month_window_and_buckets(self):
        for day in ("2023-06-30", "2023-07-01", "2023-12-31", "2024-01-01"):
            self.analytics.record(datetime.fromisoformat(day + "T12:00:00").timestamp(),
                                  "TRANSIT", "magnet")
        report = self.analytics.report(dataset="live")
        self.assertEqual(report["start"], "2023-07-01")
        self.assertEqual(len(report["counts"]), 184)
        self.assertEqual(report["total"], 2)
        report = self.analytics.report(dataset="live", bucket="hour")
        self.assertEqual(report["counts"]["12:00"], 2)
        self.assertEqual(report["busiest"], ["12:00"])
