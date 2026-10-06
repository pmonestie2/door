"""Image storage checks; no hardware or web-server tests."""

from pathlib import Path
import tempfile
import unittest

from door_camera import CameraImages, CameraCommands, MAX_IMAGE_BYTES


class CameraImagesTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name) / "images"
        self.storage = CameraImages(self.directory)
        # Framing checks do not claim to decode JPEGs.
        self.image = b"\xff\xd8example\xff\xd9"

    def test_latest_survives_restart_with_capture_and_receipt_times(self):
        saved = self.storage.save(self.image, "2026-10-05T08:00:00-07:00")
        metadata, image = CameraImages(self.directory).latest()
        self.assertEqual(image, self.image)
        self.assertEqual(metadata, saved)
        self.assertEqual(metadata["captured_at"], "2026-10-05T15:00:00+00:00")
        self.assertIn("received_at", metadata)

    def test_push_keeps_all_images_and_their_metadata(self):
        names = [self.storage.save(self.image, "2026-10-05T15:00:00Z")["filename"]
                 for _ in range(7)]
        for name in names:
            self.assertTrue((self.directory / name).exists())
            self.assertTrue((self.directory / name).with_suffix(".json").exists())
        self.assertEqual(len(list(self.directory.glob("*.jpg"))), 7)
        self.assertEqual(self.storage.latest()[0]["filename"], names[-1])

    def test_pull_keeps_five_latest_images_and_matching_metadata(self):
        """Prune old uploads on pull, including after reopening storage."""
        names = [self.storage.save(self.image, "2026-10-05T15:00:00Z")["filename"]
                 for _ in range(7)]
        unrelated = self.directory / "reference.jpg"
        unrelated.write_bytes(self.image)
        storage = CameraImages(self.directory)
        for _ in range(2):
            latest = storage.save(self.image, "2026-10-05T15:00:00Z", "a" * 32)
            names.append(latest["filename"])
            for name in names:
                expected = name in names[-5:]
                self.assertEqual((self.directory / name).exists(), expected)
                self.assertEqual((self.directory / name).with_suffix(".json").exists(), expected)
            self.assertEqual(storage.latest(), (latest, self.image))
            self.assertTrue(unrelated.exists())

    def test_rejects_bad_images_and_capture_times(self):
        for image, captured in ((b"not JPEG", "2026-10-05T15:00:00Z"),
                                (self.image, "invalid"),
                                (self.image, "2026-10-05T15:00:00"),
                                (b"\xff\xd8" + b"x" * MAX_IMAGE_BYTES + b"\xff\xd9",
                                 "2026-10-05T15:00:00Z")):
            with self.subTest(captured=captured, length=len(image)):
                with self.assertRaises(ValueError):
                    self.storage.save(image, captured)
        self.assertFalse(self.directory.exists())

    def test_no_latest_before_first_upload(self):
        with self.assertRaises(FileNotFoundError):
            self.storage.latest()


class CameraCommandTests(unittest.TestCase):
    def setUp(self):
        self.now = 0
        self.commands = CameraCommands(clock=lambda: self.now)

    def test_command_is_claimed_once_and_matched_to_uploaded_image(self):
        request = self.commands.request_capture()
        command = self.commands.wait_for_command(timeout=0)
        self.assertEqual(command["request_id"], request["request_id"])
        self.assertIsNone(self.commands.wait_for_command(timeout=0))
        self.commands.complete("wrong-id", {"filename": "wrong.jpg"})
        self.assertEqual(self.commands.status(request["request_id"])["status"], "capturing")
        self.commands.complete(request["request_id"], {"filename": "right.jpg"})
        result = self.commands.status(request["request_id"])
        self.assertEqual(result["status"], "complete")
        self.assertEqual(result["image"]["filename"], "right.jpg")

    def test_duplicate_request_is_rejected_until_timeout(self):
        request = self.commands.request_capture()
        with self.assertRaises(RuntimeError):
            self.commands.request_capture()
        self.now = 30
        self.assertEqual(self.commands.status(request["request_id"])["status"], "timed_out")
        self.assertIsNone(self.commands.wait_for_command(timeout=0))
        self.assertNotEqual(self.commands.request_capture()["request_id"], request["request_id"])

    def test_late_photo_does_not_complete_expired_request(self):
        request = self.commands.request_capture()
        self.commands.wait_for_command(timeout=0)
        self.now = 31
        self.commands.complete(request["request_id"], {"filename": "late.jpg"})
        self.assertEqual(self.commands.status(request["request_id"])["status"], "timed_out")

    def test_disconnected_phone_does_not_claim_command(self):
        request = self.commands.request_capture()
        self.assertIsNone(self.commands.wait_for_command(timeout=0, disconnected=lambda: True))
        self.assertEqual(self.commands.status(request["request_id"])["status"], "waiting")
        self.assertEqual(self.commands.wait_for_command(timeout=0)["request_id"], request["request_id"])

    def test_waiting_phone_wakes_for_new_command(self):
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(max_workers=1) as pool:
            waiting = pool.submit(self.commands.wait_for_command, 2)
            request = self.commands.request_capture()
            self.assertEqual(waiting.result(timeout=3)["request_id"], request["request_id"])
