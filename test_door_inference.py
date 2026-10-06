"""Fresh-photo selection and classifier integration tests without a phone."""

from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import door_inference


class InferenceTests(unittest.TestCase):
    def test_checks_requested_image_instead_of_latest_upload(self):
        """A concurrent upload must not replace the photo sent to inference."""
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / "requested.jpg").write_bytes(b"requested photo")
            (directory / "latest.jpg").write_bytes(b"unrelated photo")
            with patch.object(door_inference, "DEFAULT_DIRECTORY", directory), \
                    patch.object(door_inference, "pull_photo",
                                 return_value={"filename": "requested.jpg"}) as capture, \
                    patch.object(door_inference, "run_inference", return_value=False) as inference:
                self.assertFalse(door_inference.check_before_close())
                capture.assert_called_once_with()
                inference.assert_called_once_with(b"requested photo")

    def test_capture_failure_does_not_run_inference(self):
        """Missing captures cannot fall back to an old photo."""
        with patch.object(door_inference, "pull_photo", side_effect=TimeoutError), \
                patch.object(door_inference, "run_inference") as inference:
            with self.assertRaises(TimeoutError):
                door_inference.check_before_close()
            inference.assert_not_called()

    def test_only_unobstructed_model_result_allows_closing(self):
        """Pass the requested bytes to the model and reject obstructions or unknown labels."""
        for label in ("obstructed", "unobstructed", "unknown"):
            with self.subTest(label=label), patch.object(door_inference, "_get_classifier") as get_model:
                get_model.return_value.predict.return_value = {"label": label}
                self.assertEqual(door_inference.run_inference(b"photo"), label == "unobstructed")
                image = get_model.return_value.predict.call_args.args[0]
                self.assertEqual(image.getvalue(), b"photo")

    def test_empty_input_does_not_load_model(self):
        """Reject empty data before loading the classifier."""
        with patch.object(door_inference, "_get_classifier") as get_model:
            with self.assertRaises(ValueError):
                door_inference.run_inference(b"")
            get_model.assert_not_called()

    def test_model_failure_propagates_to_closing_guard(self):
        """A failed prediction must not allow closing."""
        with patch.object(door_inference, "_get_classifier") as get_model:
            get_model.return_value.predict.side_effect = OSError("unreadable image")
            with self.assertRaises(OSError):
                door_inference.run_inference(b"bad image")
