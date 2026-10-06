"""Check a fresh phone photo with the trained obstruction classifier."""

from io import BytesIO
import threading
import time

from door_camera import DEFAULT_DIRECTORY
from pull_photo import pull_photo


_classifier = None
_inference_lock = threading.Lock()


def _get_classifier():
    """Returns:
        Classifier: The cached model; call while holding the inference lock.
    """
    global _classifier
    if _classifier is None:
        import torch
        from fat_cat_model.inference import Classifier
        torch.set_num_threads(2)
        _classifier = Classifier()
    return _classifier


def run_inference(image, log_message=print):
    """Returns:
        bool: Whether the trained classifier labels these JPEG bytes unobstructed.

    Raises:
        ValueError: If no image bytes were supplied or the model output is invalid.
        OSError: If the model or image cannot be read.
    """
    if not image:
        raise ValueError("Cannot run inference without an image")
    with _inference_lock:
        result = _get_classifier().predict(BytesIO(image))
    log_message("inference: result=%s score=%.6f threshold=%.6f" %
                (result["label"], result["obstructed_score"], result["threshold"]))
    return result["label"] == "unobstructed"


def warm_up_model(log_message=print):
    """Load and warm the cached model with a synthetic image; log readiness or failure."""
    started = time.monotonic()
    log_message("warming up obstruction model")
    try:
        from PIL import Image
        image = BytesIO()
        Image.new("RGB", (384, 384), (124, 116, 104)).save(image, format="JPEG")
        # Uses the same lock and classifier as real checks; the result is discarded.
        image.seek(0)
        with _inference_lock:
            _get_classifier().predict(image)
    except Exception as error:
        log_message("model warm-up failed: %s" % error)
        return
    log_message("obstruction model ready in %.1f seconds" % (time.monotonic() - started))


def check_before_close(log_message=print):
    """Request a new photo and run inference on that exact saved upload.

    Returns:
        bool: Whether inference allows closing.

    Raises:
        TimeoutError: If the phone does not supply the requested photo in time.
        OSError: If the server or saved photo cannot be accessed.
        ValueError: If the response or image is invalid.
        KeyError: If the response lacks the image filename.
    """
    metadata = pull_photo()
    image = (DEFAULT_DIRECTORY / metadata["filename"]).read_bytes()
    return run_inference(image, log_message=log_message)
