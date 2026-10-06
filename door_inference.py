"""Check a fresh phone photo with the trained obstruction classifier."""

from io import BytesIO
import threading

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


def run_inference(image):
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
    return result["label"] == "unobstructed"


def check_before_close():
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
    return run_inference(image)
