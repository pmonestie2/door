"""Persistent storage for phone snapshots; independent of door control and inference."""

from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import uuid

MAX_IMAGE_BYTES = 2 * 1024 * 1024
KEEP_IMAGES_AFTER_PULL = 5
DEFAULT_DIRECTORY = Path(__file__).with_name("camera_images")


class CameraImages:
    def __init__(self, directory=DEFAULT_DIRECTORY):
        """Configure storage without creating files until the first upload."""
        self.directory = Path(directory)
        self.lock = threading.Lock()

    def save(self, image, captured_at, request_id=None):
        """Returns:
            dict: Image filename, phone capture time, and server receipt time in UTC.

        Raises:
            ValueError: If the timestamp or basic JPEG framing is invalid.
            OSError: If the image cannot be saved.
        """
        if not 4 <= len(image) <= MAX_IMAGE_BYTES or not (
                image.startswith(b"\xff\xd8") and image.endswith(b"\xff\xd9")):
            raise ValueError("Expected a JPEG no larger than 2 MiB")
        if request_id is not None and (len(request_id) != 32 or any(c not in "0123456789abcdef" for c in request_id)):
            raise ValueError("Invalid capture request ID")
        captured = datetime.fromisoformat(captured_at.replace("Z", "+00:00"))
        if captured.tzinfo is None:
            raise ValueError("Capture time must include a timezone")
        received = datetime.now(timezone.utc)
        name = received.strftime("%Y%m%dT%H%M%S%fZ_") + uuid.uuid4().hex[:8] + ".jpg"
        metadata = dict(filename=name, captured_at=captured.astimezone(timezone.utc).isoformat(),
                        received_at=received.isoformat())
        if request_id is not None:
            metadata["request_id"] = request_id
        with self.lock:
            self.directory.mkdir(parents=True, exist_ok=True)
            image_path = self.directory / name
            image_path.write_bytes(image)
            image_path.with_suffix(".json").write_text(json.dumps(metadata))
            temporary = self.directory / "latest.tmp"
            temporary.write_text(json.dumps(metadata))
            temporary.replace(self.directory / "latest.json")
            if request_id is not None:
                self._prune_images(name)
        return metadata

    def _prune_images(self, newest):
        """Keep five snapshots after a pull; call while holding the storage lock."""
        images = sorted(path for path in self.directory.glob("????????T????????????Z_????????.jpg")
                        if path.name != newest)
        for path in images[:-(KEEP_IMAGES_AFTER_PULL - 1)]:
            path.unlink()
            path.with_suffix(".json").unlink(missing_ok=True)

    def latest(self):
        """Returns:
            tuple[dict, bytes]: Latest image metadata and JPEG, read under the storage lock.

        Raises:
            FileNotFoundError: If no image has been received.
        """
        with self.lock:
            metadata = json.loads((self.directory / "latest.json").read_text())
            image = (self.directory / metadata["filename"]).read_bytes()
        return metadata, image


class CameraCommands:
    def __init__(self, clock=None):
        """Track one outstanding capture and a bounded history of request results."""
        import time
        self.clock = time.monotonic if clock is None else clock
        self.condition = threading.Condition()
        self.requests = {}

    def _expire(self):
        """Mark unanswered requests as timed out; call while holding the condition."""
        for item in self.requests.values():
            if item["status"] in ("waiting", "capturing") and self.clock() >= item["deadline"]:
                item["status"] = "timed_out"

    def request_capture(self):
        """Returns:
            dict: A new request ID and waiting status, valid for thirty seconds.

        Raises:
            RuntimeError: If another capture is still outstanding.
        """
        with self.condition:
            self._expire()
            if any(item["status"] in ("waiting", "capturing") for item in self.requests.values()):
                raise RuntimeError("A camera request is already pending")
            request_id = uuid.uuid4().hex
            item = dict(request_id=request_id, status="waiting", deadline=self.clock() + 30)
            self.requests[request_id] = item
            while len(self.requests) > 100:
                del self.requests[next(iter(self.requests))]
            self.condition.notify_all()
            return dict(request_id=request_id, status="waiting")

    def wait_for_command(self, timeout=20, disconnected=None):
        """Returns:
            dict or None: One unclaimed capture command, or None after the wait expires.
        """
        import time
        stop = time.monotonic() + timeout
        with self.condition:
            while True:
                if disconnected is not None and disconnected():
                    return None
                self._expire()
                for item in self.requests.values():
                    if item["status"] == "waiting":
                        item["status"] = "capturing"
                        return dict(command="capture", request_id=item["request_id"])
                remaining = stop - time.monotonic()
                if remaining <= 0:
                    return None
                self.condition.wait(min(remaining, 1))

    def complete(self, request_id, metadata):
        """Match a saved photo to its active request; expired requests remain timed out."""
        with self.condition:
            self._expire()
            item = self.requests.get(request_id)
            if item is not None and item["status"] == "capturing":
                item.update(status="complete", image=metadata)

    def status(self, request_id):
        """Returns:
            dict or None: A request's status and matching image metadata, if known.
        """
        with self.condition:
            self._expire()
            item = self.requests.get(request_id)
            return {key: value for key, value in item.items() if key != "deadline"} if item else None
