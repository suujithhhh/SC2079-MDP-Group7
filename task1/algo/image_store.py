"""RAW camera images, RPi to PC (`PROTOCOL.md` §7.1): validation and storage.

The RPi sends the photo it took after each `SNAP`; the web page shows it live.
This module is display-only plumbing: it does no recognition, draws no box,
chooses no "best" image and never touches a plan or an STM command. The one
recognition-related value it carries is the optional `target_id` the RPi
reports; it is stored and shown as given, never checked against a picture. It holds
the photos in memory only (nothing is written to disk, a restart clears them)
and has no HTTP code: `server.py` calls it.

Rules (all enforced here, tested in `tests/test_server_images.py`):
- one photo per obstacle, a newer photo of the same obstacle replaces it;
- a photo with a different `mission_id` starts a new set and clears the old
  one, so at most 8 photos (at most 16 MB) are ever held;
- the PC names the image (`image_id`), a client never names a file;
- `target_id` is optional: absent means "no ID reported", present it is an
  integer from 11 to 40 (the NTU image-ID range) or the upload is rejected;
- the body is only checked for being a complete JPEG (`FF D8 FF` ... `FF D9`)
  and for its size; the resolution is not enforced and the image is not decoded.
"""

import re
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

MAX_IMAGE_BYTES = 2 * 1024 * 1024
MAX_MISSION_ID_CHARS = 128
OBSTACLE_ID_MIN = 1
OBSTACLE_ID_MAX = 8
TARGET_ID_MIN = 11
TARGET_ID_MAX = 40

_OBSTACLE_ID_RE = re.compile(r"[1-8]")  # exactly one digit: no sign, space or leading zero
_TARGET_ID_RE = re.compile(r"[1-4][0-9]")  # exactly two digits: no sign, space or leading zero
IMAGE_ID_RE = re.compile(r"[0-9]{1,2}-[0-9]{1,12}")
_JPEG_START = b"\xff\xd8\xff"
_JPEG_END = b"\xff\xd9"


class ImageRejected(Exception):
    """A client error: carries the HTTP status and a short public message."""

    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status = status
        self.message = message


def parse_obstacle_id(text) -> int:
    """A single digit 1-8 (`1.5`, `+3`, ` 3`, `08`, `0`, `9` and `` are rejected)."""
    if not isinstance(text, str) or not _OBSTACLE_ID_RE.fullmatch(text):
        raise ImageRejected(400, "obstacle_id must be an integer from 1 to 8")
    value = int(text)
    if not OBSTACLE_ID_MIN <= value <= OBSTACLE_ID_MAX:
        raise ImageRejected(400, "obstacle_id must be an integer from 1 to 8")
    return value


def parse_target_id(text) -> int:
    """Two digits, 11-40 (`1`, `011`, `+22`, ` 22`, `22.0`, `41`, `10` and `` are rejected)."""
    if not isinstance(text, str) or not _TARGET_ID_RE.fullmatch(text):
        raise ImageRejected(400, f"target_id must be an integer from {TARGET_ID_MIN} to {TARGET_ID_MAX}")
    value = int(text)
    if not TARGET_ID_MIN <= value <= TARGET_ID_MAX:
        raise ImageRejected(400, f"target_id must be an integer from {TARGET_ID_MIN} to {TARGET_ID_MAX}")
    return value


def validate_mission_id(value) -> str:
    """A non-empty string of at most 128 characters with no control characters."""
    if not isinstance(value, str) or not value:
        raise ImageRejected(400, "mission_id must be a non-empty string")
    if len(value) > MAX_MISSION_ID_CHARS:
        raise ImageRejected(400, f"mission_id must be at most {MAX_MISSION_ID_CHARS} characters")
    if any(ord(ch) < 32 or ord(ch) == 127 for ch in value):
        raise ImageRejected(400, "mission_id must not contain control characters")
    return value


def validate_jpeg(data: bytes) -> None:
    """Complete-JPEG check only: start marker, end marker, within the size limit."""
    if not data:
        raise ImageRejected(400, "request body must not be empty")
    if len(data) > MAX_IMAGE_BYTES:
        raise ImageRejected(413, f"image is larger than {MAX_IMAGE_BYTES} bytes")
    if not (len(data) >= 5 and data.startswith(_JPEG_START) and data.endswith(_JPEG_END)):
        raise ImageRejected(400, "body is not a complete JPEG")


@dataclass(frozen=True)
class StoredImage:
    image_id: str
    obstacle_id: int
    data: bytes
    received_at_s: float
    target_id: Optional[int] = None


class ImageStore:
    """Thread-safe, in-memory, one photo per obstacle for the current mission."""

    def __init__(self, clock: Callable[[], float] = time.time):
        self._clock = clock
        self._lock = threading.Lock()
        self._mission_id: Optional[str] = None
        self._by_obstacle: dict[int, StoredImage] = {}
        self._counter = 0
        self._revision = 0

    def store(self, mission_id: str, obstacle_id: int, data: bytes, target_id: Optional[int] = None) -> dict:
        """Validates and stores one photo; returns the `IMAGE_STORED` reply fields.

        `target_id` is the RPi's reported image ID (11-40) or None for "none
        reported". A newer photo replaces the older one completely, including
        its `target_id`: re-uploading without one clears it."""
        mission_id = validate_mission_id(mission_id)
        if not isinstance(obstacle_id, int) or isinstance(obstacle_id, bool) \
                or not OBSTACLE_ID_MIN <= obstacle_id <= OBSTACLE_ID_MAX:
            raise ImageRejected(400, "obstacle_id must be an integer from 1 to 8")
        if target_id is not None and (not isinstance(target_id, int) or isinstance(target_id, bool)
                                      or not TARGET_ID_MIN <= target_id <= TARGET_ID_MAX):
            raise ImageRejected(400, f"target_id must be an integer from {TARGET_ID_MIN} to {TARGET_ID_MAX}")
        validate_jpeg(data)
        with self._lock:
            if mission_id != self._mission_id:
                self._by_obstacle = {}
                self._mission_id = mission_id
            replaced = obstacle_id in self._by_obstacle
            self._counter += 1
            self._revision += 1
            image_id = f"{obstacle_id}-{self._counter}"
            self._by_obstacle[obstacle_id] = StoredImage(
                image_id=image_id, obstacle_id=obstacle_id, data=bytes(data), received_at_s=self._clock(),
                target_id=target_id,
            )
            return {
                "image_id": image_id,
                "mission_id": mission_id,
                "obstacle_id": obstacle_id,
                "target_id": target_id,
                "bytes": len(data),
                "replaced": replaced,
            }

    def list_images(self) -> dict:
        """The `IMAGE_LIST` reply: oldest first, so photos appear in snap order."""
        with self._lock:
            images = sorted(self._by_obstacle.values(), key=lambda im: (im.received_at_s, im.obstacle_id))
            return {
                "type": "IMAGE_LIST",
                "mission_id": self._mission_id,
                "revision": self._revision,
                "images": [
                    {
                        "image_id": im.image_id,
                        "obstacle_id": im.obstacle_id,
                        "target_id": im.target_id,
                        "bytes": len(im.data),
                        "received_at_s": im.received_at_s,
                    }
                    for im in images
                ],
            }

    def get(self, image_id: str) -> Optional[bytes]:
        """The JPEG bytes for a currently stored `image_id`, else None."""
        if not isinstance(image_id, str) or not IMAGE_ID_RE.fullmatch(image_id):
            return None
        with self._lock:
            for image in self._by_obstacle.values():
                if image.image_id == image_id:
                    return image.data
        return None
