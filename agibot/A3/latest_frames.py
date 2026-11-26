"""Newest-frame cache shared by the A3 camera mirror paths.

The mirror has two writers (relay forward, in-container re-encode) and one
reader (the snapshot card). This module is that seam. It owns the pieces the
card and the writers used to know separately:

- payload normalisation: rclpy hands a ``uint8[]`` over as ``array('B')`` while
  the in-container encoder assigns ``bytes``; both become ``bytes`` here;
- admission: only streams some card registered are kept, and a stream declared
  as ``jpeg`` never stores a payload without the JPEG SOI/EOI markers;
- the timestamp contract: every frame is stamped with the injected monotonic
  clock, so callers never pass or interpret timestamps;
- freshness: ``get`` refuses frames older than the requested age (or with a
  timestamp from the future, which means the clock went backwards).

The relay publishes ten camera streams; the card reads one. Keys nobody
registered are dropped without copying, so the cache stays proportional to what
is consumed instead of what is published.
"""

import time
from array import array
from typing import NamedTuple

JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"


def looks_like_jpeg(data):
    return data.startswith(JPEG_SOI) and data.endswith(JPEG_EOI)


class Frame(NamedTuple):
    """A cached frame plus its age at the moment it was read."""

    data: bytes
    age_s: float


class LatestFrameCache:
    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._consumers = {}
        self._frames = {}

    # -- interest --

    def register(self, key, jpeg=False):
        """Declare that some card consumes this stream.

        Only registered streams are copied into the cache. ``jpeg`` also gates
        admission on the JPEG markers; depth streams arrive zlib-compressed and
        must not be declared this way."""
        self._consumers[key] = bool(jpeg)
        return self

    def is_registered(self, key):
        return key in self._consumers

    def registered_keys(self):
        return tuple(sorted(self._consumers))

    # -- writers --

    def put(self, key, payload):
        """Keep the newest frame if the stream has a consumer and is valid.

        ``payload`` may be a message with ``.data`` (CompressedImage) or the
        raw payload itself. Returns True when the frame was kept; an ignored
        payload leaves the previous frame in place."""
        if key not in self._consumers:
            return False
        data = getattr(payload, "data", payload)
        if not isinstance(data, (bytes, bytearray, memoryview, array)):
            return False
        data = bytes(data)
        if self._consumers[key] and not looks_like_jpeg(data):
            return False
        self._frames[key] = (data, self._clock())
        return True

    # -- readers --

    def get(self, key, max_age_s=None):
        """Return the newest frame no older than ``max_age_s``, else None."""
        entry = self._frames.get(key)
        if entry is None:
            return None
        data, stamped = entry
        age = self._clock() - stamped
        if age < 0 or (max_age_s is not None and age > max_age_s):
            return None
        return Frame(data, age)

    def age(self, key):
        """Seconds since the newest frame, or None when nothing is cached.

        May return a negative value if the clock went backwards; ``get``
        refuses such frames, callers that only report it should clamp."""
        entry = self._frames.get(key)
        if entry is None:
            return None
        return self._clock() - entry[1]
