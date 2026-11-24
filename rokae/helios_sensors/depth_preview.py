"""Encode a small colour preview while leaving native depth measurements intact."""

from io import BytesIO
import zlib

import numpy as np
from PIL import Image

from .orbbec import HEIGHT, WIDTH


# Distances in millimetres. Nearby surfaces get the strongest colours.
_DEPTH_STOPS = (
    (0, 94, 20, 16),
    (300, 168, 33, 15),
    (800, 232, 116, 30),
    (1450, 243, 196, 82),
    (1800, 201, 204, 142),
    (2100, 180, 210, 204),
    (5000, 230, 228, 221),
)
_DISTANCES, *_COLORS = zip(*_DEPTH_STOPS)
_COLOR_TABLE = np.column_stack([
    np.interp(np.arange(5001), _DISTANCES, channel) for channel in _COLORS
]).astype(np.uint8)
_COLOR_TABLE[0] = 0


def depth_preview_jpeg(compressed_depth):
    try:
        decoder = zlib.decompressobj()
        raw = decoder.decompress(compressed_depth, HEIGHT * WIDTH * 2 + 1)
    except zlib.error as exc:
        raise ValueError("invalid compressed depth preview input") from exc
    if not decoder.eof or len(raw) != HEIGHT * WIDTH * 2:
        raise ValueError("invalid depth preview frame length")
    depth = np.frombuffer(raw, dtype="<u2").reshape(HEIGHT, WIDTH)
    pixels = _COLOR_TABLE[np.minimum(depth[::2, ::2], len(_COLOR_TABLE) - 1)]
    with BytesIO() as output:
        Image.fromarray(pixels).save(output, format="JPEG", quality=65)
        return output.getvalue()
