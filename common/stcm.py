"""Read the explore grid from Slamtec's length-prefixed STCM layers.

The layout is inferred from files, not taken from a Slamtec spec: verified
against all 16 maps saved by Tianyi 2.0's chassis (2026-11). Each holds five
layers — the explore grid first, then virtual walls, virtual tracks, POIs and
rectangle areas — and the four after the grid are present (421 bytes) even
when empty, which is why counting the grid back from the end of the file is
always wrong. A file this decoder rejects is evidence the layout differs.
"""

import math
import struct


def decode_stcm(raw: bytes) -> dict:
    """Return grid geometry and row-major uint8 cells; reject malformed containers."""
    if len(raw) < 18 or raw[:4] != b"STCM":
        raise ValueError("invalid STCM header")
    # Little-endian container header; each layer length includes its uint32 prefix.
    header_size = struct.unpack_from("<H", raw, 4)[0]
    layer_count = struct.unpack_from("<I", raw, 12)[0]
    if not 18 <= header_size <= len(raw):
        raise ValueError("invalid STCM header size")

    offset = header_size
    grid = None
    for _ in range(layer_count):
        if offset + 6 > len(raw):
            raise ValueError("truncated STCM layer header")
        size, property_count = struct.unpack_from("<IH", raw, offset)
        end = offset + size
        if size < 6 or end > len(raw):
            raise ValueError("invalid STCM layer length")
        cursor = offset + 6

        def read_string():
            nonlocal cursor
            if cursor + 2 > end:
                raise ValueError("truncated STCM property length")
            length = struct.unpack_from("<H", raw, cursor)[0]
            cursor += 2
            if cursor + length > end:
                raise ValueError("STCM property exceeds layer")
            value = raw[cursor:cursor + length]
            cursor += length
            return value

        properties = {}
        for _ in range(property_count):
            key, value = read_string(), read_string()
            if key in properties:
                raise ValueError("duplicate STCM property")
            properties[key] = value
        if (properties.get(b"type") == b"vnd.slamtec.map-layer/vnd.grid-map+binary"
                and properties.get(b"usage") == b"explore"):
            if grid is not None:
                raise ValueError("multiple STCM explore grids")
            try:
                width = int(properties[b"dimension_width"])
                height = int(properties[b"dimension_height"])
                origin = (float(properties[b"origin_x"]), float(properties[b"origin_y"]))
                rx, ry = float(properties[b"resolution_x"]), float(properties[b"resolution_y"])
            except (KeyError, ValueError) as exc:
                raise ValueError("invalid STCM grid metadata") from exc
            if (width <= 0 or height <= 0 or rx <= 0 or ry <= 0
                    or not all(math.isfinite(v) for v in (*origin, rx, ry))):
                raise ValueError("invalid STCM grid geometry")
            if end - cursor != width * height:
                raise ValueError("STCM grid payload size does not match dimensions")
            grid = {"width": width, "height": height, "origin": origin,
                    "resolution_x": rx, "resolution_y": ry, "cells": raw[cursor:end]}
        offset = end
    if offset != len(raw):
        raise ValueError("STCM layer count does not cover file")
    if grid is None:
        raise ValueError("STCM does not contain an explore grid")
    return grid
