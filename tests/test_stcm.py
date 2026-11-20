"""Synthetic STCM containers: no robot maps or POI data."""

import struct

import pytest

from common.stcm import decode_stcm
from simulator.generic.tools.import_stcm import read_stcm


def layer(properties, payload=b""):
    body = struct.pack("<H", len(properties))
    for key, value in properties.items():
        for item in (key, str(value)):
            encoded = item.encode()
            body += struct.pack("<H", len(encoded)) + encoded
    body += payload
    return struct.pack("<I", len(body) + 4) + body


def container(*layers, extension=b""):
    return b"STCM" + struct.pack("<HHHHIH", 18 + len(extension), 1, 1, 1, len(layers), 0) + extension + b"".join(layers)


def grid_layer(cells=b"\x7f\x00\xff\x00\x32\x7f", **overrides):
    properties = {"dimension_width": 3, "dimension_height": 2,
                  "origin_x": -2, "origin_y": 3,
                  "resolution_x": 0.2, "resolution_y": 0.4,
                  "type": "vnd.slamtec.map-layer/vnd.grid-map+binary", "usage": "explore"}
    properties.update(overrides)
    return layer(properties, cells)


SEMANTIC = layer({"type": "vnd.slamtec.map-layer/vnd.line-map+binary",
                  "usage": "virtual_walls", "count": 0})


@pytest.mark.parametrize("layers", [(grid_layer(),), (SEMANTIC, grid_layer()),
                                   (grid_layer(), SEMANTIC), (SEMANTIC, grid_layer(), SEMANTIC)])
def test_grid_payload_is_bounded_by_its_layer(layers, tmp_path):
    raw = container(*layers, extension=b"extra header")
    grid = decode_stcm(raw)
    assert grid == {"width": 3, "height": 2, "origin": (-2, 3),
                    "resolution_x": 0.2, "resolution_y": 0.4,
                    "cells": b"\x7f\x00\xff\x00\x32\x7f"}
    path = tmp_path / "synthetic.stcm"
    path.write_bytes(raw)
    assert read_stcm(path) == {"width": 3, "height": 2, "origin": (-2, 3),
                               "resolution": 0.2, "cells": grid["cells"]}


def test_metadata_is_scoped_to_the_explore_layer():
    decoy = grid_layer(origin_x=999, usage="other")
    assert decode_stcm(container(decoy, grid_layer(), SEMANTIC))["origin"] == (-2, 3)


def test_grid_after_large_semantic_payload_is_not_limited_to_metadata_prefix():
    preceding = layer({"type": "other", "usage": "other"}, b"x" * 70000)
    assert decode_stcm(container(preceding, grid_layer()))["width"] == 3


def test_missing_numeric_metadata_is_rejected():
    raw = container(layer({"type": "vnd.slamtec.map-layer/vnd.grid-map+binary",
                           "usage": "explore"}))
    with pytest.raises(ValueError, match="metadata"):
        decode_stcm(raw)


def test_every_truncation_is_rejected():
    raw = container(SEMANTIC, grid_layer(), SEMANTIC)
    for length in range(len(raw)):
        with pytest.raises(ValueError):
            decode_stcm(raw[:length])


@pytest.mark.parametrize("offset,fmt,value", [
    (4, "<H", 0), (4, "<H", 65535),
    (12, "<I", 0), (12, "<I", 2),
    (18, "<I", 0), (18, "<I", 5), (18, "<I", 2**32 - 1),
    (22, "<H", 65535), (24, "<H", 65535),
    (41, "<H", 65535),  # first property's value length
])
def test_invalid_lengths_and_counts_are_rejected(offset, fmt, value):
    raw = bytearray(container(grid_layer()))
    struct.pack_into(fmt, raw, offset, value)
    with pytest.raises(ValueError):
        decode_stcm(bytes(raw))


@pytest.mark.parametrize("change", [
    {"dimension_width": 0}, {"dimension_height": -1}, {"dimension_width": 4},
    {"resolution_x": 0}, {"resolution_y": -0.1}, {"resolution_x": "nan"},
    {"origin_y": "inf"}, {"dimension_width": "invalid"},
])
def test_invalid_grid_geometry_is_rejected(change):
    with pytest.raises(ValueError):
        decode_stcm(container(grid_layer(**change)))


@pytest.mark.parametrize("raw", [container(SEMANTIC), container(grid_layer(usage="other")),
                                container(grid_layer(), grid_layer()),
                                container(grid_layer(b"\x7f")),
                                container(grid_layer()) + b"trailing"])
def test_missing_duplicate_or_malformed_grid_is_rejected(raw):
    with pytest.raises(ValueError):
        decode_stcm(raw)
