"""Native protocol and two-channel assembly, without SDK or hardware."""

import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "linkerhand/l20_lite"))
from glove_direct_reader import GloveAssembler
from glove_protocol import Decoder


def packet(wire, seq=1, values=None, kind=1):
    values = values or [[0, 0, 0, 100]] * 6
    payload = bytes(v & 255 for x, y, z, w in values for v in (w, x, y, z))
    raw = bytes([0xA0, wire, seq, kind, len(payload)]) + payload
    check = 0
    for value in raw:
        check ^= value
    return raw + bytes([check, 0x0A])


@pytest.mark.parametrize(
    "wire,side,channel",
    [
        (0x20, "left", "root"),
        (0x10, "right", "root"),
        (0x2A, "left", "top"),
        (0x1A, "right", "top"),
    ],
)
def test_wire_channels_and_signed_component_order(wire, side, channel):
    values = [[-60, 0, 0, 80], [0, 0, 0, 0], *[[0, 0, 100, 0]] * 4]
    sample = Decoder().feed(packet(wire, 255, values))[0]
    assert (sample["side"], sample["channel"], sample["sequence"]) == (
        side,
        channel,
        255,
    )
    assert sample["quaternions"][0] == [-0.6, 0, 0, 0.8]
    assert sample["quaternions"][1] is None


def test_fragmented_mixed_stream_recovers_and_retains_sequence_wrap():
    stream = b"\x00\xff" + packet(0x20, 255) + packet(0x2A, 0) + packet(0x10, 1)
    for width in (1, 2, 5, 17, len(stream)):
        parser = Decoder()
        samples = []
        for offset in range(0, len(stream), width):
            samples.extend(parser.feed(stream[offset : offset + width]))
        assert [(s["side"], s["channel"], s["sequence"]) for s in samples] == [
            ("left", "root", 255),
            ("left", "top", 0),
            ("right", "root", 1),
        ]


@pytest.mark.parametrize(
    "fault", ["checksum", "terminator", "checksum_is_0a", "length"]
)
def test_bad_frames_never_publish_and_valid_successor_recovers(fault):
    bad = bytearray(packet(0x20))
    if fault == "checksum":
        bad[-2] ^= 1
    elif fault == "terminator":
        bad[-1] = 0
    elif fault == "checksum_is_0a":
        bad[-2] = 0x0A
    else:
        bad[4] = 255
    parser = Decoder()
    assert parser.feed(bad) == []
    assert parser.feed(packet(0x20, 2))[0]["sequence"] == 2


@pytest.mark.parametrize("wire", [0x20, 0x10, 0x2A, 0x1A])
@pytest.mark.parametrize("flags", [0, 0xC0])
def test_invalid_length_is_rejected_from_header_before_waiting_for_payload(wire, flags):
    for length in range(256):
        if length == 24:
            continue
        parser = Decoder()
        bad_header = bytes([0xA0, wire | flags, 7, 1 | flags, length])
        assert parser.feed(bad_header[:4]) == []
        assert parser.rejected == 0  # Length byte has not arrived yet.
        assert parser.feed(bad_header[4:]) == []
        assert parser.rejected == 1  # No advertised payload was required.
        assert len(parser.buffer) <= 1  # Only a possible next A0 sync byte remains.
        recovered = []
        for byte in packet(wire, seq=2):
            recovered.extend(parser.feed(bytes([byte])))
        assert len(recovered) == 1 and recovered[0]["sequence"] == 2
        assert parser.accepted == 1 and not parser.buffer


def update(assembler, wire, now, seq=1, values=None):
    for sample in Decoder().feed(packet(wire, seq, values)):
        assembler.update(sample, now)


def test_single_live_channel_cannot_keep_stale_pair_alive():
    a = GloveAssembler()
    update(a, 0x20, 1)
    update(a, 0x2A, 1)
    assert a.sample("left", 1) is not None
    update(a, 0x2A, 1.25, 2)
    assert a.sample("left", 1.25) is None
    update(a, 0x20, 1.26, 2)
    assert a.sample("left", 1.26) is not None
    assert a.sample("right", 1.26) is None


def test_invalid_roots_use_correct_tip_slots_but_missing_palm_is_rejected():
    a = GloveAssembler()
    roots = [[0, 0, 0, 100], [60, 0, 0, 80], *[[0, 0, 0, 0]] * 4]
    tips = [[0, 0, 100, 0], [0, 0, 0, 0], *[[0, 60, 0, 80]] * 4]
    update(a, 0x20, 1, values=roots)
    update(a, 0x2A, 1, values=tips)
    sample = a.sample("left", 1, wall_ms=1000)
    assert sample["root_quaternions"][1] == [0.6, 0, 0, 0.8]
    assert sample["tip_quaternions"][1] is None
    assert sample["quaternion_layout"] == "hex-root-top-v1"
    assert all(math.isfinite(v) for v in sample["pitch"] + sample["yaw"])
    roots[0] = [0, 0, 0, 0]
    update(a, 0x20, 1.1, values=roots)
    assert a.sample("left", 1.1) is None


def test_only_new_input_can_emit_even_when_sequence_repeats():
    a = GloveAssembler()
    update(a, 0x20, 1, 255)
    update(a, 0x2A, 1, 255)
    first = a.sample("left", 1)
    assert a.sample("left", 1.1) is None
    update(a, 0x20, 1.11, 255)
    second = a.sample("left", 1.11)
    assert second["sequence"] == first["sequence"]
    assert second["frame_marker"] != first["frame_marker"]
