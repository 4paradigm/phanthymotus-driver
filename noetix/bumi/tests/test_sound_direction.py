import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sound_direction import estimate_angle, estimate_signature


def test_estimate_signature_uses_interleaved_head_microphones():
    rng = np.random.default_rng(7)
    source = rng.normal(0, 2000, 4096).astype(np.int16)
    shifts = (0, 2, -3, 1)
    channels = [np.roll(source, shift) for shift in shifts]
    # 5-6 are reserved; 7-8 are speaker loopback and must not affect DOA.
    channels.extend([np.zeros_like(source) for _ in range(4)])
    audio = np.stack(channels, axis=1).reshape(-1)

    signature = estimate_signature(audio, channels=8, sample_rate=16000)

    assert signature is not None
    assert np.allclose(signature, (2, -3, 1), atol=0.25)


def test_two_known_directions_calibrate_full_circle():
    front = (2.0, -1.0, 0.0)
    right = (0.0, 1.0, -2.0)

    assert estimate_angle(front, front, right) == 0
    assert estimate_angle(right, front, right) == 90
    assert estimate_angle(tuple(-x for x in front), front, right) == 180
    assert estimate_angle(tuple(-x for x in right), front, right) == 270


def test_silence_and_degenerate_calibration_do_not_report_a_direction():
    assert estimate_signature(np.zeros(8 * 2048, dtype=np.int16), 8, 16000) is None
    assert estimate_angle((1, 0, 0), (1, 0, 0), (2, 0, 0)) is None


def test_unrelated_loud_sounds_do_not_look_like_one_source():
    rng = np.random.default_rng(11)
    unrelated = rng.normal(0, 2000, (4096, 8)).astype(np.int16)

    assert estimate_signature(unrelated.reshape(-1), 8, 16000) is None
