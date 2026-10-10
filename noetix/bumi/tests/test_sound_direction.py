import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sound_direction import SoundActivityGate, estimate_angle, estimate_signature, is_voiced_audio


def test_activity_gate_detects_normal_speech_with_dc_offset_and_short_pause():
    gate = SoundActivityGate()

    def frame(level, dc=100):
        wave = np.tile([dc + level, dc - level], 320)
        return np.stack([wave] * 4 + [np.zeros_like(wave)] * 4, axis=1).reshape(-1)

    assert not any(gate.accepts(frame(40)) for _ in range(8))
    assert not gate.accepts(frame(85))  # 单帧升高不等于持续说话。
    assert gate.accepts(frame(85))
    assert not any(gate.accepts(frame(40)) for _ in range(6))
    assert not gate.accepts(frame(60))  # 停顿期不以较低门限刷新声源方向。
    assert not gate.accepts(frame(85))
    assert gate.accepts(frame(85))  # 下一次明显发声后继续定位。
    assert not gate.accepts(np.zeros(8 * 640 + 1, dtype=np.int16))


def test_activity_gate_does_not_extend_voice_on_modest_continuous_noise():
    gate = SoundActivityGate()

    def frame(level):
        wave = np.tile([level, -level], 320)
        return np.stack([wave] * 4 + [np.zeros_like(wave)] * 4, axis=1).reshape(-1)

    assert not any(gate.accepts(frame(40)) for _ in range(8))
    assert not gate.accepts(frame(85))
    assert gate.accepts(frame(85))
    # 短暂发声之后，风扇声约为背景的 1.33 倍，不应持续刷新方向。
    assert not any(gate.accepts(frame(53)) for _ in range(16))
    assert not gate.accepts(frame(60))  # 保持期已经结束。


def test_activity_gate_uses_user_thresholds_without_changing_defaults():
    def frame(level):
        wave = np.tile([level, -level], 320)
        return np.stack([wave] * 4 + [np.zeros_like(wave)] * 4, axis=1).reshape(-1)

    default = SoundActivityGate()
    strict = SoundActivityGate(onset_ratio=2.5, burst_ratio=4.0)
    for _ in range(8):
        assert not default.accepts(frame(40))
        assert not strict.accepts(frame(40))
    assert not default.accepts(frame(85))
    assert default.accepts(frame(85))
    assert not strict.accepts(frame(85))
    assert not strict.accepts(frame(85))


def test_estimator_rejects_bad_inputs_and_off_basis_direction():
    front, right = (2.0, -1.0, 0.0), (0.0, 1.0, -2.0)
    assert estimate_signature(np.zeros(8 * 100, dtype=np.int16), 8, 16000) is None
    assert estimate_signature(np.zeros(8 * 2048 + 1, dtype=np.int16), 8, 16000) is None
    assert estimate_signature(np.zeros(8 * 2048, dtype=np.int16), 4, 16000) is None
    assert estimate_angle(tuple(a + b for a, b in zip(front, right)), front, right) == 45
    assert estimate_angle((1, 1, 10), front, right) is None
    assert estimate_angle((float("nan"), 1, 0), front, right) is None
    assert estimate_angle(front, (float("inf"), 0, 0), right) is None


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


def test_estimate_signature_resolves_subsample_delays():
    rng = np.random.default_rng(19)
    source = rng.normal(0, 2000, 4096)
    shifts = (0.0, 1.5, -2.25, 0.75)
    channels = [np.fft.irfft(
        np.fft.rfft(source) * np.exp(-2j * np.pi * np.fft.rfftfreq(len(source)) * shift),
        n=len(source)).astype(np.int16) for shift in shifts]
    channels.extend([np.zeros(len(source), dtype=np.int16) for _ in range(4)])

    signature = estimate_signature(np.stack(channels, axis=1).reshape(-1), 8, 16000)

    assert signature is not None
    assert np.allclose(signature, shifts[1:], atol=0.35)


def test_calibration_voice_check_rejects_coherent_noise_and_tone():
    rng = np.random.default_rng(31)
    t = np.arange(32000) / 16000
    speech_like = (400 + 500 * np.sin(2 * np.pi * 2 * t) ** 2) * sum(
        np.sin(2 * np.pi * 150 * harmonic * t) / harmonic
        for harmonic in range(1, 9))
    white_noise = rng.normal(0, 800, len(t))
    single_tone = 900 * np.sin(2 * np.pi * 500 * t)

    def microphone_audio(source):
        channels = [np.roll(source.astype(np.int16), shift)
                    for shift in (0, 2, -3, 1)]
        channels.extend([np.zeros(len(source), dtype=np.int16) for _ in range(4)])
        return np.stack(channels, axis=1).reshape(-1)

    assert is_voiced_audio(microphone_audio(speech_like), 8, 16000)
    assert estimate_signature(microphone_audio(white_noise), 8, 16000) is not None
    assert not is_voiced_audio(microphone_audio(white_noise), 8, 16000)
    assert not is_voiced_audio(microphone_audio(single_tone), 8, 16000)
