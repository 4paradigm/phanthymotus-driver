import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sound_direction import SoundActivityGate, estimate_angle, estimate_signature


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


def test_activity_gate_keeps_accepting_during_sustained_speech():
    gate = SoundActivityGate()

    def frame(level, dc=100):
        wave = np.tile([dc + level, dc - level], 320)
        return np.stack([wave] * 4 + [np.zeros_like(wave)] * 4, axis=1).reshape(-1)

    assert not any(gate.accepts(frame(40)) for _ in range(8))
    assert not gate.accepts(frame(85))
    assert gate.accepts(frame(85))
    # 连续说话不能把自己抬成背景；同电平发声仍要持续定位。
    accepted = [gate.accepts(frame(85)) for _ in range(50)]
    assert sum(accepted) >= 40
    assert gate.accepts(frame(85))


def test_activity_gate_tracks_continuous_noise_without_true_silence():
    gate = SoundActivityGate()

    def frame(level, dc=100):
        wave = np.tile([dc + level, dc - level], 320)
        return np.stack([wave] * 4 + [np.zeros_like(wave)] * 4, axis=1).reshape(-1)

    assert not any(gate.accepts(frame(40)) for _ in range(8))
    # 持续噪声没有真静音；电平变低后底噪必须跟得上，否则后面的发声会被旧底噪卡住。
    assert not any(gate.accepts(frame(20)) for _ in range(40))
    # 40 落在旧底噪 40×2.0=80 与新底噪 20×2.0=40 之间：只有底噪跟着下调才会被判定为发声。
    assert not gate.accepts(frame(40))
    assert gate.accepts(frame(40))


def test_activity_gate_rejects_oscillating_fan_noise():
    gate = SoundActivityGate()

    def frame(level, dc=100):
        wave = np.tile([dc + level, dc - level], 320)
        return np.stack([wave] * 4 + [np.zeros_like(wave)] * 4, axis=1).reshape(-1)

    assert not any(gate.accepts(frame(40)) for _ in range(8))
    # 机器人风扇是安静段/轰鸣段交替的双峰噪声：轰鸣段 75 超过 1.8×40=72
    # 会长得和语音 onset 一模一样，必须整体拒掉。
    fan = [40.0, 40.0, 75.0, 75.0, 75.0] * 30
    assert not any(gate.accepts(frame(level)) for level in fan)


def test_activity_gate_rejects_fan_noise_after_speech():
    gate = SoundActivityGate()

    def frame(level, dc=100):
        wave = np.tile([dc + level, dc - level], 320)
        return np.stack([wave] * 4 + [np.zeros_like(wave)] * 4, axis=1).reshape(-1)

    assert not any(gate.accepts(frame(40)) for _ in range(8))
    assert not gate.accepts(frame(85))  # 单帧升高不等于发声。
    assert gate.accepts(frame(85))
    for _ in range(10):
        gate.accepts(frame(85))
    # 说话结束后底噪保持在风扇安静段，轰鸣段同样不能被当成下一次发声。
    fan = [40.0, 75.0, 75.0, 40.0, 75.0] * 20
    assert not any(gate.accepts(frame(level)) for level in fan)


def test_activity_gate_burst_path_requires_four_times_background():
    gate = SoundActivityGate()

    def frame(level, dc=100):
        wave = np.tile([dc + level, dc - level], 320)
        return np.stack([wave] * 4 + [np.zeros_like(wave)] * 4, axis=1).reshape(-1)

    assert not any(gate.accepts(frame(40)) for _ in range(8))
    # 单帧 140 落在 3×40=120 与 4×40=160 之间：强声旁路不应再放行它。
    assert not gate.accepts(frame(140))
    assert not gate.accepts(frame(40))  # 清掉连续发声计数。
    # 真正的强声（单帧 ≥4×40=160）仍然立刻放行，无需两帧确认。
    assert gate.accepts(frame(165))


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
