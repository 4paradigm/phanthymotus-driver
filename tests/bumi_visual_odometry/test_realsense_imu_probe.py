from types import SimpleNamespace as NS
from pathlib import Path
import subprocess

import pytest

from noetix.bumi.realsense_imu_probe import capture, motion_summary, transform


def test_motion_rate_uses_sensor_timestamps_and_reports_stationary_gravity():
    samples = [(1000+i*5, 'hardware_clock', [0., 0., -9.81]) for i in range(201)]
    result = motion_summary(samples, 200, 1.)
    assert result['rate_hz'] == 200
    assert result['max_gap_ms'] == 5
    assert result['mean'] == pytest.approx([0, 0, -9.81])
    assert result['mean_vector_norm'] == pytest.approx(9.81)
    assert result['timestamp_domain'] == 'hardware_clock'


@pytest.mark.parametrize('samples', [
    [], [(0, 'hardware_clock', [0, 0, 0])],
    [(0, 'hardware_clock', [0, 0, 0]), (0, 'hardware_clock', [0, 0, 0])],
    [(5, 'hardware_clock', [0, 0, 0]), (0, 'hardware_clock', [0, 0, 0])],
    [(0, 'hardware_clock', [0, 0, 0]), (5, 'global_time', [0, 0, 0])],
    [(0, 'hardware_clock', [0, 0, 0]), (5, 'hardware_clock', [0, float('nan'), 0])],
    [(0, 'hardware_clock', [0, 0, 0]), (100, 'hardware_clock', [0, 0, 0])],
])
def test_invalid_or_low_rate_motion_is_refused(samples):
    with pytest.raises(ValueError):
        motion_summary(samples, 200, 1.)


def test_a_short_burst_does_not_prove_a_continuous_recording():
    samples = [(i*5, 'hardware_clock', [0., 0., -9.81]) for i in range(10)]
    with pytest.raises(ValueError):
        motion_summary(samples, 200, 10.)


def test_extrinsics_convert_sdk_column_major_layout_and_keep_direction():
    result = transform(NS(rotation=[0, 1, 0, -1, 0, 0, 0, 0, 1],
                          translation=[1, 2, 3]))
    assert result['rotation_row_major'] == [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    assert result['translation_m'] == [1, 2, 3]
    assert result['equation'] == 'p_color = R_color_motion * p_motion + t_color_motion'


def test_unusable_extrinsics_are_refused():
    for rotation in ([0]*9, [1, 0, 0, 0, 1, 0, 0, 0, -1], [float('nan')]*9):
        with pytest.raises(ValueError):
            transform(NS(rotation=rotation, translation=[0, 0, 0]))


@pytest.mark.parametrize('marker', [None, 'another-source-revision'])
def test_builder_refuses_unverified_existing_sources(tmp_path, marker):
    source = tmp_path / 'source'
    source.mkdir()
    (source / 'CMakeLists.txt').write_text('unverified source')
    if marker is not None:
        (source / '.verified-archive-sha256').write_text(marker)
    script = Path(__file__).resolve().parents[2] / 'noetix/bumi/build_realsense_rsusb.sh'
    result = subprocess.run(['bash', str(script), str(tmp_path)],
                            capture_output=True, text=True)
    assert result.returncode == 1
    assert 'Existing source is unverified' in result.stderr
    assert not (tmp_path / 'build').exists()


@pytest.mark.parametrize('gap_s', [0., .03])
def test_capture_fixes_clock_excludes_startup_and_preserves_failed_diagnostics(monkeypatch, gap_s):
    calls = []
    extrinsics = NS(rotation=[1, 0, 0, 0, 1, 0, 0, 0, 1], translation=[0, 0, 0])
    profiles = [NS(stream_type=lambda kind=kind: kind, format=lambda: 'bgr8',
                   fps=lambda rate=rate: rate,
                   as_video_stream_profile=lambda: NS(width=lambda: 640, height=lambda: 480),
                   get_extrinsics_to=lambda _: extrinsics)
                for kind, rate in [('color', 30), ('gyro', 200), ('accel', 200)]]
    clock = [0.]
    polls = [0]

    def poll(_):
        clock[0] += .0025
        polls[0] += 1
        if polls[0] % 100 == 0:
            clock[0] += gap_s
        profile = profiles[1+polls[0] % 2]
        stamp = 999999 if clock[0] <= 2 else (clock[0]-2)*1000
        return True, NS(get_profile=lambda: profile, get_timestamp=lambda: stamp,
                        get_frame_timestamp_domain=lambda: 'hardware_clock',
                        as_motion_frame=lambda: NS(get_motion_data=lambda: NS(x=0., y=0., z=-9.81)))

    queue = NS(try_wait_for_frame=poll)
    sensor = NS(get_stream_profiles=lambda: profiles,
                set_option=lambda *args: calls.append(('option', args)),
                open=lambda _: calls.append(('open',)), start=lambda _: None,
                stop=lambda: calls.append(('stop',)), close=lambda: calls.append(('close',)))
    device = NS(query_sensors=lambda: [sensor], get_info=lambda _: 'test device')
    sdk = NS(context=lambda: NS(query_devices=lambda: [device]),
             frame_queue=lambda *args, **kwargs: queue,
             stream=NS(color='color', gyro='gyro', accel='accel'), format=NS(bgr8='bgr8'),
             option=NS(global_time_enabled='global_time_enabled'),
             camera_info=NS(name='name', firmware_version='firmware'))

    monkeypatch.setattr('noetix.bumi.realsense_imu_probe.time.monotonic', lambda: clock[0])
    result = capture(sdk, 1.)
    assert calls[:2] == [('option', ('global_time_enabled', 0.)), ('open',)]
    assert calls[-2:] == [('stop',), ('close',)]
    assert result['motion_to_color']['gyro']['translation_m'] == [0, 0, 0]
    assert result['passed'] == (gap_s == 0.)
    if gap_s:
        assert 'max gap' in result['motion']['gyro']['error']
    else:
        assert result['motion']['gyro']['count'] >= 199
        assert result['motion']['accel']['rate_hz'] == pytest.approx(200)
