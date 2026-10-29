"""Regressions for the production hand_grasp adaptive contact-stop card."""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand" / "l20"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))

from test_linkerhand_l20 import (
    HandPlugin,
    L20CanClient,
    MockCanBus,
    MockCanMessage,
    _SingleHandController,
    build_plugins,
)
from can_client import PositionQueryTimeout, TactileQueryTimeout
from adaptive_grasp_device import (
    DEFAULT_CLOSE_LIMIT_POSE,
    DEFAULT_PREP_POSE,
    FINGER_JOINTS,
    AdaptiveGraspPlugin,
    TestGraspFault as GraspFault,
    _JointStallDetector,
)


class MockAdaptiveTactileBus(MockCanBus):
    """Simulates CAN bus with simulated tactile matrix peaks on grasp."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Configurable peak matrix responses per finger
        self.sim_matrix_peaks = {
            "thumb": 0,
            "index": 35,   # >= 25 threshold
            "middle": 40,  # >= 25 threshold
            "ring": 10,
            "pinky": 5,
        }

    def send(self, msg):
        # Tactile matrix request 0xB1-0xB5 with 0xC6
        if 0xB1 <= msg.data[0] <= 0xB5 and len(msg.data) == 2 and msg.data[1] == 0xC6:
            finger_idx = msg.data[0] - 0xB1
            finger_names = ["thumb", "index", "middle", "ring", "pinky"]
            fname = finger_names[finger_idx]
            peak = self.sim_matrix_peaks.get(fname, 0)
            self.sent_messages.append(msg)
            # Reply with 12 rows of 6 bytes (row offsets 0, 16, 32 ... 176)
            for row in range(12):
                # Place the peak at the last taxel, not merely the first cell.
                cells = [0] * 6
                if row == 11:
                    cells[5] = peak
                reply_data = [msg.data[0], row * 16, *cells]
                self._inbox.put(MockCanMessage(self.can_id, reply_data))
            return
        super().send(msg)


@pytest.fixture
def test_setup():
    bus = MockAdaptiveTactileBus(can_id=0x28)
    client = L20CanClient(
        side="left",
        can_id=0x28,
        recv_timeout_s=0.05,
        bus_factory=lambda *_: bus,
    )
    ctrl = _SingleHandController({}, "left", "can0", threading.RLock(), can_id=0x28)
    ctrl._client = client
    ctrl._open_pose = [255] * 20
    hand = HandPlugin({"left_can_id": 0x28, "right_can_id": 0x27}, {})
    hand.get_controller_for_side = lambda *_: ctrl

    cfg = {
        "approach_speed": 85,
        "release_speed": 80,
        "motion_current": 230,
        "tactile_threshold": 25,
        "step_size": 20,  # larger step for fast tests
        "poll_interval_s": 0.005,
        "matrix_timeout_s": 0.1,
        "pose_timeout_s": 0.2,
        "timeout_s": 2.0,  # Each cycle now includes fresh matrix AND position queries.
    }
    plugin = AdaptiveGraspPlugin(hand, cfg, global_config={"can_channel": "can0"})

    events = []
    done = threading.Event()

    def completed(aid, status, result):
        events.append((aid, status, result))
        done.set()

    plugin._acp_callback = completed
    plugin._test_completions = events

    bus.sent_messages.clear()
    try:
        yield plugin, client, bus, ctrl, hand, events, done
    finally:
        plugin.stop()
        client.close()


def test_tool_definition(test_setup):
    plugin, _, _, _, _, _, _ = test_setup
    tool_def = plugin.get_tool()
    assert tool_def["name"] == "hand_grasp"
    assert tool_def["type"] == "actuator"
    assert "adaptive_grasp" in tool_def["inputSchema"]["properties"]["action"]["enum"]
    assert "release" in tool_def["inputSchema"]["properties"]["action"]["enum"]
    assert tool_def["multiInstance"] is True


def test_build_plugins_registers_hand_grasp():
    config = {
        "plugins": {
            "hand": {"enabled": False},
            "hand_gesture": {"enabled": False},
            "hand_tactile": {"enabled": False},
            "hand_grasp": {"enabled": True},
        }
    }
    plugins = build_plugins(config, "", None)
    assert len(plugins) == 1
    assert isinstance(plugins[0], AdaptiveGraspPlugin)


def test_dispatch_lifecycle(test_setup):
    plugin, _, _, _, _, _, _ = test_setup
    assert plugin.dispatch("config", {})["state"] == "configured"
    assert plugin.dispatch("start", {})["state"] == "ready"
    info = plugin.dispatch("info", {"bound_side": "left"})
    assert info["state"] == "ready"
    assert info["bound_side"] == "left"
    stop_res = plugin.dispatch("stop", {"bound_side": "left"})
    assert stop_res["state"] == "idle"


def test_adaptive_grasp_validation(test_setup):
    plugin, _, _, _, _, _, _ = test_setup
    with pytest.raises(ValueError, match="confirm_motion"):
        plugin.dispatch("adaptive_grasp", {"bound_side": "left", "confirm_motion": False})
    with pytest.raises(ValueError, match="bound_side"):
        plugin.dispatch("adaptive_grasp", {"bound_side": "invalid", "confirm_motion": True})
    with pytest.raises(ValueError, match="wait_for_result"):
        plugin.dispatch("adaptive_grasp", {"bound_side": "left", "confirm_motion": True, "wait_for_result": True})


def test_adaptive_grasp_execution(test_setup):
    plugin, client, bus, ctrl, hand, events, done = test_setup
    # Index (35) and Middle (40) will trigger tactile threshold (>= 25)
    bus.sim_matrix_peaks = {
        "thumb": 5,
        "index": 35,
        "middle": 40,
        "ring": 10,
        "pinky": 5,
    }

    res = plugin.dispatch("adaptive_grasp", {"bound_side": "left", "confirm_motion": True})
    assert res["state"] == "running"
    action_id = res["action_id"]

    assert done.wait(timeout=2.0), "adaptive_grasp timed out"
    assert len(events) == 1
    aid, status, result = events[0]
    assert aid == action_id
    assert status == "completed"
    assert result["holding"] is True
    assert result["contact_count"] == 2
    assert result["contacts"]["index"] is True
    assert result["contacts"]["middle"] is True
    assert result["contacts"]["thumb"] is False
    for finger in ('index', 'middle'):
        assert result['joint_states'][finger] == {'root': 'tactile_threshold', 'tip': 'tactile_threshold'}
        assert result['target_positions'][f'{finger}_pitch'] == DEFAULT_PREP_POSE[FINGER_JOINTS[finger]['root']]
        assert result['target_positions'][f'{finger}_tip'] == DEFAULT_PREP_POSE[FINGER_JOINTS[finger]['tip']]
        assert not any(result['stalled_joints'][finger].values())


def test_release_execution(test_setup):
    plugin, client, bus, ctrl, hand, events, done = test_setup
    res = plugin.dispatch("release", {"bound_side": "left", "confirm_motion": True})
    assert res["state"] == "running"
    action_id = res["action_id"]

    assert done.wait(timeout=2.0), "release timed out"
    assert len(events) == 1
    aid, status, result = events[0]
    assert aid == action_id
    assert status == "completed"
    assert result["hand_opened"] is True
    assert result["torque_released"] is True
    assert result["holding"] is False


def matrix(peak=0):
    rows = [[0] * 6 for _ in range(12)]
    rows[11][5] = peak
    return rows


def run_grasp(test_setup, *, completion_timeout=3):
    plugin, _, _, _, _, events, done = test_setup
    reply = plugin.dispatch('adaptive_grasp', {'bound_side': 'left', 'confirm_motion': True})
    assert done.wait(completion_timeout), 'test card failed to deliver completion'
    assert len(events) == 1 and events[0][0] == reply['action_id']
    return events[0][1], events[0][2]


def test_complete_zero_matrix_is_data_and_never_replaced_with_scalar(test_setup, monkeypatch, capsys):
    plugin, client, bus, _, _, _, _ = test_setup
    bus.sim_matrix_peaks = dict.fromkeys(FINGER_JOINTS, 0)
    bus.sim_touch = [255] * 5  # Deliberately disagree with matrix feedback.
    def forbidden_scalar(*args, **kwargs):
        pytest.fail('hand_grasp must not use scalar/cache feedback')
    for name in ('get_fresh_tactile', 'get_tactile_snapshot', 'request_tactile_scalar'):
        monkeypatch.setattr(client, name, forbidden_scalar)
    status, result = run_grasp(test_setup)
    assert status == 'completed' and result['contact_count'] == 0
    assert result['peaks'] == dict.fromkeys(FINGER_JOINTS, 0)
    assert result['end_reason'] == 'all_fingers_settled' and not result['timed_out']
    assert result['tactile_status']['matrix_valid'] and result['tactile_status']['sample_state'] == 'ok'
    assert result['tactile_status']['sample_sequence'] > 0
    assert result['tactile_status']['source'] == 'matrix_peak'
    assert result['tactile_status']['position_valid']
    assert result['actual_positions'] == result['target_positions'] == result['sampled_target_positions']
    assert all(state == 'command_limit' for states in result['joint_states'].values() for state in states.values())
    assert not any(stalled for joints in result['stalled_joints'].values() for stalled in joints.values())
    assert not result['tactile_status']['monitoring']
    assert 'thumb=0' in result['message'] and '不代表已抓稳' in result['message']
    assert '已收到五指完整12×6阵列' in capsys.readouterr().out
    requests = [list(msg.data) for msg in bus.sent_messages if 0xB1 <= msg.data[0] <= 0xB5]
    assert requests and all(len(data) == 2 and data[1] == 0xC6 for data in requests)


def test_missing_matrix_stops_before_any_closing_and_keeps_timeout_details(test_setup, monkeypatch):
    plugin, client, _, _, _, _, _ = test_setup
    positions = []
    send = client.set_joint_positions
    def capture(target):
        positions.append(list(target))
        return send(target)
    monkeypatch.setattr(client, 'set_joint_positions', capture)
    count_at_query = []
    details = {'query_type': 'matrix', 'query_phase': 'receive_rows',
               'missing_rows': {'index': [11]}, 'elapsed_s': .25}
    def missing(*args, **kwargs):
        count_at_query.append(len(positions))
        raise TactileQueryTimeout('index row 11 missing', details)
    monkeypatch.setattr(client, 'get_fresh_matrix_touch', missing)
    status, result = run_grasp(test_setup)
    assert status == 'error' and result['error_code'] == 'tactile_query_timeout'
    assert result['missing_rows'] == details['missing_rows']
    assert result['query_phase'] == 'receive_rows'
    assert result['tactile_status']['sample_state'] == 'error'
    assert not result['tactile_status']['matrix_valid']
    assert result['tactile_status']['peaks'] == dict.fromkeys(FINGER_JOINTS, None)
    assert result['tactile_status']['sample_sequence'] == 0
    assert len(positions) == count_at_query[0] and positions[-1] == plugin.prep_pose
    assert result['torque_released'] and not result['holding']
    assert client._currents == [0] * 5
    assert '未继续收紧' in result['message']
    assert not plugin._active and not plugin._sessions


@pytest.mark.parametrize('bad', ['missing_finger', 'row_count', 'row_width', 'negative',
                                'too_large', 'boolean', 'float', 'not_a_matrix'])
def test_invalid_matrix_is_not_silently_treated_as_zero(test_setup, monkeypatch, bad):
    _, client, _, _, _, _, _ = test_setup
    values = {name: matrix() for name in FINGER_JOINTS}
    if bad == 'missing_finger':
        del values['index']
    elif bad == 'row_count':
        values['index'].pop()
    elif bad == 'row_width':
        values['index'][0].pop()
    elif bad == 'not_a_matrix':
        values = None
    else:
        values['index'][11][5] = {'negative': -1, 'too_large': 256, 'boolean': True, 'float': 1.0}[bad]
    monkeypatch.setattr(client, 'get_fresh_matrix_touch', lambda *a, **k: values)
    status, result = run_grasp(test_setup)
    assert status == 'error' and result['error_code'] == 'invalid_tactile_matrix'
    assert result['torque_released'] and not result['tactile_status']['matrix_valid']


def test_timeout_is_distinguished_from_position_limit(test_setup):
    plugin, _, bus, _, _, _, _ = test_setup
    bus.sim_matrix_peaks = dict.fromkeys(FINGER_JOINTS, 0)
    plugin.timeout_s = .01  # One matrix query already exceeds this observation window.
    status, result = run_grasp(test_setup)
    assert status == 'completed' and result['end_reason'] == 'timeout' and result['timed_out']
    assert not all(result['reached_limit'].values())
    assert '收拢观察超时' in result['message']
    assert result['outcome'] == 'timeout_hold' and not result['grasp_confirmed']
    assert result['tactile_status']['matrix_valid'] and result['tactile_status']['sample_sequence'] == 1


def test_completion_budget_includes_preparation_and_observation_time(test_setup):
    plugin, *_ = test_setup
    plugin.pose_timeout_s = 5
    plugin.timeout_s = 15
    assert plugin.get_tool()['inputSchema']['x-completion']['timeout'] >= 4 * 5 + 15 + 2


def test_live_info_reports_sample_alongside_public_status_action(test_setup, monkeypatch):
    plugin, client, _, _, _, _, done = test_setup
    second_query = threading.Event()
    resume = threading.Event()
    count = []
    def query(*args, **kwargs):
        count.append(1)
        if len(count) == 2:
            second_query.set()
            assert resume.wait(2)
        return {name: matrix(0 if len(count) == 1 else 35) for name in FINGER_JOINTS}
    monkeypatch.setattr(client, 'get_fresh_matrix_touch', query)
    plugin.dispatch('adaptive_grasp', {'bound_side': 'left', 'confirm_motion': True})
    try:
        assert second_query.wait(2)
        info = plugin.dispatch('info', {'bound_side': 'left'})
        sample = info['tactile_status']
        assert info['active_action_id'] and sample['sample_sequence'] == 1
        assert sample['source'] == 'matrix_peak' and sample['matrix_shape'] == [12, 6]
        assert sample['matrix_valid'] and sample['sample_timestamp'] > 0
        assert sample['peaks'] == dict.fromkeys(FINGER_JOINTS, 0)
        assert set(plugin.get_tool()['inputSchema']['x-action-params']) == {
            'adaptive_grasp', 'release', 'get_state', 'config'}
    finally:
        resume.set()
        assert done.wait(3)
    assert plugin.dispatch('info', {'bound_side': 'left'})['tactile_status']['phase'] == 'holding'


def test_release_never_requires_tactile_data(test_setup, monkeypatch):
    plugin, client, _, _, _, events, done = test_setup
    def forbidden(*args, **kwargs):
        pytest.fail('release must remain available without tactile feedback')
    monkeypatch.setattr(client, 'get_fresh_matrix_touch', forbidden)
    plugin.dispatch('release', {'bound_side': 'left', 'confirm_motion': True})
    assert done.wait(3)
    assert events[0][1] == 'completed' and events[0][2]['hand_opened']
    assert events[0][2]['tactile_status']['sample_state'] == 'not_requested'


def test_stall_requires_elapsed_time_and_at_least_three_samples():
    detector = _JointStallDetector(20, 1, .5)
    assert not detector.update(1, 170, 200, 0, closing=True)
    assert not detector.update(1, 165, 201, .6, closing=True), 'two samples do not establish a stall'
    assert detector.update(1, 160, 200, .7, closing=True)


def test_stall_uses_full_window_spread_instead_of_only_adjacent_samples():
    detector = _JointStallDetector(20, 1, .5)
    # Adjacent differences are only one, but the joint keeps moving.
    for i in range(10):
        assert not detector.update(1, 100, 210 - i, i * .3, closing=True)


@pytest.mark.parametrize('command,actual,closing', [(180, 200, True), (220, 200, True), (100, 200, False)])
def test_stall_ignores_small_or_opposite_error_and_preparation(command, actual, closing):
    detector = _JointStallDetector(20, 1, .5)
    for timestamp in (0, .5, 1):
        assert not detector.update(1, command, actual, timestamp, closing)
    assert not detector.windows


def test_stall_window_resets_when_tracking_recovers_and_is_independent_per_joint():
    detector = _JointStallDetector(20, 1, .5)
    assert not detector.update(1, 100, 200, 0, True)
    assert not detector.update(1, 100, 200, .3, True)
    assert not detector.update(1, 190, 200, .4, True)
    assert not detector.update(1, 100, 200, .6, True)
    assert not detector.update(16, 100, 200, .7, True)
    assert not detector.update(1, 100, 200, .9, True)
    assert detector.update(1, 100, 200, 1.2, True)
    assert detector.windows[16]['samples'] == 1


def test_trial_stall_defaults_are_separate_from_pose_tolerance(test_setup):
    plugin, *_ = test_setup
    assert plugin.stall_error_threshold == 20
    assert plugin.stall_position_tolerance == 3
    assert plugin.stall_duration_s == .5
    configured = AdaptiveGraspPlugin(None, {'stall_error_threshold': 30, 'stall_position_tolerance': 2,
                                    'stall_duration_s': .8, 'position_tolerance': 19})
    assert configured.stall_error_threshold == 30 and configured.position_tolerance == 19


def test_trial_stall_tolerates_small_jitter_but_requires_time_and_three_samples():
    plugin = AdaptiveGraspPlugin(None, {})
    detector = _JointStallDetector(plugin.stall_error_threshold,
                                    plugin.stall_position_tolerance, plugin.stall_duration_s)
    assert not detector.update(1, 170, 200, 0, True)
    assert not detector.update(1, 165, 203, .2, True)
    assert not detector.update(1, 160, 201, .4, True)
    assert detector.update(1, 155, 202, .6, True)
    assert detector.windows[1]['maximum'] - detector.windows[1]['minimum'] == 3


def test_trial_stall_does_not_freeze_a_joint_still_moving_beyond_tolerance():
    detector = _JointStallDetector(20, 3, .5)
    for sequence in range(10):
        assert not detector.update(1, 100, 210 - sequence * 2, sequence * .2, True)


@pytest.mark.parametrize('setting,value', [
    ('fast_step_size', 0), ('fast_step_size', 4), ('fast_step_size', 256),
    ('fast_step_size', True), ('fast_step_size', 10.5),
    ('slowdown_error_threshold', 0), ('slowdown_error_threshold', 255),
    ('slowdown_error_threshold', True), ('slowdown_error_threshold', 10.5),
])
def test_invalid_adaptive_step_settings_fail_before_device_access(setting, value):
    with pytest.raises(ValueError, match=setting):
        AdaptiveGraspPlugin(None, {setting: value})


@pytest.mark.parametrize('lag,expected_steps', [(0, [10, 10, 10, 10]), (9, [10, 10, 10, 10]),
                                               (10, [10, 10, 5, 10]), (15, [10, 10, 5, 10]),
                                               (-15, [10, 10, 10, 10])])
def test_feedback_selects_steps_per_joint_without_rebasing_targets_or_changing_speed(
        test_setup, monkeypatch, lag, expected_steps):
    plugin, client, _, _, _, _, _ = test_setup
    plugin.step_size = 5
    plugin.fast_step_size = 10
    query_count = []
    writes = []
    samples = []
    speeds = []
    read_position = client.get_positions_vector
    send_position = client.set_joint_positions
    send_speed = client.set_speeds
    publish = plugin._set_tactile_status

    def matrix_query(*args, **kwargs):
        query_count.append(1)
        return {finger: matrix(35 if len(query_count) == 5 else 0) for finger in FINGER_JOINTS}

    def position_query():
        actual = read_position()
        if len(query_count) == 3:
            actual[1] += lag  # Index root lags independently of its tip.
        return actual

    def capture_position(target):
        if query_count:
            writes.append(list(target))
        return send_position(target)

    def capture_speed(target):
        speeds.append(list(target))
        return send_speed(target)

    def capture_sample(**fields):
        if fields.get('sample_sequence'):
            samples.append(fields)
        return publish(**fields)

    monkeypatch.setattr(client, 'get_fresh_matrix_touch', matrix_query)
    monkeypatch.setattr(client, 'get_positions_vector', position_query)
    monkeypatch.setattr(client, 'set_joint_positions', capture_position)
    monkeypatch.setattr(client, 'set_speeds', capture_speed)
    monkeypatch.setattr(plugin, '_set_tactile_status', capture_sample)
    status, result = run_grasp(test_setup)
    assert status == 'completed' and result['end_reason'] == 'all_fingers_settled'
    assert len(writes) == 4, 'tactile contact must prevent the next advance'
    previous = plugin.prep_pose[1]
    for target, sample, expected in zip(writes, samples, expected_steps):
        assert previous - target[1] == expected
        assert sample['planned_steps']['index']['root'] == expected
        assert sample['planned_steps']['index']['tip'] == 10
        previous = target[1]
    assert [target[16] for target in writes] == [245, 235, 225, 215]
    assert speeds == [[85] * 5], 'adaptive stepping must not change motor speed'
    assert not any(result['stalled_joints']['index'].values())
    assert all(step == 0 for joints in samples[-1]['planned_steps'].values() for step in joints.values())
    assert result['tactile_status']['step_settings'] == {
        'step_size': 5, 'fast_step_size': 10, 'slowdown_error_threshold': 10}


def test_small_step_tracking_lag_does_not_add_a_travel_budget_or_claim_contact(test_setup, monkeypatch):
    plugin, client, _, _, _, _, _ = test_setup
    plugin.step_size = 5
    plugin.fast_step_size = 10
    plugin.timeout_s = 10  # Allow the full small-step stroke under ARM64 emulation.
    query_count = []
    read_position = client.get_positions_vector

    def matrix_query(*args, **kwargs):
        query_count.append(1)
        return {finger: matrix() for finger in FINGER_JOINTS}

    def position_query():
        actual = read_position()
        if query_count:
            actual[1] = min(255, actual[1] + 15)
        return actual

    monkeypatch.setattr(client, 'get_fresh_matrix_touch', matrix_query)
    monkeypatch.setattr(client, 'get_positions_vector', position_query)
    status, result = run_grasp(test_setup, completion_timeout=12)
    assert status == 'completed' and result['end_reason'] == 'all_fingers_settled'
    assert result['target_positions']['index_pitch'] == 0
    assert result['actual_positions']['index_pitch'] == 15
    assert result['joint_states']['index']['root'] == 'command_limit'
    assert not result['stalled_joints']['index']['root']
    assert not result['contacts']['index'] and result['contact_count'] == 0


@pytest.mark.parametrize('setting,value', [
    ('stall_error_threshold', -1), ('stall_error_threshold', 255),
    ('stall_error_threshold', True), ('stall_error_threshold', 20.5),
    ('stall_position_tolerance', -1), ('stall_position_tolerance', True),
    ('stall_duration_s', 0), ('stall_duration_s', float('nan')),
    ('stall_duration_s', float('inf')), ('stall_duration_s', True),
])
def test_invalid_stall_settings_fail_before_any_device_access(setting, value):
    with pytest.raises(ValueError, match=setting):
        AdaptiveGraspPlugin(None, {setting: value})


def simulate_blocked_joints(test_setup, monkeypatch, blocked):
    """Keep selected actual joints above 200 while commands continue closing."""
    plugin, client, bus, *_ = test_setup
    plugin.stall_duration_s = .05  # Compress the observation, keep >=3 fresh samples.
    plugin.timeout_s = 3
    bus.sim_matrix_peaks = dict.fromkeys(FINGER_JOINTS, 0)
    closing = threading.Event()
    query = client.get_fresh_matrix_touch
    def matrix_query(*args, **kwargs):
        values = query(*args, **kwargs)
        closing.set()
        return values
    monkeypatch.setattr(client, 'get_fresh_matrix_touch', matrix_query)
    writes = []
    send = client.set_joint_positions
    def capture(target):
        writes.append(list(target))
        value = send(target)
        if closing.is_set():
            for finger, joint in blocked:
                index = list(FINGER_JOINTS).index(finger)
                actual = bus.sim_pitch if joint == 'root' else bus.sim_tip
                actual[index] = max(200, actual[index])
        return value
    monkeypatch.setattr(client, 'set_joint_positions', capture)
    return writes


@pytest.mark.parametrize('joint,index,other,other_index', [('root', 1, 'tip', 16), ('tip', 16, 'root', 1)])
def test_only_stalled_joint_freezes_and_sibling_keeps_closing(test_setup, monkeypatch,
                                                           joint, index, other, other_index):
    writes = simulate_blocked_joints(test_setup, monkeypatch, [('index', joint)])
    status, result = run_grasp(test_setup)
    assert status == 'completed' and result['end_reason'] == 'all_fingers_settled'
    assert result['joint_states']['index'] == {joint: 'position_stalled', other: 'command_limit'}
    evidence = result['stall_evidence']['index'][joint]
    frozen = evidence['command']
    assert frozen < evidence['actual'] == 200
    assert evidence['tracking_error'] > 20 and evidence['samples'] >= 3
    assert evidence['observed_s'] >= .05
    first_frozen = next(i for i, target in enumerate(writes) if target[index] == frozen)
    assert all(target[index] == frozen for target in writes[first_frozen:]), 'frozen joint advanced or rebased to actual'
    assert writes[-1][other_index] == 0 and writes[first_frozen][other_index] > 0
    assert result['contact_count'] == 0 and not result['contacts']['index']
    assert result['actual_positions'][f'index_{"pitch" if joint == "root" else "tip"}'] == 200
    assert result['holding']


def test_all_joints_stalled_can_complete_without_claiming_tactile_contact(test_setup, monkeypatch):
    blocked = [(finger, joint) for finger in FINGER_JOINTS for joint in FINGER_JOINTS[finger]]
    simulate_blocked_joints(test_setup, monkeypatch, blocked)
    status, result = run_grasp(test_setup)
    assert status == 'completed' and result['holding'] and not result['timed_out']
    assert all(state == 'position_stalled' for states in result['joint_states'].values() for state in states.values())
    assert result['contact_count'] == 0 and not any(result['contacts'].values())
    assert '位置持续受阻' in result['message'] and '不代表已抓稳' in result['message']


def test_position_timeout_after_valid_matrix_stops_closing_and_preserves_details(test_setup, monkeypatch):
    plugin, client, _, _, _, _, _ = test_setup
    matrix_seen = threading.Event()
    query = client.get_fresh_matrix_touch
    def matrix_query(*args, **kwargs):
        values = query(*args, **kwargs)
        matrix_seen.set()
        return values
    monkeypatch.setattr(client, 'get_fresh_matrix_touch', matrix_query)
    writes = []
    send = client.set_joint_positions
    def capture(target):
        writes.append(list(target))
        return send(target)
    monkeypatch.setattr(client, 'set_joint_positions', capture)
    read = client.get_positions_vector
    details = {'query_type': 'position', 'query_phase': 'receive_reply',
               'missing_frames': ['0x45'], 'missing_fingers': ['pinky'], 'elapsed_s': .05}
    writes_at_failure = []
    def missing_position():
        if matrix_seen.is_set():
            writes_at_failure.append(len(writes))
            raise PositionQueryTimeout('pinky position missing', details)
        return read()
    monkeypatch.setattr(client, 'get_positions_vector', missing_position)
    status, result = run_grasp(test_setup)
    assert status == 'error' and result['error_code'] == 'position_query_timeout'
    assert result['missing_frames'] == ['0x45'] and result['query_phase'] == 'receive_reply'
    sample = result['tactile_status']
    assert sample['matrix_valid'] and not sample['position_valid']
    assert sample['actual_positions'] is None and sample['sample_sequence'] == 0
    assert len(writes) == writes_at_failure[0] and writes[-1] == plugin.prep_pose
    assert result['torque_released'] and not result['holding']


@pytest.mark.parametrize('invalid', [[0] * 19, [-1] * 20, [True] * 20, [0.5] * 20])
def test_invalid_actual_positions_are_not_used_for_stall_detection(test_setup, monkeypatch, invalid):
    _, client, *_ = test_setup
    matrix_seen = threading.Event()
    query = client.get_fresh_matrix_touch
    def matrix_query(*args, **kwargs):
        values = query(*args, **kwargs)
        matrix_seen.set()
        return values
    monkeypatch.setattr(client, 'get_fresh_matrix_touch', matrix_query)
    read = client.get_positions_vector
    monkeypatch.setattr(client, 'get_positions_vector', lambda: invalid if matrix_seen.is_set() else read())
    status, result = run_grasp(test_setup)
    assert status == 'error' and result['error_code'] == 'invalid_position_feedback'
    assert result['torque_released'] and not result['tactile_status']['position_valid']


def test_release_interrupts_adaptive_grasp_while_position_detection_is_active(test_setup, monkeypatch):
    plugin, _, _, _, _, events, _ = test_setup
    sampled = threading.Event()
    publish = plugin._set_tactile_status
    def capture(**fields):
        value = publish(**fields)
        if fields.get('sample_sequence') == 2:
            sampled.set()
        return value
    monkeypatch.setattr(plugin, '_set_tactile_status', capture)
    first = plugin.dispatch('adaptive_grasp', {'bound_side': 'left', 'confirm_motion': True})
    assert sampled.wait(3)
    release = plugin.dispatch('release', {'bound_side': 'left', 'confirm_motion': True})
    deadline = time.monotonic() + 3
    while len(events) < 2 and time.monotonic() < deadline:
        time.sleep(.005)
    assert len(events) == 2
    outcomes = {aid: (status, result) for aid, status, result in events}
    assert outcomes[first['action_id']][0] == 'cancelled'
    assert outcomes[release['action_id']][0] == 'completed'
    assert outcomes[release['action_id']][1]['hand_opened']


def test_transient_matrix_timeout_recovers_without_advancing_during_failed_sample(test_setup, monkeypatch, capsys):
    plugin, client, bus, _, _, _, _ = test_setup
    bus.sim_matrix_peaks = dict.fromkeys(FINGER_JOINTS, 0)
    writes = []
    send = client.set_joint_positions
    def capture(target):
        writes.append(list(target))
        return send(target)
    monkeypatch.setattr(client, 'set_joint_positions', capture)
    query = client.get_fresh_matrix_touch
    counts = []
    def intermittent(*args, **kwargs):
        counts.append(len(writes))
        if len(counts) == 2:
            raise TactileQueryTimeout('pinky row missing', {'query_type': 'matrix', 'missing_rows': {'pinky': [11]}})
        return query(*args, **kwargs)
    monkeypatch.setattr(client, 'get_fresh_matrix_touch', intermittent)
    status, result = run_grasp(test_setup)
    assert status == 'completed' and result['end_reason'] == 'all_fingers_settled'
    assert counts[1] == counts[2], 'a failed query must not advance the command'
    assert result['matrix_retries_total'] == result['matrix_recoveries'] == 1
    assert result['contact_count'] == 0 and result['actual_positions'] == result['target_positions']
    assert result['tactile_status']['error_code'] is None
    assert '触觉采集已恢复' in capsys.readouterr().out


def test_matrix_recovery_restarts_complete_query_and_resets_stall_observation(monkeypatch):
    plugin = AdaptiveGraspPlugin(None, {})
    detector = _JointStallDetector(20, 1, .5)
    detector.update(1, 100, 200, 0, True)
    values = {name: matrix(35) for name in FINGER_JOINTS}
    calls = []
    def query(*, timeout_s, cancel):
        calls.append(timeout_s)
        if len(calls) < 3:
            raise TactileQueryTimeout('partial reply', {'missing_rows': {'pinky': [11]}})
        return values
    returned, recovery = plugin._read_matrix_with_recovery(SimpleNamespace(get_fresh_matrix_touch=query), detector)
    assert returned is values and recovery['matrix_query_attempts'] == 3 and recovery['matrix_retries'] == 2
    assert not detector.windows, 'an observation gap cannot count toward continuous stagnation'
    assert all(0 < timeout <= .25 for timeout in calls)


@pytest.mark.parametrize('retries', [0, 2])
def test_matrix_recovery_stops_at_attempt_limit_and_retains_missing_rows(retries):
    plugin = AdaptiveGraspPlugin(None, {'matrix_retry_count': retries})
    detector = _JointStallDetector(20, 1, .5)
    calls = []
    details = {'query_type': 'matrix', 'query_phase': 'receive_rows', 'missing_rows': {'pinky': [9, 11]}}
    def query(**kwargs):
        calls.append(kwargs)
        raise TactileQueryTimeout('incomplete', details)
    with pytest.raises(GraspFault) as error:
        plugin._read_matrix_with_recovery(SimpleNamespace(get_fresh_matrix_touch=query), detector)
    assert len(calls) == retries + 1
    assert error.value.code == 'tactile_query_timeout'
    assert error.value.details['missing_rows'] == {'pinky': [9, 11]}
    assert error.value.details['matrix_query_attempts'] == retries + 1
    assert error.value.details['matrix_retries'] == retries


def test_matrix_recovery_total_budget_caps_last_attempt(monkeypatch):
    import adaptive_grasp_device
    clock = [0.0]
    monkeypatch.setattr(adaptive_grasp_device, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    plugin = AdaptiveGraspPlugin(None, {'matrix_retry_count': 10, 'matrix_timeout_s': .25,
                               'matrix_recovery_timeout_s': .6})
    monkeypatch.setattr(plugin, '_wait', lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    budgets = []
    def query(*, timeout_s, cancel):
        budgets.append(timeout_s)
        clock[0] += timeout_s
        raise TimeoutError('missing rows')
    with pytest.raises(GraspFault) as error:
        plugin._read_matrix_with_recovery(SimpleNamespace(get_fresh_matrix_touch=query), _JointStallDetector(20, 1, .5))
    assert len(budgets) == 3
    assert budgets[:2] == [.25, .25] and budgets[-1] == pytest.approx(.06)
    assert error.value.details['matrix_recovery_elapsed_s'] == pytest.approx(.6)


def test_malformed_matrix_is_not_retried():
    plugin = AdaptiveGraspPlugin(None, {})
    calls = []
    def query(**kwargs):
        calls.append(kwargs)
        return {'thumb': matrix(0)}
    with pytest.raises(GraspFault, match='index') as error:
        plugin._read_matrix_with_recovery(SimpleNamespace(get_fresh_matrix_touch=query), _JointStallDetector(20, 1, .5))
    assert error.value.code == 'invalid_tactile_matrix' and len(calls) == 1


def test_stop_interrupts_matrix_retry_wait(monkeypatch):
    plugin = AdaptiveGraspPlugin(None, {})
    calls = []
    def query(**kwargs):
        calls.append(kwargs)
        raise TimeoutError('missing rows')
    def stop_during_wait(seconds):
        plugin._stop.set()
        plugin._check_stop()
    monkeypatch.setattr(plugin, '_wait', stop_during_wait)
    with pytest.raises(InterruptedError):
        plugin._read_matrix_with_recovery(SimpleNamespace(get_fresh_matrix_touch=query), _JointStallDetector(20, 1, .5))
    assert len(calls) == 1


@pytest.mark.parametrize('setting,value', [('matrix_retry_count', -1), ('matrix_retry_count', True),
                                        ('matrix_retry_count', 1.5), ('matrix_recovery_timeout_s', 0),
                                        ('matrix_recovery_timeout_s', float('inf')),
                                        ('matrix_recovery_timeout_s', True)])
def test_invalid_matrix_recovery_settings_fail_before_device_access(setting, value):
    with pytest.raises(ValueError, match=setting):
        AdaptiveGraspPlugin(None, {setting: value})


def test_late_complete_matrix_cannot_bypass_recovery_deadline(monkeypatch):
    import adaptive_grasp_device
    clock = [0.0]
    monkeypatch.setattr(adaptive_grasp_device, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    plugin = AdaptiveGraspPlugin(None, {})
    def late(**kwargs):
        clock[0] = 1.1
        return {name: matrix(35) for name in FINGER_JOINTS}
    with pytest.raises(GraspFault) as error:
        plugin._read_matrix_with_recovery(SimpleNamespace(get_fresh_matrix_touch=late), _JointStallDetector(20, 1, .5))
    assert error.value.details['matrix_query_attempts'] == 1
    assert error.value.details['matrix_recovery_elapsed_s'] == pytest.approx(1.1)


def test_completion_budget_accounts_for_matrix_recovery(test_setup):
    plugin, *_ = test_setup
    plugin.pose_timeout_s = 5
    plugin.timeout_s = 15
    plugin.matrix_recovery_timeout_s = 3
    assert plugin.get_tool()['inputSchema']['x-completion']['timeout'] >= 4 * 5 + 15 + 3 + 2


def test_user_card_settings_and_actions_keep_threshold_separate_from_speed():
    plugin = AdaptiveGraspPlugin(None, {})
    definition = plugin.get_tool()
    settings = definition['configSchema']['properties']
    schema = definition['inputSchema']
    assert set(settings) == {'motion_current', 'tactile_threshold'}
    assert settings['motion_current']['default'] == 230
    assert settings['tactile_threshold']['default'] == 15
    assert all(s['scope'] == 'shared' for s in settings.values())
    assert set(schema['x-action-params']['adaptive_grasp']['params']) == {
        'bound_side', 'speed_ratio', 'confirm_motion'}
    assert schema['x-action-params']['release']['params'] == ['bound_side', 'confirm_motion']
    assert schema['x-action-params']['get_state']['params'] == ['bound_side']
    assert set(schema['x-completion']['actions']) == {'adaptive_grasp', 'release'}
    assert not set(settings).intersection(schema['properties'])


@pytest.mark.parametrize('name', ['motion_current', 'tactile_threshold'])
@pytest.mark.parametrize('value', [0, -1, 256, True, False, 15.5, '15', '', None])
def test_invalid_user_settings_rejected_at_startup_and_config_without_can(name, value, monkeypatch):
    with pytest.raises(ValueError, match=name):
        AdaptiveGraspPlugin(None, {name: value})
    plugin = AdaptiveGraspPlugin(None, {})
    monkeypatch.setattr(plugin, '_get_client_and_ctrl', lambda *a: pytest.fail('config accessed CAN'))
    with pytest.raises(ValueError, match=name):
        plugin.dispatch('config', {name: value})
    assert plugin.motion_current == 230 and plugin.tactile_threshold == 15


def test_shared_user_settings_update_atomically_and_do_not_change_other_cards():
    others = {'plugins': {'hand_grasp': {'motion_current': 123}, 'hand_tactile': {'contact_threshold': 8}}}
    plugin = AdaptiveGraspPlugin(None, {}, global_config=others)
    with pytest.raises(ValueError, match='tactile_threshold'):
        plugin.dispatch('config', {'motion_current': 200, 'tactile_threshold': 0})
    assert plugin.motion_current == 230
    configured = plugin.dispatch('config', {'motion_current': 180, 'tactile_threshold': 12})
    assert configured['state'] == 'configured'
    assert configured['motion_current'] == plugin.config['motion_current'] == 180
    assert configured['tactile_threshold'] == plugin.config['tactile_threshold'] == 12
    assert others == {'plugins': {'hand_grasp': {'motion_current': 123}, 'hand_tactile': {'contact_threshold': 8}}}
    assert plugin.get_tool()['configSchema']['properties']['tactile_threshold']['default'] == 12


@pytest.mark.parametrize('ratio,raw', [('1.0x', 85), ('1.5x', 128), ('2.0x', 170), ('2.5x', 212), ('3.0x', 255)])
def test_selected_speed_keeps_fixed_85_baseline_and_applies_configured_limit(test_setup, ratio, raw):
    plugin, client, _, _, _, events, done = test_setup
    plugin.approach_speed = 200  # Legacy raw-speed setting cannot redefine 1.0x.
    plugin.dispatch('config', {'motion_current': 180, 'tactile_threshold': 15})
    reply = plugin.dispatch('adaptive_grasp', {'bound_side': 'left', 'speed_ratio': ratio, 'confirm_motion': True})
    assert reply['state'] == 'running' and done.wait(3)
    aid, status, result = events[0]
    assert aid == reply['action_id'] == result['action_id'] and status == 'completed'
    assert result['speed_ratio'] == ratio and result['approach_speed'] == raw
    assert client._speeds == [raw] * 5
    assert result['motion_current'] == 180 and client._currents == [180] * 5
    assert result['tactile_threshold'] == 15 and result['outcome'] == 'contact_hold'


@pytest.mark.parametrize('ratio', ['', '4.0x', '1.0', 1, True, None])
def test_invalid_speed_rejected_before_device_access(ratio, monkeypatch):
    plugin = AdaptiveGraspPlugin(None, {})
    monkeypatch.setattr(plugin, '_get_client_and_ctrl', lambda *a: pytest.fail('invalid speed accessed CAN'))
    with pytest.raises(ValueError, match='speed_ratio'):
        plugin.dispatch('adaptive_grasp', {'bound_side': 'left', 'speed_ratio': ratio, 'confirm_motion': True})


def test_contact_at_15_is_latched_even_if_peak_later_drops(test_setup, monkeypatch):
    plugin, client, _, _, _, _, _ = test_setup
    plugin.dispatch('config', {'tactile_threshold': 15})
    calls = []
    def query(**kwargs):
        calls.append(1)
        return {finger: matrix(15 if finger == 'index' and len(calls) == 1 else 0)
                for finger in FINGER_JOINTS}
    monkeypatch.setattr(client, 'get_fresh_matrix_touch', query)
    status, result = run_grasp(test_setup)
    assert status == 'completed' and len(calls) > 1
    assert result['contacts']['index'] and result['peaks']['index'] == 0
    assert result['target_positions']['index_pitch'] == plugin.prep_pose[1]
    assert result['target_positions']['index_tip'] == plugin.prep_pose[16]
    assert result['joint_states']['index'] == {'root': 'tactile_threshold', 'tip': 'tactile_threshold'}
    assert result['outcome'] == 'contact_hold' and not result['grasp_confirmed']


def test_get_state_reads_final_result_without_can_or_motion(test_setup, monkeypatch):
    plugin, client, bus, _, _, events, _ = test_setup
    run_grasp(test_setup)
    monkeypatch.setattr(client, 'get_positions_vector', lambda: pytest.fail('status queried CAN'))
    monkeypatch.setattr(plugin, '_get_client_and_ctrl', lambda *a: pytest.fail('status opened CAN'))
    count = len(bus.sent_messages)
    result = plugin.dispatch('get_state', {'bound_side': 'left'})
    assert len(bus.sent_messages) == count
    assert result['state'] == 'holding' and result['holding'] and not result['monitoring']
    assert result['active_action_id'] is None and result['action_id'] == events[0][0]
    assert result['last_result']['result'] == events[0][2]
    assert result['message'] == events[0][2]['message']
    assert '收拢完成' in result['message'] and '已保持' in result['message']
    other = plugin.dispatch('get_state', {'bound_side': 'right'})
    assert other['state'] == 'ready' and other['last_result'] is None


@pytest.mark.parametrize('side', [None, '', 'invalid'])
def test_get_state_requires_explicit_side_without_confirmation(side):
    plugin = AdaptiveGraspPlugin(None, {})
    with pytest.raises(ValueError, match='bound_side'):
        plugin.dispatch('get_state', {'bound_side': side})
    assert plugin.dispatch('info', {})['selection_required']


def test_user_settings_cannot_change_during_running_or_holding(test_setup, monkeypatch):
    plugin, client, _, _, _, _, done = test_setup
    sampled = threading.Event()
    resume = threading.Event()
    def query(**kwargs):
        sampled.set()
        assert resume.wait(3)
        return {finger: matrix(35) for finger in FINGER_JOINTS}
    monkeypatch.setattr(client, 'get_fresh_matrix_touch', query)
    reply = plugin.dispatch('adaptive_grasp', {'bound_side': 'left', 'confirm_motion': True})
    try:
        assert sampled.wait(2)
        state = plugin.dispatch('get_state', {'bound_side': 'left'})
        assert state['active_action_id'] == reply['action_id'] and state['monitoring']
        with pytest.raises(RuntimeError, match='先 release'):
            plugin.dispatch('config', {'tactile_threshold': 30})
        assert plugin.dispatch('config', {'tactile_threshold': 25})['state'] == 'configured'
    finally:
        resume.set()
        assert done.wait(3)
    with pytest.raises(RuntimeError, match='先 release'):
        plugin.dispatch('config', {'motion_current': 100})
    assert plugin.motion_current == 230 and plugin.tactile_threshold == 25


def test_holding_blocks_other_card_until_release_and_stale_speed_cannot_block_release(test_setup):
    plugin, client, bus, ctrl, hand, events, done = test_setup
    run_grasp(test_setup)
    count = len(bus.sent_messages)
    with pytest.raises(RuntimeError, match='正在抓握或保持'):
        hand.dispatch('open', {'bound_side': 'left', 'confirm_motion': True})
    assert len(bus.sent_messages) == count
    assert ctrl._holding_torque == 230 and ctrl._grasp_owner == (id(plugin), ('left', 'can0'))
    done.clear()
    release = plugin.dispatch('release', {'bound_side': 'left', 'confirm_motion': True,
                                          'speed_ratio': 'invalid'})
    assert done.wait(3) and len(events) == 2
    assert events[-1][0] == release['action_id'] and events[-1][1] == 'completed'
    assert events[-1][2]['hand_opened'] and not events[-1][2]['holding']
    assert client._currents == [0] * 5 and ctrl._holding_torque == 0
    assert '_grasp_owner' not in vars(ctrl) and '_test_grasp_owner' not in vars(ctrl)
    assert plugin.dispatch('config', {'motion_current': 100})['state'] == 'configured'


def test_stop_clears_holding_ownership_and_does_not_present_old_result_as_current(test_setup):
    plugin, client, _, ctrl, _, events, _ = test_setup
    run_grasp(test_setup)
    stopped = plugin.dispatch('stop', {'bound_side': 'left'})
    assert stopped['state'] == 'idle' and stopped['torque_released']
    state = plugin.dispatch('get_state', {'bound_side': 'left'})
    assert state['state'] == 'idle' and not state['holding'] and not state['monitoring']
    assert state['last_result']['action_id'] == events[0][0]
    assert '上一动作结果仅供查阅' in state['message']
    assert '_grasp_owner' not in vars(ctrl) and client._currents == [0] * 5


def test_stop_during_sampling_delivers_one_cancelled_result_with_original_task_id(test_setup, monkeypatch):
    plugin, client, _, ctrl, _, events, done = test_setup
    entered = threading.Event()
    def query(*, timeout_s, cancel):
        entered.set()
        assert cancel.wait(3)
        raise InterruptedError('motion stopped')
    monkeypatch.setattr(client, 'get_fresh_matrix_touch', query)
    reply = plugin.dispatch('adaptive_grasp', {'bound_side': 'left', 'confirm_motion': True})
    try:
        assert entered.wait(2)
        stopped = plugin.dispatch('stop', {'bound_side': 'left'})
        assert stopped['state'] == 'idle' and done.wait(3)
        assert len(events) == 1 and events[0][:2] == (reply['action_id'], 'cancelled')
        result = plugin.dispatch('get_state', {'bound_side': 'left'})
        assert result['action_id'] == reply['action_id']
        assert result['last_result']['status'] == 'cancelled'
        assert not result['holding'] and not result['monitoring'] and not result['active_action_id']
        assert client._currents == [0] * 5 and '_grasp_owner' not in vars(ctrl)
    finally:
        if not done.is_set():
            plugin.dispatch('stop', {'bound_side': 'left'})


@pytest.mark.parametrize('value', [1, 255])
def test_user_setting_boundaries_are_valid(value):
    plugin = AdaptiveGraspPlugin(None, {'motion_current': value, 'tactile_threshold': value})
    result = plugin.dispatch('config', {'motion_current': value, 'tactile_threshold': value})
    assert result['motion_current'] == value and result['tactile_threshold'] == value


def test_adaptive_grasp_replaces_old_card():
    config = yaml.safe_load((DRIVER / "config.yaml").read_text())
    plugins = build_plugins(config, "", None)
    tools = {plugin.get_tool()["name"]: plugin.get_tool() for plugin in plugins}
    assert set(tools) == {"hand", "hand_gesture", "hand_grasp", "hand_tactile"}
    assert next(plugin for plugin in plugins if plugin.PREFIX == "hand_grasp").__class__ is AdaptiveGraspPlugin
    schema = tools["hand_grasp"]["inputSchema"]
    assert set(schema["x-action-params"]) == {"adaptive_grasp", "release", "get_state", "config"}
    assert set(schema["x-completion"]["actions"]) == {"adaptive_grasp", "release"}


def test_adaptive_grasp_requires_explicit_motion_confirmation():
    plugin = AdaptiveGraspPlugin(None, {})
    try:
        result = plugin.dispatch("adaptive_grasp", {"bound_side": "left"})
    except (ValueError, PermissionError) as exc:
        assert "confirm" in str(exc).lower() or "确认" in str(exc)
    else:
        assert result is None or result.get("state") == "error"
