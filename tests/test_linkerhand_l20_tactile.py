"""Tests for LinkerHand L20 Tactile Sensor Card (hand_tactile)."""

import json
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest import mock
import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand" / "l20"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))
sys.path.insert(0, str(ROOT / "tests"))

from common.vendor_runtime import DriverBundle
from hand_device import HandPlugin, build_plugins
from tactile_device import TactilePlugin
from can_client import FINGER_NAMES


class FakeCanClient:
    def __init__(self):
        self._touch = [0, 5, 25, 12, 1]
        self._normal_force = [0, 5, 25, 12, 1]
        self._tangential_force = [0, 0, 0, 0, 0]
        self._tangential_dir = [0, 0, 0, 0, 0]
        self._matrix_touch = [[[0] * 6 for _ in range(12)] for _ in range(5)]
        # Add some touch on middle finger matrix
        self._matrix_touch[2][5][3] = 42
        self.matrix_requested = 0
        self.scalar_requested = 0
        self.scalar_online = True
        self.matrix_online = True

    def get_fresh_tactile(self, *, timeout_s, cancel=None):
        self.scalar_requested += 1
        if cancel is not None and cancel.is_set():
            raise InterruptedError('stopped')
        if not self.scalar_online:
            raise TimeoutError('missing fresh scalar replies')
        return {**self.get_tactile_snapshot(), 'scalar_age_s': {n: 0.0 for n in FINGER_NAMES}}

    def get_fresh_matrix_touch(self, *, timeout_s):
        self.matrix_requested += 1
        if not self.matrix_online:
            raise TimeoutError('missing fresh matrix rows')
        return self.get_matrix_touch()

    def request_matrix_touch(self):
        self.matrix_requested += 1

    def get_tactile_snapshot(self):
        return {
            "touch_scalar": {name: self._touch[i] for i, name in enumerate(FINGER_NAMES)},
            "normal_force": {name: self._normal_force[i] for i, name in enumerate(FINGER_NAMES)},
            "tangential_force": {name: self._tangential_force[i] for i, name in enumerate(FINGER_NAMES)},
            "tangential_dir": {name: self._tangential_dir[i] for i, name in enumerate(FINGER_NAMES)},
        }

    def get_matrix_touch(self):
        return {name: self._matrix_touch[i] for i, name in enumerate(FINGER_NAMES)}


class FakeController:
    def __init__(self, client):
        self._client = client

    def get(self):
        return self._client


@pytest.fixture
def fake_hand():
    hand = mock.Mock(spec=HandPlugin)
    client = FakeCanClient()
    ctrl = FakeController(client)
    hand.config = {"side": "left", "can_channel": "can0"}
    hand.get_controller_for_side = mock.Mock(return_value=ctrl)
    return hand, client


def test_tactile_tool_metadata(fake_hand):
    hand, _ = fake_hand
    plugin = TactilePlugin(hand, {"poll_hz": 20.0, "auto_start": False})
    tool_def = plugin.get_tool()

    assert tool_def["name"] == "hand_tactile"
    assert tool_def["type"] == "sensor"
    assert tool_def["multiInstance"] is True
    assert tool_def["topic_out"] == [
        {"topic": "/hand/left/tactile/heatmap", "format": "image/jpeg"},
        {"topic": "/hand/left/tactile", "format": "data/json"},
    ]

    schema = tool_def["inputSchema"]
    assert set(schema['properties']) == {'action', 'bound_side'}
    assert set(tool_def['configSchema']['properties']) == {'can_channel', 'poll_hz'}
    assert tool_def['configSchema']['properties']['can_channel']['scope'] == 'instance'
    assert "action" in schema["properties"]
    assert set(schema["properties"]["action"]["enum"]) == {"info", "get_state", "start", "stop", "config"}


def test_tactile_info_and_config(fake_hand):
    hand, _ = fake_hand
    plugin = TactilePlugin(hand, {"poll_hz": 20.0, "auto_start": False})

    info = plugin.dispatch("info", {})
    assert info["state"] == "idle"
    assert info["bound_side"] == "left"
    assert info["can_channel"] == "can0"

    cfg = plugin.dispatch("config", {"instance_id": "inst_1", "bound_side": "right"})
    assert cfg["state"] == "configured"
    assert cfg["bound_side"] == "right"


@pytest.mark.parametrize('args', [{}, {'poll_hz': 10}, {'instance_id': 'sensor_1'},
                                  {'instance_id': 'sensor_1', 'poll_hz': 10}])
def test_partial_sensor_config_does_not_require_or_create_binding(fake_hand, args):
    hand, client = fake_hand
    plugin = TactilePlugin(hand, {'auto_start': False})
    plugin.dispatch('config', {'instance_id': 'sensor_1', 'bound_side': 'right',
                               'can_channel': 'can1'})
    result = plugin.dispatch('config', args)
    assert result['state'] == 'configured'
    assert plugin._bindings == {'sensor_1': 'right'}
    assert plugin._channels == {'sensor_1': 'can1'}
    assert plugin.dispatch('info', {'instance_id': 'sensor_1'})['bound_side'] == 'right'
    assert plugin.poll_hz == args.get('poll_hz', 20)
    assert client.scalar_requested == client.matrix_requested == 0


def test_core_shared_then_instance_config_sequence(fake_hand):
    hand, _ = fake_hand
    plugin = TactilePlugin(hand, {'auto_start': False})
    # Core plan_config_calls sends shared settings with no instance or side.
    plugin.dispatch('config', {'poll_hz': 10})
    plugin.dispatch('config', {'instance_id': 'sensor_1', 'bound_side': 'right',
                               'can_channel': 'can1'})
    result = plugin.dispatch('get_state', {'instance_id': 'sensor_1'})
    assert result['side'] == 'right' and result['can_channel'] == 'can1'
    assert result['scalar_valid'] and plugin.poll_hz == 10
    hand.get_controller_for_side.assert_called_with('right', 'can1')
    # Applying only a channel later preserves the selected side.
    plugin.dispatch('config', {'instance_id': 'sensor_1', 'can_channel': 'can2'})
    info = plugin.dispatch('info', {'instance_id': 'sensor_1'})
    assert info['bound_side'] == 'right' and info['can_channel'] == 'can2'


@pytest.mark.parametrize('value', ['', None, False, 1, 'LEFT', 'unknown'])
def test_explicit_invalid_config_side_still_rejected(fake_hand, value):
    hand, _ = fake_hand
    plugin = TactilePlugin(hand, {'auto_start': False})
    with pytest.raises(ValueError, match='bound_side'):
        plugin.dispatch('config', {'bound_side': value, 'poll_hz': 10})
    assert plugin._bindings == {} and plugin.poll_hz == 20


def test_tactile_get_state(fake_hand):
    hand, client = fake_hand
    plugin = TactilePlugin(hand, {"poll_hz": 20.0, "auto_start": False, "contact_threshold": 8})

    res = plugin.dispatch("get_state", {})
    assert res["action"] == "get_state"
    assert res["state"] == "ok"
    assert res["bound_side"] == "left"
    assert res['scalar_valid'] and res['matrix_valid'] and res['matrix_fresh']
    assert client.scalar_requested == client.matrix_requested == 1
    assert res['force_fields_valid'] is False

    # Scalar pressures
    assert res["touch_scalar"]["middle"] == 25
    assert res["touch_scalar"]["thumb"] == 0

    # Matrix touch & sums
    assert res["touch_matrix"]["middle"][5][3] == 42
    assert res["matrix_sums"]["middle"] == 42
    assert res["matrix_sums"]["thumb"] == 0

    # Contact flags (middle >= 8 or sum > 0)
    assert res["contact_flags"]["middle"] is True
    assert res["contact_flags"]["ring"] is True
    assert res["contact_flags"]["thumb"] is False


def test_tactile_start_and_stop_streaming(fake_hand):
    hand, client = fake_hand
    plugin = TactilePlugin(hand, {"poll_hz": 30.0, "auto_start": False})

    start_res = plugin.dispatch("start", {})
    assert start_res["state"] == "running"
    time.sleep(0.08)

    info = plugin.dispatch("info", {})
    assert info["state"] == "running"

    stop_res = plugin.dispatch("stop", {})
    assert stop_res["state"] == "stopped"

    info2 = plugin.dispatch("info", {})
    assert info2["state"] == "idle"


def test_build_plugins_includes_tactile_when_configured():
    config = {
        "plugins": {
            "hand": {"enabled": True},
            "hand_gesture": {"enabled": True},
            "hand_grasp": {"enabled": True},
            "hand_tactile": {"enabled": True},
        }
    }
    plugins = build_plugins(config, "", None)
    assert len(plugins) == 4
    names = [p.get_tool()["name"] for p in plugins]
    assert names == ["hand", "hand_gesture", "hand_grasp", "hand_tactile"]
    tactile_plugin = next(p for p in plugins if p.get_tool()["name"] == "hand_tactile")
    assert tactile_plugin.get_tool()["type"] == "sensor"


def test_scalar_timeout_is_invalid_instead_of_cached_success(fake_hand):
    hand, client = fake_hand
    plugin = TactilePlugin(hand, {'auto_start': False})
    client.scalar_online = False
    result = plugin.dispatch('get_state', {})
    assert result['state'] == 'error' and not result['scalar_valid']
    assert all(v is None for v in result['touch_scalar'].values())
    assert 'scalar' in result['errors']
    assert result['matrix_valid']  # A different sensor cannot prove scalar freshness.
    client.scalar_online = True
    recovered = plugin.dispatch('get_state', {})
    assert recovered['state'] == 'ok' and recovered['scalar_valid']


def test_matrix_timeout_does_not_fabricate_zero_matrix(fake_hand):
    hand, client = fake_hand
    plugin = TactilePlugin(hand, {'auto_start': False})
    client.matrix_online = False
    result = plugin.dispatch('get_state', {})
    assert result['state'] == 'degraded' and result['scalar_valid']
    assert not result['matrix_valid']
    assert all(value is None for value in result['touch_matrix'].values())
    assert all(value is None for value in result['matrix_sums'].values())


def test_missing_controller_has_no_invented_zero_pressure():
    plugin = TactilePlugin(None, {'auto_start': False})
    result = plugin.dispatch('get_state', {})
    assert result['state'] == 'error'
    assert not result['scalar_valid'] and not result['matrix_valid']
    assert all(v is None for v in result['contact_flags'].values())


@pytest.mark.parametrize('finger,value', [('thumb', -1), ('index', 256), ('middle', True), ('ring', 2.5)])
def test_invalid_scalar_values_are_not_published_as_valid(fake_hand, finger, value):
    hand, client = fake_hand
    client._touch[FINGER_NAMES.index(finger)] = value
    result = TactilePlugin(hand, {}).dispatch('get_state', {})
    assert not result['scalar_valid'] and result['state'] == 'error'
    assert all(v is None for v in result['touch_scalar'].values())


def test_partial_matrix_is_rejected_even_with_valid_scalar(fake_hand):
    hand, client = fake_hand
    client._matrix_touch[2] = [[0] * 6] * 11
    result = TactilePlugin(hand, {}).dispatch('get_state', {})
    assert result['scalar_valid'] and not result['matrix_valid']
    assert result['state'] == 'degraded'


def test_matrix_rate_limit_reports_age_and_never_borrows_other_side(fake_hand):
    hand, client = fake_hand
    plugin = TactilePlugin(hand, {'matrix_request_interval_s': 60})
    first = plugin.dispatch('get_state', {})
    second = plugin.dispatch('get_state', {})
    assert first['matrix_fresh'] and not second['matrix_fresh']
    assert second['matrix_valid'] and second['matrix_age_s'] >= 0
    assert client.scalar_requested == 2 and client.matrix_requested == 1
    plugin.dispatch('get_state', {'bound_side': 'right'})
    assert client.matrix_requested == 2


def test_expired_matrix_and_sample_force_fresh_validation(fake_hand):
    hand, client = fake_hand
    plugin = TactilePlugin(hand, {'matrix_request_interval_s': 60})
    first = plugin.dispatch('get_state', {})
    plugin._matrix_cache[('left', 'can0')]['received_at'] -= 3
    result = plugin.dispatch('get_state', {})
    assert not result['matrix_valid'] and result['state'] == 'degraded'
    plugin._latest_data = {**first, 'timestamp': time.time() - 3}
    reads = client.scalar_requested
    result = plugin.dispatch('get_state', {})
    assert client.scalar_requested == reads + 1
    assert result['sample_age_s'] < 1


def test_matrix_can_be_disabled_without_disabling_pressure_queries(fake_hand):
    hand, client = fake_hand
    result = TactilePlugin(hand, {'matrix_enabled': False}).dispatch('get_state', {})
    assert result['scalar_valid'] and result['state'] == 'ok'
    assert client.scalar_requested == 1 and client.matrix_requested == 0
    assert not result['matrix_valid']


def test_instance_channel_and_shared_poll_rate_are_applied(fake_hand):
    hand, _ = fake_hand
    plugin = TactilePlugin(hand, {})
    configured = plugin.dispatch('config', {'instance_id': 'right', 'bound_side': 'right',
                                           'can_channel': 'can1', 'poll_hz': 10})
    assert configured['stream_side'] == 'right'
    result = plugin.dispatch('get_state', {'instance_id': 'right'})
    assert result['side'] == 'right' and result['can_channel'] == 'can1'
    hand.get_controller_for_side.assert_called_with('right', 'can1')
    assert plugin.poll_hz == 10


@pytest.mark.parametrize('name,value', [('poll_hz', 0), ('poll_hz', True), ('poll_hz', float('nan')),
                                     ('contact_threshold', 256), ('auto_start', 'false'),
                                     ('matrix_enabled', 1), ('tactile_timeout_s', 0)])
def test_invalid_sensor_configuration_is_rejected(name, value):
    with pytest.raises(ValueError):
        TactilePlugin(None, {name: value})


@pytest.fixture
def ros_runtime(monkeypatch):
    published = []
    images = []
    class String:
        data = ''
    class CompressedImage:
        data = b''
        format = ''
    class Publisher:
        def __init__(self, topic):
            self.topic = topic
        def publish(self, msg):
            if isinstance(msg, String):
                published.append(json.loads(msg.data))
            else:
                images.append({'topic': self.topic, 'data': msg.data})
        def get_subscription_count(self):
            return 2
    class Node:
        def __init__(self, *args, **kwargs):
            self.destroyed = False
        def create_publisher(self, msg_type, topic, qos):
            return Publisher(topic)
        def destroy_node(self):
            self.destroyed = True
    class QoS:
        def __init__(self, **kwargs):
            self.settings = kwargs
    monkeypatch.setitem(sys.modules, 'rclpy.node', SimpleNamespace(Node=Node))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', SimpleNamespace(QoSProfile=QoS,
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1),
        DurabilityPolicy=SimpleNamespace(VOLATILE=1), HistoryPolicy=SimpleNamespace(KEEP_LAST=1)))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', SimpleNamespace(String=String))
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', SimpleNamespace(CompressedImage=CompressedImage))
    monkeypatch.setitem(sys.modules, 'tactile_heatmap', SimpleNamespace(
        render_heatmap_jpeg=lambda matrix, scalar, side: b'jpeg:' + side.encode()))
    return SimpleNamespace(ctx_core=object(), executor_core=mock.Mock(), images=images), published


def wait_for_stream(predicate):
    end = time.monotonic() + 3
    while not predicate() and time.monotonic() < end:
        time.sleep(.005)
    assert predicate()


def test_card_selection_moves_bootstrap_json_and_heatmap_to_right(fake_hand, ros_runtime):
    hand, _ = fake_hand
    ros, published = ros_runtime
    plugin = TactilePlugin(hand, {'poll_hz': 50}, ros2=ros, namespace='robot')
    try:
        plugin.start()
        wait_for_stream(lambda: len(published) > 0)
        result = plugin.dispatch('get_state', {'instance_id': 'card_r', 'bound_side': 'right'})
        assert result['side'] == 'right'
        wait_for_stream(lambda: any(i['data'] == b'jpeg:right' for i in ros.images))
        info = plugin.dispatch('info', {'instance_id': 'card_r'})
        assert info['state'] == 'running' and info['stream_side'] == 'right'
        assert info['topic_out'] == [
            {'topic': '/robot/hand/right/tactile/heatmap', 'format': 'image/jpeg'},
            {'topic': '/robot/hand/right/tactile', 'format': 'data/json'}]
        assert info['heatmap_published_frames'] > 0 and info['heatmap_publish_error'] is None
        assert plugin._worker_thread is None  # Unclaimed default-left worker was stopped.
        count = len(published)
        wait_for_stream(lambda: len(published) > count)
        assert all(v['side'] == 'right' for v in published[count:])
    finally:
        plugin.stop()


def test_left_and_right_cards_keep_independent_streams_and_stop(fake_hand, ros_runtime):
    hand, _ = fake_hand
    ros, published = ros_runtime
    plugin = TactilePlugin(hand, {'poll_hz': 50, 'auto_start': False}, ros2=ros)
    try:
        plugin.dispatch('start', {'instance_id': 'l', 'bound_side': 'left'})
        plugin.dispatch('start', {'instance_id': 'r', 'bound_side': 'right'})
        wait_for_stream(lambda: {v['side'] for v in published} == {'left', 'right'})
        wait_for_stream(lambda: {i['data'] for i in ros.images} == {b'jpeg:left', b'jpeg:right'})
        assert plugin.dispatch('info', {'instance_id': 'l'})['stream_side'] == 'left'
        assert plugin.dispatch('info', {'instance_id': 'r'})['stream_side'] == 'right'
        plugin.dispatch('config', {'poll_hz': 10})  # Core's shared call has no side or ID.
        assert all(stream.poll_hz == 10 for stream in plugin._streams.values())
        plugin.dispatch('stop', {'instance_id': 'l'})
        count = len(published)
        wait_for_stream(lambda: len(published) > count)
        assert all(v['side'] == 'right' for v in published[count:])
        assert plugin.dispatch('info', {'instance_id': 'r'})['state'] == 'running'
        # A later start without explicit side remembers this card's selection.
        plugin.dispatch('stop', {'instance_id': 'r'})
        assert plugin.dispatch('start', {'instance_id': 'r'})['stream_side'] == 'right'
    finally:
        streams = list(plugin._streams.values())
        plugin.stop()
    assert all(not stream._running() and stream._pub is None for stream in streams)


def test_same_target_shares_worker_and_switch_does_not_stop_other_owner(fake_hand, ros_runtime):
    hand, _ = fake_hand
    ros, _ = ros_runtime
    plugin = TactilePlugin(hand, {'auto_start': False}, ros2=ros)
    try:
        plugin.dispatch('start', {'instance_id': 'a', 'bound_side': 'left'})
        plugin.dispatch('start', {'instance_id': 'b', 'bound_side': 'left'})
        assert len(plugin._streams) == 1
        plugin.dispatch('get_state', {'instance_id': 'a', 'bound_side': 'right'})
        assert plugin._running()  # b still needs left.
        assert len(plugin._streams) == 2
        plugin.dispatch('stop', {'instance_id': 'b'})
        assert not plugin._running()
        assert plugin.dispatch('info', {'instance_id': 'a'})['state'] == 'running'
    finally:
        plugin.stop()


def test_running_card_channel_change_moves_both_topics(fake_hand, ros_runtime):
    hand, _ = fake_hand
    ros, published = ros_runtime
    plugin = TactilePlugin(hand, {'poll_hz': 50, 'auto_start': False}, ros2=ros)
    try:
        plugin.dispatch('start', {'instance_id': 'r', 'bound_side': 'right'})
        plugin.dispatch('config', {'instance_id': 'r', 'can_channel': 'can1'})
        info = plugin.dispatch('info', {'instance_id': 'r'})
        assert info['stream_side'] == 'right' and info['stream_can_channel'] == 'can1'
        assert info['topic_out'][0]['topic'] != '/hand/right/tactile/heatmap'
        wait_for_stream(lambda: any(v['side'] == 'right' and v['can_channel'] == 'can1' for v in published))
        wait_for_stream(lambda: any(i['topic'] == info['topic_out'][0]['topic'] for i in ros.images))
        hand.get_controller_for_side.assert_any_call('right', 'can1')
        assert ('right', 'can0') not in plugin._streams
    finally:
        plugin.stop()


def test_stream_publishes_valid_and_error_json_and_recovers(fake_hand, ros_runtime):
    hand, client = fake_hand
    ros, published = ros_runtime
    plugin = TactilePlugin(hand, {'poll_hz': 50, 'auto_start': False}, ros2=ros)
    def wait_for(predicate):
        end = time.monotonic() + 2
        while not predicate() and time.monotonic() < end:
            time.sleep(.005)
        assert predicate()
    try:
        plugin.start()  # auto_start=false must not start lifecycle streaming.
        assert plugin._worker_thread is None
        plugin.dispatch('start', {})
        wait_for(lambda: len(published) >= 2)
        client.scalar_online = False
        wait_for(lambda: any(frame['state'] == 'error' for frame in published))
        client.scalar_online = True
        wait_for(lambda: published[-1]['state'] == 'ok')
        plugin.dispatch('stop', {})
        info = plugin.dispatch('info', {})
        assert info['published_frames'] == len(published)
        assert info['last_publish_time'] is not None and info['publish_error'] is None
        assert len({frame['sequence'] for frame in published}) == len(published)
    finally:
        plugin.stop()
    assert not plugin.dispatch('info', {})['publisher_ready']
    count = len(published)
    time.sleep(.03)
    assert len(published) == count


def test_publisher_start_failure_is_visible_and_can_retry(fake_hand, ros_runtime, monkeypatch):
    hand, _ = fake_hand
    ros, _ = ros_runtime
    plugin = TactilePlugin(hand, {'auto_start': False}, ros2=ros)
    constructor = sys.modules['rclpy.node'].Node
    def fail(*args, **kwargs):
        raise RuntimeError('DDS unavailable')
    monkeypatch.setattr(sys.modules['rclpy.node'], 'Node', fail)
    assert not plugin._setup_ros_publisher()
    info = plugin.dispatch('info', {})
    assert not info['publisher_ready'] and 'DDS unavailable' in info['publish_error']
    monkeypatch.setattr(sys.modules['rclpy.node'], 'Node', constructor)
    assert plugin._setup_ros_publisher()
    assert plugin.dispatch('info', {})['publisher_ready']
    plugin.stop()


def test_thread_start_failure_cleans_up_publisher(fake_hand, ros_runtime, monkeypatch):
    hand, _ = fake_hand
    ros, _ = ros_runtime
    plugin = TactilePlugin(hand, {'auto_start': False}, ros2=ros)
    monkeypatch.setattr(threading.Thread, 'start', mock.Mock(side_effect=RuntimeError('no worker')))
    with pytest.raises(RuntimeError, match='no worker'):
        plugin.dispatch('start', {})
    assert plugin._worker_thread is None and plugin._node is None and plugin._pub is None


def test_real_can_queries_require_new_scalar_and_all_matrix_rows_without_motor_commands():
    from test_linkerhand_l20 import MockCanBus, MockCanMessage, L20CanClient
    class MatrixBus(MockCanBus):
        matrix_online = True
        def send(self, msg):
            if 0xB1 <= msg.data[0] <= 0xB5 and list(msg.data[1:]) == [0xC6]:
                self.sent_messages.append(msg)
                for row in range(12 if self.matrix_online else 11):
                    self._inbox.put(MockCanMessage(self.can_id, [msg.data[0], row * 16] + [row] * 6))
            else:
                super().send(msg)
    bus = MatrixBus(can_id=0x28)
    client = L20CanClient(side='left', can_id=0x28, recv_timeout_s=.05, bus_factory=lambda *_: bus)
    hand = mock.Mock(spec=HandPlugin)
    hand.config = {'side': 'left', 'can_channel': 'can0'}
    hand.get_controller_for_side.return_value = FakeController(client)
    plugin = TactilePlugin(hand, {'matrix_request_interval_s': .01, 'matrix_timeout_s': .05})
    try:
        result = plugin.dispatch('get_state', {})
        assert result['scalar_valid'] and result['matrix_valid']
        assert result['touch_scalar']['index'] == 60
        assert result['touch_matrix']['thumb'][11] == [11] * 6
        bus.matrix_online = False
        plugin._matrix_cache[('left', 'can0')]['requested_at'] -= 1
        result = plugin.dispatch('get_state', {})
        assert result['scalar_valid'] and not result['matrix_valid']
        assert result['state'] == 'degraded' and result['touch_matrix']['thumb'] is None
        assert all(m.data[0] in (*range(0xB1, 0xB6), 0xC0, 0xC1, 0xC2) for m in bus.sent_messages)
    finally:
        client.close()
