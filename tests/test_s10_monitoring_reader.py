"""Read-only monitoring discovery, per-device retention and source lifecycle."""

from __future__ import annotations

import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from deep_robotics.s10_pro import reader  # noqa: E402
from deep_robotics.s10_pro.bridge import SOURCES  # noqa: E402


class Clock:
    value = 100.0

    def __call__(self):
        return self.value


class ReadOnlyNode:
    def __init__(self):
        self.created = []
        self.destroyed = []
        self.graph_calls = 0
        self.topics = [('/HES_STATUS', ['drdds/msg/StdMsgInt32']),
                       ('/fault_aggregator', ['drdds/msg/FaultEventArray'])]
        self.nodes = [('motion_master', '/')]
        self.endpoints = {
            '/HES_STATUS': [SimpleNamespace(node_name='motion_master', node_namespace='/',
                                           topic_type='drdds/msg/StdMsgInt32')],
            '/fault_aggregator': [],
        }
        self.graph_error = False

    def create_subscription(self, kind, topic, callback, qos):
        subscription = SimpleNamespace(kind=kind, topic=topic, callback=callback)
        self.created.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)

    def get_topic_names_and_types(self):
        self.graph_calls += 1
        if self.graph_error:
            raise RuntimeError('private body must not be logged')
        return self.topics

    def get_node_names_and_namespaces(self):
        return self.nodes

    def get_publishers_info_by_topic(self, topic):
        return self.endpoints.get(topic, [])

    def create_publisher(self, *_args, **_kwargs):
        pytest.fail('monitoring reader must never create publishers')

    def create_client(self, *_args, **_kwargs):
        pytest.fail('monitoring reader must never create service clients')


def build(get_message=lambda kind: kind):
    node, clock, events, purges, logs = ReadOnlyNode(), Clock(), [], [], []
    instance = reader.SubscriptionReader(
        node, reader.source_specs({}), get_message, events.append, 'best-effort-depth-one',
        monotonic=clock, wall_time=lambda: clock.value + 1000,
        purge=lambda keys: purges.append(set(keys)), logger=logs.append,
    )
    return instance, node, clock, events, purges, logs


def test_monitoring_source_names_match_captured_vendor_graph():
    specs = reader.source_specs({})
    assert specs['hard_estop'] == {'topic': '/HES_STATUS', 'type': 'drdds/msg/StdMsgInt32'}
    for key, topic, kind in (
        ('exception', '/EXCEPTION_NOTIFICATION', 'Exception'),
        ('fault_event', '/fault_event', 'FaultEvent'),
        ('fault_aggregator', '/fault_aggregator', 'FaultEventArray'),
        ('cpu_status', '/cpu_status', 'CPUStatus'),
        ('memory_status', '/memory_status', 'MemoryStatus'),
        ('filesystem_status', '/filesystem_status', 'FileSystemStatus'),
        ('network_status', '/network_status', 'NetworkStatus'),
    ):
        assert specs[key] == {'topic': topic, 'type': f'drdds/msg/{kind}', 'optional': True}
    assert SOURCES == frozenset(specs)
    assert not {'gps', 'charge_status', 'rear_lidar', 'video'} & SOURCES


@pytest.mark.parametrize('config', [{'topics': {'ros_graph': '/redirect'}},
                                    {'message_types': {'ros_graph': 'foo/msg/Bar'}}])
def test_internal_graph_cannot_be_redirected_to_a_robot_topic(config):
    with pytest.raises(ValueError, match='internal read-only'):
        reader.source_specs(config)


def test_graph_inspection_creates_no_subscription_and_does_not_claim_sensor_health():
    def getter(kind):
        if kind == 'drdds/msg/FaultEventArray':
            raise ModuleNotFoundError(kind)
        return kind

    instance, node, clock, events, _, _ = build(getter)
    instance.apply_control({'active': ['ros_graph']})
    assert not node.created
    graph = events[-1]
    assert graph['key'] == 'ros_graph' and graph['received_monotonic'] == clock.value
    data = graph['data']
    assert data['read_only'] and data['command_publishers_created'] == data['services_called'] == 0
    assert data['nodes'] == [{'name': 'motion_master', 'namespace': '/'}]
    hes = data['sources']['hard_estop']
    assert hes['type_available'] is True and hes['topic_discovered'] is True
    assert hes['publisher_count'] == hes['matching_publishers'] == 1
    assert hes['last_received_at_ms'] is None
    fault = data['sources']['fault_aggregator']
    assert fault['type_available'] is False and fault['publisher_count'] == 0
    assert fault['last_received_at_ms'] is None
    assert 'data' not in fault and 'healthy' not in fault
    assert data['sources']['exception']['topic_discovered'] is False


def test_optional_types_fail_independently_without_fake_empty_fault_messages():
    def getter(kind):
        if kind.endswith('/FaultEventArray'):
            raise ImportError('private')
        return kind

    instance, node, _, events, _, logs = build(getter)
    instance.apply_control({'active': ['hard_estop', 'fault_aggregator', 'ros_graph']})
    assert set(instance.subscriptions) == {'hard_estop'}
    instance.subscriptions['hard_estop'].callback({'value': 0})
    assert events[-1]['data'] == {'value': 0}
    assert not any(event['key'] == 'fault_aggregator' for event in events)
    assert len(node.created) == 1
    assert all('private' not in line for line in logs)


def test_graph_rate_is_capped_and_failure_expires_instead_of_fabricating_empty_graph():
    instance, node, clock, events, _, logs = build()
    instance.apply_control({'active': ['ros_graph'], 'rates_hz': {'ros_graph': 200}})
    clock.value += 0.5
    instance.reconcile()
    assert node.graph_calls == 1 and len(events) == 1
    clock.value += 0.5
    node.graph_error = True
    instance.reconcile()
    assert node.graph_calls == 2 and len(events) == 1
    assert logs == ['ros_graph discovery failed (RuntimeError)']
    clock.value += 1
    node.graph_error = False
    instance.reconcile()
    assert len(events) == 2


def test_graph_receipt_proof_is_validated_and_cleared_when_source_stops():
    instance, _, clock, events, purges, _ = build()
    instance.apply_control({'active': ['hard_estop', 'ros_graph']})
    callback = instance.subscriptions['hard_estop'].callback
    callback(object())
    clock.value += 1
    instance.reconcile()
    assert events[-1]['data']['sources']['hard_estop']['last_received_at_ms'] is None
    callback({'value': 1})
    clock.value += 1
    instance.reconcile()
    assert events[-1]['data']['sources']['hard_estop']['last_received_at_ms'] == 1101000
    instance.apply_control({'active': ['ros_graph']})
    clock.value += 1
    callback({'value': 0})  # Callback from destroyed generation.
    instance.reconcile()
    assert events[-1]['data']['sources']['hard_estop']['last_received_at_ms'] is None
    assert purges[-1] == {'hard_estop'}


def test_graph_bounds_never_report_omitted_sources_as_absent(monkeypatch):
    monkeypatch.setattr(reader, 'MAX_GRAPH_TOPICS', 1)
    monkeypatch.setattr(reader, 'MAX_GRAPH_NODES', 1)
    monkeypatch.setattr(reader, 'MAX_GRAPH_ENDPOINTS', 1)
    instance, node, _, events, _, _ = build()
    node.nodes.append(('another', '/'))
    node.endpoints['/HES_STATUS'].append(node.endpoints['/HES_STATUS'][0])
    instance.apply_control({'active': ['ros_graph']})
    data = events[-1]['data']
    assert data['truncated'] and len(data['topics']) == len(data['nodes']) == 1
    assert data['topics'][0]['publisher_count'] == 2
    assert len(data['topics'][0]['publishers']) == 1
    assert data['sources']['hard_estop']['matching_publishers'] is None
    assert data['sources']['fault_aggregator']['topic_discovered'] is None
    assert data['sources']['fault_aggregator']['publisher_count'] is None


def test_system_status_retains_all_devices_before_output_throttle_and_ages_individually():
    instance, _, clock, events, _, _ = build()
    instance.apply_control({'active': ['cpu_status'], 'rates_hz': {'cpu_status': 1}})
    callback = instance.subscriptions['cpu_status'].callback
    callback({'soc_id': 'CPU_103', 'package_temp': 50, 'avg_util': 25})
    clock.value += 0.2
    callback({'soc_id': 'CPU_106', 'package_temp': 45, 'avg_util': 20})
    assert len(events) == 1
    clock.value += 0.8
    callback({'soc_id': 'CPU_103', 'package_temp': 51, 'avg_util': 26})
    envelope = events[-1]['data']
    assert envelope['latest']['package_temp'] == 51 and not envelope['truncated']
    units = {entry['data']['soc_id']: entry for entry in envelope['items']}
    assert units['CPU_103']['age_s'] == 0
    assert units['CPU_106']['age_s'] == pytest.approx(0.8)
    assert units['CPU_106']['received_at_ms'] == 1100200


@pytest.mark.parametrize('source,base,field', [
    ('network_status', {'soc_id': 'CPU_106'}, 'iface'),
    ('filesystem_status', {'soc_id': 'CPU_106', 'device_path': '/dev/root'}, 'mount_point'),
])
def test_status_peripheral_identities_are_bounded_and_start_without_old_units(source, base, field):
    instance, _, clock, events, _, _ = build()
    instance.apply_control({'active': [source]})
    callback = instance.subscriptions[source].callback
    for number in range(reader.MAX_STATUS_ITEMS + 3):
        clock.value += 1
        callback({**base, field: str(number)})
    data = events[-1]['data']
    assert data['truncated'] and len(data['items']) == reader.MAX_STATUS_ITEMS
    assert data['items'][0]['data'][field] == '3'
    instance.apply_control({'active': []})
    instance.apply_control({'active': [source]})
    callback({**base, field: 'old_callback'})
    instance.subscriptions[source].callback({**base, field: 'new'})
    data = events[-1]['data']
    assert not data['truncated'] and len(data['items']) == 1
    assert data['items'][0]['data'][field] == 'new'


def test_import_validation_allows_absent_optional_types_but_required_hes_stays_strict(monkeypatch):
    ros, qos, utilities = ModuleType('rclpy'), ModuleType('rclpy.qos'), ModuleType('rosidl_runtime_py.utilities')
    qos.QoSProfile = lambda **options: options
    qos.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT='best_effort')
    qos.HistoryPolicy = SimpleNamespace(KEEP_LAST='keep_last')
    qos.DurabilityPolicy = SimpleNamespace(VOLATILE='volatile')
    optional = {spec['type'] for spec in reader.DEFAULT_SOURCES.values() if spec.get('optional')}

    def get_message(kind):
        assert kind != 'internal/ros_graph'
        if kind in optional:
            raise ImportError(kind)
        return kind

    utilities.get_message = get_message
    for name, module in [('rclpy', ros), ('rclpy.qos', qos),
                         ('rosidl_runtime_py', ModuleType('rosidl_runtime_py')),
                         ('rosidl_runtime_py.utilities', utilities)]:
        monkeypatch.setitem(sys.modules, name, module)
    reader.import_ros(reader.source_specs({}), validate_messages=True)
    utilities.get_message = lambda _kind: (_ for _ in ()).throw(ImportError('required missing'))
    with pytest.raises(ImportError):
        reader.import_ros({'hard_estop': reader.DEFAULT_SOURCES['hard_estop']}, validate_messages=True)
