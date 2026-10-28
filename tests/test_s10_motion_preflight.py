"""The graph probe cannot create a command publisher or infer control rights."""
from types import SimpleNamespace
import pytest
from deep_robotics.s10_pro.motion_preflight import SPECS, observe_graph, report


class Graph:
    def get_publishers_info_by_topic(self, topic):
        return [SimpleNamespace(node_name='vendor', node_namespace='/', topic_type=next(v['type'] for v in SPECS.values() if v['topic'] == topic))]

    def get_subscriptions_info_by_topic(self, topic):
        return self.get_publishers_info_by_topic(topic)

    def create_publisher(self, *args):
        pytest.fail('graph probe must not create publishers')


def test_graph_endpoint_presence_does_not_grant_authority_or_infer_mode():
    now = [0.]
    def spin(node, timeout_sec):
        now[0] += timeout_sec
    rows = observe_graph(Graph(), seconds=.2, clock=lambda: now[0], spin_once=spin)
    assert len(rows) == 3 and all(v['expected_type_present'] for v in rows.values())
    result = report({}, rows)
    assert result['control_ownership'] == result['operating_mode'] == result['robot_deadman'] == 'unknown'
    assert result['command_publishers_created'] == result['robot_commands_sent'] == 0
    assert result['tls_required'] is False and result['execution_available'] is False


@pytest.mark.parametrize('seconds', [0, -1, 11, float('nan'), True])
def test_graph_discovery_is_bounded(seconds):
    with pytest.raises(ValueError):
        observe_graph(Graph(), seconds=seconds)
