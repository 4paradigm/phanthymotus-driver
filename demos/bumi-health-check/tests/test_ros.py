import sys
from types import SimpleNamespace
from unittest.mock import Mock
from health_service.service import HealthService
from health_service.ros import RosReceiver


def test_ros_receiver_subscribes_to_exact_topics_and_binds_callbacks(monkeypatch):
    subscriptions = []
    node = Mock()
    def subscribe(message_type, topic, callback, qos):
        subscriptions.append((topic, callback, qos))
        return object()
    node.create_subscription.side_effect = subscribe
    ros = SimpleNamespace(init=Mock(), create_node=Mock(return_value=node),
                          ok=lambda: True, shutdown=Mock())
    executor = Mock()
    monkeypatch.setitem(sys.modules, "rclpy", ros)
    monkeypatch.setitem(sys.modules, "rclpy.executors",
                        SimpleNamespace(SingleThreadedExecutor=lambda: executor))
    monkeypatch.setitem(sys.modules, "rclpy.qos", SimpleNamespace(
        QoSProfile=lambda **kw: kw, ReliabilityPolicy=SimpleNamespace(BEST_EFFORT="best"),
        DurabilityPolicy=SimpleNamespace(VOLATILE="volatile")))
    monkeypatch.setitem(sys.modules, "std_msgs", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(String=object))
    monkeypatch.setattr("health_service.ros.threading.Thread", Mock())
    service = HealthService("robot_a")
    receiver = RosReceiver(service)
    receiver.start()
    assert [s[0] for s in subscriptions] == list(service.config["topics"].values())
    for source, (_, callback, qos) in zip(service.config["topics"], subscriptions):
        callback(SimpleNamespace(data='{"source": "' + source + '"}'))
        assert service.samples[source]["data"]["source"] == source
        assert qos["reliability"] == "best"
    node.create_publisher.assert_not_called()
    receiver.close()
    node.destroy_node.assert_called_once()
    ros.shutdown.assert_called_once()
