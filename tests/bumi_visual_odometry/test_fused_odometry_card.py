"""ROS-free integration check for the separate fused sensor card."""
import importlib
import json
import math
import sys
from types import ModuleType, SimpleNamespace as NS


def test_plugin_outputs_and_does_not_copy_old_segment_frame(monkeypatch):
    class Msg:
        def __init__(self, data=''):
            self.data = data

    class FakeNode:
        def __init__(self, name):
            self.name = name
            self.pubs = {}
            self.subs = {}
        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=1_475_000_000))
        def create_publisher(self, klass, topic, qos):
            self.pubs[topic] = []
            return NS(publish=self.pubs[topic].append)
        def create_subscription(self, klass, topic, callback, qos):
            self.subs[topic] = callback
            self.sub_qos = qos
            return NS()
        def create_timer(self, period, callback):
            self.tick = callback
            return NS()

    rclpy = ModuleType('rclpy')
    rclpy_node = ModuleType('rclpy.node')
    rclpy_node.Node = FakeNode
    qos = ModuleType('rclpy.qos')
    qos.qos_profile_sensor_data = 'best_effort'
    monkeypatch.setitem(sys.modules,'rclpy.qos',qos)
    messages = ModuleType('std_msgs.msg')
    messages.String = Msg
    monkeypatch.setitem(sys.modules,'rclpy',rclpy)
    monkeypatch.setitem(sys.modules,'rclpy.node',rclpy_node)
    monkeypatch.setitem(sys.modules,'std_msgs.msg',messages)
    common = ModuleType('common')
    odom = ModuleType('common.odom')
    def build_sample(*, stamp_ms, twist, pose, vendor):
        return dict(schema='motus.odom/1',stamp_ms=stamp_ms,frame='body',
                    twist=twist,pose=pose,vendor=vendor)
    odom.build_sample = build_sample
    monkeypatch.setitem(sys.modules,'common',common)
    monkeypatch.setitem(sys.modules,'common.odom',odom)
    legs = ModuleType('leg_odometry')
    class LegOdometry:
        def __init__(self, slip_scale=1.):
            self.position=[0.,0.,0.]
            self.stance_samples=0
    legs.LegOdometry=LegOdometry
    monkeypatch.setitem(sys.modules,'leg_odometry',legs)

    from noetix.bumi import fused_odometry
    importlib.reload(fused_odometry)
    plugin = fused_odometry.FusedOdometryPlugin({},'robot', NS(add_node=lambda node: None),object())
    assert plugin.topic.endswith('/bumi_fused_odometry/state/odom')
    assert plugin.visual_topic.endswith('/bumi_visual_odometry/state/odom')
    assert plugin.node.sub_qos == 'best_effort'
    assert [x['format'] for x in plugin.get_tools()[0]['topic_out']] == ['data/json','state/odom']
    plugin.active=True
    for i in range(20):
        plugin.filter.predict(1_000_000_000+i*25_000_000,0,(.002,0),True)
    for t,x in [(1_400_000_000,11.0),(1_425_000_000,11.012)]:
        visual = dict(schema='motus.odom/1',vendor={
            'status':'tracking','pose_reference':'body','segment_id':'s1',
            'visual_stamp_ns':t,'visual_diagnostic':{'inliers':35}},pose={
            'position':[x,0,0], 'quaternion':[0,0,0,1],
            'covariance':[0.0]*36})
        plugin._on_visual(Msg(json.dumps(visual)))
    plugin._publish()
    payload=json.loads(plugin.node.pubs[plugin.topic][-1].data)
    assert payload['vendor']['status']=='tracking'
    assert payload['vendor']['planar_only'] is True
    assert payload['pose']['frame_id']=='bumi_fused_odom'
    assert payload['pose']['covariance'][14]==1.0
    assert plugin.dispatch('info',{})['odom_interface']['pose_drift']=='unbounded'
    plugin.stop()
    plugin._publish()
    stopped=json.loads(plugin.node.pubs[plugin.topic][-1].data)
    assert stopped['pose'] is None
