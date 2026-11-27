"""Temporarily select existing ROS velocity publishers at MPC subscribers.

Uses ROS1's subscriber publisherUpdate API; never rewrites vendor files or
stops joystick nodes. /joy and /stop_robot subscriptions remain untouched.
"""
import xmlrpc.client
import time

CARD_PUBLISHERS = {'/rosbridge_websocket'}
REMOTE_PUBLISHERS = {'/mobile_manipulator_joy_command_node', '/humanoid_joy_control_auto_gait_with_vel'}
CONSUMERS = ('/nodelet_manager', '/mobile_manipulator_sqp_mpc_node')
TOPICS = ('/cmd_vel', '/cmd_vel_world')


class BoundedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection=super().make_connection(host)
        connection.timeout=.5
        return connection


class UpperInputGate:
    def __init__(self, master):
        self.master=master
        self.owner=None
        self.checked_at=0.
        self.original={}

    def query(self, uri, method, *args):
        with xmlrpc.client.ServerProxy(uri,transport=BoundedTransport()) as proxy:
            code,_,value=getattr(proxy,method)('/motus_upper_input_gate',*args)
        if code!=1:raise RuntimeError('upper_bridge_input_gate_rpc_failed')
        return value

    def select(self, owner):
        if owner not in ('card','remote','hold'):raise ValueError('invalid_input_owner')
        graph=self.query(self.master,'getSystemState')
        pubs,subs=dict(graph[0]),dict(graph[1])
        allowed=CARD_PUBLISHERS if owner=='card' else REMOTE_PUBLISHERS if owner=='remote' else set()
        expected={}
        for topic in TOPICS:
            source_names=set(pubs.get(topic,[])) & allowed
            if owner=='card' and topic=='/cmd_vel' and source_names!=CARD_PUBLISHERS:
                raise RuntimeError('upper_bridge_card_publisher_missing')
            if owner=='remote' and topic=='/cmd_vel' and not source_names:
                raise RuntimeError('upper_bridge_remote_publisher_missing')
            expected[topic]={self.query(self.master,'lookupNode',name) for name in source_names}
        for name in CONSUMERS:
            uri=self.query(self.master,'lookupNode',name)
            for topic in TOPICS:
                if name not in subs.get(topic,[]):raise RuntimeError('upper_bridge_mpc_subscription_missing')
                self.original[(uri,topic)]=[self.query(self.master,'lookupNode',p) for p in pubs.get(topic,[])]
                self.query(uri,'publisherUpdate',topic,sorted(expected[topic]))
        # Acknowledgement is not sufficient: old TCPROS links must be gone.
        end=time.monotonic()+1.
        while time.monotonic()<end:
            ready=True
            for name in CONSUMERS:
                uri=self.query(self.master,'lookupNode',name)
                bus=self.query(uri,'getBusInfo')
                for item in bus:
                    if len(item)>=6 and item[2]=='i' and item[4] in TOPICS and item[5]:
                        if item[1] not in expected[item[4]]:ready=False
            if ready:
                self.owner,self.checked_at=owner,time.monotonic()
                return
            time.sleep(.02)
        raise RuntimeError('upper_bridge_input_gate_connections_unconfirmed')

    def restore(self):
        # Re-query the master rather than restoring dead publisher URIs.
        graph=self.query(self.master,'getSystemState')
        pubs=dict(graph[0])
        for uri,topic in self.original:
            peers=[self.query(self.master,'lookupNode',p) for p in pubs.get(topic,[])]
            self.query(uri,'publisherUpdate',topic,peers)
        self.owner=None
