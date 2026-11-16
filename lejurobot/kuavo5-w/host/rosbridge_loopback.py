#!/usr/bin/env python3
"""Process-local ROS1 rosbridge fixes: loopback binding and low-latency TCPROS."""
from functools import partial
import runpy
import sys


def configure_subscribers(rospy, subscribers):
    # The old bridge constructs rospy.Subscriber without TCP_NODELAY. In this
    # deployment that coalesces odom/images into ~0.5s bursts. Keep queue_size
    # unchanged (all odom samples), and request low-latency TCPROS from publishers.
    # Larger image reads avoid repeatedly draining a 64KB receive buffer.
    subscribers.Subscriber = partial(rospy.Subscriber, tcp_nodelay=True, buff_size=2_000_000)


def listen_loopback(factory, contextFactory=None, backlog=50, interface=''):
    return _listen(factory, contextFactory, backlog, interface='127.0.0.1')


if __name__ == '__main__':
    import rospy
    import rosbridge_library.internal.subscribers as subscribers
    from autobahn.twisted import websocket
    _listen = websocket.listenWS
    configure_subscribers(rospy, subscribers)
    websocket.listenWS = listen_loopback
    if len(sys.argv) < 2:
        raise SystemExit('Expected the rosbridge node executable')
    sys.argv = sys.argv[1:]
    runpy.run_path(sys.argv[0], run_name='__main__')
