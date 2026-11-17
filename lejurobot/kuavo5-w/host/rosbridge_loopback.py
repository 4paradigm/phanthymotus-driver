#!/usr/bin/env python3
"""Process-local ROS1 rosbridge fixes: loopback binding and low-latency TCPROS."""
from functools import partial
from pathlib import Path
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


def configure_messages(root):
    overlay = Path(root) / 'ros1_messages'
    if all((overlay / 'kuavo_msgs' / part / '__init__.py').is_file()
           for part in ('msg', 'srv')):
        sys.path.insert(0, str(overlay))
        return str(overlay)
    return None


if __name__ == '__main__':
    # Optional deployment-local generated classes copied from the actual ROS1
    # controller. Application and controller workspaces can carry incompatible
    # kuavo_msgs MD5s. Keep this override isolated to our bridge/rosapi processes.
    configure_messages(Path(__file__).resolve().parent)
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
