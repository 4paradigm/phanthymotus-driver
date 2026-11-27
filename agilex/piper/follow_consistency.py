"""Read-only ALOHA follow diagnostics over existing PiPER SocketCAN buses."""

import json
import math
import re
import socket
import struct
import threading
import time

from common.vendor_runtime import action_schema, run_driver, tool
from agilex.piper.follow_analysis import FRAME_IDS, FollowPair


NAME = 'piper_follow_consistency'
CAN_FRAME = struct.Struct('=IB3x8s')
CAN_FILTER = struct.Struct('=II')
# SocketCAN reserves CAN_ERR_FLAG in a filter mask for error subscriptions.
# Match the standard data identifier and reject extended and RTR frames.
CAN_MASK = 0xC00007FF
KERNEL_TIMESPEC = struct.Struct('@ll')
# SO_TIMESTAMPNS_OLD on the supported 64-bit Linux image architectures.
SO_TIMESTAMPNS = getattr(socket, 'SO_TIMESTAMPNS', 35)


def receive_socket(interface):
    connection = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        connection.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                             b''.join(CAN_FILTER.pack(frame_id, CAN_MASK) for frame_id in FRAME_IDS))
        connection.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
        connection.settimeout(0.15)
        connection.bind((interface,))
    except OSError:
        connection.close()
        raise
    return connection


def receive_sample(connection):
    packet, ancillary, flags, _ = connection.recvmsg(CAN_FRAME.size, socket.CMSG_SPACE(KERNEL_TIMESPEC.size))
    # SocketCAN marks frames originating from another local socket as DONTROUTE.
    if flags & (socket.MSG_DONTROUTE | socket.MSG_TRUNC | socket.MSG_CTRUNC):
        return None
    if len(packet) != CAN_FRAME.size:
        return None
    frame_id, size, payload = CAN_FRAME.unpack(packet)
    if frame_id not in FRAME_IDS or size != 8:
        return None
    for level, kind, data in ancillary:
        if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS and len(data) == KERNEL_TIMESPEC.size:
            seconds, nanos = KERNEL_TIMESPEC.unpack(data)
            age = time.time() - (seconds + nanos / 1_000_000_000)
            if -0.005 <= age <= 0.5:
                return frame_id, payload, time.monotonic() - max(age, 0.0)
    return None


def parameter(config, key, default, minimum, maximum):
    value = config.get(key, default)
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
            or not minimum <= value <= maximum):
        raise ValueError(f'follow_consistency.{key} must be between {minimum} and {maximum}')
    return float(value)


class FollowConsistency:
    """Each configured bus carries one external leader target and follower feedback."""

    def __init__(self, config, namespace, ros2):
        pairs = config.get('pairs')
        if (not isinstance(pairs, dict) or not 1 <= len(pairs) <= 2
                or any(not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,15}', name)
                       or not isinstance(interface, str) or not re.fullmatch(r'[a-zA-Z0-9_.-]{1,15}', interface)
                       for name, interface in pairs.items())
                or len(set(pairs.values())) != len(pairs)):
            raise ValueError('follow_consistency.pairs requires one or two named, distinct CAN interfaces')
        self.interfaces = dict(pairs)
        self.topic = f'/{namespace}/{NAME}'
        self.ros2 = ros2
        self.tolerance = parameter(config, 'tolerance_deg', 3, 0.1, 90)
        self.persistence = parameter(config, 'persistence_ms', 250, 20, 5000)
        self.stale = parameter(config, 'stale_ms', 350, 100, 2000)
        self.max_gap = parameter(config, 'max_alignment_gap_ms', 80, 10, self.stale)
        self.pairs = {name: FollowPair(self.tolerance, self.persistence / 1000,
                                       self.stale / 1000, self.max_gap / 1000) for name in pairs}
        self.errors = {name: None for name in pairs}
        self.lifecycle_lock = threading.RLock()
        self.data_lock = threading.Lock()
        self.finished = threading.Event()
        self.node = None
        self.workers = []

    def get_tool(self):
        schema = action_schema({'info': ([], 'Read monitoring state and current joint comparison'),
                                'start': ([], 'Start passive CAN monitoring'),
                                'stop': ([], 'Stop this monitor')}, {})
        result = tool(NAME, 'sensor',
                      'Compares external PiPER joint targets with measured follower positions. '
                      'Does not issue CAN commands or change operating modes.', schema,
                      topic_out=[{'topic': self.topic, 'format': 'data/json'}])
        result['title'] = 'ALOHA 主从跟随检测'
        return result

    def start(self):
        from std_msgs.msg import String

        with self.lifecycle_lock:
            if self.node is not None:
                return
            if self.ros2.executor_core is None:
                raise RuntimeError('Follow monitoring requires the Core DDS domain')
            node = self.ros2.rclpy.create_node(f'{NAME}_{self.topic.split("/")[1]}', context=self.ros2.ctx_core)
            try:
                publisher = node.create_publisher(String, self.topic, 10)
                node.create_timer(0.2, lambda: self._publish(node, publisher, String))
                self.ros2.executor_core.add_node(node)
            except Exception:
                node.destroy_node()
                raise
            self.finished.clear()
            with self.data_lock:
                for name, pair in self.pairs.items():
                    pair.reset()
                    self.errors[name] = None
            self.node = node
            try:
                for name, interface in self.interfaces.items():
                    worker = threading.Thread(target=self._listen, args=(name, interface),
                                              daemon=True, name=f'{NAME}_{name}')
                    worker.start()
                    self.workers.append(worker)
            except Exception:
                self.stop()
                raise

    def stop(self):
        with self.lifecycle_lock:
            if self.node is None:
                return
            self.finished.set()
            for worker in self.workers:
                worker.join()
            self.workers.clear()
            self.ros2.executor_core.remove_node(self.node)
            self.node.destroy_node()
            self.node = None

    def _listen(self, name, interface):
        while not self.finished.is_set():
            try:
                with receive_socket(interface) as connection:
                    with self.data_lock:
                        self.errors[name] = None
                    while not self.finished.is_set():
                        try:
                            sample = receive_sample(connection)
                        except socket.timeout:
                            continue
                        if sample is not None:
                            with self.data_lock:
                                self.pairs[name].accept(*sample)
            except (AttributeError, OSError) as exc:
                with self.data_lock:
                    self.pairs[name].reset()
                    self.errors[name] = str(exc)
                self.finished.wait(0.5)

    def _results(self, *, include_lag=False):
        with self.data_lock:
            now = time.monotonic()
            results = {}
            for name, pair in self.pairs.items():
                result = pair.snapshot(now, include_lag=include_lag)
                result['interface'] = self.interfaces[name]
                if self.errors[name] is not None:
                    result.update(state='interface_unavailable', interface_error=self.errors[name])
                results[name] = result
        return results

    def _publish(self, node, publisher, message_type):
        with self.lifecycle_lock:
            if self.node is not node:
                return
            summary = {}
            for name, result in self._results().items():
                for key, value in result.items():
                    if key not in {'joints', 'estimated_lag_ms'}:
                        summary[f'{name}_{key}'] = value
            message = message_type()
            message.data = json.dumps(summary, allow_nan=False)
            publisher.publish(message)

    def dispatch(self, action, args):
        if action == 'start':
            self.start()
        elif action == 'stop':
            self.stop()
        elif action not in {'info', NAME}:
            raise ValueError('Follow monitoring supports only start, stop and info')
        with self.lifecycle_lock:
            running = self.node is not None
            return dict(state='running' if running else 'idle', topic_out=self.get_tool()['topic_out'],
                        tolerance_deg=self.tolerance, persistence_ms=self.persistence,
                        pairs=self._results(include_lag=True) if running else {})


def build_monitor_only(config, namespace, ros2):
    follow = config.get('follow_consistency', {})
    if follow.get('enabled') is not True:
        raise ValueError('The standalone follow monitor requires follow_consistency.enabled: true')
    return [FollowConsistency(follow, namespace, ros2)]


if __name__ == '__main__':
    run_driver(__file__, 'agilex-piper-follow-consistency', NAME, build_monitor_only)
