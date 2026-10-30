"""Typed S10 commands for the isolated vendor-domain worker."""
from __future__ import annotations
import math
import threading
import time
from collections.abc import Mapping

COMMAND_SPECS = {
    'state': {'topic': '/MOTION_STATE', 'type': 'drdds/msg/MotionState'},
    'velocity': {'topic': '/NAV_CMD', 'type': 'drdds/msg/NavCmd'},
}

STATE_VALUES = frozenset({1, 4, 17})
VELOCITY_LIMITS = {'vx': 1.67, 'vy': .4, 'yaw_rate': 1.0}
EXPECTED_FIELDS = {
    'drdds/msg/MotionState': {'header': 'drdds/msg/MetaType', 'data': 'drdds/msg/MotionStateValue'},
    'drdds/msg/NavCmd': {'header': 'drdds/msg/MetaType', 'data': 'drdds/msg/NavCmdValue'},
    'drdds/msg/MetaType': {'frame_id': 'uint64', 'stamp': 'builtin_interfaces/msg/Time'},
    'drdds/msg/MotionStateValue': {'state': 'int32'},
    'drdds/msg/NavCmdValue': {'x_vel': 'float', 'y_vel': 'float', 'yaw_vel': 'float'},
    'builtin_interfaces/msg/Time': {'sec': 'int32', 'nanosec': 'uint32'},
}

class CommandChannelNotReady(RuntimeError):
    pass

def _integer(value, minimum, maximum, name):
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(name + ' must be an integer in range')
    return value


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name + ' must be a finite number')
    return float(value)


def _type_name(value):
    if not isinstance(value, str):
        return None
    value = value.replace('::', '/')
    if value == 'float32':
        return 'float'
    parts = value.split('/')
    if len(parts) == 2:
        return parts[0] + '/msg/' + parts[1]
    return value


def _default_message_getter(name):
    # Importing this file and constructing a codec do not import ROS packages.
    from rosidl_runtime_py.utilities import get_message
    return get_message(name)


class MessageCodec:
    """Construct documented, typed messages after checking installed schemas.

    The repository's actual MetaType uses ``stamp`` with builtin Time, rather
    than the guide's illustrative ``timestamp`` spelling. Reject a mismatching
    installed schema instead of guessing its field name or primitive width.
    """
    def __init__(self, message_getter=None, *, clock_ns=time.time_ns):
        self._getter = message_getter or _default_message_getter
        self._clock_ns = clock_ns
        self._classes = {}
        self._lock = threading.Lock()
        self._next_frame_id = 1

    def _class(self, name):
        if name in self._classes:
            return self._classes[name]
        cls = self._getter(name)
        fields = cls.get_fields_and_field_types()
        if not isinstance(fields, Mapping) or {k: _type_name(v) for k, v in fields.items()} != EXPECTED_FIELDS[name]:
            raise ValueError('installed ROS message schema mismatch for ' + name)
        # Validate all nested message types before constructing the outer one.
        for value in EXPECTED_FIELDS[name].values():
            if '/msg/' in value:
                self._class(value)
        self._classes[name] = cls
        return cls

    def prepare(self):
        """Preload class metadata before the bounded command channel is ready.

        This creates no message instance, DDS participant or publisher. ROS
        import latency belongs to explicit worker startup, not a live command.
        """
        with self._lock:
            # Type metadata loading creates no publisher or command.
            specs = COMMAND_SPECS.values()
            for spec in specs:
                self._class(spec['type'])

    def state_message_type(self):
        """Return validated type metadata without constructing a command."""
        with self._lock:
            return self._class(COMMAND_SPECS['state']['type'])

    def velocity_message_type(self):
        """Return validated NAV_CMD metadata without constructing a command."""
        with self._lock:
            return self._class(COMMAND_SPECS['velocity']['type'])

    def _header(self, message, frame_id, stamp_ns):
        if frame_id is None:
            frame_id = self._next_frame_id
            _integer(frame_id, 0, 2**64 - 1, 'frame_id')
            self._next_frame_id += 1
        else:
            frame_id = _integer(frame_id, 0, 2**64 - 1, 'frame_id')
        stamp_ns = self._clock_ns() if stamp_ns is None else stamp_ns
        stamp_ns = _integer(stamp_ns, 0, (2**31 - 1) * 1_000_000_000 + 999_999_999, 'stamp_ns')
        message.header.frame_id = frame_id
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        return message

    def build_state(self, state, *, frame_id=None, stamp_ns=None):
        if type(state) is not int or state not in STATE_VALUES:
            raise ValueError('only documented state commands 1, 4 and 17 are allowed')
        with self._lock:
            message = self._class(COMMAND_SPECS['state']['type'])()
            self._header(message, frame_id, stamp_ns)
            message.data.state = state
            return message

    def build_velocity(self, vx=0., vy=0., yaw_rate=0., *, frame_id=None, stamp_ns=None):
        values = {'vx': _finite(vx, 'vx'), 'vy': _finite(vy, 'vy'), 'yaw_rate': _finite(yaw_rate, 'yaw_rate')}
        if any(abs(value) > VELOCITY_LIMITS[key] for key, value in values.items()):
            raise ValueError('physical velocity exceeds S10 documented limits')
        with self._lock:
            message = self._class(COMMAND_SPECS['velocity']['type'])()
            self._header(message, frame_id, stamp_ns)
            message.data.x_vel, message.data.y_vel, message.data.yaw_vel = values.values()
            return message

class DDSCommandAdapter:
    def __init__(self, node, *, execution_enabled=False, codec=None):
        self.node, self.enabled = node, execution_enabled
        self.codec = codec or MessageCodec()
        self.publishers = {}

    def prepare_state_channel(self):
        if not self.enabled:
            raise PermissionError('motion disabled')
        for kind, spec in COMMAND_SPECS.items():
            cls = self.codec.state_message_type() if kind == 'state' else self.codec.velocity_message_type()
            self.publishers[kind] = self.node.create_publisher(cls, spec['topic'], 1)

    def graph_snapshot(self):
        return {kind: {'publisher_prepared': kind in self.publishers,
            'matched_subscribers': self.publishers[kind].get_subscription_count() if kind in self.publishers else 0,
            'topic': spec['topic'], 'type': spec['type']} for kind, spec in COMMAND_SPECS.items()}

    def _send(self, kind, build, cancel_event):
        if not self.enabled or (cancel_event is not None and cancel_event.is_set()):
            raise PermissionError('motion disabled or command expired')
        publisher = self.publishers.get(kind)
        if publisher is None or publisher.get_subscription_count() < 1:
            raise CommandChannelNotReady('no matched receiver')
        message = build()
        if cancel_event is not None and cancel_event.is_set():
            raise TimeoutError('command expired before publish')
        publisher.publish(message)
        return {'sent': True, 'topic': COMMAND_SPECS[kind]['topic'],
                'frame_id': message.header.frame_id, 'sent_monotonic': time.monotonic()}

    def publish_state(self, state, *, cancel_event=None):
        return self._send('state', lambda: self.codec.build_state(state), cancel_event)

    def publish_velocity(self, vx=0., vy=0., yaw_rate=0., *, cancel_event=None):
        return self._send('velocity', lambda: self.codec.build_velocity(vx, vy, yaw_rate), cancel_event)

    def close(self):
        for publisher in self.publishers.values():
            self.node.destroy_publisher(publisher)
        self.publishers.clear()
