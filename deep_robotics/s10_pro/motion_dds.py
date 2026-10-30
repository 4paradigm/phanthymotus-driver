"""Lazy, opt-in S10 high-level DDS messages and publisher boundary.

The S10 V1.0.2 software guide documents /MOTION_STATE and /NAV_CMD. This
module deliberately does not create a ROS context/node, source environment,
subscribe, call services, change mode, or construct a vendor SDK. Use an
independent vendor-domain process; never pass the platform-domain node here.

DDS publishing has no command acknowledgement. ``sent`` only means the local
publisher returned. Real ownership, mode and feedback must come from a trusted
parent adapter. No default authority grant is inferred from a visible topic.
"""
from __future__ import annotations

import math
import threading
import time
from collections.abc import Mapping


PROTOCOL_SOURCE = 'https://alidocs.dingtalk.com/i/p/OlnXRnDldg3J7XLp/docs/dQPGYqjpJYgQaZdQtBNLa6a6Wakx1Z5N?dontjump=true'
COMMAND_SPECS = {
    'state': {'topic': '/MOTION_STATE', 'type': 'drdds/msg/MotionState'},
    'velocity': {'topic': '/NAV_CMD', 'type': 'drdds/msg/NavCmd'},
}
# Only the requested initial card features. Damping, reset, gait, SDK-mode and
# low-level joint commands are intentionally not part of this adapter.
STATE_VALUES = frozenset({1, 4, 17})
VELOCITY_LIMITS = {'vx': 1.67, 'vy': .4, 'yaw_rate': 1.0}
RECOMMENDED_NAV_RATE_HZ = 10.0
MAX_AUTHORITY_AGE_S = .5
OPERATOR_PERMIT_SCHEMA = 's10_pro.operator_permit/1'
OPERATOR_PERMIT_SCHEMA_V2 = 's10_pro.operator_permit/2'
OPERATOR_PERMIT_SCHEMAS = frozenset({OPERATOR_PERMIT_SCHEMA, OPERATOR_PERMIT_SCHEMA_V2})
MAX_OPERATOR_ACTION_S = 15.
SUPERVISED_POSTURE_ACTIONS = frozenset({'stand', 'lie', 'move_mode'})
SUPERVISED_DISTANCE_ACTIONS = frozenset({
    'forward_30cm', 'backward_30cm', 'turn_left', 'turn_right',
    'relative_forward', 'relative_backward', 'relative_left', 'relative_right',
    'relative_turn_left', 'relative_turn_right'})
SUPERVISED_ACTIONS = SUPERVISED_POSTURE_ACTIONS | SUPERVISED_DISTANCE_ACTIONS
OPERATOR_PERMIT_FIELDS = frozenset({'schema', 'session_id', 'request_id', 'action',
    'robot_session', 'issued_monotonic', 'expires_monotonic', 'action_deadline_monotonic'})
OPERATOR_PERMIT_FIELDS_V2 = OPERATOR_PERMIT_FIELDS | {'lifetime'}
EXPECTED_FIELDS = {
    'drdds/msg/MotionState': {'header': 'drdds/msg/MetaType', 'data': 'drdds/msg/MotionStateValue'},
    'drdds/msg/NavCmd': {'header': 'drdds/msg/MetaType', 'data': 'drdds/msg/NavCmdValue'},
    'drdds/msg/MetaType': {'frame_id': 'uint64', 'stamp': 'builtin_interfaces/msg/Time'},
    'drdds/msg/MotionStateValue': {'state': 'int32'},
    'drdds/msg/NavCmdValue': {'x_vel': 'float', 'y_vel': 'float', 'yaw_vel': 'float'},
    'builtin_interfaces/msg/Time': {'sec': 'int32', 'nanosec': 'uint32'},
}


class MotionDDSDisabled(RuntimeError):
    pass


class AuthorityUnavailable(RuntimeError):
    pass


class NavigationModeRequired(AuthorityUnavailable):
    pass


class DDSAdapterClosed(RuntimeError):
    pass


class CommandChannelNotReady(RuntimeError):
    """No matched receiver for the prepared local command publisher."""


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

    def prepare(self, *, supervised_posture=False):
        """Preload class metadata before the bounded command channel is ready.

        This creates no message instance, DDS participant or publisher. ROS
        import latency belongs to explicit worker startup, not a live command.
        """
        with self._lock:
            # The supervised canvas supports one-shot posture requests and two
            # bounded distance actions. Loading type metadata creates neither a
            # DDS publisher nor a robot command.
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


def validate_authority(authority, *, now, navigation=False):
    """Validate a short-lived parent attestation; never grants ownership itself.

    ``received_monotonic`` and ``expires_monotonic`` must use this host's clock.
    A parent must not construct this attestation from DDS graph discovery alone.
    Navigation mode is required for every NavCmd, including a zero-speed stop.
    """
    if not isinstance(authority, Mapping):
        raise AuthorityUnavailable('explicit parent authority attestation is required')
    values = dict(authority)
    if isinstance(values.get('schema'), str) and values['schema'] in OPERATOR_PERMIT_SCHEMAS:
        raise AuthorityUnavailable('operator permits are not vendor control authority')
    if values.get('owned') is not True or values.get('fresh') is not True:
        raise AuthorityUnavailable('control authority is not owned and fresh')
    request_id = values.get('request_id')
    if not isinstance(request_id, str) or not request_id.strip() or len(request_id) > 128:
        raise AuthorityUnavailable('a bounded parent request_id is required')
    try:
        received = _finite(values.get('received_monotonic'), 'received_monotonic')
        expires = _finite(values.get('expires_monotonic'), 'expires_monotonic')
        age = _finite(values.get('age_s'), 'age_s')
    except ValueError as exc:
        raise AuthorityUnavailable('invalid authority clock values') from exc
    if not (0 <= now - received <= MAX_AUTHORITY_AGE_S and 0 <= age <= MAX_AUTHORITY_AGE_S
            and abs((now - received) - age) <= .1 and now < expires <= received + MAX_AUTHORITY_AGE_S):
        raise AuthorityUnavailable('authority attestation is expired or uses a different clock')
    if navigation and values.get('navigation_mode_confirmed') is not True:
        raise NavigationModeRequired('fresh navigation-mode confirmation is required for NAV_CMD')
    return values


def validate_operator_permit(permit, *, now):
    """Validate private-pipe operator scope; never claims vendor ownership.

    The parent must check its protected operator authorization before IPC. This
    isolated worker verifies the original process receipt, authorization scope
    and finite per-action deadline; it never infers a vendor control lease.
    """
    if not isinstance(permit, Mapping):
        raise AuthorityUnavailable('an exact supervised operator permit is required')
    values = dict(permit)
    schema = values.get('schema')
    if not isinstance(schema, str) or schema not in OPERATOR_PERMIT_SCHEMAS:
        raise AuthorityUnavailable('invalid supervised operator permit schema')
    expected_fields = OPERATOR_PERMIT_FIELDS_V2 if schema == OPERATOR_PERMIT_SCHEMA_V2 else OPERATOR_PERMIT_FIELDS
    if set(values) != expected_fields:
        raise AuthorityUnavailable('an exact supervised operator permit is required')
    for key in ('session_id', 'request_id', 'robot_session'):
        if not isinstance(values[key], str) or not values[key].strip() or len(values[key]) > 128:
            raise AuthorityUnavailable('invalid bounded operator permit identity')
    if values['action'] not in ({'stand', 'lie'} if schema == OPERATOR_PERMIT_SCHEMA_V2 else SUPERVISED_ACTIONS):
        raise AuthorityUnavailable('operator scope does not support this action')
    try:
        issued = _finite(values['issued_monotonic'], 'issued_monotonic')
        deadline = _finite(values['action_deadline_monotonic'], 'action_deadline_monotonic')
        observed_now = _finite(now, 'now')
    except ValueError as exc:
        raise AuthorityUnavailable('invalid operator permit clock values') from exc
    if schema == OPERATOR_PERMIT_SCHEMA_V2:
        if (values['lifetime'] != 'until_revoked' or values['expires_monotonic'] is not None
                or not 0 <= issued <= observed_now < deadline <= observed_now + MAX_OPERATOR_ACTION_S):
            raise AuthorityUnavailable('persistent operator permit has invalid lifetime or action deadline')
    else:
        try:
            expires = _finite(values['expires_monotonic'], 'expires_monotonic')
        except ValueError as exc:
            raise AuthorityUnavailable('invalid operator permit clock values') from exc
        if not (0 <= issued <= observed_now < expires <= issued + 3600
                and observed_now < deadline <= expires):
            raise AuthorityUnavailable('operator permit expired or uses a different clock')
    return values


class DDSCommandAdapter:
    """Publisher wrapper for an independently initialized vendor-domain node.

    Construction and graph inspection create no publisher. An explicitly
    enabled process plus a current parent authority attestation are required on
    *every* send. This is a transport boundary, not an authority implementation,
    action state machine, scheduler, watchdog, or verified MotionBackend.
    """
    def __init__(self, node, *, execution_enabled=False, supervised_posture=False,
                 codec=None, qos=1, clock=time.monotonic):
        if not isinstance(execution_enabled, bool):
            raise ValueError('execution_enabled must be boolean')
        if type(supervised_posture) is not bool:
            raise ValueError('supervised_posture must be boolean')
        self._node = node
        self._enabled = execution_enabled
        self._supervised_posture = supervised_posture
        self._operator_sessions = {}
        self._operator_requests = {}
        self._codec = codec or MessageCodec()
        self._qos = qos
        self._clock = clock
        self._publishers = {}
        self._lock = threading.RLock()
        self._closed = False

    def prepare_state_channel(self):
        """Advertise supervised channels; never construct/publish a command.

        Called only by the explicitly enabled supervised worker at startup.
        Discovery can then finish before the parent accepts a posture task.
        """
        with self._lock:
            if self._closed:
                raise DDSAdapterClosed('DDS command adapter is closed')
            if not self._enabled or not self._supervised_posture:
                raise MotionDDSDisabled('explicit supervised worker preparation required')
            if 'state' not in self._publishers:
                cls = self._codec.state_message_type()
                self._publishers['state'] = self._node.create_publisher(
                    cls, COMMAND_SPECS['state']['topic'], self._qos)
            if 'velocity' not in self._publishers:
                cls = self._codec.velocity_message_type()
                self._publishers['velocity'] = self._node.create_publisher(
                    cls, COMMAND_SPECS['velocity']['topic'], self._qos)

    def _channel_status(self, kind):
        if kind not in COMMAND_SPECS:
            raise ValueError('unknown command channel')
        publisher = self._publishers.get(kind)
        matched = 0 if publisher is None else publisher.get_subscription_count()
        if type(matched) is not int or matched < 0:
            raise CommandChannelNotReady('invalid matched-receiver count')
        return {'publisher_prepared': publisher is not None,
                'matched_subscribers': matched,
                'ready': publisher is not None and matched > 0,
                'receiver_acceptance_verified': False}

    def _state_channel_status(self):
        return self._channel_status('state')

    def _check(self, authority, navigation, cancel_event):
        if self._closed:
            raise DDSAdapterClosed('DDS command adapter is closed')
        if not self._enabled:
            raise MotionDDSDisabled('DDS motion execution is disabled')
        if cancel_event is not None and cancel_event.is_set():
            raise InterruptedError('DDS motion request was cancelled')
        if self._supervised_posture:
            permit = validate_operator_permit(authority, now=self._clock())
            if navigation and permit['action'] not in SUPERVISED_DISTANCE_ACTIONS:
                raise AuthorityUnavailable('only bounded navigation actions may publish NAV_CMD')
            if not navigation and permit['action'] not in SUPERVISED_POSTURE_ACTIONS:
                raise AuthorityUnavailable('navigation scope cannot publish a state command')
            return permit
        return validate_authority(authority, now=self._clock(), navigation=navigation)

    def _consume_operator_permit(self, permit, *, repeatable=False):
        session = permit['session_id']
        original = (permit['schema'], permit.get('lifetime'), permit['robot_session'],
                    permit['issued_monotonic'], permit['expires_monotonic'])
        previous = self._operator_sessions.get(session)
        if previous is not None and previous != original:
            raise AuthorityUnavailable('operator session receipt or scope cannot be renewed')
        key = (session, permit['request_id'])
        signature = (permit['action'], permit['robot_session'], permit['issued_monotonic'],
                     permit['expires_monotonic'], permit['action_deadline_monotonic'])
        if key in self._operator_requests:
            if not repeatable or self._operator_requests[key] != signature:
                raise AuthorityUnavailable('supervised request scope cannot be replayed or changed')
            return
        if len(self._operator_sessions) >= 64 and session not in self._operator_sessions:
            raise AuthorityUnavailable('operator session bound exhausted')
        if len(self._operator_requests) >= 4096:
            raise AuthorityUnavailable('operator request bound exhausted')
        self._operator_sessions[session] = original
        self._operator_requests[key] = signature

    def _send(self, kind, message_factory, *, authority, cancel_event=None):
        with self._lock:
            attestation = self._check(authority, kind == 'velocity', cancel_event)
            if self._supervised_posture and not self._channel_status(kind)['ready']:
                raise CommandChannelNotReady('prepare a matched command channel before sending')
            message = message_factory()
            # Building/importing a type may take time. Do not create a publisher
            # from an expired permit or after a concurrent cancellation.
            self._check(attestation, kind == 'velocity', cancel_event)
            spec = COMMAND_SPECS[kind]
            if kind not in self._publishers:
                self._publishers[kind] = self._node.create_publisher(type(message), spec['topic'], self._qos)
            self._check(attestation, kind == 'velocity', cancel_event)
            channel = None
            if self._supervised_posture:
                channel = self._channel_status(kind)
                if not channel['ready']:
                    raise CommandChannelNotReady('command receiver disconnected before publish')
                self._consume_operator_permit(attestation, repeatable=kind == 'velocity')
            self._publishers[kind].publish(message)
            return {'sent': True, 'accepted': None, 'confirmed': False, 'completed': False,
                    'topic': spec['topic'], 'request_id': attestation['request_id'],
                    'frame_id': message.header.frame_id, 'sent_monotonic': self._clock(),
                    **({'matched_subscribers': channel['matched_subscribers']} if channel is not None else {})}

    def publish_state(self, state, *, authority, cancel_event=None):
        if self._supervised_posture:
            permit = validate_operator_permit(authority, now=self._clock())
            if (permit['action'] not in SUPERVISED_POSTURE_ACTIONS or type(state) is not int
                    or state != {'stand': 1, 'lie': 4, 'move_mode': 17}[permit['action']]):
                raise AuthorityUnavailable('supervised operator action does not match its exact state')
        # In particular: building 1 does not send 17. The parent must wait for a
        # new confirmed stand frame before making a separate 17 request.
        return self._send('state', lambda: self._codec.build_state(state), authority=authority, cancel_event=cancel_event)

    def publish_velocity(self, vx=0., vy=0., yaw_rate=0., *, authority, cancel_event=None):
        if self._supervised_posture:
            permit = validate_operator_permit(authority, now=self._clock())
            if permit['action'] not in SUPERVISED_DISTANCE_ACTIONS:
                raise AuthorityUnavailable('only bounded navigation scope may publish NAV_CMD')
            values = tuple(_finite(value, name) for name, value in (
                ('vx', vx), ('vy', vy), ('yaw_rate', yaw_rate)))
            vx, vy, yaw_rate = values
            scope = permit['action']
            relative = scope.startswith('relative_')
            base_scope = scope.removeprefix('relative_') if relative else scope
            expected = 1 if base_scope in ('forward_30cm', 'turn_left', 'forward', 'left') else -1
            turning = base_scope.startswith('turn_')
            lateral = base_scope in ('left', 'right')
            linear_limit = .12 if relative else 1.
            yaw_limit = .20 if relative else 1.
            invalid = ((vx != 0 or vy != 0 or abs(yaw_rate) > yaw_limit
                        or (yaw_rate != 0 and yaw_rate * expected <= 0)) if turning else
                       (vx != 0 or yaw_rate != 0 or abs(vy) > .12
                        or (vy != 0 and vy * expected <= 0)) if lateral else
                       (vy != 0 or yaw_rate != 0 or abs(vx) > linear_limit
                        or (vx != 0 and vx * expected <= 0)))
            if invalid:
                raise AuthorityUnavailable('navigation command exceeds its direction or speed scope')
        return self._send('velocity', lambda: self._codec.build_velocity(vx, vy, yaw_rate), authority=authority, cancel_event=cancel_event)

    def publish_zero_velocity(self, *, authority, cancel_event=None):
        return self.publish_velocity(0., 0., 0., authority=authority, cancel_event=cancel_event)

    def graph_snapshot(self):
        """Bounded command-topic discovery. It neither subscribes nor publishes."""
        with self._lock:
            if self._closed:
                raise DDSAdapterClosed('DDS command adapter is closed')
            result = {}
            for kind, spec in COMMAND_SPECS.items():
                publishers = list(self._node.get_publishers_info_by_topic(spec['topic']))
                subscribers = list(self._node.get_subscriptions_info_by_topic(spec['topic']))
                if len(publishers) > 128 or len(subscribers) > 128:
                    raise ValueError('command graph exceeds diagnostic endpoint bound')
                result[kind] = {'topic': spec['topic'], 'type': spec['type'],
                    'publisher_count': len(publishers), 'subscriber_count': len(subscribers),
                    'publisher_types': sorted({str(p.topic_type) for p in publishers}),
                    'subscriber_types': sorted({str(p.topic_type) for p in subscribers}),
                    'authority_proven': False}
                if self._supervised_posture:
                    result[kind].update(self._channel_status(kind))
            return result

    def close(self):
        """Destroy only our publishers; closing is not a physical robot stop."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            publishers, self._publishers = self._publishers, {}
            for publisher in publishers.values():
                self._node.destroy_publisher(publisher)
