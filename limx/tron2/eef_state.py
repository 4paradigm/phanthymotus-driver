"""DACH vendor TCP poses and measured gripper openings on the Motus ROS bus."""
from collections import deque
import asyncio
import math
import time

from eef_kinematics import Kinematics
from gripper import MAX_OPENING, MAX_OPENING_MM, percentage
from skeleton import ARM_NAMES, JOINT_NAMES
from state import vector
from vendor import POSE_JOINT_POLL_SECONDS

STATE_TOPIC = '/limx_tron2/state/eef'
CONTROL_TOPIC = '/limx_tron2/control/eef'
SCHEMA = 'tron2.eef/1'
FRAME = 'base_Link'
JOINT_MAX_AGE = .35
GRIPPER_MAX_AGE = .75
GRIPPER_PERIOD = .2
SIDES = ('left', 'right')


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


class _Freshness:
    """Bind a vendor sample to the start of its first observed host query."""
    def __init__(self):
        self.robot_stamp = None
        self.requested = None
        self.wall_ms = None

    def observe(self, stamp, reception_age, round_trip, maximum):
        if not all(_number(x) and x >= 0 for x in (stamp, reception_age, round_trip)):
            return False
        if self.robot_stamp is not None and stamp < self.robot_stamp:
            return False
        age = reception_age + round_trip
        if self.robot_stamp != stamp:
            # Record late replies too. A faster retry can return the very same sample.
            self.robot_stamp = stamp
            self.requested = time.monotonic() - age
            self.wall_ms = int((time.time() - age) * 1000)
        return age <= maximum and self.fresh(maximum)

    def fresh(self, maximum):
        return self.requested is not None and time.monotonic() - self.requested <= maximum


class EefState:
    def __init__(self, adapter):
        self.adapter = adapter
        self.reader = adapter.reader
        self.kinematics = Kinematics()
        self._publishing = False
        self._connection = None
        self._published = deque(maxlen=32)
        self._joint_time = _Freshness()
        self._gripper_time = _Freshness()
        self._openings = ()

    @staticmethod
    def tool():
        return {'name': 'eef_state', 'type': 'sensor', 'multiInstance': False,
                'description': 'The eef array contains measured DACH TCP poses and nominal gripper gaps in metres. '
                               'TCP frames are L_ee_link / R_ee_link in base_Link, orientation xyzw.',
                'inputSchema': {'type': 'object', 'properties': {
                    'action': {'type': 'string', 'enum': ['info', 'start', 'stop']}},
                    'required': ['action']},
                'topic_out': [{'topic': STATE_TOPIC, 'format': 'state/eef'}]}

    def _connected(self):
        socket = self.reader.ws
        if socket is not self._connection:
            self._connection = socket
            self._published.clear()
            self._joint_time = _Freshness()
            self._gripper_time = _Freshness()
            self._openings = ()
        profile = self.reader.configuration
        return (self.reader.connected and socket is not None and
                profile.get('mode') == 'upper_limb' and profile.get('state') == 'identified')

    def _joint_data(self):
        if not self._connected():
            raise ValueError('DACH upper-limb feedback is unavailable')
        sample = self.reader.sample('joint_state')
        data = sample.get('data') or {}
        if not sample.get('available') or not self._joint_time.observe(
                data.get('robot_timestamp'), sample.get('reception_age_seconds'),
                self.reader.last_round_trip, JOINT_MAX_AGE):
            raise ValueError('Fresh DACH joint feedback is required')
        return data

    @staticmethod
    def _ordered(data, key, *, arms_only=False):
        names = data.get('names') or JOINT_NAMES
        required = ARM_NAMES if arms_only else JOINT_NAMES
        if not arms_only and len(names) != len(JOINT_NAMES):
            raise ValueError('ServoJ requires all 16 DACH joints')
        values = vector(data.get(key), len(names), key)
        if names[:len(required)] == required:
            return values[:len(required)]
        # The SDK reader has already validated the name/value pairing.
        by_name = dict(zip(names, values))
        try:
            return [by_name[name] for name in required]
        except KeyError as exc:
            raise ValueError(f'Missing DACH joint: {exc.args[0]}') from exc

    def joints(self):
        """Native SDK order, including the head, for the 16-axis ServoJ channel."""
        return self._ordered(self._joint_data(), 'q')

    def joint_snapshot(self):
        """Read one SDK frame as (vendor timestamp, q16, dq16)."""
        data = self._joint_data()
        return data['robot_timestamp'], self._ordered(data, 'q'), self._ordered(data, 'dq')

    def _measurement(self):
        q = self._ordered(self._joint_data(), 'q', arms_only=True)
        if not self._openings or not self._gripper_time.fresh(GRIPPER_MAX_AGE):
            raise ValueError('Fresh gripper openings are required')
        left, right = self._openings
        eef = (self.kinematics.forward('left', q[:7]) + [left]
               + self.kinematics.forward('right', q[7:14]) + [right])
        # Earliest local query start across the joint and gripper samples.
        return {'schema': SCHEMA, 'frame': FRAME,
                'stamp_ms': min(self._joint_time.wall_ms, self._gripper_time.wall_ms),
                'eef': eef}, q

    def packet(self):
        return self._measurement()[0]

    def values(self):
        return self.packet()['eef']

    def call(self, action):
        if action == 'start':
            self._publishing = True
        elif action == 'stop':
            self._publishing = False
        elif action != 'info':
            raise ValueError('Unsupported eef_state action')
        return self.info()

    def info(self):
        info = {'read_only': True, 'topic_out': self.tool()['topic_out']}
        if not self._publishing:
            return dict(info, state='idle', available=False)
        try:
            return dict(info, state='running', available=True, sample=self.packet())
        except ValueError as exc:
            return dict(info, state='running', available=False, reason=str(exc))

    def _needs_feedback(self):
        return self._publishing or 'servo_eef' in self.adapter.enabled

    async def _poll_grippers(self):
        while True:
            start = time.monotonic()
            if self._needs_feedback() and self._connected():
                session = self._connection
                try:
                    data = await self.reader.read_gripper()
                    if self._connected() and self._connection is session:
                        self._update_openings(data, time.monotonic() - start)
                except (ValueError, OSError, asyncio.TimeoutError):
                    self._openings = ()
            await asyncio.sleep(max(0., GRIPPER_PERIOD - (time.monotonic() - start)))

    def _update_openings(self, data, round_trip):
        if (not isinstance(data, dict) or data.get('result') != 'success'
                or not self._gripper_time.observe(data.get('timestamp'), 0., round_trip, GRIPPER_MAX_AGE)):
            raise ValueError('Fresh gripper openings are required')
        self._openings = tuple(percentage(data.get(side + '_opening'), side + '_opening')
                               * (MAX_OPENING_MM / 1000.) / MAX_OPENING for side in SIDES)

    def observation(self, stamp):
        """Return the published EEF vector and the arm encoders used for its FK."""
        if self._connection is not self.reader.ws:
            return None
        for packet, q in reversed(self._published):
            if packet['stamp_ms'] == stamp:
                return packet['eef'], q
        return None

    def _poll_joints(self):
        try:
            if not self._publishing:
                # The servo still needs to detect frozen vendor timestamps.
                self._joint_data()
                return
            packet, q = self._measurement()
        except ValueError:
            return  # Wait for the next complete measurement.
        # If two polls share a timestamp, keep the published observation unambiguous.
        if self._published and self._published[-1][0]['stamp_ms'] == packet['stamp_ms']:
            packet, _ = self._published[-1]
        else:
            self._published.append((packet, q))
        self.adapter.publish(STATE_TOPIC, packet)

    async def run(self):
        """Share feedback acquisition with the servo while gating the public topic."""
        while True:
            if not self._needs_feedback():
                await asyncio.sleep(POSE_JOINT_POLL_SECONDS)
                continue
            with self.reader.pose_feedback():
                poller = asyncio.create_task(self._poll_grippers())
                try:
                    while self._needs_feedback():
                        self._poll_joints()
                        await asyncio.sleep(POSE_JOINT_POLL_SECONDS)
                finally:
                    poller.cancel()
                    await asyncio.gather(poller, return_exceptions=True)
