"""DACH vendor TCP poses and measured gripper openings on the Motus ROS bus."""
import asyncio
import math
import time

from gripper import MAX_OPENING, MAX_OPENING_MM, percentage
from vendor import POSE_JOINT_POLL_SECONDS

STATE_TOPIC = '/limx_tron2/state/eef'
CONTROL_TOPIC = '/limx_tron2/control/eef'
SCHEMA = 'motus.eef/1'
FRAME = 'base_Link'
POSE_MAX_AGE = .35
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

    def observe(self, stamp, round_trip, maximum):
        if not all(_number(x) and x >= 0 for x in (stamp, round_trip)):
            return False
        if self.robot_stamp is not None and stamp < self.robot_stamp:
            return False
        if self.robot_stamp != stamp:
            # Record late replies too. A faster retry can return the very same sample.
            self.robot_stamp = stamp
            self.requested = time.monotonic() - round_trip
            self.wall_ms = int((time.time() - round_trip) * 1000)
        return round_trip <= maximum and self.fresh(maximum)

    def fresh(self, maximum):
        return self.requested is not None and time.monotonic() - self.requested <= maximum


class EefState:
    def __init__(self, adapter):
        self.adapter = adapter
        self.reader = adapter.reader
        self._publishing = False
        self._connection = None
        self._pose_time = _Freshness()
        self._gripper_time = _Freshness()
        self._poses = self._openings = ()

    @staticmethod
    def tool():
        return {'name': 'eef_state', 'type': 'sensor', 'multiInstance': False,
                'description': 'The eef array contains measured DACH TCP poses and nominal gripper gaps in metres. '
                               'Native MoveP end poses in base_Link, orientation xyzw.',
                'inputSchema': {'type': 'object', 'properties': {
                    'action': {'type': 'string', 'enum': ['info', 'start', 'stop']}},
                    'required': ['action']},
                'topic_out': [{'topic': STATE_TOPIC, 'format': 'state/eef'}]}

    def _connected(self):
        socket = self.reader.ws
        if socket is not self._connection:
            self._connection = socket
            self._pose_time = _Freshness()
            self._gripper_time = _Freshness()
            self._poses = self._openings = ()
        profile = self.reader.configuration
        return (self.reader.connected and socket is not None and
                profile.get('mode') == 'upper_limb' and profile.get('state') == 'identified')

    def packet(self):
        if not self._connected() or not self._poses or not self._pose_time.fresh(POSE_MAX_AGE):
            raise ValueError('Fresh DACH end poses are required')
        if not self._openings or not self._gripper_time.fresh(GRIPPER_MAX_AGE):
            raise ValueError('Fresh gripper openings are required')
        left, right = self._poses
        return {'schema': SCHEMA, 'frame': FRAME,
                'stamp_ms': min(self._pose_time.wall_ms, self._gripper_time.wall_ms),
                'eef': [*left, self._openings[0], *right, self._openings[1]]}

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
                or not self._gripper_time.observe(data.get('timestamp'), round_trip, GRIPPER_MAX_AGE)):
            raise ValueError('Fresh gripper openings are required')
        self._openings = tuple(percentage(data.get(side + '_opening'), side + '_opening')
                               * (MAX_OPENING_MM / 1000.) / MAX_OPENING for side in SIDES)

    def _update_poses(self, data, round_trip):
        if (not isinstance(data, dict) or data.get('result') != 'success'
                or not self._pose_time.observe(data.get('timestamp'), round_trip, POSE_MAX_AGE)):
            raise ValueError('Fresh DACH end poses are required')
        poses = []
        for side in SIDES:
            xyz, quat = data.get(side + '_position'), data.get(side + '_quat')
            if (not isinstance(xyz, list) or len(xyz) != 3 or not all(map(_number, xyz))
                    or not isinstance(quat, list) or len(quat) != 4 or not all(map(_number, quat))
                    or abs(math.hypot(*quat) - 1.) > .001):
                raise ValueError('Invalid native MoveP pose for ' + side)
            poses.append([*xyz, *quat[1:], quat[0]])
        self._poses = poses

    def _publish(self):
        if not self._publishing:
            return
        try:
            packet = self.packet()
        except ValueError:
            return
        self.adapter.publish(STATE_TOPIC, packet)

    async def run(self):
        while True:
            if not self._needs_feedback():
                await asyncio.sleep(POSE_JOINT_POLL_SECONDS)
                continue
            # This lease also keeps the single shared WS receive pump responsive to EEF replies.
            with self.reader.pose_feedback():
                poller = asyncio.create_task(self._poll_grippers())
                try:
                    while self._needs_feedback():
                        start = time.monotonic()
                        if self._connected():
                            session = self._connection
                            try:
                                data = await self.reader.read_move_pose()
                                if self._connected() and self._connection is session:
                                    self._update_poses(data, time.monotonic() - start)
                                    self._publish()
                            except (ValueError, OSError, asyncio.TimeoutError):
                                self._poses = ()
                        await asyncio.sleep(max(0., POSE_JOINT_POLL_SECONDS - (time.monotonic() - start)))
                finally:
                    poller.cancel()
                    await asyncio.gather(poller, return_exceptions=True)
