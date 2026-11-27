"""Point admission and execution gating by the live G1 locomotion mode.

The historical class and callback names are retained for the installed point
backend. The callback's ``stationary`` field is an admission flag in that API;
it now reports a supported locomotion mode, not measured body stationarity.
Arm feedback, command limits and stop handling belong to the shared executor.
"""
import json
import math
import threading
import time


class DirectionalStationaryGuard:
    # Existing point YAML may contain stationary settings. Only the FSM feedback
    # timeout is used by this mode guard; the other names remain load-compatible.
    DEFAULTS = dict(hold_s=0., feedback_timeout_s=.25, lowstate_timeout_s=.1,
                    max_sample_gap_s=.25, max_speed_m_s=.02,
                    max_yaw_rate_rad_s=.03, max_drift_m=.02,
                    max_yaw_drift_rad=.03, max_tilt_deg=10.,
                    max_leg_speed_rad_s=.1, max_knee_flexion_rad=.65,
                    max_hip_pitch_rad=.65)

    def __init__(self, low_state, config=None, *, clock=time.monotonic,
                 wall_clock=time.time):
        self.clock, self.wall_clock = clock, wall_clock
        self.config = dict(self.DEFAULTS)
        for key, value in (config or {}).items():
            if (key not in self.DEFAULTS or isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < 0
                    or (key != 'hold_s' and value == 0)):
                raise ValueError('invalid_stationary_setting:' + str(key))
            if key != 'hold_s' and value > self.DEFAULTS[key]:
                raise ValueError('stationary_limit_too_permissive:' + key)
            self.config[key] = float(value)
        self._lock = threading.RLock()
        self._fsm = None
        self._fsm_stamp = None
        self._closed = False
        self._reason = 'fsm_missing'

    def update_fsm(self, data):
        with self._lock:
            if self._closed:
                return
            try:
                stamp = data['ts']
                if (type(data['fsm_id']) is not int or type(data['fsm_mode']) is not int
                        or type(stamp) not in (int, float) or not math.isfinite(stamp)):
                    raise ValueError('invalid_fsm')
                source_age = self.wall_clock() - stamp
                if not 0 <= source_age <= self.config['feedback_timeout_s']:
                    self._fsm, self._reason = None, 'fsm_stale'
                    return
                if self._fsm_stamp is not None and stamp <= self._fsm_stamp:
                    self._fsm, self._reason = None, 'fsm_time_not_advancing'
                    return
                self._fsm_stamp = stamp
                self._fsm = dict(fsm_id=data['fsm_id'], fsm_mode=data['fsm_mode'],
                                 source_stamp=stamp, received=self.clock())
                self._reason = 'fsm_missing'
            except (KeyError, TypeError, ValueError, OverflowError):
                self._fsm, self._reason = None, 'fsm_invalid'

    def update_odom(self, data):
        """Compatibility for existing callers; odometry does not gate pointing."""

    def check(self):
        with self._lock:
            reason = None
            received_age = source_age = None
            if self._closed:
                reason = 'closed'
            elif self._fsm is None:
                reason = self._reason
            else:
                received_age = self.clock() - self._fsm['received']
                source_age = self.wall_clock() - self._fsm['source_stamp']
                timeout = self.config['feedback_timeout_s']
                if not (0 <= received_age <= timeout and 0 <= source_age <= timeout):
                    reason = 'fsm_stale'
                elif self._fsm['fsm_id'] != 500 or self._fsm['fsm_mode'] not in (0, 1):
                    reason = 'fsm_not_operational'
            allowed = reason is None
            return dict(stationary=allowed, mode_allowed=allowed,
                        guard_kind='locomotion_mode', reason=reason or 'normal_loco_mode',
                        fsm_id=self._fsm['fsm_id'] if self._fsm else None,
                        fsm_mode=self._fsm['fsm_mode'] if self._fsm else None,
                        fsm_received_age_s=received_age, fsm_source_age_s=source_age,
                        time_basis='fsm_local_receive_and_publish_wall_time')

    def close(self):
        with self._lock:
            self._closed = True
            self._fsm = None
            self._reason = 'closed'


def create_stationary_node(namespace, guard):
    """Subscribe to the existing mode publisher for the point callback."""
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String
    node = Node('g1_directional_stationary')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)

    def receive(message):
        try:
            data = json.loads(message.data)
        except (ValueError, TypeError):
            data = {}
        guard.update_fsm(data if isinstance(data, dict) else {})

    try:
        node.create_subscription(String, f'/{namespace}/loco/motion_state', receive, qos)
    except Exception:
        guard.close()
        node.destroy_node()
        raise
    return node
