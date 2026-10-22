"""Pinned vendor SDK execution and continuous endpoint observation."""
import json
import logging
import math
import threading
import time
from urllib.parse import urlparse

SDK_COMMIT = 'd38b6258f4c5a9607a231da1771e96cd1bdbccec'


class StableWindow:
    """Observe range over time; adjacent position differences are not velocity."""
    def __init__(self):
        self.stamp = None
        self.reset()

    def reset(self):
        self.since = self.last = None
        self.low = self.high = None

    def add(self, now, stamp, q, grip):
        stamp = stamp if isinstance(stamp, tuple) else (stamp,)
        if not all(type(v) in (int, float) and math.isfinite(v) and v >= 0 for v in stamp):
            raise ValueError('invalid_feedback_timestamp')
        if self.stamp is not None and any(a < b for a,b in zip(stamp,self.stamp)):
            raise ValueError('feedback_timestamp_regressed')
        if self.stamp is not None and any(a == b for a,b in zip(stamp,self.stamp)):
            if self.last is not None and now-self.last > .1:
                self.reset()
            return False
        self.stamp = stamp
        values = list(q)+list(grip)
        if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
            raise ValueError('invalid_feedback')
        limits = [math.radians(.2)]*len(q)+[1]*len(grip)
        if self.since is None or now-self.last > .1:
            self.since, self.low, self.high = now, values[:], values[:]
        else:
            self.low = [min(a,b) for a,b in zip(self.low,values)]
            self.high = [max(a,b) for a,b in zip(self.high,values)]
            if any(hi-lo > limit for lo,hi,limit in zip(self.low,self.high,limits)):
                self.since, self.low, self.high = now, values[:], values[:]
        self.last = now
        return now-self.since >= .5


class SDKReplay:
    def __init__(self, url, frames, head, initial, speed, force):
        address = urlparse(url)
        if address.scheme != 'ws' or not address.hostname or address.path not in ('', '/'):
            raise ValueError('unsupported_sdk_robot_url')
        self.host, self.port = address.hostname, address.port or 5000
        self.frames, self.head, self.initial = frames, head, initial
        self.speed, self.force = speed, force
        self.stop = threading.Event()
        self.reason = None
        self.closing = False
        self.phase = 'checking_start'
        self.transport = self.controller = None
        self.motion_sent = False
        self.sent = 0

    def fail(self, reason):
        if self.closing:
            return
        if self.reason is None:
            self.reason = str(reason)
        self.stop.set()

    def check(self):
        if self.stop.is_set():
            raise RuntimeError(self.reason or 'operator_stop')
        if self.controller:
            self.controller.raise_if_faulted()

    def create_controller(self):
        from tron2_env import Tron2Config
        from tron2_env.motion import MotionController
        from tron2_env.transport import WebsocketTransport
        worker = self

        class Transport(WebsocketTransport):
            # The pinned SDK retries connections and logs some send failures.
            # Latch those failures so a failed task cannot reconnect or resume.
            def _run_websocket_attempts(self, client, reset_connection_state, label):
                try:
                    client.run_forever()
                except Exception as exc:
                    worker.fail(exc)
                finally:
                    reset_connection_state()
                    if not worker.closing:
                        worker.fail('sdk_connection_lost')

            def _send_request(self, title, data=None, **kwargs):
                if worker.stop.is_set() or worker.closing:
                    return False
                if not title.startswith('request_get_'):
                    worker.motion_sent = True
                sent = super()._send_request(title, data, **kwargs)
                if not sent:
                    worker.fail('sdk_send_failed: '+title)
                return sent

            def _on_message(self, ws, message):
                # This SDK revision ignores negative motion acknowledgements.
                try:
                    packet = json.loads(message)
                    if packet.get('title') in ('response_movej', 'response_moveh',
                            'response_servoj', 'response_set_limx_2fclaw_cmd'):
                        data = packet.get('data', {})
                        if not isinstance(data, dict) or data.get('result') != 'success':
                            worker.fail('sdk_command_rejected: '+packet['title'])
                except (ValueError, AttributeError):
                    worker.fail('sdk_invalid_response')
                super()._on_message(ws, message)

            def _sleep_during_init(self, generation, duration):
                # Keep the SDK's initialization order and duration, but allow stop.
                end = time.monotonic()+duration
                while time.monotonic() < end:
                    worker.check()
                    self._require_init_session(generation)
                    worker.stop.wait(min(.02, max(0, end-time.monotonic())))
                worker.check()
                self._require_init_session(generation)

            def set_gripper(self, left_opening=0, right_opening=0, **kwargs):
                return super().set_gripper(left_opening, right_opening,
                    left_speed=worker.speed, right_speed=worker.speed,
                    left_force=worker.force, right_force=worker.force)

        config = Tron2Config(robot_ip=self.host, port=self.port,
                            init_joints=self.initial, init_head=self.head)
        self.transport = Transport.__new__(Transport)
        self.transport.__init__(config)
        self.check()
        self.controller = MotionController(self.transport, publish_rate=300,
                                          eta_default=.02, state_max_age=.5)
        self.controller.start()

    def wait_stable(self, target, grip):
        after = time.monotonic()
        window = StableWindow()
        previous = None
        while time.monotonic()-after < 8:
            self.check()
            # shortcut: pinned SDK public getters omit receipt time; use its
            # snapshots until the public API exposes the same freshness metadata.
            arm = self.transport._arm_head_snapshot.get(timeout=.1, max_age=.5)
            claw = self.transport._gripper_snapshot.get(timeout=.1, max_age=.5)
            now = time.monotonic()
            ids = (arm.received_at, claw.received_at)
            stamps = (arm.robot_timestamp, claw.robot_timestamp)
            q, g = arm.values.tolist(), (claw.values*100).tolist()
            if len(q) != 16 or len(g) != 2 or not all(math.isfinite(v) for v in q+g):
                raise ValueError('invalid_sdk_feedback')
            if previous and any(a < b for a,b in zip(stamps,previous)):
                raise ValueError('feedback_timestamp_regressed')
            fresh = min(ids) >= after and all(0 <= now-v <= .1 for v in ids)
            advancing = previous is None or all(a > b for a,b in zip(stamps,previous))
            error = max(abs(a-b) for a,b in zip(q,target))
            grip_error = max(abs(a-b) for a,b in zip(g,grip))
            if not fresh or error > math.radians(2) or grip_error > 3:
                window.reset()
            elif advancing and window.add(now, stamps, q, g):
                return dict(stable_seconds=now-window.since, arm_head_error_deg=math.degrees(error),
                    gripper_error=grip_error, feedback_age_seconds=max(now-v for v in ids),
                    joint_drift_deg=math.degrees(max(b-a for a,b in zip(window.low[:16],window.high[:16]))),
                    gripper_drift=max(b-a for a,b in zip(window.low[16:],window.high[16:])))
            if advancing:
                previous = stamps
            self.stop.wait(.01)
        raise ValueError('start_feedback_not_verified' if self.phase == 'verifying_start' else 'terminal_feedback_not_verified')

    def run(self):
        worker = self
        class Errors(logging.Handler):
            def emit(self, record):
                if not worker.closing and record.levelno >= logging.WARNING:
                    worker.fail('sdk_error: '+record.getMessage())
        logger = logging.getLogger('tron2_env')
        handler = Errors()
        logger.addHandler(handler)
        result = dict(status='error', sdk_commit=SDK_COMMIT, physical_stop_verified=False,
                      target_submission_complete=False, source='measured_state', head_policy='hold_live_start')
        try:
            self.check()
            self.phase = 'moving_to_start'
            self.create_controller()
            self.phase = 'verifying_start'
            first = self.frames[0]
            self.controller.set_gripper(*first['gripper'])
            result['start_check'] = self.wait_stable(first['q']+self.head, first['gripper'])
            self.phase = 'replaying'
            origin = time.monotonic()
            last_grip = first['gripper']
            for index, frame in enumerate(self.frames):
                self.stop.wait(max(0, origin+frame['t_ns']/1e9-time.monotonic()))
                self.check()
                if frame['gripper'] != last_grip:
                    self.controller.set_gripper(*frame['gripper'])
                    last_grip = frame['gripper']
                eta = (self.frames[index+1]['t_ns']-frame['t_ns'])/1e9 if index+1 < len(self.frames) else .02
                self.controller.command_joints(frame['q']+self.head, eta=eta)
                self.sent += 1
            result.update(target_submission_complete=True, submission_elapsed_seconds=time.monotonic()-origin)
            self.phase = 'verifying_end'
            result['end_check'] = self.wait_stable(self.frames[-1]['q']+self.head, last_grip)
            self.check()
            result.update(status='completed', physical_stop_verified=True)
        except Exception as exc:
            result.update(status='cancelled' if self.stop.is_set() and self.reason is None else 'error', reason=str(exc))
        finally:
            self.closing = True
            output_stopped = True
            if self.controller:
                try:
                    self.controller.pause(timeout=1)
                except Exception as exc:
                    result.update(status='error', reason='sdk_pause: '+str(exc))
            if self.controller or self.transport:
                try:
                    (self.controller or self.transport).disconnect()
                except Exception as exc:
                    output_stopped = False
                    result.update(status='error', reason='sdk_disconnect: '+str(exc))
            logger.removeHandler(handler)
            result.update(samples_sent=self.sent, motion_sent=self.motion_sent,
                output_stopped=output_stopped, phase=self.phase,
                motion_may_continue=(self.motion_sent and not result['physical_stop_verified']) or not output_stopped)
            if self.stop.is_set() and result['status'] == 'completed':
                result.update(status='error' if self.reason else 'cancelled', reason=self.reason or 'operator_stop')
        return result
