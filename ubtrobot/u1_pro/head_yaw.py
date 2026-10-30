"""Read-only horizontal neck feedback; no clients, publishers or motion calls."""
import math
import time


def wrap_degrees(angle):
    result = (angle + 180) % 360 - 180
    return 180.0 if result == -180 else 0.0 if result == 0 else result


class HeadYaw:
    def __init__(self, config):
        self.enabled = config.get('enabled', False)
        self.sign = config.get('yaw_sign', 1)
        self.offset = config.get('zero_offset_deg', 0.0)
        if type(self.enabled) is not bool or type(self.sign) not in (int, float) or self.sign not in (-1, 1):
            raise ValueError('frame_compensation requires boolean enabled and yaw_sign +/-1')
        if type(self.offset) not in (int, float) or not math.isfinite(self.offset) or abs(self.offset) > 180:
            raise ValueError('zero_offset_deg must be finite within +/-180')
        self.reset()

    def reset(self):
        self.latest = None
        self.reason = 'waiting_for_head_feedback'

    def update(self, message, received=None):
        now = time.monotonic() if received is None else received
        try:
            names = list(message.name)
            if names.count('neck_base_joint') != 1:
                raise ValueError('missing_or_duplicate_neck_base_joint')
            angle = float(message.position[names.index('neck_base_joint')])
            stamp = message.header.stamp
            stamp_s = int(stamp.sec) + int(stamp.nanosec) / 1e9
            if not math.isfinite(angle) or abs(angle) > math.pi:
                raise ValueError('invalid_head_angle')
            if stamp_s < 1e9 or not -.2 <= time.time() - stamp_s <= .5:
                raise ValueError('stale_head_header')
            self.latest = (self.sign * math.degrees(angle) + self.offset, now, round(stamp_s * 1000))
            self.reason = ''
        except (ValueError, TypeError, AttributeError, IndexError, OverflowError) as exc:
            self.latest = None
            self.reason = str(exc)[:128]

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        result = {'enabled': self.enabled, 'state': 'disabled' if not self.enabled else 'unavailable',
                  'yaw_sign': self.sign, 'zero_offset_deg': self.offset,
                  'basis': 'joint_feedback_at_doa_receipt_approximation',
                  'reason': self.reason}
        if not self.enabled:
            return result
        if self.latest is None:
            return result
        angle, received, stamp_ms = self.latest
        age = now - received
        result['age_ms'] = round(age * 1000)
        if not 0 <= age <= .3:
            result['reason'] = 'stale_head_feedback'
            return result
        result.update(state='estimated', head_yaw_deg=angle, timestamp_ms=stamp_ms, reason='')
        return result
