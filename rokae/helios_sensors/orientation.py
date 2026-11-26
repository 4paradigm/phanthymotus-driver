"""Gravity-referenced orientation from the Gemini's accelerometer and gyro."""

import math


_GRAVITY = 9.80665
_MAX_ACCEL_ERROR = 0.12 * _GRAVITY
_INIT_MAX_RATE = 0.12
_INIT_TIME_S = 0.30
_MAX_GAP_S = 0.25
_TILT_GAIN = 1.5


def _tilt_quaternion(ax, ay, az):
    """IMU to gravity-up frame, with initial yaw set to zero."""
    roll = math.atan2(ay, az) / 2
    pitch = math.atan2(-ax, math.hypot(ay, az)) / 2
    return (math.sin(roll) * math.cos(pitch),
            math.cos(roll) * math.sin(pitch),
            -math.sin(roll) * math.sin(pitch),
            math.cos(roll) * math.cos(pitch))


def _rotate(orientation, angular_rate, dt):
    """Apply a body-frame angular increment to a quaternion in XYZW order."""
    x, y, z, w = orientation
    gx, gy, gz = angular_rate
    rate = math.hypot(gx, gy, gz)
    if rate == 0:
        return orientation
    half_angle = rate * dt / 2
    scale = math.sin(half_angle) / rate
    a, b, c = gx * scale, gy * scale, gz * scale
    d = math.cos(half_angle)
    rotated = (w*a + x*d + y*c - z*b,
               w*b - x*c + y*d + z*a,
               w*c + x*b - y*a + z*d,
               w*d - x*a - y*b - z*c)
    length = math.hypot(*rotated)
    return tuple(component / length for component in rotated)


class OrientationFilter:
    """Integrate gyro on device time and correct tilt using gravity."""

    def __init__(self):
        self.orientation = None
        self.last_stamp_us = None
        self.init_since_us = None

    def update(self, stamp_us, accel, gyro):
        ax, ay, az = accel
        magnitude = math.hypot(ax, ay, az)
        accel_error = abs(magnitude - _GRAVITY)
        dt = 0.0
        if self.last_stamp_us is not None:
            dt = (stamp_us - self.last_stamp_us) * 1e-6
            if dt <= 0:
                return self.orientation
            if dt > _MAX_GAP_S:
                self.orientation = self.init_since_us = None
        self.last_stamp_us = stamp_us

        if self.orientation is None:
            if accel_error >= _MAX_ACCEL_ERROR or math.hypot(*gyro) > _INIT_MAX_RATE:
                self.init_since_us = None
                return None
            if self.init_since_us is None:
                self.init_since_us = stamp_us
            if (stamp_us - self.init_since_us) * 1e-6 >= _INIT_TIME_S:
                self.orientation = _tilt_quaternion(ax, ay, az)
            return self.orientation

        gx, gy, gz = gyro
        if accel_error < _MAX_ACCEL_ERROR:
            x, y, z, w = self.orientation
            # The reference up axis expressed in the IMU frame.
            ux, uy, uz = 2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)
            gain = _TILT_GAIN * (1 - accel_error / _MAX_ACCEL_ERROR) / magnitude
            gx += gain * (ay*uz - az*uy)
            gy += gain * (az*ux - ax*uz)
            gz += gain * (ax*uy - ay*ux)
        self.orientation = _rotate(self.orientation, (gx, gy, gz), dt)
        return self.orientation
