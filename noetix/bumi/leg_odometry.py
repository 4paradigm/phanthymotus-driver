"""Body displacement from the two leg chains and the SDK joint positions.

No camera and no external calibration: the chain parameters come from the
vendor URDF `BUMI_V3.0_260119.urdf` (base_link is the body frame, x forward,
y left, z up; every joint origin has rpy = 0). The SDK exposes the 21 motor
positions; the stance foot is stationary in the world, so over one sample the
body moved by

    delta = R_prev * p_prev - R_cur * p_cur

where p is the foot position in the body frame and R maps the body frame to
the world using the SDK orientation quaternion. The foot whose implied delta is
smallest is the stance foot; when both are similar the body is in double
support and the deltas are averaged.

This is a kinematic estimate, not a measurement: foot roll/slip, the sole
contact point (modelled as the ankle-roll origin projected onto the sole at
z = -0.0511 m, from the ankle_roll_link mesh) and the unknown IMU-to-base_link
mounting rotation are approximations. `accuracy_validated` stays false.
"""
import math

# URDF chain per leg: (joint name, origin xyz in the parent link, joint axis).
LEG_CHAINS = {
    'left': (
        ('l_leg_pitch_joint', (0.0010184, 0.1, 0.03875), (0.0, 1.0, 0.0)),
        ('l_leg_roll_joint', (0.0, 0.0, -0.0665), (1.0, 0.0, 0.0)),
        ('l_leg_yaw_joint', (0.0, 0.0, -0.098), (0.0, 0.0, 1.0)),
        ('l_knee_pitch_joint', (0.0, 0.0, -0.101), (0.0, 1.0, 0.0)),
        ('l_ankle_pitch_joint', (0.00066801, -0.0005, -0.20237), (0.0, 1.0, 0.0)),
        ('l_ankle_roll_joint', (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
    ),
    'right': (
        ('r_leg_pitch_joint', (0.0010184, -0.1, 0.03875), (0.0, 1.0, 0.0)),
        ('r_leg_roll_joint', (0.0, 0.0, -0.0665), (1.0, 0.0, 0.0)),
        ('r_leg_yaw_joint', (0.0, 0.0, -0.098), (0.0, 0.0, 1.0)),
        ('r_knee_pitch_joint', (0.0, 0.0, -0.101), (0.0, 1.0, 0.0)),
        ('r_ankle_pitch_joint', (0.00066801, 0.0005, -0.20237), (0.0, 1.0, 0.0)),
        ('r_ankle_roll_joint', (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
    ),
}

# SDK motor order (device._BUMI_JOINT_NAMES / model-motion _JOINT_NAMES_BY_ID).
JOINT_ORDER = (
    'l_arm_pitch_joint', 'l_arm_roll_joint', 'l_arm_yaw_joint', 'l_elbow_pitch_joint',
    'l_leg_pitch_joint', 'l_leg_roll_joint', 'l_leg_yaw_joint',
    'l_knee_pitch_joint', 'l_ankle_pitch_joint', 'l_ankle_roll_joint',
    'r_arm_pitch_joint', 'r_arm_roll_joint', 'r_arm_yaw_joint', 'r_elbow_pitch_joint',
    'r_leg_pitch_joint', 'r_leg_roll_joint', 'r_leg_yaw_joint',
    'r_knee_pitch_joint', 'r_ankle_pitch_joint', 'r_ankle_roll_joint',
    'waist_yaw_joint',
)
JOINT_INDEX = {name: index for index, name in enumerate(JOINT_ORDER)}

# Sole centre in the ankle-roll link frame, from the ankle_roll_link mesh
# bounding box (x: heel -0.0689 .. toe 0.1258, y: +-0.0413, sole z: -0.0511).
# A fixed point in the foot frame is continuous. The previous "lowest sole
# corner" model followed the heel-to-toe roll but jumped between corners on a
# single sample, which showed up as a 2.6 cm step and -1.33 m/s at the first
# step of the 2026-10-21 18:58 run; that is a false fault, not motion.
SOLE_CENTRE = (0.02845, 0.0, -0.0511)

# The largest plausible body step per sample at ~50 Hz (1.0 m/s). A larger
# implied step is a contact/kinematics discontinuity, not body motion.
MAX_STEP_M = 0.02
# Two implied steps this close are treated as double support and averaged.
DOUBLE_SUPPORT_M = 0.01


def _axis_rotation(axis, angle):
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return [[t*x*x+c, t*x*y-s*z, t*x*z+s*y],
            [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
            [t*x*z-s*y, t*y*z+s*x, t*z*z+c]]


def _matrix_multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _apply(rotation, vector):
    return [sum(rotation[i][j]*vector[j] for j in range(3)) for i in range(3)]


def rotation_from_quaternion(quaternion):
    """Body-to-world rotation from the SDK xyzw quaternion."""
    x, y, z, w = (float(v) for v in quaternion)
    norm = math.hypot(x, y, z, w)
    if not math.isfinite(norm) or norm < 0.8:
        raise ValueError('leg odometry requires a finite SDK quaternion')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def joint_angles(joint_positions):
    """The 12 leg joint positions from a 21-motor SDK joint-state sequence.

    Accepts either raw numbers or SDK `MotorState` objects (which expose
    `.pos`), so the control loop can pass `get_joint_state()` directly.
    """
    try:
        values = []
        for leg in LEG_CHAINS.values():
            for name, _, _ in leg:
                motor = joint_positions[JOINT_INDEX[name]]
                values.append(float(getattr(motor, 'pos', motor)))
    except (TypeError, IndexError) as exc:
        raise ValueError('leg odometry requires all 21 SDK joint positions') from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError('leg odometry joint positions must be finite')
    return values


def foot_position(angles, chain):
    """Foot contact point in the body frame for one leg's six joint angles.

    The contact is the sole centre, a fixed point in the foot frame: continuous
    in the joint angles, which the stance-foot integration relies on. The real
    centre of pressure moves as the foot rolls; that residual is one of the
    approximations this estimate carries.
    """
    position = [0.0, 0.0, 0.0]
    rotation = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    for (_, origin, axis), angle in zip(chain, angles):
        offset = _apply(rotation, origin)
        position = [position[i]+offset[i] for i in range(3)]
        rotation = _matrix_multiply(rotation, _axis_rotation(axis, angle))
    contact = _apply(rotation, SOLE_CENTRE)
    return [position[i]+contact[i] for i in range(3)]


class LegOdometry:
    """Integrate body displacement from successive foot positions."""

    def __init__(self):
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.samples = 0
        self.stance_samples = 0
        self.feet = None
        self._feet = None
        self._rotation = None
        self._time = None

    def reset_position(self):
        """Zero the accumulated position/velocity, keeping the previous feet."""
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]

    def observe(self, joint_positions, quaternion, now):
        """One SDK joint-state + orientation sample."""
        values = joint_angles(joint_positions)
        split = len(LEG_CHAINS['left'])
        feet = [foot_position(values[:split], LEG_CHAINS['left']),
                foot_position(values[split:], LEG_CHAINS['right'])]
        self.observe_feet(feet, rotation_from_quaternion(quaternion), now)

    def observe_feet(self, feet, rotation, now):
        """One already-computed (left, right) body-frame foot sample."""
        if len(feet) != 2:
            raise ValueError('leg odometry requires two feet')
        feet = [[float(v) for v in foot] for foot in feet]
        if not all(math.isfinite(v) for foot in feet for v in foot):
            raise ValueError('leg odometry foot positions must be finite')
        rotation = [[float(v) for v in row] for row in rotation]
        if self._feet is not None:
            dt = now-self._time
            if not math.isfinite(dt) or not 0 < dt <= 0.1:
                raise ValueError('leg odometry sampling interval invalid or longer than 0.1 s')
            deltas = []
            for previous, current in zip(self._feet, feet):
                start = _apply(self._rotation, previous)
                end = _apply(rotation, current)
                deltas.append([start[i]-end[i] for i in range(3)])
            lengths = [math.hypot(*delta) for delta in deltas]
            plausible = [index for index in (0, 1) if lengths[index] <= MAX_STEP_M]
            if plausible:
                if len(plausible) == 2 and abs(lengths[0]-lengths[1]) <= DOUBLE_SUPPORT_M:
                    delta = [(deltas[0][i]+deltas[1][i])/2 for i in range(3)]
                else:
                    best = min(plausible, key=lambda index: lengths[index])
                    delta = deltas[best]
                self.position = [self.position[i]+delta[i] for i in range(3)]
                self.velocity = [value/dt for value in delta]
                self.stance_samples += 1
            else:
                self.velocity = [0.0, 0.0, 0.0]
        self.feet = feet
        self._feet, self._rotation, self._time = feet, rotation, now
        self.samples += 1
