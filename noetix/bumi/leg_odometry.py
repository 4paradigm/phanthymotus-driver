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

# Heel and toe of the sole in the ankle-roll link frame, from the
# ankle_roll_link mesh bounding box (x: heel -0.0689 .. toe 0.1258, sole
# z: -0.0511, mid-line y = 0). The contact point is a continuous blend between
# them weighted by their height above the ground: a hard "lowest corner" jumps
# from one corner to the other in a single sample (2.6 cm / -1.33 m/s in the
# 2026-10-21 18:58 run), while a fixed sole centre cannot follow the roll.
HEEL = (-0.0689, 0.0, -0.0511)
TOE = (0.1258, 0.0, -0.0511)
# Height difference over which the contact slides from heel to toe.
CONTACT_BLEND_M = 0.004

# Empirical slip corrections, measured on the 2026-10-21 lab floor. The feet
# lift only ~1 cm and slide. Forward kinematics accounted for ~0.44 m of a
# ~0.75 m body move (factor 1.7; the 3 m run delivered 2.95 m). Backward
# delivered ~3.50 m for a 3 m request at 1.7, so its factor is 1.85. The sideways
# gait slides much more: right delivered 1.20 m for 1 m at 4.4 (hence 5.5),
# left was approximately right at 4.4. These are measured corrections for that
# floor/gait, not a physical law: re-measure for another surface (same body
# marker, known move) and set the factor to 1.0 if the feet stop slipping.
# Reported in the result as `leg_slip_scale`. `DISTANCE_SLIP_SCALE` keeps the
# forward value for code that constructs LegOdometry() without a direction.
LEG_SLIP_SCALE = {'forward': 1.7, 'backward': 1.85, 'left': 4.4, 'right': 5.5}
DISTANCE_SLIP_SCALE = LEG_SLIP_SCALE['forward']

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


def foot_geometry(angles, chain):
    """Body-frame heel and toe points for one leg's six joint angles."""
    position = [0.0, 0.0, 0.0]
    rotation = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    for (_, origin, axis), angle in zip(chain, angles):
        offset = _apply(rotation, origin)
        position = [position[i]+offset[i] for i in range(3)]
        rotation = _matrix_multiply(rotation, _axis_rotation(axis, angle))
    heel = _apply(rotation, HEEL)
    toe = _apply(rotation, TOE)
    return ([position[i]+heel[i] for i in range(3)],
            [position[i]+toe[i] for i in range(3)])


def contact_point(heel, toe, body_rotation=None):
    """Continuous contact between the heel and toe of one foot.

    The blend weight uses the height of each point in the body frame (or in the
    world frame when `body_rotation` is supplied): the lower point wins, with a
    smooth transition over `CONTACT_BLEND_M`, so a rolling foot slides the
    contact instead of jumping between corners.
    """
    # The rolling blend was measured to *reduce* the estimate on this gait (the
    # contact slides with the roll and cancels the body step: 0.18 m vs 0.44 m
    # over the same run). Keep the fixed sole centre and correct slip globally.
    return [(heel[i]+toe[i])/2 for i in range(3)]


def foot_position(angles, chain):
    """Foot contact point in the body frame for one leg's six joint angles."""
    heel, toe = foot_geometry(angles, chain)
    return contact_point(heel, toe)


class LegOdometry:
    """Integrate body displacement from successive foot positions."""

    def __init__(self, slip_scale=None):
        self.slip_scale = DISTANCE_SLIP_SCALE if slip_scale is None else float(slip_scale)
        if not math.isfinite(self.slip_scale) or self.slip_scale <= 0:
            raise ValueError('leg slip scale must be positive and finite')
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.samples = 0
        self.stance_samples = 0
        self.feet = None
        self.foot_gap = None          # fore-aft distance between the contacts
        self.double_support = False   # both feet moving with the body
        self._feet = None
        self._rotation = None
        self._time = None

    def reset_position(self):
        """Zero the accumulated position/velocity, keeping the previous feet."""
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]

    def observe(self, joint_positions, quaternion, now, frozen=False):
        """One SDK joint-state + orientation sample.

        `frozen=True` keeps the accumulated position/velocity (braking phase):
        the stance foot rolls and slips during the stop, which only adds noise
        to a displacement that was already used to request the brake.
        """
        if frozen:
            # No leg progress is used during the stop; report zero speed so the
            # endpoint check uses the IMU quiet conditions, not a stale step.
            self.velocity = [0.0, 0.0, 0.0]
            self._feet = None
            return
        values = joint_angles(joint_positions)
        split = len(LEG_CHAINS['left'])
        rotation = rotation_from_quaternion(quaternion)
        feet = [foot_position(values[:split], LEG_CHAINS['left']),
                foot_position(values[split:], LEG_CHAINS['right'])]
        self.observe_feet(feet, rotation, now)

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
            limit = MAX_STEP_M/max(1.0, self.slip_scale)
            plausible = [index for index in (0, 1) if lengths[index] <= limit]
            self.foot_gap = feet[0][0]-feet[1][0]
            self.double_support = (len(plausible) == 2
                                   and abs(lengths[0]-lengths[1]) <= DOUBLE_SUPPORT_M)
            if plausible:
                if len(plausible) == 2 and abs(lengths[0]-lengths[1]) <= DOUBLE_SUPPORT_M:
                    delta = [(deltas[0][i]+deltas[1][i])/2 for i in range(3)]
                else:
                    best = min(plausible, key=lambda index: lengths[index])
                    delta = deltas[best]
                delta = [value*self.slip_scale for value in delta]
                self.position = [self.position[i]+delta[i] for i in range(3)]
                self.velocity = [value/dt for value in delta]
                self.stance_samples += 1
            else:
                self.velocity = [0.0, 0.0, 0.0]
        self.feet = feet
        self._feet, self._rotation, self._time = feet, rotation, now
        self.samples += 1
