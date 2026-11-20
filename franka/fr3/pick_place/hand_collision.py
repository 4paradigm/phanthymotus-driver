"""Hand V2 geometry in the measured SDK frame, independent of grasp corrections.

Finger boxes: franka_description 7aeeddc449edf8d62b594f9e36a81da53e7796f9,
end_effectors/common/franka_hand.xacro. Housing uses its collision-mesh AABB.
All parallel-jaw widths between 0 and 80 mm are covered by the endpoint hull.
"""
import itertools
import math
import numpy as np

SIGNS = np.array(list(itertools.product((-1., 1.), repeat=3)))
FINGER_BOXES = (
    ([0, .0185, .011], [0, 0, 0], [.022, .015, .020]),
    ([0, .0068, .0022], [0, 0, 0], [.022, .0088, .0038]),
    ([0, .0159, .02835], [math.pi/6, 0, 0], [.0175, .007, .0235]),
    ([0, .00758, .04525], [0, 0, 0], [.0175, .0152, .0185]),
)


def frame(profile, measured=None):
    from .geometry import transform
    from .planning import rigid, rotation
    calibrated = profile.get('tool_frame')
    flange_to_hand = rigid(rotation((0, 0, 1), -math.pi/4), [0, 0, 0])
    if calibrated:
        expected = transform(calibrated['F_T_EE'], 'tool_frame.F_T_EE')
        if measured is None:
            raise ValueError('Measured SDK F_T_EE is required for the calibrated hand model')
        actual = transform(np.asarray(measured).reshape(4, 4, order='F'), 'F_T_EE')
        if not np.allclose(actual, expected, atol=1e-5):
            raise ValueError('SDK tool frame changed; recheck the hand calibration')
    else:
        # Standard URDF TCP; deployed profiles use the explicit measured frame.
        actual = flange_to_hand.copy(); actual[2, 3] = .1034
    return np.linalg.solve(actual, flange_to_hand)


def vertices(ee_to_hand):
    from .planning import rigid, rotation
    chunks = []
    low, high = np.array([-.03164, -.10399, -.02593]), np.array([.03164, .10043, .06596])
    chunks.append((low+high)/2+SIGNS*(high-low)/2)
    for side in (0, 1):
        joint_r = rotation((0, 0, 1), side*math.pi)
        for width in (0., .08):
            joint = rigid(joint_r, [0, 0, .0584])
            joint[:3, 3] += joint_r @ np.array([0, width/2, 0])
            for index, (position, rpy, size) in enumerate(FINGER_BOXES):
                if side and index == 2: rpy = [-math.pi/6, 0, math.pi]
                x, y, z = rpy
                r = rotation((0, 0, 1), z) @ rotation((0, 1, 0), y) @ rotation((1, 0, 0), x)
                part = joint @ rigid(r, position)
                chunks.append((SIGNS*np.asarray(size)/2) @ part[:3, :3].T+part[:3, 3])
    points = np.vstack(chunks)
    return points @ ee_to_hand[:3, :3].T+ee_to_hand[:3, 3]


def table_clearance(ee, points, table, normal):
    return float(np.min((points @ ee[:3, :3].T+ee[:3, 3]-table) @ normal))


def check_table(ee, points, table, normal, stage, *, minimum_gap=.005):
    gap = table_clearance(ee, points, table, normal)
    if gap < minimum_gap:
        raise ValueError(f'Hand/finger table clearance {gap*1000:.2f} mm below {minimum_gap*1000:g} mm during {stage}; grasp target unchanged')
    return gap



def check_path_table(model, q, points, table, normal, stage, profile):
    """Check live depth and the independent commissioned bench plane."""
    planes = [(np.asarray(table), np.asarray(normal))]
    fixed = profile.get('table_plane')
    if fixed:
        planes.append((np.asarray(fixed['point_base_mm'])/1000, np.asarray(fixed['normal_base'])))
    for point, axis in planes:
        check_table(model.pose(q), points, point, axis, stage)
        if model.table_clearance(q, point, axis) < .005:
            raise ValueError(f'Arm/table clearance below 5 mm during {stage}')

def check_descent_curve(model, curve, points, planes, check, *, minimum_gap=.005):
    """Bound hand clearance and tilt between samples of the executed spline."""
    start, stop = curve.x
    velocity_bound = np.max(np.abs(curve.derivative().c[:, 0]), axis=0).sum()
    tilt_limit = math.radians(3)
    minimum = math.inf
    intervals = [(start, stop, 0)]
    while intervals:
        check()
        left, right, depth = intervals.pop()
        middle = (left + right) / 2
        ee = model.pose(curve(middle))
        gaps = [check_table(ee, points, table, normal, 'descend', minimum_gap=minimum_gap)
                for table, normal in planes]
        tilt = max(math.acos(np.clip(ee[:3, 2] @ -normal, -1., 1.)) for _, normal in planes)
        if tilt > tilt_limit:
            raise ValueError(f'Hand tilt {math.degrees(tilt):.2f} deg exceeds 3 deg during descend')
        joint_travel = velocity_bound * (right - left) / 2
        # Two metres bounds the distance from every FR3 joint axis to any
        # vertex in the Hand V2 model. Sum joint motion to bound tip travel.
        lower_bound = min(gaps) - 2 * joint_travel
        if lower_bound >= minimum_gap and tilt + joint_travel <= tilt_limit:
            minimum = min(minimum, lower_bound)
            continue
        if depth >= 12:
            raise ValueError(f'Cannot verify {minimum_gap*1000:g} mm hand clearance and 3 deg tilt throughout descent')
        intervals.extend(((middle, right, depth+1), (left, middle, depth+1)))
    return minimum
