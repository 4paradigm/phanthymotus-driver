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


def check_table(ee, points, table, normal, stage):
    gap = table_clearance(ee, points, table, normal)
    if gap < .005:
        raise ValueError(f'Hand/finger table clearance {gap*1000:.2f} mm below 5 mm during {stage}; grasp target unchanged')
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

def observation_path(state, target_degrees, profile, lifts=(.10, .15, .20)):
    """Check opening at the current pose and the joint return before motion."""
    plane = profile.get('table_plane')
    if plane is None:
        if profile.get('tool_frame'):
            raise ValueError('Calibrated table plane is required before observing')
        return  # Legacy offline fixtures have no physical bench calibration.
    from .kinematics import Kinematics
    q = np.asarray(state['q'])
    ee = np.asarray(state['O_T_EE']).reshape(4, 4, order='F')
    model = Kinematics(q, ee)
    points = vertices(frame(profile, state.get('F_T_EE')))
    table = np.asarray(plane['point_base_mm'])/1000
    normal = np.asarray(plane['normal_base'])
    if (table.shape != (3,) or normal.shape != (3,) or not np.isfinite(table).all()
            or not np.isfinite(normal).all() or not np.isclose(np.linalg.norm(normal), 1, atol=.001)):
        raise ValueError('Invalid calibrated table plane')
    target = np.radians(target_degrees)
    from .trajectory import samples
    def validate(controls):
        # Validate the same continuous spline sent to the native executor.
        max_step = np.max(np.abs(np.diff(controls, axis=0)))
        subdivisions = max(40, int(np.ceil(max_step/math.radians(.1))))
        for value in samples(controls, subdivisions):
            if np.any(value < model.low) or np.any(value > model.high):
                raise ValueError('Observation path exceeds joint limits')
            check_table(model.pose(value), points, table, normal, 'opening/observation return')
            if model.table_clearance(value, table, normal) < .005:
                raise ValueError('Arm/table clearance below 5 mm during observation return')
            if model.clearance(value) <= .02:
                raise ValueError('Observation path fails arm self-collision clearance')
        return np.asarray(controls)
    if np.max(np.abs(target-q)) <= math.radians(.2):
        return validate(np.vstack((q, target)))
    # Open only if the complete jaw envelope is clear at the starting pose.
    check_table(ee, points, table, normal, 'opening')
    errors = []
    # Lift away from the table before turning the shoulder toward observation.
    for lift in lifts:
        try:
            controls = [q.copy()]; value = q.copy()
            for fraction in np.linspace(0, 1, 11)[1:]:
                goal = ee.copy(); goal[:3,3] += normal*lift*fraction
                value = model.solve(value, goal); controls.append(value.copy())
            controls.extend(np.linspace(value, target, 11)[1:])
            return validate(np.asarray(controls))
        except ValueError as exc:
            errors.append(str(exc))
    raise ValueError('No table-clear observation return: '+'; '.join(dict.fromkeys(errors)))


def check_observation_path(state, target_degrees, profile):
    return observation_path(state, target_degrees, profile)
