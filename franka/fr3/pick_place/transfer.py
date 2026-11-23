"""Transfer inputs and targets for FR3 MoveIt planning."""
import math
import numpy as np

from .geometry import locate, transform
from .planning import rigid, rotation


def number(args, name, limit=None, default=None):
    value = args.get(name, default)
    if (type(value) not in (int,float) or not math.isfinite(value)
            or (limit is not None and abs(value)>limit)):
        raise ValueError(f'{name} must be a finite number'+(f' in [-{limit}, {limit}]' if limit is not None else ''))
    return float(value)


def parameters(action, args):
    start = [number(args,k,1) for k in ('start_point_x','start_point_y')]
    angle = number(args,'rotation_deg',180,0)
    if action == 'grab_by':
        delta = [number(args,k) for k in ('delta_x','delta_y')]
        if not any(delta) and not angle:
            raise ValueError('At least one of delta_x, delta_y and rotation_deg must be nonzero')
        return dict(position=start,rotation_deg=angle,displacement_mm=delta)
    return dict(position=start,rotation_deg=angle,
                target_position=[number(args,k,1) for k in ('target_point_x','target_point_y')])


def image_table_axes(camera, normal):
    right = camera[:3,0]-normal*(camera[:3,0]@normal)
    if np.linalg.norm(right)<1e-6:
        raise ValueError('Observation image right is perpendicular to the table')
    right /= np.linalg.norm(right)
    down = np.cross(normal,right)
    if down@camera[:3,1]<0: down=-down
    return np.column_stack((right, down))


def destination(observation, scene, handeye, options):
    normal = np.asarray(scene['table_normal_base'],dtype=float)
    table = np.asarray(scene['table_point_base_mm'])/1000
    source = np.asarray(scene['centre_base_mm'])/1000
    actual = np.asarray(observation['metadata']['O_T_EE']).reshape(4,4,order='F')
    camera = actual @ transform(handeye['EE_T_camera'],'EE_T_camera')
    if 'displacement_mm' in options:
        delta = image_table_axes(camera, normal) @ np.asarray(options['displacement_mm']) / 1000
    else:
        surface = locate(observation,options['target_position'],handeye)
        ray = np.asarray(surface['base_xyz_mm'])/1000-camera[:3,3]
        denominator = ray@normal
        if denominator>=-1e-6:
            raise ValueError('Placement ray does not point toward the table')
        target = camera[:3,3]+ray*((table-camera[:3,3])@normal/denominator)
        delta = target-(source-normal*((source-table)@normal))
    return delta


def moveit_targets(observation, pick, scene, handeye, options, *, delta=None):
    """Build SDK-frame targets for remote planning without solving local IK."""
    approach, grasp, lift = np.asarray(pick['poses'], dtype=float)
    normal = np.asarray(scene['table_normal_base'], dtype=float)
    if delta is None:
        delta = destination(observation, scene, handeye, options)
    initial = grasp[:3, :3]
    final = rotation(-normal, math.radians(options['rotation_deg'])) @ initial
    pad = pick['sdk_to_pad_offset_mm'] / 1000
    grasp_centre = np.asarray(pick.get('placement_reference', grasp[:3, 3])) + pad * initial[:, 2]
    raised = lift[:3, 3] + pad * initial[:, 2] + .05 * normal
    land = grasp_centre + delta - pad * final[:, 2]
    carry = [rigid(initial, raised - pad * initial[:, 2])]
    if options['rotation_deg']:
        carry.append(rigid(final, raised - pad * final[:, 2]))
    carry.extend((rigid(final, raised + delta - pad * final[:, 2]), rigid(final, land)))
    return dict(approach=approach, descent=grasp, lift=lift, carry=carry,
                place_lift=rigid(final, land + .15 * normal))
