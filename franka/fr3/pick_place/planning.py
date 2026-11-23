"""Generate wrist grasp targets for MoveIt."""
import math

import numpy as np

from hardware import JOINT_LOWER, JOINT_UPPER
from .geometry import transform


def rotation(axis, angle):
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross@cross)


def rigid(r, p):
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = r, p
    return result


def segment_distance(a, b, c, d):
    u, v = b-a, d-c
    matrix = np.column_stack((u, -v))
    s, t = np.linalg.lstsq(matrix, c-a, rcond=None)[0]
    distances = []
    if 0 <= s <= 1 and 0 <= t <= 1:
        distances.append(np.linalg.norm(a-c+s*u-t*v))
    for p, start, direction in ((a,c,v), (b,c,v), (c,a,u), (d,a,u)):
        f = np.clip((p-start)@direction/max(direction@direction, 1e-15), 0, 1)
        distances.append(np.linalg.norm(p-start-f*direction))
    return min(distances)


def grasp_targets(observation, scene, profile, *, extra_retreat_mm=0, only_yaw=None):
    diameter_mm = scene['diameter_mm']
    if type(diameter_mm) not in (int, float) or not 0 < diameter_mm < 79.4:
        raise ValueError('Object diameter must be between 0 and 79.4 mm')
    metadata = observation['metadata']
    actual = transform(np.asarray(metadata['O_T_EE']).reshape(4, 4, order='F'), 'O_T_EE')
    camera = actual @ transform(profile['handeye']['EE_T_camera'], 'EE_T_camera')
    normal = np.array(scene['table_normal_base'], dtype=float)
    if normal.shape != (3,) or not np.isclose(np.linalg.norm(normal), 1, atol=.001):
        raise ValueError('Invalid table plane')
    normal /= np.linalg.norm(normal)
    centre = np.asarray(scene['centre_base_mm']) / 1000
    q0 = np.radians(metadata['joint_degree'])
    low, high = np.asarray(JOINT_LOWER)+.05, np.asarray(JOINT_UPPER)-.05
    if q0.shape != (7,) or not np.isfinite(q0).all() or not np.all((low <= q0)&(q0 <= high)):
        raise ValueError('Observation joints lack required limit margin')

    initial_r = actual[:3,:3]
    down = -normal
    # Per-arm commissioning correction to the SDK-to-pad reference, along tool Z.
    compensation = profile.get('depth_compensation_mm', 0)
    if type(compensation) not in (int, float) or not math.isfinite(compensation) or not -50 <= compensation <= 50:
        raise ValueError('depth_compensation_mm must be in [-50, 50]')
    compensation = profile.get('grasp_depth_offset_mm', 0) + profile.get('z_compensation_mm', compensation)
    pad_offset = .020 - compensation/1000
    from .transfer import image_table_axes
    xy = np.array([profile.get(f'{axis}_compensation_mm', 0) for axis in 'xy'])
    shift = image_table_axes(camera, normal) @ xy / 1000 if np.any(xy) else np.zeros(3)
    offset = np.asarray(profile.get('grasp_offset_mm', [0, 0, 0]), dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all() or np.any(np.abs(offset) > 50):
        raise ValueError('grasp_offset_mm must contain three finite values in [-50, 50]')
    horizontal = image_table_axes(camera, normal) @ offset[:2] / 1000 if np.any(offset[:2]) else np.zeros(3)
    vertical = normal * offset[2] / 1000
    closing = initial_r[:,1] - (initial_r[:,1] @ down)*down
    if np.linalg.norm(closing) < 1e-6:
        closing = np.cross(down, initial_r[:,0])
    closing /= np.linalg.norm(closing)
    yaw_candidates = (0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180)
    if scene.get('footprint_base_mm'):
        # Fit a rectangle to the outline in the table plane. Close across
        # its short side first, then try the other pair of parallel faces.
        footprint = np.asarray(scene['footprint_base_mm'])
        edges = np.roll(footprint, -1, axis=0)-footprint
        edges -= (edges @ down)[:,None]*down
        edges = edges[np.linalg.norm(edges,axis=1)>1e-6]
        if len(edges):
            axes = edges/np.linalg.norm(edges,axis=1)[:,None]
            perpendicular = np.cross(down, axes)
            widths = np.ptp(footprint @ axes.T,axis=0)
            lengths = np.ptp(footprint @ perpendicular.T,axis=0)
            best = int(np.argmin(widths*lengths))
            short = axes[best] if widths[best] <= lengths[best] else perpendicular[best]
            yaw = math.degrees(math.atan2(down @ np.cross(closing,short), closing @ short))
            yaw_candidates = (yaw, yaw+180, yaw+90, yaw-90)
    failures, candidates = [], []
    for yaw in yaw_candidates if only_yaw is None else (only_yaw,):
        goal_r = rotation(down, math.radians(yaw)) @ np.column_stack((np.cross(closing, down), closing, down))
        retreat = profile.get('shallow_retreat_mm', 0)
        if type(retreat) not in (int, float) or not 0 <= retreat < 150:
            raise ValueError('shallow_retreat_mm must be in [0, 150)')
        section_width = None
        retreat_adjusted = False
        if scene.get('visible_surface_base_mm'):
            # Compare the hand housing sweep and pad section to observed points.
            local = (np.asarray(scene['visible_surface_base_mm'])/1000-centre) @ goal_r
            across = ((abs(local[:,0]) <= .03164)
                      & (local[:,1] >= -.10399) & (local[:,1] <= .10043))
            pad = abs(local[:,0]) <= .00875
            while retreat < 150:
                z = local[:,2] + retreat/1000
                housing_z = z + pad_offset
                blocked = np.any(across & (housing_z >= -.27933) & (housing_z <= -.03744))
                section = local[pad & (abs(z-.00025) <= .00925),1]
                if not blocked and len(section) >= 2 and section.min() < 0 < section.max():
                    width = float(np.ptp(section)*1000)
                    if 0 < width <= 79:
                        if not extra_retreat_mm or retreat_adjusted:
                            section_width = width
                            break
                        retreat += extra_retreat_mm
                        retreat_adjusted = True
                        continue
                if retreat_adjusted:
                    break
                retreat += 10
            else:
                failures.append(f"{yaw:.1f}°: no usable grasp section")
                continue
        if extra_retreat_mm and section_width is None:
            continue
        grasp = centre - goal_r[:,2]*retreat/1000
        grasp_width = diameter_mm
        if scene.get('footprint_base_mm'):
            grasp_axis = goal_r[:,1]
            tangent = grasp_axis-normal*(grasp_axis @ normal)
            length = np.linalg.norm(tangent)
            if length < .5:
                continue
            footprint = np.asarray(scene['footprint_base_mm'])
            grasp_width = float(np.ptp(footprint @ (tangent/length))/length)
        if section_width is not None:
            grasp_width = section_width
        if not 0 < grasp_width <= 79:
            continue
        sdk_grasp = grasp + shift - pad_offset*goal_r[:,2]
        hover = sdk_grasp - (.15 - extra_retreat_mm/1000) * goal_r[:, 2]
        corrected_grasp = sdk_grasp + horizontal + vertical
        ends = [hover + horizontal + vertical, corrected_grasp, corrected_grasp + .01 * normal]
        candidates.append(dict(
            yaw_deg=yaw, observation_id=metadata['observation_id'],
            poses=[rigid(goal_r, point).tolist() for point in ends],
            grasp_width_mm=grasp_width,
            placement_reference=(sdk_grasp + vertical).tolist(),
            sdk_to_pad_offset_mm=pad_offset*1000,
        ))
    if not candidates:
        raise ValueError('No usable vertical grasp target: ' + ('; '.join(failures) or 'invalid grasp width'))
    return candidates
