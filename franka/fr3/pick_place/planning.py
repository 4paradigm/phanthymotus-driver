"""Offline preview for a spherical target resting on a calibrated table."""
import itertools
import math

import numpy as np

from hardware import JOINT_LOWER, JOINT_UPPER
from .geometry import locate, transform
from .kinematics import Kinematics
from .trajectory import samples


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


def preview(observation, position, diameter_mm, profile, scene=None):
    if type(diameter_mm) not in (int, float) or not 0 < diameter_mm < 79.4:
        raise ValueError('Object diameter must be between 0 and 79.4 mm')
    surface = locate(observation, position, profile['handeye'])
    from .hand_collision import frame, vertices, check_path_table
    metadata = observation['metadata']
    hand_points = vertices(frame(profile, metadata.get('F_T_EE')))
    actual = transform(np.asarray(metadata['O_T_EE']).reshape(4, 4, order='F'), 'O_T_EE')
    camera = actual @ transform(profile['handeye']['EE_T_camera'], 'EE_T_camera')
    normal = np.asarray(profile['table_normal_base'], dtype=float)
    table = np.asarray(profile['table_point_base_mm'], dtype=float)/1000
    if normal.shape != (3,) or table.shape != (3,) or not np.isfinite(table).all() or not np.isclose(np.linalg.norm(normal), 1, atol=.001):
        raise ValueError('Invalid table plane')
    normal /= np.linalg.norm(normal)
    radius = diameter_mm/2000
    ray = np.asarray(surface['base_xyz_mm'])/1000-camera[:3, 3]
    if scene is not None:
        centre = np.asarray(scene['centre_base_mm'])/1000
    else:
        if ray@normal >= -1e-6:
            raise ValueError('Target ray does not point toward the table')
        depth_factor = (radius+(table-camera[:3, 3])@normal)/(ray@normal)
        if depth_factor < 1 or abs(depth_factor-1)*np.linalg.norm(ray) > radius*1.25:
            raise ValueError('Observed surface is inconsistent with the assumed diameter and table')
        centre = camera[:3, 3]+depth_factor*ray
    q0 = np.radians(metadata['joint_degree'])
    low, high = np.asarray(JOINT_LOWER)+.05, np.asarray(JOINT_UPPER)-.05
    if q0.shape != (7,) or not np.isfinite(q0).all() or not np.all((low <= q0)&(q0 <= high)):
        raise ValueError('Observation joints lack required limit margin')
    model = Kinematics(q0, actual)
    frames, offset = model.frames, model.offset
    vector, clearance = model.vector, model.clearance

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
    closing = initial_r[:,1] - (initial_r[:,1] @ down)*down
    if np.linalg.norm(closing) < 1e-6:
        closing = np.cross(down, initial_r[:,0])
    closing /= np.linalg.norm(closing)
    yaw_candidates = (0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180)
    if scene and scene.get('footprint_base_mm'):
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
    failures = []
    for yaw in yaw_candidates:
        goal_r = rotation(down, math.radians(yaw)) @ np.column_stack((np.cross(closing, down), closing, down))
        retreat = profile.get('shallow_retreat_mm', 0)
        if type(retreat) not in (int, float) or not 0 <= retreat < 150:
            raise ValueError('shallow_retreat_mm must be in [0, 150)')
        section_width = None
        if scene and scene.get('visible_surface_base_mm'):
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
                        section_width = width
                        break
                retreat += 10
            else:
                failures.append(f"{yaw:.1f}°: no usable grasp section")
                continue
        grasp = centre - goal_r[:,2]*retreat/1000
        grasp_width = diameter_mm
        if scene and scene.get('footprint_base_mm'):
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
        ends = [sdk_grasp-.15*goal_r[:,2], sdk_grasp, sdk_grasp+.01*normal]
        delta = goal_r @ initial_r.T
        turn = math.acos(np.clip((np.trace(delta)-1)/2,-1,1))
        def solve(q, point, iterations=120):
            goal = np.r_[point, .25*goal_r.ravel()]
            for _ in range(iterations):
                f = vector(q)
                error = goal-f
                if np.linalg.norm(error) < .00008:
                    return q
                jac = np.column_stack([(vector(q+d)-f)/1e-5 for d in np.eye(7)*1e-5])
                inverse = np.linalg.solve(jac@jac.T+1e-5*np.eye(len(error)), jac).T
                bias = ((low+high)/2-q)/(high-low)
                q = np.clip(q+np.clip(inverse@error+.02*(bias-inverse@(jac@bias)), -.03, .03), low, high)
            return None

        path = []
        rng = np.random.default_rng(42)
        seeds = [q0, (low+high)/2, *rng.uniform(low, high, size=(16,7))]
        # Rank redundant IK solutions by travel from the captured pose.
        # Shoulder movement is expensive; never pick a random branch merely
        # because it was the first feasible solver result.
        solutions = []
        for seed in seeds:
            q = solve(seed, ends[0], iterations=300)
            if q is not None:
                solutions.append(q)
        weights = np.array([4., 3., 3., 2., 1., 1., 1.])
        for q in sorted(solutions, key=lambda value: np.linalg.norm((value-q0)*weights)):
            candidate = np.linspace(q0, q, 101).tolist()
            for start, end in zip(ends,ends[1:]):
                for point in np.linspace(start,end,101)[1:]:
                    q = solve(q,point)
                    if q is None:
                        break
                    candidate.append(q.tolist())
                if q is None:
                    break
            if (len(candidate) == 301
                    and np.max(np.abs(np.diff(candidate, axis=0))) <= math.radians(2)):
                path = candidate
                break
        if not path:
            failures.append(f"{yaw:.1f}°: no overhead solution with a complete vertical descent and lift")
        if len(path) == 301:
            # The executor sends three continuous splines, not 300 stop/start moves.
            phases = [samples(np.asarray(path)[first:last+1:5])
                      for first,last in ((0,100),(100,200),(200,300))]
            checked = np.vstack(phases)
            tilt = max(math.degrees(math.acos(np.clip(
                (frames(q)[-1]@offset)[:3,2] @ down, -1, 1)))
                for q in np.vstack(phases[1:]))
            if tilt > 3:
                continue
            if scene and scene.get('visible_surface_base_mm'):
                observed = np.asarray(scene['visible_surface_base_mm'])/1000
                housing_low = np.array([-.03164,-.10399,-.12933])
                housing_high = np.array([.03164,.10043,-.03744])
                corners = np.array(list(itertools.product(*zip(housing_low,housing_high))))
                for pose_q in np.vstack(phases[:2]):
                    ee = frames(pose_q)[-1] @ offset
                    r, p = ee[:3,:3], ee[:3,3]
                    local = (observed-p) @ r
                    if (np.any(np.all((local >= housing_low) & (local <= housing_high),axis=1))
                            or np.min((corners@r.T+p-table)@normal) <= 0):
                        failures.append(f"{yaw:.1f}°: hand housing intersects the target or table")
                        break
                else:
                    pose_q = None
                if pose_q is not None:
                    continue
            try:
                for pose_q in checked:
                    check_path_table(model, pose_q, hand_points, table, normal, 'pick/descent/lift', profile)
            except ValueError as exc:
                failures.append(f"{yaw:.1f}°: {exc}")
                continue
            gap = min(clearance(q) for q in checked)
            if gap > .02:
                return dict(ok=True, observation_id=metadata['observation_id'],
                            grasp_centre_base_mm=(grasp*1000).tolist(),
                            sdk_grasp_base_mm=(sdk_grasp*1000).tolist(),
                            sdk_to_pad_offset_mm=pad_offset*1000, object_centre_base_mm=(centre*1000).tolist(),
                            depth_compensation_mm=compensation,
                            shallow_retreat_mm=retreat, grasp_mode="shallow" if retreat else "centre",
                            wrist_adjustment_deg=math.degrees(turn),
                            max_descent_tilt_deg=tilt,
                            table_yaw_deg=yaw, grasp_width_mm=grasp_width,
                            arm_capsule_clearance_mm=float(gap*1000), joint_path_rad=path,
                            joint_travel_deg=np.degrees(np.ptp(checked, axis=0)).tolist(),
                            checked_samples=len(checked), object_model=scene['model'] if scene else 'table_supported_sphere',
                            collision_scope=('sampled_arm_self_collision_fingers_table_and_visible_surface_housing'
                                             if scene and scene.get('visible_surface_base_mm')
                                             else 'sampled_arm_self_collision_and_fingers_table'),
                            trajectory_timed=False,
                            phase_end_indices=[100,200,300], motion_sent=False)
    raise ValueError('No complete vertical grasp trajectory: ' + ('; '.join(failures) or 'path checks failed'))
