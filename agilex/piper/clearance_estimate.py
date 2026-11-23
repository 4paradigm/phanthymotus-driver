"""Plane-height estimate from a site's reduced URDF profile, without CAN access.

The profile is deployment data, not a physical calibration. No SciPy or mesh
loader is required at runtime; the source preview reduces meshes offline.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def load_profile(path):
    path = Path(path)
    if not path.is_absolute() or path.stat().st_size > 2 * 1024 * 1024:
        raise ValueError('模型 profile 必须为不超过 2 MiB 的本地绝对路径')
    raw = path.read_bytes()
    profile = json.loads(raw)
    if profile.get('schema') != 'piper.gripper_estimate/1':
        raise ValueError('unsupported_estimate_profile')
    if profile.get('root') != 'world' or not isinstance(profile.get('commit'), str):
        raise ValueError('invalid_estimate_provenance')
    def array(value, shape):
        result = np.array(value, dtype=float)
        if result.shape != shape or not np.isfinite(result).all():
            raise ValueError('invalid_estimate_geometry')
        return result
    joints = profile['joints']
    if not isinstance(joints, list) or not 1 <= len(joints) <= 64:
        raise ValueError('invalid_estimate_chain')
    seen, names = {'world'}, set()
    for joint in joints:
        if (joint['parent'] not in seen or joint['child'] in seen or joint['name'] in names
                or joint['kind'] not in {'fixed', 'revolute', 'prismatic'}):
            raise ValueError('invalid_estimate_joint')
        matrix = array(joint['origin'], (4, 4))
        if (not np.allclose(matrix[3], [0, 0, 0, 1])
                or not np.allclose(matrix[:3,:3].T @ matrix[:3,:3], np.eye(3))
                or not np.isclose(np.linalg.det(matrix[:3,:3]), 1)):
            raise ValueError('invalid_estimate_transform')
        if joint['kind'] != 'fixed':
            axis = array(joint['axis'], (3,))
            if not np.isclose(np.linalg.norm(axis), 1):
                raise ValueError('invalid_estimate_axis')
            mimic = joint.get('mimic')
            if mimic:
                if mimic['joint'] not in names or not all(math.isfinite(float(mimic[k])) for k in ['multiplier','offset']):
                    raise ValueError('invalid_estimate_mimic')
            elif joint['name'] not in {*(f'joint{i}' for i in range(1,7)), 'gripper'}:
                raise ValueError('unknown_estimate_input')
        seen.add(joint['child']); names.add(joint['name'])
    if not {*(f'joint{i}' for i in range(1,7)), 'gripper'} <= names:
        raise ValueError('incomplete_estimate_chain')
    vertices = profile['vertices_m']
    if set(vertices) != {'gripper_link1','gripper_link2','gripper_base','flange_link'}:
        raise ValueError('incomplete_estimate_coverage')
    for name, points in vertices.items():
        if name not in seen or not isinstance(points, list) or not 4 <= len(points) <= 4096:
            raise ValueError('invalid_estimate_vertices')
        array(points, (len(points),3))
    bounds = array(profile['joint_bounds_deg'], (6,2))
    opening = array(profile['opening_range_mm'], (2,))
    if not (bounds[:,0] < bounds[:,1]).all() or not 0 <= opening[0] < opening[1] <= 1000:
        raise ValueError('invalid_estimate_limits')
    profile['sha256'] = hashlib.sha256(raw).hexdigest()
    return profile


def estimate(snapshot, cfg, now, profile):
    bad = dict(state='unavailable', clearance_mm=None, lowest_point_base_mm=None,
        conservative_clearance_mm=None, estimated=True, calibration_verified=False)
    if snapshot.get('connected') is not True or snapshot.get('can_interface') != cfg['can_interface']:
        return dict(bad, reason='estimate_source_unavailable')
    grip = snapshot.get('gripper') or {}
    stamps = [snapshot.get('feedback_time'), grip.get('feedback_time')]
    if not all(type(t) in (int,float) and math.isfinite(t) and 0 <= now-t <= .5 for t in stamps):
        return dict(bad, reason='missing_or_stale_feedback')
    if abs(stamps[0]-stamps[1]) > .1:
        return dict(bad, reason='joint_gripper_time_skew')
    try:
        with np.errstate(over='raise',invalid='raise',divide='raise'):
            q = np.asarray(snapshot.get('joints_deg'),dtype=float)
            width = grip.get('width_mm')
            if (q.shape != (6,) or not np.isfinite(q).all() or type(width) not in (int,float)
                    or not math.isfinite(width)):
                return dict(bad,reason='invalid_feedback')
            if any(not lo<=v<=hi for v,(lo,hi) in zip(q,profile['joint_bounds_deg'])):
                return dict(bad,reason='joint_reference_range')
            if not profile['opening_range_mm'][0]<=width<=profile['opening_range_mm'][1]:
                return dict(bad,reason='opening_out_of_model_range')
            values={f'joint{i+1}':math.radians(float(v)) for i,v in enumerate(q)}
            values['gripper']=width/1000
            frames={'world':np.eye(4)}
            for joint in profile['joints']:
                move=np.eye(4)
                if joint['kind']!='fixed':
                    mimic=joint.get('mimic')
                    value=(values[mimic['joint']]*mimic['multiplier']+mimic['offset']) if mimic else values[joint['name']]
                    values[joint['name']]=value
                    axis=np.asarray(joint['axis'])
                    if joint['kind']=='prismatic': move[:3,3]=axis*value
                    else:
                        x,y,z=axis
                        cross=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
                        move[:3,:3]=np.eye(3)+math.sin(value)*cross+(1-math.cos(value))*(cross@cross)
                frames[joint['child']]=frames[joint['parent']] @ np.array(joint['origin']) @ move
            heights, lowest = {}, {}
            for name, vertices in profile['vertices_m'].items():
                frame=frames[name]
                points=np.array(vertices) @ frame[:3,:3].T+frame[:3,3]
                if not np.isfinite(points).all(): raise FloatingPointError()
                index=int(np.argmin(points[:,2]))
                lowest[name]=(points[index]*1000).tolist()
                heights[name]=float(points[index,2]*1000-cfg['table_z_mm'])
            closest=min(heights,key=heights.get)
            distance=heights[closest]
            if not all(math.isfinite(v) for v in heights.values()): raise FloatingPointError()
    except (FloatingPointError,OverflowError):
        return dict(bad,reason='nonfinite_geometry')
    return dict(state='estimated',reason='model_estimate_not_calibrated',clearance_mm=distance,
        conservative_clearance_mm=None,lowest_point_base_mm=lowest[closest],
        estimated=True,calibration_verified=False,closest_part=closest,part_heights_mm=heights,
        opening_mm=width,feedback_time=min(stamps),unit='mm',base_frame='model_world',
        model_commit=profile['commit'],model_sha256=profile['sha256'],
        model_source=profile.get('source'),table_z_mm=cfg['table_z_mm'],
        assumptions=['installed_tool_matches_profile','table_parallel_to_mounting_plane'],
        collision_protection_active=False,depth_used=False)
