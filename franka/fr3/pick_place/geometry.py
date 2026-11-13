"""Image-centred targets to FR3 base coordinates using the captured EE pose."""
import math
import numpy as np
from .alignment import decode_depth


def transform(value, name):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-4) or
            not math.isclose(np.linalg.det(matrix[:3, :3]), 1, abs_tol=1e-4)):
        raise ValueError(f'Invalid rigid transform: {name}')
    return matrix


def locate(observation, position, handeye):
    """Return a measured surface point, never a grasp pose or motion command."""
    meta = observation['metadata']
    calibration = meta['calibration']
    if (meta['arm_endpoint'] != handeye.get('robot_ip') or
            meta['input_identity']['camera_serial'] != handeye.get('camera_serial') or
            calibration['serial_number'] != handeye.get('camera_serial') or
            calibration['session_id'] != meta['input_identity']['session_id']):
        raise ValueError('Hand-eye profile, robot or camera snapshot identity mismatch')
    if meta.get('depth_aligned_to') != 'color' or meta.get('depth_encoding') != 'zlib/uint16-le':
        raise ValueError('Expected RGB-aligned uint16 millimetre depth')
    if (not isinstance(position, (list, tuple)) or len(position) != 2 or
            any(type(v) not in (int, float) or not math.isfinite(v) or abs(v)>1 for v in position)):
        raise ValueError('Target must be image-centred normalized XY in [-1,1]')
    intr = calibration['rgb_intrinsics']; width, height = meta['width'], meta['height']
    if (width != intr['width'] or height != intr['height'] or
            not 1 <= width <= 4096 or not 1 <= height <= 4096):
        raise ValueError('Snapshot and RGB intrinsics dimensions differ')
    x,y = [min(round((v+1)*n/2), n-1) for v,n in zip(position,(width,height))]
    depth = decode_depth(observation['depth_zlib'], width, height)
    patch = depth[max(0,y-2):y+3, max(0,x-2):x+3]
    valid = patch[patch>0]
    if valid.size < max(3, patch.size*.6) or np.ptp(valid)>30:
        raise ValueError(f'Target ({position[0]:.4f}, {position[1]:.4f}) at RGB pixel ({x}, {y}) has no reliable depth: '
                         f'{valid.size}/{patch.size} valid nearby pixels. Select the actual object in the observation image.')
    z = float(np.median(valid)) / 1000
    values = [intr[k] for k in ('fx','fy','ppx','ppy')]+intr['coeffs']
    if (not all(type(v) in (int,float) and math.isfinite(v) for v in values) or
            min(intr['fx'],intr['fy'])<=0 or len(intr['coeffs'])!=5):
        raise ValueError('Invalid RGB intrinsics')
    if any(intr['coeffs']):
        import pyrealsense2 as rs
        optics=rs.intrinsics()
        for key in ('width','height','fx','fy','ppx','ppy','coeffs'):setattr(optics,key,intr[key])
        optics.model=getattr(rs.distortion,intr['model'].rsplit('.',1)[-1])
        point=np.asarray(rs.rs2_deproject_pixel_to_point(optics,[x,y],z))
    else:
        point=np.array([(x-intr['ppx'])*z/intr['fx'],(y-intr['ppy'])*z/intr['fy'],z])
    ee=transform(np.asarray(meta['O_T_EE']).reshape(4,4,order='F'),'O_T_EE')
    extr=transform(handeye['EE_T_camera'],'EE_T_camera')
    base=(ee@extr@np.r_[point,1])[:3]
    return dict(observation_id=meta['observation_id'],pixel=[x,y],depth_mm=z*1000,
                camera_xyz_mm=(point*1000).tolist(),base_xyz_mm=(base*1000).tolist(),
                depth_spread_mm=int(np.ptp(valid)),handeye_accepted=handeye.get('accepted') is True,
                target_semantics='visible_surface_not_grasp_centre')
