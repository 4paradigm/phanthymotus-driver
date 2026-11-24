"""Panther computation contracts, independent of ROS and hardware ownership.

Registered RGB-D ray rectification follows PR519's Tianyi make_geometry.
Physical calibration and capture headers always come from the current robot.
"""
from __future__ import annotations

import math
import numpy as np

CLOCK_ID = "ros_domain_0"


def mount_offset_matrix(config):
    """Explicit CAD-origin coincidence assumption or configured installation offset."""
    xyz = np.asarray(config.get("translation_m", [0., 0., 0.]), dtype=float)
    rpy = np.asarray(config.get("rpy_rad", [0., 0., 0.]), dtype=float)
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(np.r_[xyz, rpy]).all():
        raise ValueError("camera mount estimate needs finite xyz metres and rpy radians")
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                       [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    matrix[:3, 3] = xyz
    return matrix


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def make_geometry(color, depth, *, registered):
    if not registered or not color.header.frame_id or color.header.frame_id != depth.header.frame_id:
        raise ValueError('vendor-registered common optical reference is required')
    for info in (color, depth):
        k = np.asarray(info.k, float).reshape(3, 3)
        if (info.width <= 0 or info.height <= 0 or not np.isfinite(k).all() or
                k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]) or
                k[0, 1] != 0 or k[1, 0] != 0):
            raise ValueError('invalid source pinhole intrinsics')
    if np.any(np.asarray(depth.d) != 0):
        raise ValueError('registered depth must have rectified rays')
    if color.distortion_model not in ('plumb_bob', 'rational_polynomial') or len(color.d) not in (5, 8):
        raise ValueError('unsupported source color distortion')
    distortion = np.zeros(8)
    distortion[:len(color.d)] = color.d
    if not np.isfinite(distortion).all():
        raise ValueError('invalid color distortion')
    width, height = depth.width, depth.height
    crop_w, crop_h = min(width, height * 4 // 3), min(height, width * 3 // 4)
    left, top = (width - crop_w) // 2, (height - crop_h) // 2
    k = np.asarray(depth.k, float).reshape(3, 3).copy()
    sx, sy = 640 / crop_w, 480 / crop_h
    k[0, 0] *= sx
    k[1, 1] *= sy
    k[0, 2] = (k[0, 2] - left + .5) * sx - .5
    k[1, 2] = (k[1, 2] - top + .5) * sy - .5
    u, v = np.meshgrid(np.arange(640), np.arange(480))
    def project(matrix):
        x, y = (u - matrix[0, 2]) / matrix[0, 0], (v - matrix[1, 2]) / matrix[1, 1]
        r2 = x*x + y*y
        k1, k2, p1, p2, k3, k4, k5, k6 = distortion
        radial = (1 + k1*r2 + k2*r2**2 + k3*r2**3) / (1 + k4*r2 + k5*r2**2 + k6*r2**3)
        xd = x*radial + 2*p1*x*y + p2*(r2 + 2*x*x)
        yd = y*radial + p1*(r2 + 2*y*y) + 2*p2*x*y
        maps = dict(color_x=(color.k[0]*xd + color.k[2]).astype(np.float32),
                    color_y=(color.k[4]*yd + color.k[5]).astype(np.float32),
                    depth_x=(depth.k[0]*x + depth.k[2]).astype(np.float32),
                    depth_y=(depth.k[4]*y + depth.k[5]).astype(np.float32))
        valid = np.ones((480, 640), dtype=bool)
        for kind, camera in (('color', color), ('depth', depth)):
            mx, my = maps[kind+'_x'], maps[kind+'_y']
            valid &= (np.isfinite(mx) & np.isfinite(my) & (mx >= 0) & (mx <= camera.width-1)
                      & (my >= 0) & (my <= camera.height-1))
        return maps, valid

    maps, valid = project(k)
    if not valid.all():
        # Keep a centered 4:3 window fully inside both measured image fields.
        # A one-pixel margin protects interpolation between the sampled rays.
        radius = np.maximum(np.abs(u-320.)/320., np.abs(v-240.)/240.)
        extent = float(radius[~valid].min()) - 1./240.
        if extent <= 0:
            raise ValueError('RGB-D has no usable common central field of view')
        zoom = 1./extent
        k[:2] *= zoom
        k[0, 2] += 320.*(1.-zoom)
        k[1, 2] += 240.*(1.-zoom)
        maps, valid = project(k)
        if not valid.all():
            raise ValueError('RGB-D common field cannot be rectified without missing image rays')
    return dict(K=k, frame_id=depth.header.frame_id, **maps)


def metric_depth(message, unit_mm=1.0):
    """Decode stride/endian-aware depth to LE uint16 axial millimetres."""
    encoding = str(message.encoding)
    if encoding not in ("16UC1", "mono16", "32FC1"):
        raise ValueError(f"unsupported depth encoding: {encoding}")
    size = 4 if encoding == "32FC1" else 2
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width <= 0 or height <= 0 or step < width * size:
        raise ValueError("invalid depth layout")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < height * step:
        raise ValueError("truncated depth image")
    rows = raw[:height*step].reshape(height, step)[:, :width*size].copy()
    dtype = (">" if message.is_bigendian else "<") + ("f4" if size == 4 else "u2")
    values = rows.view(dtype).reshape(height, width).astype(np.float64)
    scale = 1000. if size == 4 else float(unit_mm)
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("source depth unit must be positive finite millimetres")
    values *= scale
    values[~np.isfinite(values) | (values < 0) | (values > 65535)] = 0.
    return np.rint(values).astype("<u2")



def transform_matrix(transform):
    """Column-vector target_from_source, in metres; reject invalid transforms."""
    t = transform.transform.translation
    q = transform.transform.rotation
    values = np.asarray([t.x, t.y, t.z, q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(values).all():
        raise ValueError("nonfinite TF")
    norm = np.linalg.norm(values[3:])
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError("zero TF quaternion")
    x, y, z, w = values[3:] / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                       [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                       [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = values[:3]
    return matrix


def imu_packet(message, matrix, camera_frame, epoch):
    stamp = stamp_ns(message)
    if stamp <= 0 or not message.header.frame_id or not camera_frame:
        raise ValueError("IMU measurement timestamp and frames required")
    for covariance in (message.angular_velocity_covariance, message.linear_acceleration_covariance):
        if len(covariance) and covariance[0] == -1:
            raise ValueError("IMU gyro/acceleration unavailable")
    gyro = message.angular_velocity
    accel = message.linear_acceleration
    vectors = [gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z]
    if not all(math.isfinite(v) for v in vectors):
        raise ValueError("nonfinite IMU vector")
    return dict(schema="motus.imu/1", source_id="/camera/gyro_accel/sample", epoch=epoch,
                stamp_ns=stamp, accel_stamp_ns=stamp, stamp_source="measurement",
                clock_id=CLOCK_ID, clock_mapping={"verified": False, "method": "source_ros_header"},
                camera_frame_id=camera_frame, gyro_frame_id=message.header.frame_id,
                accel_frame_id=message.header.frame_id,
                camera_from_gyro=matrix[:3, :3].tolist(), camera_from_accel=matrix[:3, :3].tolist(),
                camera_from_gyro_translation=matrix[:3, 3].tolist(),
                camera_from_accel_translation=matrix[:3, 3].tolist(),
                rotation_source="tf", translation_source="tf", calibration_id="source_tf",
                units={"angular_velocity": "rad/s", "linear_acceleration": "m/s^2"},
                angular_velocity=vectors[:3], linear_acceleration=vectors[3:])


def odom_pose(message, segment):
    """Keep the ROS pose's own precise measurement time; never fabricate pose."""
    stamp = stamp_ns(message)
    quality = dict(profile="motus_slam/1", status="unavailable", stamp_source="unknown",
                   segment_id=segment, clock_id=CLOCK_ID, pose_reference="body",
                   pose_source="ros_odometry", reason="invalid source pose/time/frames")
    p, q = message.pose.pose.position, message.pose.pose.orientation
    position = [p.x, p.y, p.z]
    quaternion = [q.x, q.y, q.z, q.w]
    values = position + quaternion
    norm = math.sqrt(sum(v*v for v in quaternion))
    if (stamp <= 0 or not message.header.frame_id or not message.child_frame_id
            or not all(math.isfinite(v) for v in values) or not math.isfinite(norm) or norm < 1e-6):
        return None, quality
    covariance = list(message.pose.covariance)
    if not all(math.isfinite(v) for v in covariance):
        covariance = None
    pose = dict(frame_id=message.header.frame_id, child_frame_id=message.child_frame_id,
                position=position, quaternion=[v/norm for v in quaternion],
                covariance=covariance, stamp_ms=stamp//1_000_000)
    quality.update(status="tracking", stamp_source="measurement", stamp_ns=stamp, reason="",
                   measurement={"schema": "motus.slam_frame/1", "stamp_ns": stamp,
                                "frame_id": message.header.frame_id, "clock_id": CLOCK_ID,
                                "clock_epoch": segment, "source_generation": segment,
                                "source_clock": "ros_header", "source_stamp": stamp, "source_unit": "ns",
                                "mapping": {
                       "method": "source_ros_header", "verified": False}})
    return pose, quality
