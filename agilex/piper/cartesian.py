"""Small base-frame translations using the installed SDK FK and guarded joint motion."""
import math
import time

import numpy as np


def rotation(pose):
    r, p, y = np.radians(pose[3:])
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def rotation_error(target, actual):
    relative = rotation(target) @ rotation(actual).T
    angle = math.acos(float(np.clip((np.trace(relative)-1)/2, -1, 1)))
    if angle > math.pi-.01:
        raise ValueError('姿态差接近 180°，请重新选择起始姿态')
    skew = np.array([relative[2,1]-relative[1,2], relative[0,2]-relative[2,0], relative[1,0]-relative[0,1]])
    return skew * (angle/(2*math.sin(angle)) if angle > 1e-7 else .5)


def pose_error(target, actual):
    return np.concatenate((np.array(target[:3])-actual[:3], 100*rotation_error(target, actual)))


def read_pose(driver):
    """Check all three pose frames, rather than trusting an aggregate timestamp."""
    driver.get_state()
    msg = driver.arm.GetArmEndPoseMsgs()
    stamp = msg.time_stamp
    if hasattr(driver.arm, 'frame_times'):
        with driver.arm.frame_lock:
            stamps = [driver.arm.frame_times.get(i, 0.) for i in (0x2A2, 0x2A3, 0x2A4)]
        stamp = min(stamps)
        if any(not math.isfinite(s) or abs(time.time()-s) > .5 for s in stamps):
            raise RuntimeError('末端位置的某个 CAN 帧超过 0.5 秒未更新')
    if not math.isfinite(stamp) or abs(time.time()-stamp) > .5:
        raise RuntimeError('末端反馈超过 0.5 秒未更新')
    values = [getattr(msg.end_pose, key)/1000 for key in
              ('X_axis', 'Y_axis', 'Z_axis', 'RX_axis', 'RY_axis', 'RZ_axis')]
    if not all(math.isfinite(v) for v in values):
        raise RuntimeError('末端反馈包含无效数值')
    return {'pose': values, 'frame': 'base', 'position_unit': 'mm', 'rotation_unit': 'deg',
            'feedback_time': stamp, 'fresh': True, 'reference': 'SDK end pose; no custom TCP offset'}


def fk_model(driver, joints, measured):
    from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics
    matches = []
    for offset in (0, 1):
        model = C_PiperForwardKinematics(dh_is_offset=offset)
        pose = model.CalFK(list(np.radians(joints)))[-1]
        if np.linalg.norm(np.array(pose[:3])-measured[:3]) <= 2 and np.linalg.norm(rotation_error(pose, measured)) <= math.radians(1):
            matches.append((model, offset))
    if len(matches) != 1:
        raise RuntimeError('SDK 两种 DH 模型与末端反馈无法唯一匹配；暂停运动并核对固件、坐标与工具偏移')
    return matches[0]


def solve_target(fk, start, target, bounds):
    """Local damped IK; refuse large joint changes or a rank-deficient pose."""
    q = np.array(start, dtype=float)
    lo = np.maximum(np.array(bounds)[:,0], q-12)
    hi = np.minimum(np.array(bounds)[:,1], q+12)
    if np.any(q < lo) or np.any(q > hi):
        raise ValueError('当前姿态超出有效关节限位')
    for _ in range(80):
        pose = fk(q)
        error = pose_error(target, pose)
        jac = np.column_stack([pose_error(fk(q+np.eye(6)[i]*.01), pose)/.01 for i in range(6)])
        singular = np.linalg.svd(jac, compute_uv=False)
        if singular[-1] < .01:
            raise ValueError('当前姿态接近运动学奇异点，请先安排调整起始姿态')
        if np.linalg.norm(error[:3]) <= .05 and np.linalg.norm(error[3:]) <= .05:
            return tuple(np.round(q, 3))
        step = np.linalg.solve(jac.T @ jac + np.eye(6)*.001, jac.T @ error)
        step *= min(1., 1./max(float(np.max(np.abs(step))), 1e-9))
        q = np.clip(q+step, lo, hi)
    raise ValueError('在当前限位及各轴 12° 范围内未找到保持姿态的空间位移解')


def check_path(model, joints, finish, start, target, bounds):
    fk = lambda q: model.CalFK(list(np.radians(q)))[-1]
    delta = np.array(finish)-joints
    deviation = 0.
    for u in np.linspace(0, 1, 41):
        q = np.array(joints)+u*delta
        pose = fk(q)
        expected = np.array(start[:3]) + u*(np.array(target[:3])-start[:3])
        deviation = max(deviation, float(np.linalg.norm(np.array(pose[:3])-expected)))
        if deviation > 2 or np.linalg.norm(rotation_error(target, pose)) > math.radians(1):
            raise ValueError('关节插补路径偏差超过 2 mm 或姿态变化超过 1°，请减小位移')
        if any(not low <= value <= high for value, (low, high) in zip(q, bounds)):
            raise ValueError('插补路径超出关节限位')
    return deviation


def plan(driver, state, axis, distance, rate, speed):
    from agilex.piper.control import ready_to_move
    ready_to_move(state)
    measured = read_pose(driver)['pose']
    model, offset = fk_model(driver, state.joints_deg, measured)
    fk = lambda q: model.CalFK(list(np.radians(q)))[-1]
    start = measured
    target = list(start)
    target['xyz'.index(axis)] += distance
    bounds = driver.joint_bounds_deg()
    finish = solve_target(fk, state.joints_deg, target, bounds)
    joint_plan = driver.joint_plan(finish, rate, speed)
    # The existing cubic interpolator follows this joint segment. Check its
    # Cartesian samples and disclose the path approximation in the preview.
    deviation = check_path(model, state.joints_deg, finish, start, target, bounds)
    return {**joint_plan, 'axis': axis, 'distance_mm': distance, 'frame': 'base',
            'start_pose': measured, 'target_pose': target, 'dh_is_offset': offset,
            'path_type': 'joint_interpolation', 'sampled_path_deviation_mm': deviation,
            'executed': False, 'collision_checked': False,
            'message': '保持末端姿态的小幅空间位移；关节插补，需确认整个机械臂、相机和夹爪活动空间'}, model


def execute(driver, preview, model):
    start, target = preview['start_pose'], preview['target_pose']
    state = driver.get_state()
    measured = read_pose(driver)['pose']
    _, offset = fk_model(driver, state.joints_deg, measured)
    if offset != preview['dh_is_offset']:
        raise RuntimeError('运动学模型发生变化，请重新预览')
    driver.joint_plan(preview['target_deg'], preview['speed_deg_s'], preview['speed_pct'])
    check_path(model, state.joints_deg, preview['target_deg'], start, target, driver.joint_bounds_deg())
    driver._check_cancelled()
    vector = np.array(target[:3])-start[:3]
    length2 = float(vector @ vector)
    def monitor(state):
        measured = read_pose(driver)['pose']
        predicted = model.CalFK(list(np.radians(state.joints_deg)))[-1]
        if np.linalg.norm(np.array(measured[:3])-predicted[:3]) > 3 or np.linalg.norm(rotation_error(predicted, measured)) > math.radians(2):
            raise RuntimeError('末端反馈与运动学模型不一致，已取消运动')
        u = float(np.clip((np.array(measured[:3])-start[:3]) @ vector/max(length2, 1e-9), 0, 1))
        nearest = np.array(start[:3])+u*vector
        if np.linalg.norm(np.array(measured[:3])-nearest) > 5 or np.linalg.norm(rotation_error(target, measured)) > math.radians(2):
            raise RuntimeError('末端偏离本次位移范围，已取消运动')
    state = driver._move_joint_target(preview['target_deg'], preview['speed_deg_s'], preview['speed_pct'], monitor=monitor)
    report = read_pose(driver)
    error_mm = float(np.linalg.norm(np.array(report['pose'][:3])-target[:3]))
    error_deg = math.degrees(float(np.linalg.norm(rotation_error(target, report['pose']))))
    if error_mm > 2 or error_deg > 1:
        raise RuntimeError('关节运动结束，但末端位置或姿态尚未达到 2 mm / 1° 容差；请读取反馈')
    return {'executed': True, 'confirmed': True, 'end_pose': report,
            'position_error_mm': error_mm, 'orientation_error_deg': error_deg,
            'joints_deg': state.joints_deg, 'motor_torque_released': False}
