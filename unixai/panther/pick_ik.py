"""Offline Pink planner. Run in a separate Python environment from ROS/OpenCV."""

import json
import math
import sys


def plan(request):
    import numpy as np
    import pinocchio as pin
    import pink
    from pink.barriers import PositionBarrier
    from pink.tasks import FrameTask, PostureTask

    cfg, names = request["config"], request["names"]
    full = pin.buildModelFromXML(request["urdf"])
    neutral = pin.neutral(full)
    unlocked = set()
    for name in names:
        if not full.existJointName(name) or full.joints[full.getJointId(name)].nq != 1:
            raise ValueError(f"expected scalar Panther joint: {name}")
        unlocked.add(full.getJointId(name))
    tcp, base = cfg["tcp_frame"], cfg["base_frame"]
    if not full.existFrame(tcp) or not full.existFrame(base):
        raise ValueError("TCP/base frame missing from live URDF")
    if not unlocked.issubset({i for i in range(1, full.njoints) if full.names[i] in request["measured"]}):
        raise ValueError("selected arm feedback is incomplete")
    for frame in (tcp, base):
        joint_id = full.frames[full.getFrameId(frame)].parentJoint
        ancestors = set()
        while joint_id:
            ancestors.add(joint_id)
            if full.names[joint_id] not in request["measured"]:
                raise ValueError(f"missing ancestor joint feedback: {full.names[joint_id]}")
            if frame == base and joint_id in unlocked:
                raise ValueError("base frame cannot be downstream of the moving arm")
            joint_id = full.parents[joint_id]
        if frame == tcp and not unlocked.issubset(ancestors):
            raise ValueError("TCP must be downstream of every joint of the selected arm")
    for joint_id in range(1, full.njoints):
        joint = full.joints[joint_id]
        value = request["measured"].get(full.names[joint_id])
        if value is not None:
            if not math.isfinite(value):
                raise ValueError("non-finite joint feedback")
            if joint.nq == 1:
                neutral[joint.idx_q] = value
            elif joint.nq == 2 and joint.nv == 1:
                neutral[joint.idx_q:joint.idx_q + 2] = [math.cos(value), math.sin(value)]
            else:
                raise ValueError("unsupported measured joint representation")
    model = pin.buildReducedModel(full, [i for i in range(1, full.njoints) if i not in unlocked], neutral)
    if model.nq != len(names) or model.nv != len(names):
        raise ValueError("unexpected reduced model size")
    order = [model.joints[model.getJointId(name)].idx_q for name in names]
    current = np.empty(model.nq)
    current[order] = [request["measured"][name] for name in names]
    for name, index in zip(names, order):
        low, high = request["limits"][name]
        # Relax lower limit by 1e-3 rad to account for floating-point precision
        # in joint position readings (e.g., encoder zero offset)
        model.lowerPositionLimit[index] = min(low - 1e-3, model.lowerPositionLimit[index])
        model.upperPositionLimit[index] = min(high, model.upperPositionLimit[index])
    data = model.createData()
    state = pink.Configuration(model, data, current)
    world_from_base = state.get_transform_frame_to_world(base).copy()
    base_from_world = world_from_base.inverse()
    lower, upper = np.array(cfg["workspace_min_m"]), np.array(cfg["workspace_max_m"])
    if not np.allclose(world_from_base.rotation, np.eye(3), atol=1e-6):
        raise ValueError("workspace axes must align with the fixed model root")
    barrier = PositionBarrier(tcp, p_min=lower + world_from_base.translation,
                              p_max=upper + world_from_base.translation)

    def pose(q):
        state.update(q)
        state.check_limits()
        if min(barrier.compute_barrier(state)) < -1e-8:
            raise ValueError("TCP outside commissioned workspace")
        return base_from_world * state.get_transform_frame_to_world(tcp)

    max_delta = math.radians(request["max_delta_deg"])

    def interpolate_pose(start, end, fraction):
        return start * pin.exp6(pin.log6(start.inverse() * end).vector * fraction)

    def check_segment(q0, q1, start=None, end=None):
        if max(abs(q1 - q0)) > max_delta:
            raise ValueError("planned joint segment exceeds arm_control step limit")
        for fraction in np.linspace(0., 1., 21):
            actual = pose(pin.interpolate(model, q0, q1, fraction))
            if start is not None:
                expected = interpolate_pose(start, end, fraction)
                error = pin.log6(expected.inverse() * actual).vector
                if np.linalg.norm(error[:3]) > .006 or np.linalg.norm(error[3:]) > .06:
                    raise ValueError("joint interpolation does not follow the Cartesian segment")

    initial = pose(current)
    observation = current.copy()
    observation[order] = np.deg2rad(cfg["observation_joints_deg"])

    def joint_waypoints(source, target):
        count = max(1, math.ceil(max(abs(target - source)) / (.95 * max_delta)))
        result = []
        previous = source
        for i in range(1, count + 1):
            point = pin.interpolate(model, source, target, i / count)
            check_segment(previous, point)
            result.append(np.rad2deg(point[order]).tolist())
            previous = point
        return result

    if request["action"] == "observe":
        return [["observation_pose", joint_waypoints(current, observation)]]

    # Preserve RealMan's tabletop recipe: horizontal transfer at observation
    # height and fixed vertical descents. No approach through shelf geometry.
    if initial.rotation[2, 2] > -math.cos(math.radians(5)):
        raise ValueError("calibrated TCP +Z must point down at the observation pose")
    a, b = [np.array(p, dtype=float) for p in request["targets"]]
    z = float(initial.translation[2])
    r = initial.rotation.copy()
    theta = -math.radians(request["rotation_deg"])
    c, s = math.cos(theta), math.sin(theta)
    rotated = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]) @ r
    above_a = pin.SE3(r, np.r_[a, z]); above_b = pin.SE3(rotated, np.r_[b, z])
    at_a = pin.SE3(r, np.r_[a, z-cfg["pick_descent_m"]])
    at_b = pin.SE3(rotated, np.r_[b, z-cfg["place_descent_m"]])
    sequence = [("approach", above_a), ("open", None), ("descend_pick", at_a),
                ("close", None), ("lift", above_a)]
    if theta:
        sequence.append(("rotate_gripper", pin.SE3(rotated, above_a.translation)))
    sequence += [("transfer", above_b), ("descend_place", at_b), ("release", None), ("retract", above_b)]
    task = FrameTask(tcp, 1.0, 1.0, lm_damping=.001)
    posture = PostureTask(cost=.0001)
    q = current.copy()
    start = initial
    output = []
    for label, end in sequence:
        if end is None:
            output.append([label, None])
            continue
        distance = np.linalg.norm(end.translation - start.translation)
        angle = np.linalg.norm(pin.log3(start.rotation.T @ end.rotation))
        count = max(1, math.ceil(distance / .04), math.ceil(angle / .15))
        waypoints = []
        previous = start
        for i in range(1, count + 1):
            goal = interpolate_pose(start, end, i / count)
            state.update(q)
            task.set_target(world_from_base * goal)
            posture.set_target(q)
            sample = q.copy()
            for _ in range(100):
                error = pin.log6(goal.inverse() * pose(sample)).vector
                if np.linalg.norm(error[:3]) <= .0015 and np.linalg.norm(error[3:]) <= .015:
                    break
                v = pink.solve_ik(state, [task, posture], .02, solver="daqp", barriers=[barrier])
                sample = pin.integrate(model, sample, v * .02)
            else:
                raise ValueError(f"Pink did not reach {label}")
            check_segment(q, sample, previous, goal)
            q, previous = sample, goal
            waypoints.append(np.rad2deg(q[order]).tolist())
        output.append([label, waypoints])
        start = end
    return output + [["return_observation", joint_waypoints(q, observation)]]


if __name__ == "__main__":
    try:
        print(json.dumps({"stages": plan(json.load(sys.stdin))}, allow_nan=False))
    except Exception as exc:
        print(json.dumps({"error": f"{type(exc).__name__}: {exc}"}))
