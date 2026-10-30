"""Standalone diagnostic, never imported by the driver. Default: observe only."""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

SERVICE = "/mc/uworld_sound_source"
STATE = "/mc/neck_trj_controller/state"
JOINTS = "/mc/joint_states"


def bounded(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name}: finite number required")
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name}: expected [{low}, {high}]")
    return float(value)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--send-once", action="store_true")
    p.add_argument("--contract", type=Path)
    p.add_argument("--exclusive-neck-confirmed", action="store_true")
    p.add_argument("--confirm-no-cancel", action="store_true")
    for name in ("degree", "max-vel", "max-acc", "prismatic-max-vel", "prismatic-max-acc"):
        p.add_argument("--" + name, type=float)
    p.add_argument("--seconds", type=float, default=12)
    return p


def plan(args):
    bounded(args.seconds, "seconds", 8, 60)
    values = (args.degree, args.max_vel, args.max_acc,
              args.prismatic_max_vel, args.prismatic_max_acc)
    if not args.send_once:
        if any(v is not None for v in values) or args.contract is not None:
            raise ValueError("motion arguments require --send-once; no request sent")
        return None
    if not args.exclusive_neck_confirmed or not args.confirm_no_cancel:
        raise ValueError("exclusive neck ownership and absence of verified cancellation must be acknowledged")
    if args.contract is None:
        raise ValueError("confirmed vendor semantics contract required; no trial values are inferred")
    c = json.loads(args.contract.read_text())
    for key in ("evidence", "reference_frame", "zero_definition", "positive_direction",
                "revolute_velocity_unit", "revolute_acceleration_unit",
                "prismatic_velocity_unit", "prismatic_acceleration_unit", "zero_zones_meaning"):
        if not isinstance(c.get(key), str) or not c[key].strip() or c[key].startswith("UNCONFIRMED"):
            raise ValueError(f"contract.{key} must cite confirmed semantics")
    if c.get("mode") not in ("absolute", "relative"):
        raise ValueError("contract.mode must be absolute or relative")
    if c.get("angle_unit") != "degree" or c.get("waist_disabled_supported") is not True:
        raise ValueError("degree unit and waist-disabled operation must be confirmed")
    degree = bounded(args.degree, "degree", -5, 5)
    if degree == 0:
        raise ValueError("zero is not a validated center command")
    # These are diagnostic input caps, NOT verified hardware safety limits.
    bounded(degree, "degree/vendor range", bounded(c.get("min_degree"), "min_degree", -180, 0),
            bounded(c.get("max_degree"), "max_degree", 0, 180))
    req = {"degree": degree, "with_waist": False}
    for group, vel, acc, vmax, amax in (
        ("revolute_setup", args.max_vel, args.max_acc, .1, .1),
        ("prismatic_setup", args.prismatic_max_vel, args.prismatic_max_acc, .01, .04),
    ):
        req[group] = {
            "max_vel": bounded(vel, group + ".max_vel", 1e-6,
                               min(vmax, bounded(c.get(group + "_max_vel"), "vendor velocity cap", 1e-6, vmax))),
            "max_acc": bounded(acc, group + ".max_acc", 1e-6,
                               min(amax, bounded(c.get(group + "_max_acc"), "vendor acceleration cap", 1e-6, amax))),
            "toy_zone": 0.0, "servo_zone": 0.0,
        }
    return req


def point(value):
    return {key: list(getattr(value, key, []))[:16]
            for key in ("positions", "velocities", "accelerations", "effort")}


def log_value(value, depth=0):
    """Bound untrusted DDS text/collections before JSON serialization."""
    if depth > 8:
        return "[depth limit]"
    if isinstance(value, str):
        return value[:256].encode("unicode_escape").decode("ascii")[:256]
    if isinstance(value, dict):
        return {str(k)[:64]: log_value(v, depth + 1) for k, v in list(value.items())[:32]}
    if isinstance(value, (list, tuple)):
        return [log_value(v, depth + 1) for v in value[:16]]
    return value


def state(message):
    return {"joint_names": list(message.joint_names)[:16],
            "desired": point(message.desired), "actual": point(message.actual),
            "error": point(message.error), "encoder_provenance": "unverified"}


def ready(latest, now):
    """Fresh, finite paired observations; this is not proof of motor ownership."""
    for topic in (STATE, JOINTS):
        if topic not in latest or now - latest[topic][0] > .5:
            return False
        data = latest[topic][1]
        if "neck_base_joint" not in data.get("joint_names", []):
            return False
        index = data["joint_names"].index("neck_base_joint")
        arrays = [data["actual"]["positions"], data["desired"]["positions"]] if topic == STATE else [data["positions"]]
        if any(len(a) <= index or not math.isfinite(a[index]) for a in arrays):
            return False
    return True


def run(args, request):
    # ROS is imported only after all input/contract gates pass. Offline tests
    # cannot discover services or accidentally construct motion requests.
    import rclpy
    from control_msgs.msg import JointTrajectoryControllerState
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.utilities import get_rmw_implementation_identifier

    rclpy.init(domain_id=2)
    node = rclpy.create_node("phanthy_head_diagnostic")
    latest = {}
    emitted = {}

    def emit(event, **data):
        print(json.dumps({"event": event, "wall_time": time.time(),
                          "monotonic": time.monotonic(), **log_value(data)}, allow_nan=False), flush=True)

    def receive(topic, data):
        now = time.monotonic()
        # Reject invalid values without crashing the observation session.
        try:
            json.dumps(data, allow_nan=False)
        except ValueError:
            latest.pop(topic, None)
            return
        latest[topic] = now, data
        if now - emitted.get(topic, 0) >= .2:
            emit("feedback", topic=topic, data=data)
            emitted[topic] = now

    client = None
    attempted = False
    try:
        emit("session", read_only=request is None, backend=SERVICE, domain=2,
             rmw=get_rmw_implementation_identifier(), physical_stop_supported=False,
             warning="Ctrl-C/timeout ends observation only; it does not cancel motion")
        node.create_subscription(JointTrajectoryControllerState, STATE,
                                 lambda m: receive(STATE, state(m)), qos_profile_sensor_data)
        node.create_subscription(JointState, JOINTS,
                                 lambda m: receive(JOINTS, {"joint_names": list(m.name)[:16],
                                                          "positions": list(m.position)[:16],
                                                          "velocities": list(m.velocity)[:16]}), qos_profile_sensor_data)
        node.create_subscription(JointTrajectory, "/mc/neck_trj_controller/joint_trajectory",
                                 lambda m: receive("trajectory", {
                                     "joint_names": list(m.joint_names)[:16],
                                     "points": [point(v) for v in m.points[:16]]}), qos_profile_sensor_data)
        start = time.monotonic()
        # Observe BEFORE any send. Discovery failure must not trigger retries.
        while time.monotonic() - start < 5:
            rclpy.spin_once(node, timeout_sec=.1)
        for topic in (STATE, JOINTS, "/mc/neck_trj_controller/joint_trajectory"):
            emit("publishers", topic=topic, endpoints=[
                {"name": x.node_name, "namespace": x.node_namespace,
                 "type": x.topic_type, "gid": list(x.endpoint_gid)}
                for x in node.get_publishers_info_by_topic(topic)])
        emit("services", names=[v for v in node.get_service_names_and_types()
                                if any(k in v[0] for k in ("neck", "uworld_motion", "sound_source"))])
        future = None
        if request is not None:
            if get_rmw_implementation_identifier() != "rmw_fastrtps_cpp":
                raise RuntimeError("motion requires vendor FastDDS runtime; no fallback")
            if not ready(latest, time.monotonic()):
                raise RuntimeError("fresh finite neck state AND joint state required; no request sent")
            from uworld_motion_msgs.srv import SoundSource
            client = node.create_client(SoundSource, SERVICE)
            if not client.wait_for_service(timeout_sec=2):
                raise RuntimeError("service unavailable; no request sent")
            if not ready(latest, time.monotonic()):
                raise RuntimeError("feedback stale before submission; no request sent")
            msg = SoundSource.Request()
            msg.degree, msg.with_waist = request["degree"], False
            for group in ("revolute_setup", "prismatic_setup"):
                for key, value in request[group].items():
                    setattr(getattr(msg, group), key, value)
            emit("request_attempt", service=SERVICE, request=request)
            attempted = True  # An exception can still leave a request in flight.
            future = client.call_async(msg)  # The ONLY motion submission site; never retried.
        replied = False
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            if future is not None and future.done() and not replied:
                replied = True
                reply = future.result()
                emit("receipt", success=reply.success, status=reply.status,
                     message=reply.message, arrival_confirmed=False)
        emit("end", request_attempted=future is not None, receipt_received=replied,
             arrival_confirmed=False, physical_stop_confirmed=False,
             feedback_fresh=ready(latest, time.monotonic()))
        return 2 if future is not None and not replied else 0
    except BaseException as exc:
        emit("error", error_type=type(exc).__name__, message=str(exc),
             request_attempted=attempted, arrival_confirmed=False,
             physical_stop_confirmed=False, outcome_unknown=attempted)
        raise
    finally:
        if client is not None:
            node.destroy_client(client)
        node.destroy_node()
        rclpy.shutdown()


def main():
    p = parser()
    args = p.parse_args()
    try:
        request = plan(args)
    except (ValueError, OSError, TypeError) as exc:
        p.error(str(exc))
    try:
        return run(args, request)
    except KeyboardInterrupt:
        print('Observation interrupted; physical stop NOT confirmed.', flush=True)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
