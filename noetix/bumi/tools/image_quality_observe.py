"""Observe only ImageQualityGuard String results; never subscribe to images.

Run on an explicitly selected ROS2 domain after the operator starts the Card.
Optional JSONL contains quality metrics only, with the original analysis time.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time


def parse_result(data, run_id=None):
    def reject_constant(value):
        raise ValueError("nonfinite JSON constant: " + value)

    value = json.loads(data, parse_constant=reject_constant)
    if not isinstance(value, dict) or value.get("schema") != "motus.image_quality/1":
        raise ValueError("unexpected quality result schema")
    if value.get("state") not in {"UNKNOWN", "HEALTHY", "DEGRADED"}:
        raise ValueError("invalid quality state")
    if not isinstance(value.get("run_id"), str) or not value["run_id"]:
        raise ValueError("quality result has no run_id")
    if run_id is not None and value["run_id"] != run_id:
        return None
    for name in ("reasons", "candidate_reasons"):
        if (not isinstance(value.get(name), list)
                or any(item not in {"LOW_LIGHT", "OVEREXPOSED", "BLURRED"}
                       for item in value[name])):
            raise ValueError("invalid quality reasons")
    metrics = value.get("metrics")
    if metrics is not None:
        expected = {"mean_luminance", "dark_pixel_ratio", "bright_pixel_ratio", "laplacian_variance"}
        if not isinstance(metrics, dict) or set(metrics) != expected:
            raise ValueError("invalid metric names")
        if any(type(item) not in (int, float) or not math.isfinite(item)
               for item in metrics.values()):
            raise ValueError("nonfinite quality metric")
    # Only this allowlist is ever recorded; never retain unknown payload fields.
    allowed = {"schema", "run_id", "timestamp_ms", "timestamp_kind", "input_topic",
               "state", "metrics", "reasons", "candidate_reasons", "error"}
    return {name: value.get(name) for name in allowed}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True, help="absolute quality output Topic")
    parser.add_argument("--run-id", help="record only this original Core ticket")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--output", type=Path, help="optional metrics-only JSONL file")
    args = parser.parse_args(argv)
    if not args.topic.startswith("/") or not args.topic.endswith("/image_quality_guard"):
        parser.error("topic must be the absolute ImageQualityGuard output")
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 3600:
        parser.error("seconds must be in (0, 3600]")
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("image_quality_guard_observer")
    output = args.output.open("a", encoding="utf-8") if args.output else None
    counts = {"accepted": 0, "invalid": 0}

    def receive(message):
        try:
            result = parse_result(message.data, args.run_id)
            if result is None:
                return
            encoded = json.dumps(result, ensure_ascii=False, allow_nan=False)
            counts["accepted"] += 1
            print(encoded, flush=True)
            if output:
                output.write(encoded + "\n")
                output.flush()
        except (ValueError, TypeError) as exc:
            counts["invalid"] += 1
            node.get_logger().warning("Rejected quality result: " + str(exc)[:200])

    try:
        node.create_subscription(String, args.topic, receive,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.VOLATILE))
        deadline = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.2, max(0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        pass
    finally:
        if output:
            output.close()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({"observer_summary": counts}), flush=True)
    return 0 if counts["accepted"] and not counts["invalid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
