"""A local fixed-point inspection workflow with integrated readiness checks."""
from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import tempfile
import time
import uuid

from patrol_baseline import PatrolBaselinePlugin, _id, _text


CARD = "patrol"


def number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return float(value)


class PatrolTelemetry:
    def __init__(self, *, clock=time.monotonic):
        self.clock = clock
        self._lock = threading.Lock()
        self._samples = {}

    def receive(self, channel, value):
        try:
            if isinstance(value, str):
                value = json.loads(value)
            if not isinstance(value, dict):
                raise ValueError("expected telemetry object")
            json.dumps(value, allow_nan=False)
        except (ValueError, TypeError):
            value = None
        with self._lock:
            self._samples[channel] = (self.clock(), deepcopy(value))

    def clear(self):
        with self._lock:
            self._samples.clear()

    def check(self, min_soc, max_age):
        with self._lock:
            samples = deepcopy(self._samples)
        reasons, review, observed = [], [], {}
        for channel in ("battery", "motion"):
            received, sample = samples.get(channel, (None, None))
            age = None if received is None else self.clock() - received
            if sample is None or age is None or not 0 <= age <= max_age:
                review.append(channel + "_missing_invalid_or_stale")
            else:
                observed[channel] = {"age_s": round(age, 3), "sample": sample}
        try:
            if "battery" in observed:
                battery = observed["battery"]["sample"]
                soc = number(battery.get("soc"), "battery.soc")
                alarm = number(battery.get("alarm"), "battery.alarm")
                if not 0 <= soc <= 100 or alarm < 0:
                    raise ValueError("invalid battery fields")
                if soc < min_soc:
                    reasons.append("low_battery")
                if alarm != 0:
                    reasons.append("battery_alarm")
            if "motion" in observed:
                motion = observed["motion"]["sample"]
                if motion.get("fresh") is not True:
                    review.append("motion_source_unhealthy")
                workmode = motion.get("workmode", {}).get("code")
                if type(workmode) is not int:
                    raise ValueError("missing workmode")
                if workmode != 2:
                    reasons.append("workmode_not_walking")
                joints = motion.get("joint_states")
                if not isinstance(joints, list) or len(joints) != 21:
                    raise ValueError("expected 21 joint states")
                ids = [joint.get("motor_id") for joint in joints]
                if any(type(index) is not int for index in ids) or set(ids) != set(range(21)):
                    raise ValueError("invalid joint mapping")
                for joint in joints:
                    error = joint.get("error")
                    if type(error) is not int or error < 0:
                        raise ValueError("missing raw joint error")
                    if error:
                        review.append("joint_raw_error_" + str(joint["motor_id"]) + "_" + str(error))
                    if abs(number(joint.get("velocity"), "joint.velocity")) >= 0.15:
                        reasons.append("robot_is_moving")
        except (ValueError, TypeError, AttributeError):
            review.append("telemetry_fields_invalid")
        return {"state": "blocked" if reasons else "needs_review" if review else "ready",
                "reasons": list(dict.fromkeys(reasons + review)), "observed": observed,
                "freshness_source": "local_ROS_receipt", "hardware_command": False}


class ROSPatrolInputs:
    def __init__(self, namespace, executor, telemetry):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        self.node = Node("bumi_patrol_inputs")
        self.executor = executor
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(String, f"/{namespace}/state/battery",
                                      lambda msg: telemetry.receive("battery", msg.data), qos)
        self.node.create_subscription(String, f"/{namespace}/motion/state",
                                      lambda msg: telemetry.receive("motion", msg.data), qos)
        try:
            executor.add_node(self.node)
        except Exception:
            self.node.destroy_node()
            raise

    def close(self):
        try:
            self.executor.remove_node(self.node)
        finally:
            self.node.destroy_node()


class PatrolPlugin:
    def __init__(self, plugin_config, namespace, executor, capture_plugin=None,
                 *, telemetry=None, input_factory=ROSPatrolInputs):
        self._baseline = PatrolBaselinePlugin(plugin_config, namespace, executor)
        self._namespace, self._executor = namespace, executor
        self._capture = capture_plugin
        self.telemetry = telemetry or PatrolTelemetry()
        self._input_factory = input_factory
        self._inputs = None
        self._min_soc = number(plugin_config.get("min_battery_percent", 20), "min_battery_percent")
        self._max_age = number(plugin_config.get("telemetry_max_age_s", 3), "telemetry_max_age_s")
        if not 0 <= self._min_soc <= 100 or not 0 < self._max_age <= 30:
            raise ValueError("invalid patrol readiness thresholds")
        self._report_dir = Path(plugin_config.get("report_dir", "data/patrol"))
        self._lock = threading.RLock()
        self._inspection_lock = threading.Lock()
        self._generation = 0
        self._run = None
        self._observations = {}
        self._records = {}
        self._finished = None

    def get_tool(self):
        tool = deepcopy(self._baseline.get_tool())
        tool.update(name=CARD, title="Bumi 固定点巡检", type="actuator",
                    description="同一张巡检卡完成电量、运动模式和关节检查，固定点拍照、"
                                "读取标准、记录 Agent 结论及保存本地报告。需操作员先放到固定点位。")
        schema = tool["inputSchema"]
        schema["properties"] = {key: value for key, value in schema["properties"].items()
                                if key in ("action", "point_id", "check_id", "run_id", "assessment", "priority")}
        actions = ["info", "start", "stop", "preflight", "begin", "plan", "read", "inspect", "record", "finish"]
        schema["properties"]["action"]["enum"] = actions
        schema["properties"].update({
            "inspection_id": {"type": "string"},
            "decision": {"type": "string", "enum": ["normal", "deviation", "needs_review"]},
        })
        schema["x-action-params"] = {
            "start": {"params": []}, "stop": {"params": []}, "info": {"params": []},
            "preflight": {"params": []},
            "begin": {"params": ["run_id"], "required": ["run_id"]},
            "plan": {"params": []},
            "read": {"params": ["point_id"], "required": ["point_id"]},
            "inspect": {"params": ["point_id"], "required": ["point_id"]},
            "record": {"params": ["inspection_id", "check_id", "decision", "assessment", "priority"],
                       "required": ["inspection_id", "check_id", "decision", "assessment"]},
            "finish": {"params": []},
        }
        return tool

    def start(self):
        with self._lock:
            if self._inputs is None:
                self._inputs = self._input_factory(self._namespace, self._executor, self.telemetry)
        return {"state": "ready", "hardware_command": False}

    def stop(self):
        with self._lock:
            self._generation += 1
            inputs, self._inputs = self._inputs, None
            self._run = None
            self._observations.clear()
            self._records.clear()
            self._finished = None
        if inputs is not None:
            inputs.close()
        self.telemetry.clear()
        return {"state": "idle", "hardware_command": False}

    def _preflight(self):
        result = self.telemetry.check(self._min_soc, self._max_age)
        try:
            info = self._capture.dispatch("info", {}) if self._capture is not None else {}
            age = info.get("latest_frame_age_s")
            camera_ready = info.get("ok") is True and 0 <= number(age, "camera age") <= 3
        except Exception:
            camera_ready = False
        if not camera_ready:
            result["reasons"].append("camera_unavailable_or_stale")
            if result["state"] == "ready":
                result["state"] = "needs_review"
        return result

    def dispatch(self, action, args):
        if not isinstance(args, dict):
            raise ValueError("patrol arguments must be an object")
        if "baseline_path" in args or "report_dir" in args:
            raise ValueError("paths are operator configuration")
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "preflight":
            return self._preflight()
        if action in ("plan", "read"):
            return self._baseline.dispatch(action, args)
        if action == "info":
            with self._lock:
                return {"state": "active" if self._run else "idle", "run_id": self._run,
                        "records": deepcopy(list(self._records.values())),
                        "preflight": self._preflight(), "hardware_command": False,
                        "topic_in": [], "topic_out": []}
        if action == "begin":
            run_id = _id(args.get("run_id"), "run_id")
            with self._lock:
                check = self._preflight()
                if check["state"] != "ready":
                    return check
                if self._run is not None and not self._finished:
                    return {"state": "active" if self._run == run_id else "busy", "run_id": self._run}
                if self._finished and self._run == run_id:
                    return deepcopy(self._finished)
                self._generation += 1
                self._run, self._finished = run_id, None
                self._records.clear()
                self._observations.clear()
                return {"state": "active", "run_id": run_id, "preflight": check,
                        "plan": self._baseline.dispatch("plan", {}), "hardware_command": False}
        if action == "inspect":
            if not self._inspection_lock.acquire(blocking=False):
                return {"state": "busy", "hardware_command": False}
            try:
                return self._inspect(args)
            finally:
                self._inspection_lock.release()
        if action == "record":
            return self._record(args)
        if action == "finish":
            return self._finish()
        return None

    def _inspect(self, args):
        with self._lock:
            if self._run is None or self._finished:
                return {"state": "error", "error": "begin an active patrol first"}
            point = self._baseline.dispatch("read", args)
            check = self._preflight()
            if check["state"] != "ready":
                return check
            generation = self._generation
        try:
            capture = self._capture.dispatch("capture_photo", {})
        except Exception as exc:
            capture = {"ok": False, "error": str(exc)}
        with self._lock:
            if generation != self._generation:
                return {"state": "cancelled", "hardware_command": False}
            if capture.get("ok") is not True:
                return {"state": "needs_review", "error": "capture_failed", "capture": capture}
            inspection_id = uuid.uuid4().hex
            record = {"inspection_id": inspection_id, "point": point, "capture": deepcopy(capture)}
            self._observations[inspection_id] = record
            return {"state": "awaiting_assessment", "inspection_id": inspection_id,
                    "point": point, "capture": capture, "hardware_command": False}

    def _record(self, args):
        with self._lock:
            if self._run is None or self._finished:
                return {"state": "error", "error": "begin an active patrol first"}
            observation = self._observations.get(args.get("inspection_id"))
            if observation is None:
                raise ValueError("unknown inspection_id for current patrol")
            point, capture = observation["point"], observation["capture"]
            check_id = args.get("check_id")
            checks = {item["check_id"]: item for item in point["checks"]}
            if check_id not in checks:
                raise ValueError("unknown check_id")
            decision = args.get("decision")
            if decision not in ("normal", "deviation", "needs_review"):
                raise ValueError("invalid inspection decision")
            assessment = _text(args.get("assessment"), "assessment", 480)
            evidence = {"point_id": point["point_id"], "view_id": point["view_id"], "kind": "visual",
                        "captured_at": capture.get("frame_captured_at"),
                        "reference": capture.get("file_path"), "observed": assessment}
            request = {"run_id": self._run, "revision": point["revision"],
                       "point_id": point["point_id"], "check_id": check_id,
                       "decision": "deviation" if decision == "normal" else decision,
                       "assessment": assessment, "priority": args.get("priority", "routine"),
                       "evidence": evidence}
            report = self._baseline.dispatch("prepare_report", request)
            readiness = self._preflight()
            if readiness["state"] != "ready":
                report = self._baseline.dispatch("prepare_report", {**request, "decision": "needs_review"})
                report["review_reason"] = "patrol_readiness_changed"
            if decision == "normal" and report["status"] == "deviation_report":
                report.update(status="normal", priority="routine", agent_decision="normal",
                              report_key=report["report_key"].rsplit("/", 1)[0] + "/normal")
            report.update(inspection_id=args["inspection_id"], preflight=readiness, agent_decision=decision,
                          timestamp_source=capture.get("timestamp_source", "unknown"))
            key = (point["point_id"], check_id)
            self._records[key] = report
            return deepcopy(report)

    def _finish(self):
        with self._lock:
            if self._run is None:
                return {"state": "error", "error": "no active patrol"}
            if self._finished is not None:
                return deepcopy(self._finished)
            plan = self._baseline.dispatch("plan", {})
            expected = []
            for point in plan["points"]:
                detail = self._baseline.dispatch("read", {"point_id": point["point_id"]})
                expected.extend((point["point_id"], check["check_id"]) for check in detail["checks"])
            missing = [list(key) for key in expected if key not in self._records]
            if missing:
                return {"state": "incomplete", "missing_checks": missing, "run_id": self._run}
            result = {"state": "completed", "run_id": self._run,
                      "finished_at": datetime.now(timezone.utc).isoformat(),
                      "records": deepcopy(list(self._records.values())), "sent": False,
                      "all_normal": all(record["status"] == "normal" for record in self._records.values())}
            self._report_dir.mkdir(parents=True, exist_ok=True)
            path = self._report_dir / (self._run + "-" + uuid.uuid4().hex + ".json")
            temporary_path = None
            try:
                with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self._report_dir,
                                                 prefix=".patrol-", suffix=".tmp", delete=False) as output:
                    temporary_path = Path(output.name)
                    json.dump(result, output, ensure_ascii=False, indent=2, allow_nan=False)
                    output.write("\n")
                os.replace(temporary_path, path)
            finally:
                if temporary_path is not None:
                    temporary_path.unlink(missing_ok=True)
            result["report_path"] = str(path)
            self._finished = result
            self._generation += 1
            return deepcopy(result)
