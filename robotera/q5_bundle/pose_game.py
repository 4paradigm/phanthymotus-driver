"""Bounded Pose round recorder using the existing Q5 media bridge."""

from __future__ import annotations

import threading
import time
import uuid

from pose_recorder import PoseRecorder


POSE_TOPIC = "/nvidia_desktop/camera/rgb/poses"


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client,
                 clock_ns=time.monotonic_ns):
        del namespace, executor
        config = dict(plugin_config or {})
        topic = str(config.get("topic") or POSE_TOPIC)
        if topic != POSE_TOPIC:
            raise ValueError(f"pose_game only supports {POSE_TOPIC}")
        self._topic = topic
        self._client = client
        self._clock_ns = clock_ns
        self._barrier_timeout = max(
            0.001, float(config.get("barrier_timeout_seconds", 1.0)))
        self._recorder = PoseRecorder(
            max_frames=config.get("max_frames", 600),
            max_rounds=config.get("max_rounds", 20),
            max_frames_per_round=config.get("max_frames_per_round", 600),
            max_raw_bytes=config.get("max_raw_bytes", 1_048_576),
            clock_ns=clock_ns,
        )
        self._lifecycle_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._pump_thread = None
        self._barrier_condition = threading.Condition()
        self._seen_barriers = {}
        self._awaited_barriers = set()

    def start(self):
        with self._lifecycle_lock:
            if self._pump_thread is not None:
                return {"state": "running"}
            self._stop_event.clear()
            self._client.configure_pose(self._topic)
            self._pump_thread = threading.Thread(
                target=self._pump, daemon=True, name="q5_pose_recorder")
            self._pump_thread.start()
        return {"state": "running", "topic": self._topic}

    def stop(self):
        with self._lifecycle_lock:
            thread = self._pump_thread
            if thread is None:
                return {"state": "idle"}
            self._client.configure_pose("")
            self._stop_event.set()
            thread.join(timeout=2.0)
            if thread.is_alive():
                raise RuntimeError("Pose recorder pump did not stop")
            if self._pump_thread is thread:
                self._pump_thread = None
        return {"state": "idle"}

    def _pump(self):
        while not self._stop_event.is_set():
            try:
                item = self._client.pop_pose()
            except Exception:
                item = None
            if not isinstance(item, dict):
                self._stop_event.wait(0.005)
                continue
            if item.get("kind") == "barrier":
                token = str(item.get("token") or "")
                with self._barrier_condition:
                    if token in self._awaited_barriers:
                        self._seen_barriers[token] = (
                            int(item["bridge_received_monotonic_ns"]),
                            max(0, int(item.get("bridge_transport_drops", 0))),
                        )
                        self._barrier_condition.notify_all()
                continue
            if item.get("kind") != "pose":
                continue
            try:
                self._recorder.ingest(
                    item.get("raw_json"),
                    int(item["bridge_received_monotonic_ns"]),
                    int(self._clock_ns()),
                )
            except (KeyError, TypeError, ValueError):
                continue

    def _wait_for_barrier(self):
        token = uuid.uuid4().hex
        with self._barrier_condition:
            self._seen_barriers.pop(token, None)
            self._awaited_barriers.add(token)
        self._client.request_pose_barrier(token)
        deadline = time.monotonic() + self._barrier_timeout
        with self._barrier_condition:
            try:
                while token not in self._seen_barriers:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(
                            "Pose barrier was not observed before freeze")
                    self._barrier_condition.wait(remaining)
                return self._seen_barriers[token]
            finally:
                self._seen_barriers.pop(token, None)
                self._awaited_barriers.discard(token)

    def get_tool(self):
        return {
            "name": "pose_game",
            "type": "sensor",
            "multiInstance": False,
            "description": "Bounded Pose capture windows for offline action scoring",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start_round", "finish_round",
                                 "export_round", "info"],
                        "oneOf": [
                            {"const": "start_round", "title": "开始本轮采集"},
                            {"const": "finish_round", "title": "冻结本轮采集"},
                            {"const": "export_round", "title": "导出冻结轮次"},
                            {"const": "info", "title": "查看采集状态"},
                        ],
                    },
                    "round_id": {
                        "type": "string", "minLength": 1,
                        "description": "Unique round identifier retained with the snapshot.",
                    },
                    "outcome": {
                        "type": "string",
                        "enum": [
                            "COMPLETED", "PLAYBACK_FAILED", "TIMEOUT",
                            "MANUAL_TERMINATED", "SUBMIT_FAILED",
                            "UNTRUSTED_COMPLETION",
                        ],
                        "description": "Terminal round outcome; COMPLETED additionally requires trusted playback fields.",
                    },
                    "playback_result": {
                        "type": "object", "x-widget": "json",
                        "description": "Required only for COMPLETED. Offline Mock input until real tracked TTS integration is verified.",
                    },
                    "completion_observed_monotonic_ns": {
                        "type": "integer", "minimum": 0,
                        "description": "Required only for COMPLETED: Driver monotonic time when trusted completion was observed.",
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start_round": {
                        "params": ["round_id"],
                        "description": "Start capture before submitting TTS.",
                    },
                    "finish_round": {
                        "params": ["round_id", "outcome", "playback_result",
                                   "completion_observed_monotonic_ns"],
                        "description": "Freeze after a terminal outcome; playback evidence fields apply only to COMPLETED.",
                    },
                    "export_round": {
                        "params": ["round_id"],
                        "description": "Export one retained frozen round.",
                    },
                    "info": {
                        "params": [],
                        "description": "Read active round, capacities, and counters.",
                    },
                },
            },
            "topic_out": [],
        }

    def dispatch(self, action, args):
        args = dict(args or {})
        if action == "start_round":
            try:
                stats = self._client.pose_stats()
            except Exception:
                stats = {}
            return self._recorder.start_round(
                args["round_id"], int(self._clock_ns()),
                (stats or {}).get("bridge_transport_drops", 0))
        if action == "finish_round":
            barrier_ns, bridge_drops = self._wait_for_barrier()
            return self._recorder.finish_round(
                args["round_id"], args["outcome"], barrier_ns,
                args.get("playback_result"),
                args.get("completion_observed_monotonic_ns"),
                bridge_drops)
        if action == "export_round":
            return self._recorder.export_round(args["round_id"])
        if action in ("info", "pose_game"):
            try:
                stats = self._client.pose_stats()
            except Exception:
                stats = {}
            return self._recorder.status(
                (stats or {}).get("bridge_transport_drops", 0))
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
