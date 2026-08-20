"""Isolated ROS2 process for Q5 D455 capture and media encoding."""

from __future__ import annotations

import multiprocessing as mp
import os
import queue
import threading
import time


class CameraWorker:
    """Keep camera subscriptions/NumPy work out of the Q5 control process."""

    def __init__(self, configs: dict):
        self._ctx = mp.get_context("spawn")
        self._commands = self._ctx.Queue()
        self._media = self._ctx.Queue(maxsize=12)
        self._configs = configs
        self._process = None
        self._running = False

    def start(self):
        if self._running:
            return
        self._process = self._ctx.Process(
            target=_run_camera_worker,
            args=(self._commands, self._media, self._configs),
            name="q5_camera_worker", daemon=True,
        )
        self._process.start()
        self._running = True
        print(f"[CameraWorker] subprocess started -> pid={self._process.pid}", flush=True)

    def command(self, kind: str, action: str):
        if self._running:
            try:
                self._commands.put_nowait({"kind": kind, "action": action})
            except Exception:
                pass

    def drain(self, sender):
        """Forward only the newest frame per stream to the media bridge."""
        newest = {}
        while True:
            try:
                frame = self._media.get_nowait()
            except queue.Empty:
                break
            if isinstance(frame, dict) and frame.get("kind"):
                newest[frame["kind"]] = frame
        for frame in newest.values():
            sender(frame)

    def stop(self):
        if not self._running:
            return
        try:
            self._commands.put_nowait("shutdown")
            self._process.join(timeout=5)
        except Exception:
            pass
        if self._process and self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=2)
        self._running = False
        print("[CameraWorker] subprocess stopped", flush=True)


class CameraProxy:
    def __init__(self, config, worker: CameraWorker, kind: str):
        self._config = config
        self._worker = worker
        self._kind = kind
        self._running = False
        self._topic = str(config.get("topic", ""))
        self._format = {"rgb": "image/jpeg", "depth": "image/jpeg",
                        "pointcloud": "sensor/pointcloud"}[kind]

    def get_tool(self):
        card = {"rgb": "camera_rgb", "depth": "camera_depth",
                "pointcloud": "camera_pointcloud"}[self._kind]
        return {
            "name": card, "type": "sensor", "multiInstance": False,
            "description": "Q5 D455 live camera stream (isolated camera worker).",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": self._format}],
        }

    def start(self):
        if not self._running:
            self._worker.command(self._kind, "start")
            self._running = True

    def stop(self):
        if self._running:
            self._worker.command(self._kind, "stop")
            self._running = False

    def dispatch(self, action, args):
        del args
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "worker": "q5_camera_worker",
                    "topic_out": [{"topic": self._topic, "format": self._format}]}
        return None


def _run_camera_worker(commands, media, configs):
    os.environ["ROS_DOMAIN_ID"] = str(os.environ.get("ROS_DOMAIN_ID", "211"))
    os.environ["RMW_IMPLEMENTATION"] = os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    import rclpy
    import rclpy.executors
    from legacy_device import CameraDepthPlugin, CameraPointCloudPlugin, CameraRgbPlugin

    class Client:
        def publish_media(self, frame):
            try:
                media.put_nowait(frame)
            except Exception:
                try:
                    media.get_nowait()
                    media.put_nowait(frame)
                except Exception:
                    pass

        def snapshot(self):
            return {"joints": {}}

    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    client = Client()
    namespace = str(configs.get("namespace", "q5"))
    classes = {"rgb": CameraRgbPlugin, "depth": CameraDepthPlugin,
               "pointcloud": CameraPointCloudPlugin}
    plugins = {}
    config_names = {"camera_rgb": "rgb", "camera_depth": "depth",
                    "camera_pointcloud": "pointcloud"}
    for config_name, config in configs.get("plugins", {}).items():
        kind = config_names.get(config_name)
        if kind in classes:
            plugins[kind] = classes[kind](config, namespace, executor, client)
    for plugin in plugins.values():
        try:
            plugin.start()
        except Exception as exc:
            print(f"[CameraWorker] {type(plugin).__name__} start failed: {exc}", flush=True)

    running = True
    while running and rclpy.ok():
        try:
            while True:
                command = commands.get_nowait()
                if command == "shutdown":
                    running = False
                    break
                plugin = plugins.get(command.get("kind")) if isinstance(command, dict) else None
                if plugin is not None:
                    (plugin.start if command.get("action") == "start" else plugin.stop)()
        except queue.Empty:
            pass
        executor.spin_once(timeout_sec=0.02)
    for plugin in plugins.values():
        plugin.stop()
    executor.shutdown()
    rclpy.shutdown()
