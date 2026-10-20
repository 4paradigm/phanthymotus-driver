"""Q5 squat card proxy; pose processing lives in Agent Core's ROS domain.

Q5's main driver uses Domain 211/CycloneDDS. Perception publishes the full
pose skeleton in Domain 42/Fast DDS, so subscribing in the main driver would
silently receive no frames. A spawned child owns the ROS processor; a Pipe
carries only sparse MCP actions and their replies.
"""

from __future__ import annotations

import multiprocessing as mp
import os
import threading

from common.squat_contract import tool_definition


def _run_worker(conn, config: dict, namespace: str) -> None:
    os.environ["ROS_DOMAIN_ID"] = "42"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ.pop("CYCLONEDDS_URI", None)
    try:
        from fastdds_transport import configure_fastdds_transport
        configure_fastdds_transport()
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from common.squat_counter_card import SquatCounterPlugin

        rclpy.init()
        executor = SingleThreadedExecutor()
        card = SquatCounterPlugin(config, namespace, executor,
                                  node_name="q5_squat_counter")
        conn.send({"ready": True})
    except Exception as exc:
        try:
            conn.send({"ready": False, "error": str(exc)})
        finally:
            conn.close()
        return

    try:
        while rclpy.ok():
            if conn.poll():
                request = conn.recv()
                if request is None:
                    break
                action, args = request
                try:
                    reply = card.dispatch(action, args)
                except Exception as exc:
                    reply = {"state": "error", "error": str(exc)}
                conn.send(reply)
            executor.spin_once(timeout_sec=0.02)
    except EOFError:
        pass
    finally:
        card.stop()
        executor.shutdown()
        rclpy.shutdown()
        conn.close()


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._config = dict(plugin_config)
        self._namespace = namespace
        self._ctx = mp.get_context("spawn")
        self._lock = threading.RLock()
        self._proc = None
        self._conn = None
        self._error = "Q5 squat processor has not started"

    def get_tool(self) -> dict:
        return tool_definition(self._namespace,
                               coach_mode=self._config.get("coach_mode", False),
                               local_milestone_voice=(
                                   self._config.get("coach_mode", False)
                                   and self._config.get("local_milestone_voice", False)))

    def start(self) -> None:
        with self._lock:
            if self._proc is not None and self._proc.is_alive():
                return
            if self._conn is not None:
                self._conn.close()
                self._conn = None
            if self._proc is not None:
                self._proc.join(timeout=0)
                self._proc = None
            parent, child = self._ctx.Pipe()
            proc = self._ctx.Process(target=_run_worker,
                                     args=(child, self._config, self._namespace),
                                     name="q5_squat_counter", daemon=True)
            try:
                proc.start()
                child.close()
                if not parent.poll(5.0):
                    raise TimeoutError("Q5 squat processor startup timed out")
                ready = parent.recv()
                if not ready.get("ready"):
                    raise RuntimeError(ready.get("error") or "Q5 squat processor failed")
            except Exception as exc:
                self._error = str(exc)
                parent.close()
                child.close()
                if proc.is_alive():
                    proc.terminate()
                proc.join(timeout=2.0)
                print(f"[q5 squat_counter] {self._error}", flush=True)
                return
            self._proc = proc
            self._conn = parent
            self._error = ""

    def stop(self) -> None:
        with self._lock:
            conn, proc = self._conn, self._proc
            self._conn = self._proc = None
            if conn is not None:
                try:
                    conn.send(None)
                except (BrokenPipeError, EOFError, OSError):
                    pass
                conn.close()
            if proc is not None:
                proc.join(timeout=2.0)
                if proc.is_alive():
                    proc.terminate()
                    proc.join(timeout=2.0)

    def dispatch(self, action: str, args: dict) -> dict:
        with self._lock:
            if action == "start" and (self._proc is None or not self._proc.is_alive()):
                self.start()
            if self._proc is None or not self._proc.is_alive() or self._conn is None:
                return {"state": "error", "error": self._error or
                        "Q5 squat processor stopped; start the card again"}
            try:
                self._conn.send((action, dict(args)))
                if not self._conn.poll(5.0):
                    raise TimeoutError("Q5 squat processor did not respond")
                return self._conn.recv()
            except (BrokenPipeError, EOFError, OSError, TimeoutError) as exc:
                self._error = str(exc)
                self.stop()
                return {"state": "error", "error": self._error}


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
