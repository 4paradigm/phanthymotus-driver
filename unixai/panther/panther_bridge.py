"""Process-isolated Agent Core (domain 42) publishers for the Panther driver.

The robot's own DDS runs CycloneDDS on domain 0, bound to loopback. Agent Core is on domain
42 with FastDDS and the fleet's loopback-only profile. One process cannot hold both -- FastDDS
reads its profile once per process, and the RMW is chosen at `rclpy.init` -- so this driver
reads the robot in its main process and hands finished payloads to small child processes that
do nothing but publish on domain 42. It is the arrangement `deep_robotics/s10_pro` uses.

What crosses the process boundary is plain data -- a JSON string or encoded image bytes --
and the *child* builds the ROS message. The parent therefore never serialises a ROS message
and never imports a FastDDS-linked type, and the whole thing is testable without ROS.

Each lane is a bounded queue feeding one child. A full queue drops a sample and counts it; it
never blocks a ROS callback, because the callback shares a thread pool with every other card.
A lane's child is started on its first sample, so a camera nobody has opened costs nothing.
"""
from __future__ import annotations

import multiprocessing as mp
import os
import queue
import threading
import time

DEFAULT_PROFILE = "/opt/phanthy-motus/dds-local.xml"
RESTART_BACKOFF_S = 5.0


def safe_text(value, limit: int = 120) -> str:
    """Anything that came from outside a log line: escaped to ASCII and capped."""
    text = str(value).encode("unicode_escape").decode("ascii")
    return text if len(text) <= limit else text[: limit - 3] + "..."


def put_latest(messages, item) -> bool:
    """Put `item` on a full-or-not bounded queue, discarding the oldest entry if needed."""
    try:
        messages.put_nowait(item)
        return True
    except queue.Full:
        pass
    try:
        messages.get_nowait()
    except queue.Empty:
        # The consumer may have taken it while the queue's semaphore was still settling.
        pass
    try:
        messages.put_nowait(item)
        return True
    except queue.Full:
        return False


class CoreBridge:
    def __init__(self, profile: str = DEFAULT_PROFILE, domain: int = 42, context=None,
                 clock=time.monotonic):
        self._ctx = context or mp.get_context("spawn")
        self._profile = profile
        self._domain = int(domain)
        self._clock = clock
        self._lock = threading.Lock()
        self._lanes: dict = {}
        self._closed = False

    def register_lane(self, lane: str, queue_depth: int = 1, latest_only: bool = True) -> None:
        with self._lock:
            existing = self._lanes.get(lane)
            settings = (int(queue_depth), bool(latest_only))
            if existing is not None:
                if existing["settings"] != settings:
                    raise ValueError(f"lane {lane!r} was registered with different queue settings")
                return
            self._lanes[lane] = {
                "settings": settings, "queue": None, "process": None, "started_at": None,
                "enqueued": 0, "dropped": 0, "restarts": 0, "errors": 0, "last_error": None,
            }

    def _spawn(self, lane: str, state: dict) -> None:
        depth, _ = state["settings"]
        old = state["queue"]
        if old is not None:
            try:
                old.close()
            except Exception:
                pass
        messages = self._ctx.Queue(maxsize=depth)
        process = self._ctx.Process(
            target=_run_publisher, args=(messages, self._profile, self._domain, lane, depth),
            name=f"panther-domain{self._domain}-{lane}", daemon=True)
        process.start()
        if state["process"] is not None:
            state["restarts"] += 1
        state.update(queue=messages, process=process, started_at=self._clock())
        print(f"[dds-bridge] lane={lane} child pid={getattr(process, 'pid', '?')} started "
              f"(domain {self._domain})", flush=True)

    def send(self, lane: str, item: tuple) -> bool:
        """Queue one sample for `lane`. False if it was dropped. Never raises into a ROS callback."""
        try:
            with self._lock:
                if self._closed:
                    return False
                state = self._lanes[lane]
                process = state["process"]
                if process is None:
                    self._spawn(lane, state)
                elif not process.is_alive():
                    if self._clock() - (state["started_at"] or 0.0) >= RESTART_BACKOFF_S:
                        self._spawn(lane, state)
                depth, latest_only = state["settings"]
                messages = state["queue"]
                if latest_only:
                    accepted = put_latest(messages, item)
                else:
                    try:
                        messages.put_nowait(item)
                        accepted = True
                    except queue.Full:
                        accepted = False
                if accepted:
                    state["enqueued"] += 1
                else:
                    state["dropped"] += 1
                return accepted
        except Exception as exc:
            with self._lock:
                state = self._lanes.get(lane)
                if state is not None:
                    state["dropped"] += 1
                    state["errors"] += 1
                    count = state["errors"]
                    state["last_error"] = repr(exc)
                    if count == 1 or count % 100 == 0:
                        print(f"[dds-bridge] send failed lane={lane} count={count}: "
                              f"{safe_text(exc)}", flush=True)
            return False

    def stats(self) -> dict:
        with self._lock:
            return {lane: {
                "alive": bool(state["process"] is not None and state["process"].is_alive()),
                "enqueued": state["enqueued"], "dropped": state["dropped"],
                "restarts": state["restarts"], "last_error": state["last_error"],
            } for lane, state in self._lanes.items()}

    def stop(self) -> None:
        with self._lock:
            self._closed = True
            states = list(self._lanes.values())
        for state in states:
            if state["queue"] is not None:
                put_latest(state["queue"], None)
        for state in states:
            process = state["process"]
            if process is None:
                continue
            process.join(timeout=2)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1)


def _run_publisher(messages, profile, domain, lane, queue_depth=1):
    """Child entry point. Imports ROS only after pinning it to FastDDS on `domain`."""
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except ImportError:
        pass
    if not os.path.isfile(profile):
        # FastDDS silently ignores a missing profile and falls back to every network
        # interface, which both breaks delivery to Agent Core and leaks domain 42 to the LAN.
        print(f"[dds-bridge] lane={lane}: DDS profile {safe_text(profile)} is missing; "
              f"refusing to publish", flush=True)
        return
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    rclpy.init(domain_id=domain)
    node = Node(f"panther_core_{lane}", enable_rosout=False, start_parameter_services=False)
    qos = QoSProfile(depth=max(1, int(queue_depth)), reliability=ReliabilityPolicy.BEST_EFFORT)
    publishers: dict = {}
    published: dict = {}
    failures: dict = {}
    try:
        while True:
            try:
                item = messages.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                break
            kind, topic = item[0], item[1]
            try:
                if kind == "json":
                    msg_type, message = String, String(data=item[2])
                elif kind == "image":
                    _, _, image_format, data, frame_id = item
                    message = CompressedImage()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.header.frame_id = frame_id
                    message.format = image_format
                    message.data = data
                    msg_type = CompressedImage
                else:
                    print(f"[dds-bridge] lane={lane} unknown item kind {safe_text(kind)}", flush=True)
                    continue
                publisher = publishers.get(topic)
                if publisher is None:
                    publisher = publishers[topic] = node.create_publisher(msg_type, topic, qos)
                    print(f"[dds-bridge] lane={lane} publisher created on domain {domain}: "
                          f"{safe_text(topic)}", flush=True)
                publisher.publish(message)
                published[topic] = published.get(topic, 0) + 1
                failures[topic] = 0
                if published[topic] == 1:
                    print(f"[dds-bridge] lane={lane} first sample published: {safe_text(topic)}",
                          flush=True)
            except Exception as exc:
                failures[topic] = failures.get(topic, 0) + 1
                if failures[topic] == 1 or failures[topic] % 100 == 0:
                    print(f"[dds-bridge] lane={lane} publish failed topic={safe_text(topic)} "
                          f"count={failures[topic]}: {safe_text(exc)}", flush=True)
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
