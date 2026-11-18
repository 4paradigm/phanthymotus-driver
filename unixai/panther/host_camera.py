"""Lazy start of the robot's Orbbec camera driver, from the first camera card started.

The Panther's own bring-up does not run the Orbbec driver, so without this the camera cards
show nothing until someone logs in and launches it by hand. Ported from the Panther driver
proposed in PR #520: the first `camera_color` or `camera_depth` start creates a transient
systemd unit on the *host* (through `nsenter` into PID 1's namespaces) that runs the vendor's
`gemini2.launch.py`; both cards share it, and stopping a card does not stop it, because other
programs on the robot may be using the same camera.

This needs `privileged: true` and `pid: host` (deploy/service.yml). It is the only thing in the
driver that acts on the host, so it is deliberately small: fixed commands, no caller-supplied
text, and a unit that is reused if it already exists.

Nothing here raises into a card start: agent-core rolls a whole project back when one start
fails, so a camera that cannot be started is reported in the card's `warning`, not by failing.

Frame timing comes from the Hub (it already counts every frame), so the watchdog here only
decides when a stalled stream justifies restarting the unit -- at most `max_watchdog_restarts`
times, so a camera that is simply broken is not restarted forever.
"""
from __future__ import annotations

import subprocess
import threading
import time

KINDS = ("color", "depth")


class HostCamera:
    UNIT = "panther_orbbec.service"
    NSENTER = ("nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--")

    def __init__(self, config: dict, topics: dict, publisher_count, frame_age, *,
                 run=subprocess.run, sleep=time.sleep, monotonic=time.monotonic, watchdog=True):
        """`topics` maps "color"/"depth" to topic names; `publisher_count(topic)` and
        `frame_age(kind)` (seconds since the last frame, or None) come from the ROS layer/Hub."""
        self._cfg = config or {}
        self._topics = topics
        self._publishers = publisher_count
        self._frame_age = frame_age
        self._run = run
        self._sleep = sleep
        self._mono = monotonic
        self._lock = threading.RLock()
        self._source_lock = threading.Lock()
        self._active: set = set()
        self._armed_at = 0.0
        self._managed = False
        self._restarts = 0
        self._last_error = None
        self._closed = threading.Event()
        self._thread = None
        if watchdog:
            self._thread = threading.Thread(target=self._watch, daemon=True, name="panther-camera-watch")
            self._thread.start()

    # ── card-facing ──────────────────────────────────────────────────────────

    def start(self, kind: str) -> dict:
        if kind not in KINDS:
            raise ValueError("camera kind must be color or depth")
        with self._lock:
            already = kind in self._active
        if not already:
            with self._source_lock:
                try:
                    self._ensure_source()
                    self._last_error = None
                except Exception as exc:  # noqa: BLE001
                    self._last_error = f"{type(exc).__name__}: {str(exc)[:200]}"
                    print(f"[panther] camera source could not be started: {self._last_error}", flush=True)
            deadline = self._mono() + float(self._cfg.get("start_wait_s", 5.0))
            while self._mono() < deadline and self._publishers(self._topics[kind]) < 1:
                self._sleep(0.2)
            with self._lock:
                if not self._active:
                    self._armed_at = self._mono()
                    self._restarts = 0
                self._active.add(kind)
        return self.info(kind)

    def stop(self, kind: str) -> dict:
        with self._lock:
            self._active.discard(kind)
        return self.info(kind)

    def info(self, kind: str) -> dict:
        topic = self._topics[kind]
        with self._lock:
            result = {"host_service": self.UNIT, "host_service_managed": self._managed,
                      "publisher_count": self._publishers(topic), "source_topic": topic,
                      "watchdog_restarts": self._restarts}
        if self._last_error:
            result["start_error"] = self._last_error
        return result

    def close(self) -> None:
        self._closed.set()
        with self._lock:
            self._active.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # ── host side ────────────────────────────────────────────────────────────

    def _host(self, command, timeout: float = 15.0, check: bool = True):
        return self._run([*self.NSENTER, *command], check=check, capture_output=True, text=True,
                         timeout=timeout)

    def _launch_script(self) -> str:
        workspace = self._cfg.get("workspace", "/home/unix_ai/orbbec_ws")
        profile = self._cfg.get("cyclonedds_profile", "/home/unix_ai/config/cyclonedds.xml")
        # The camera node must sit on the same loopback-only Cyclone graph as the robot's own
        # nodes, or this driver (and the robot) would not see its topics.
        return (f"cd {workspace} && . install/setup.bash && "
                "export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
                f"[ -f {profile} ] && export CYCLONEDDS_URI=file://{profile}; "
                "exec ros2 launch orbbec_camera gemini2.launch.py")

    def _ensure_source(self) -> None:
        # Any existing publisher means something already owns the device. Starting a second
        # launch would fight it for the USB device, so this card just waits on that stream.
        if any(self._publishers(topic) for topic in self._topics.values()):
            return
        active = self._host(["systemctl", "is-active", self.UNIT], timeout=5.0, check=False)
        if active.stdout.strip() == "active":
            with self._lock:
                self._managed = True
            return
        # A transient unit that failed may still be loaded; restarting it beats creating a
        # second unit with the same name.
        again = self._host(["systemctl", "start", self.UNIT], timeout=15.0, check=False)
        if again.returncode != 0:
            self._host(["systemd-run", f"--unit={self.UNIT}", "--collect",
                        "--property=Restart=on-failure", "--property=RestartSec=2s",
                        f"--uid={self._cfg.get('user', 'unix_ai')}",
                        f"--setenv=HOME=/home/{self._cfg.get('user', 'unix_ai')}",
                        "/bin/bash", "-lc", self._launch_script()], timeout=15.0)
        with self._lock:
            self._managed = True

    # ── watchdog ─────────────────────────────────────────────────────────────

    def _watch(self) -> None:
        while not self._closed.wait(1.0):
            try:
                self.check()
            except Exception as exc:  # noqa: BLE001
                print(f"[panther] camera watchdog: {type(exc).__name__}: {str(exc)[:160]}", flush=True)

    def check(self) -> bool:
        """One watchdog pass. True if it restarted the unit."""
        first_frame = float(self._cfg.get("first_frame_grace_s", 30.0))
        stall = float(self._cfg.get("frame_stall_s", 15.0))
        limit = int(self._cfg.get("max_watchdog_restarts", 2))
        with self._lock:
            active, managed, armed_at, restarts = set(self._active), self._managed, self._armed_at, self._restarts
        if not active or not managed or not armed_at or restarts >= limit:
            return False
        now = self._mono()
        stalled = []
        for kind in active:
            age = self._frame_age(kind)
            if age is None:
                if now - armed_at >= first_frame:
                    stalled.append(kind)
            elif age >= stall:
                stalled.append(kind)
        if not stalled:
            return False
        self._host(["systemctl", "restart", self.UNIT], timeout=30.0)
        with self._lock:
            self._restarts += 1
            self._armed_at = self._mono()
        print(f"[panther] restarted {self.UNIT} ({self._restarts}/{limit}); stalled: {stalled}", flush=True)
        return True
