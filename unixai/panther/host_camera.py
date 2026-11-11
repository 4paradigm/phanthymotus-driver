"""Lazy start of the robot's Orbbec camera driver, from the first camera card started.

The Panther's own bring-up does not run the Orbbec driver, so without this the camera cards
show nothing until someone logs in and launches it by hand. Ported from the Panther driver
proposed in PR #520: the first `camera_color` or `camera_depth` start creates a transient
systemd unit on the *host* (through `nsenter` into PID 1's namespaces) that runs the vendor's
`gemini2.launch.py`; both cards share it, and stopping a *card* does not stop it, because other
programs on the robot may be using the same camera. Stopping the *container* does (`close()`),
but only a unit this driver started or found under its own name: a camera some other process was
already publishing is never touched. A unit left behind by a killed container (SIGKILL, crash)
is reused by the next start.

This needs `pid: host` and the one capability `SYS_ADMIN` (deploy/service.yml), not `privileged:
true`. `setns(2)` requires CAP_SYS_ADMIN to join the mount, PID, network, IPC and UTS namespaces of
another process, and CAP_SYS_CHROOT as well for the mount one (Docker grants that by default);
`privileged` would add every other capability, every host device and no seccomp/AppArmor
confinement, none of which this uses. Whether SYS_ADMIN alone is enough on a given robot depends on
its seccomp/AppArmor profile and on `nsenter` being allowed to read `/proc/1/ns/*`, which the
manual page does not settle: it is checked on the robot (README "Acceptance"), and if it is
refused the failure says so (`_denied`) rather than looking like a unit that does not exist.
It is the only thing in the driver that acts on the host, so it is deliberately small: fixed
commands, no caller-supplied text, and a unit that is reused if it already exists.

Nothing here raises into a card start: agent-core rolls a whole project back when one start
fails, so a camera that cannot be started is reported in the card's `warning`, not by failing.

Frame timing comes from the Hub (it already counts every frame), so the watchdog here only
decides when a stalled stream justifies restarting the unit -- at most `max_watchdog_restarts`
times, so a camera that is simply broken is not restarted forever.
"""
from __future__ import annotations

import re
import subprocess
import threading
import time

KINDS = ("color", "depth")

_ADVICE = ("this container could not enter the host's namespaces. It needs `pid: host` and "
           "`cap_add: [SYS_ADMIN]` in deploy/service.yml, and the host's seccomp/AppArmor profile "
           "must allow `setns`; see the driver README, Acceptance step 7")


class NsenterDenied(RuntimeError):
    """`nsenter` refused to enter PID 1's namespaces. Raised for every host command, including
    the ones run with check=False, so it cannot be mistaken for "the unit is not running"."""

    def __init__(self, message: str):
        super().__init__(f"could not enter the host's namespaces ({message}); {_ADVICE}")


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
        self._unit_lock = threading.Lock()                  # a restart and the final stop never interleave
        self._active: set = set()
        self._armed_at = 0.0
        self._managed = False
        self._restarts = 0
        self._last_error = None
        self._closing = False
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
        """Container shutdown: stop the camera unit this driver started, once.

        Only a unit this driver manages is stopped (`_managed`): if another process was already
        publishing the camera topics the driver never started anything and leaves them alone,
        and a run in which no camera card was ever started stops nothing. `--no-block` returns as
        soon as systemd has the request, so a camera that is slow to exit cannot eat the
        container's `stop_grace_period`, which a moving arm may need. A failure is logged and
        never raised: shutdown must finish either way.

        The watchdog is told to stand down first and is joined before the stop, so it cannot
        restart the unit it has just been asked to stop. Calling this twice stops once.
        """
        with self._lock:
            if self._closing:
                return
            self._closing = True
            self._active.clear()
            managed = self._managed
        self._closed.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if not managed:
            return
        try:
            with self._unit_lock:
                self._host(["systemctl", "stop", "--no-block", self.UNIT], timeout=10.0, check=False)
            print(f"[panther] asked systemd to stop {self.UNIT}", flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"[panther] could not stop {self.UNIT}: {type(exc).__name__}: {str(exc)[:160]}; "
                  f"stop it with `sudo systemctl stop {self.UNIT}`", flush=True)

    # ── host side ────────────────────────────────────────────────────────────

    @staticmethod
    def _nsenter_refused(result) -> str | None:
        """nsenter's own refusal message, or None.

        Told apart from `systemctl`'s ordinary non-zero exits (`is-active` is non-zero for a unit
        that is not running, `start` for one that does not exist) by the message, not the return
        code: nsenter's errors begin with its own name, possibly with a path. A message that merely
        *mentions* nsenter does not count.
        """
        stderr = str(getattr(result, "stderr", "") or "").strip()
        first = stderr.splitlines()[0] if stderr else ""
        if getattr(result, "returncode", 0) != 0 and re.match(r"^(\S*/)?nsenter:", first):
            return first
        return None

    def _host(self, command, timeout: float = 15.0, check: bool = True):
        result = self._run([*self.NSENTER, *command], check=check, capture_output=True, text=True,
                           timeout=timeout)
        refused = self._nsenter_refused(result)
        if refused:
            raise NsenterDenied(refused)
        return result

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
                        # If the camera process ignores SIGTERM, systemd's default is to wait 90 s.
                        "--property=TimeoutStopSec=15s",
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
            if self._closing:
                return False
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
        # The state above was read a moment ago. `close()` may have begun since, and a restart
        # issued after its stop would bring the camera back for good. So the decision is taken
        # again, under the same lock `close()` takes to stop the unit: whichever gets it first,
        # the other sees the result.
        with self._unit_lock:
            with self._lock:
                if self._closing:
                    return False
            self._host(["systemctl", "restart", self.UNIT], timeout=30.0)
        with self._lock:
            self._restarts += 1
            self._armed_at = self._mono()
        print(f"[panther] restarted {self.UNIT} ({self._restarts}/{limit}); stalled: {stalled}", flush=True)
        return True
