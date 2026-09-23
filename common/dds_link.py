"""The link to a robot's own DDS, brought up with retries instead of once.

`ChannelFactoryInitialize` is called at bundle start, and on r1_sz that start
happened three seconds before its network interface existed::

    09:12:40  [bundle] namespace=ubuntu mcp_port=15702
    09:12:43  [bundle] DDS init failed on 'eth10': channel factory init error.
              python3: eth10: does not match an available interface.

eth10 came up a moment later and stayed up all day. Nothing retried. Every
state topic on that robot — odometry, IMU, joints, battery, mainboard — was
empty from boot until somebody looked, while the cards that publish them sat on
the canvas looking healthy and declaring in `topic_out` that they publish at
10 Hz.

Two separate defects, and this module is about both:

**The interface list was enumerated once.** The fallback scan for an address on
`192.168.123.x` ran at the same instant as the failure, so it found nothing
either. Retrying a stale list would be pointless; the candidates have to be
recomputed on every attempt, because the thing being waited for is precisely an
interface that does not exist yet.

**Failure was reported and then forgotten.** One `print`, one `logger.warn`,
and afterwards every layer — the tool schema, `topic_out`, the canvas — said the
same thing it says when the link is fine. `status()` exists so a card can put
the truth where somebody will see it.

── on the module-level singleton ────────────────────────────────────────────

`ChannelFactory` in the vendored SDK is already a process-wide singleton, and
the DDS domain it owns genuinely is one per process. Threading a link object
through every plugin constructor would add a parameter that can only ever hold
one value, and would let a caller believe two links are possible. So this keeps
one, and says why.

── retrying is safe against the vendored SDK ────────────────────────────────

`ChannelFactory.Init` returns True immediately once `__initialized` is set, so
a retry after success costs nothing; and it leaves `__initialized` False when
domain creation raises, so a retry after failure genuinely re-attempts. Both
behaviours are load-bearing here and were checked against
`unitree_sdk2py/core/channel.py` rather than assumed.
"""

from __future__ import annotations

import threading
import time

DEFAULT_DOMAIN = 0
# The subnet Unitree robots put their own control network on. Used to find the
# interface when none was named, and re-scanned on every attempt.
ROBOT_SUBNET = "192.168.123."

_FIRST_DELAY_S = 1.0
_MAX_DELAY_S = 15.0


def candidate_interfaces(preferred: str = "") -> list:
    """Interfaces worth trying, **recomputed at the moment of the attempt**.

    Order: whatever was configured, then anything holding an address on the
    robot's own subnet, then "" for the SDK's own auto-detection.

    Re-scanning rather than caching is the whole point. The failure this exists
    for is an interface that appears seconds after the process starts, and a
    list captured at start cannot contain it.
    """
    out = [preferred] if preferred else []
    try:
        import netifaces

        for name in netifaces.interfaces():
            for addr in netifaces.ifaddresses(name).get(netifaces.AF_INET, []):
                if str(addr.get("addr", "")).startswith(ROBOT_SUBNET):
                    if name not in out:
                        out.append(name)
    except Exception:                                          # noqa: BLE001
        pass
    out.append("")
    return out


class DdsLink:
    """Brings up `ChannelFactoryInitialize` and tells subscribers when it is up.

    A card registers what it wants to subscribe with `on_ready`; the callback
    runs once the link is live, on this object's own thread. Registering after
    the link is already up runs the callback immediately, so there is no order
    to get wrong between bundle start and card construction.
    """

    def __init__(self, preferred: str = "", domain: int = DEFAULT_DOMAIN,
                 initialize=None, sleep=time.sleep):
        self._preferred = preferred
        self._domain = domain
        # Injected for tests: bringing a real DDS domain up and down in a unit
        # test is neither possible on a laptop nor the thing under test.
        self._initialize = initialize or _default_initialize
        self._sleep = sleep

        self._lock = threading.RLock()
        self._ready = False
        self._interface = ""
        self._attempts = 0
        self._last_error = ""
        self._started_at = time.monotonic()
        self._ready_at = None
        self._callbacks: list = []
        self._thread = None
        self._stop = threading.Event()

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        with self._lock:
            if self._thread is not None:
                return
            self._thread = threading.Thread(target=self._run, daemon=True,
                                            name="dds_link")
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def wait(self, timeout_s: float) -> bool:
        """Block until the link is up, or the timeout expires. For start-up
        paths that genuinely cannot proceed without it."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.ready:
                return True
            self._sleep(0.2)
        return self.ready

    def _run(self) -> None:
        delay = _FIRST_DELAY_S
        while not self._stop.is_set():
            if self._try_once():
                return
            self._sleep(delay)
            delay = min(_MAX_DELAY_S, delay * 2)

    def _try_once(self) -> bool:
        last_error = ""
        for interface in candidate_interfaces(self._preferred):
            with self._lock:
                self._attempts += 1
                attempts = self._attempts
            try:
                self._initialize(self._domain, interface)
            except Exception as error:                         # noqa: BLE001
                last_error = f"{interface or '(auto)'}: {error}"
                continue
            self._become_ready(interface)
            return True

        with self._lock:
            self._last_error = last_error
        # Loud on the transition and then rarely — a line per attempt would
        # bury every other log, and the first one already says everything the
        # rest would. Rarely is not never: a robot whose state topics are empty
        # must keep saying why, or the next person reads a quiet log as health.
        if attempts <= len(candidate_interfaces(self._preferred)) or attempts % 20 == 0:
            print(f"[dds] 还没连上机器人的 DDS（第 {attempts} 次尝试）：{last_error}",
                  flush=True)
        return False

    def _become_ready(self, interface: str) -> None:
        with self._lock:
            self._ready = True
            self._interface = interface
            self._ready_at = time.monotonic()
            callbacks = list(self._callbacks)
        waited = self._ready_at - self._started_at
        print(f"[dds] 已连上机器人的 DDS，接口 {interface or '(auto)'}"
              f"（等了 {waited:.1f}s，{self._attempts} 次尝试）", flush=True)
        for callback in callbacks:
            self._invoke(callback)

    # ── subscribers ──────────────────────────────────────────────────────────

    def on_ready(self, callback) -> None:
        """Run `callback` once the link is up. Immediately if it already is."""
        with self._lock:
            self._callbacks.append(callback)
            ready = self._ready
        if ready:
            self._invoke(callback)

    @staticmethod
    def _invoke(callback) -> None:
        try:
            callback()
        except Exception as error:                             # noqa: BLE001
            print(f"[dds] 订阅回调失败：{type(error).__name__}: {error}", flush=True)

    # ── reporting ────────────────────────────────────────────────────────────

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._ready

    def status(self) -> dict:
        """For a card's `info()`. **A card that cannot subscribe must say so
        there**, or it is indistinguishable on the canvas from one that works.
        """
        with self._lock:
            if self._ready:
                return {"state": "up", "interface": self._interface or "(auto)",
                        "attempts": self._attempts}
            return {
                "state": "connecting",
                "attempts": self._attempts,
                "waiting_s": round(time.monotonic() - self._started_at, 1),
                "last_error": self._last_error,
                "message": "还没连上机器人自己的 DDS —— 本卡片声明的话题不会有数据。"
                           "常见原因是容器启动时网口还没上来；连上之后会自动订阅，"
                           "不需要重启。",
            }


def _default_initialize(domain: int, interface: str) -> None:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize(domain, interface)


# ── the one link ─────────────────────────────────────────────────────────────

_link: DdsLink | None = None
_link_lock = threading.Lock()


def get_link() -> DdsLink | None:
    """The process's link, or None before `install` has run."""
    return _link


def install(preferred: str = "", **kwargs) -> DdsLink:
    """Create the process's link and start bringing it up. Idempotent."""
    global _link
    with _link_lock:
        if _link is None:
            _link = DdsLink(preferred, **kwargs)
            _link.start()
        return _link


def reset_for_tests() -> None:
    global _link
    with _link_lock:
        if _link is not None:
            _link.stop()
        _link = None
