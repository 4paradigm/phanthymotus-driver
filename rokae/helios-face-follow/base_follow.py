"""Turn Helios in place through the on-robot timed base_drive tool."""

import http.client
import json
import time

from tracking import finite

CHEST_CAMERA = "rokae/helios/chest_rgb"
START_OFFSET = 0.12
CENTER_OFFSET = 0.07
MIN_RATE = 3.5
MAX_RATE = 8.0
START_ALLOWANCE = 3.0


def _call(port, tool, action, **args):
    body = json.dumps({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {
        "name": tool, "arguments": {"action": action, **args},
    }}, allow_nan=False)
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=0.15)
    try:
        conn.request("POST", "/mcp", body, {"Content-Type": "application/json"})
        response = conn.getresponse()
        data = json.loads(response.read(65537)) if response.status == 200 else {}
        result = json.loads(data["result"]["content"][0]["text"])
        if not isinstance(result, dict) or result.get("error") or result.get("state") == "error":
            raise ValueError(result.get("error", "base tool rejected request") if isinstance(result, dict)
                             else "invalid base tool result")
        return result
    except (KeyError, IndexError, TypeError, RecursionError) as error:
        raise ValueError(f"invalid {tool} response") from error
    finally:
        conn.close()


class BaseFollower:
    def __init__(self, call=_call, clock=time.monotonic):
        self.call = call
        self.clock = clock
        self.camera_ok = False
        self.state = "idle"
        self.error = None
        self.action_id = None
        self.stop_at = 0.0
        self.yaw_rate = 0.0
        self.last_sample = None
        self.candidate = None
        self.retry_after = 0.0
        self.wait_detail = None

    def status(self):
        return {"backend": "base", "state": self.state, "error": self.error,
                "detail": self.wait_detail,
                "action_id": self.action_id, "yaw_rate_deg_s": self.yaw_rate}

    def reset(self):
        if self.halt("idle"):
            self.state, self.error = "idle", None
        self.camera_ok = False
        self.retry_after = 0.0
        self.wait_detail = None

    def halt(self, state):
        self.candidate = None
        if self.action_id:
            try:
                status = self._base("info")
                if "active_move" not in status:
                    raise ValueError("base status omitted the active move")
                if status["active_move"] == self.action_id:
                    result = self._base("stop_move")
                    if result.get("stopped") is not True or result.get("cancelled_move") != self.action_id:
                        raise ValueError("base did not confirm cancellation of this turn")
            except (OSError, ValueError, http.client.HTTPException) as error:
                self.state, self.error = "fault", str(error)
                return False
            self.action_id, self.yaw_rate = None, 0.0
        if not self.error:
            self.state = state
            self.wait_detail = None
        return True

    def _base(self, action, **args):
        return self.call(15782, "base_drive", action, **args)

    def _navigation_idle(self):
        status = self.call(15881, "controlled_spatial", "status")
        navigation = status.get("navigation")
        return not status.get("building") and isinstance(navigation, dict) and navigation.get("state") == "idle"

    def step(self, target, sample, valid_until):
        if self.error:
            if self.action_id:
                self.halt("fault")
            return
        if not self.camera_ok:
            self.halt("waiting_for_chest_rgb")
            return
        if target is None or self.clock() >= valid_until:
            self.halt("waiting_for_target")
            return
        x = target["offset"][0]
        threshold = START_OFFSET if self.state == "centered" else CENTER_OFFSET
        if abs(x) <= threshold:
            self.halt("centered")
            return

        magnitude = min(MAX_RATE, max(MIN_RATE, 14 * abs(x)))
        rate = -magnitude if x > 0 else magnitude
        candidate = (target["track_id"], rate > 0)
        requesting_move = False
        try:
            if self.action_id:
                if self.clock() >= self.stop_at:
                    if self.halt("fault"):
                        self.error = "base turn did not finish within the supervision window"
                elif candidate != self.candidate or not self._navigation_idle():
                    self.halt("waiting_for_target")
                elif abs(self.yaw_rate) > abs(rate) + 2:
                    self.halt("settling")
                else:
                    info = self._base("info")
                    if "active_move" not in info:
                        raise ValueError("base status omitted the active move")
                    if info["active_move"] != self.action_id:
                        last = info.get("last_move") or {}
                        failed = (isinstance(last, dict) and last.get("action_id") == self.action_id
                                  and last.get("status") == "error")
                        self.action_id, self.yaw_rate = None, 0.0
                        self.candidate = None
                        self.state = "fault" if failed else "settling"
                        if failed:
                            self.error = last.get("error") or "base turn failed"
                    elif info.get("chassis_exception_level") != 0:
                        if self.halt("fault"):
                            self.error = "base exception during turn"
                return
            if self.last_sample == sample or self.clock() < self.retry_after:
                return
            self.last_sample = sample
            if self.candidate != candidate:
                self.candidate, self.state = candidate, "confirming_target"
                return
            if not self._navigation_idle():
                self.state = "base_busy"
                return
            info = self._base("info")
            if "active_move" not in info or info["active_move"] is not None:
                self.state = "base_busy"
                return
            odom = info.get("odometry")
            if (info.get("state") != "ready" or info.get("motion_enabled") is not True
                    or info.get("sdk_available") is not True or info.get("worker_running") is not True
                    or type(info.get("chassis_exception_level")) is not int or info["chassis_exception_level"] != 0
                    or not isinstance(odom, dict) or odom.get("stale") is True
                    or not finite(odom.get("age_s")) or not 0 <= odom["age_s"] <= 1
                    or any(not finite(odom.get(k)) or abs(odom[k]) > 0.025 for k in ("vx", "vy", "w"))):
                self.state = "waiting_for_base"
                self.wait_detail = info.get("connect_error") or "底盘反馈或里程计尚未就绪"
                return
            now = self.clock()
            if now >= valid_until:
                self.halt("waiting_for_target")
                return
            duration = min(1.2, max(0.35, 1.8 * abs(x)))
            requesting_move = True
            reply = self._base("move", vx=0.0, vy=0.0, vyaw=rate, duration_s=duration)
            action_id = reply.get("action_id")
            if reply.get("state") != "running" or not isinstance(action_id, str) or not action_id:
                raise ValueError("base did not return a timed turn ID")
            self.action_id, self.yaw_rate = action_id, rate
            # The installed SDK starts timing duration after the wheels wake.
            # This deadline only applies while fresh observations keep arriving.
            self.stop_at = now + START_ALLOWANCE + duration
            self.state = "controlling"
            self.wait_detail = None
            if self.clock() >= valid_until:
                self.halt("waiting_for_target")
        except (OSError, ValueError, http.client.HTTPException) as error:
            if self.action_id or requesting_move:
                self.state, self.error = "fault", str(error)
            else:
                self.state, self.wait_detail = "waiting_for_base", str(error)
                self.retry_after = self.clock() + 0.75
