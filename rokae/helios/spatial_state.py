"""Localization validity and mapping mode reported by the chassis monitor."""

import json
import math

from .chassis_battery_pb2 import UiDatagram
from .feedback import StreamFeedbackClient


class ChassisMapState(StreamFeedbackClient):
    def __init__(self, url):
        self.last_key = None
        super().__init__(url, parser=self._parse, data_key="chassis", binary=True,
                         stale_timeout_s=3, max_msg_size=8 * 1024 * 1024)

    def _parse(self, raw):
        frame = UiDatagram.FromString(raw)
        if not frame.HasField("robot_base"):
            return None
        base = frame.robot_base
        loc = base.locate
        pool = json.loads(base.data_pool or "{}")
        mode = pool.get("mapping")
        key = (loc.update_time, loc.locate, mode)
        if key == self.last_key or not loc.update_time:
            return None
        self.last_key = key
        pose = loc.current_state
        located = (loc.locate == 1 and loc.HasField("current_state")
                   and all(math.isfinite(v) for v in (pose.x, pose.y, pose.theta)))
        return {"mapping": str(mode) == "1" if mode is not None else None, "located": located}
