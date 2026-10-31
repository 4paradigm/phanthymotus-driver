"""Squat counter that maintains pose history for LLM analysis.

Instead of hardcoding state machines, this card collects recent pose frames
and exposes them via an 'analyze' action. The LLM/skill reads this history
and makes the squat decision.

Workflow:
1. Card receives pose frames, caches last N frames per person
2. Skill/Agent calls 'analyze' action → returns structured summary
3. LLM reads summary + prompt → decides if squat completed
4. If LLM decides yes, card increments count and publishes event
"""

import math
from collections import deque


class SquatHistoryCounter:
    def __init__(self, history_frames=10, lost_s=4.0):
        """
        Args:
            history_frames: number of recent frames to keep per person.
            lost_s: seconds without tracked person before reset.
        """
        self.history_frames = int(history_frames)
        self.lost_s = float(lost_s)
        self.reset()

    def reset(self):
        self.count = 0
        self.track_id = None
        self.history = deque()  # list of (timestamp, person_dict)
        self.last_seen = None

    def _drop_track(self):
        self.track_id = None
        self.history.clear()
        self.last_seen = None

    def add_frame(self, person, now):
        """Add a pose frame to history. Returns True if person acquired."""
        if not isinstance(person, dict):
            return False

        pid = person.get("id")
        if pid is None:
            return False

        # Acquire track
        if self.track_id is None:
            self.track_id = pid
        elif self.track_id != pid:
            return False

        # Expire old history
        if self.last_seen is not None and now - self.last_seen > self.lost_s:
            self._drop_track()
            self.track_id = pid
            self.history.clear()

        self.last_seen = now
        self.history.append((now, person))

        # Trim to history_frames
        while len(self.history) > self.history_frames:
            self.history.popleft()

        return True

    def get_summary(self):
        """Return structured summary of recent history for LLM analysis.

        Returns dict with:
        - count: current squat count
        - track_id: tracked person ID
        - frame_count: number of frames in history
        - frames: list of dicts with key metrics per frame
        - trends: computed trends (bbox_height_change, posture_changes)
        """
        if not self.history:
            return {
                "count": self.count,
                "track_id": self.track_id,
                "frame_count": 0,
                "frames": [],
                "trends": {},
            }

        frames_summary = []
        bbox_heights = []
        postures = []

        for ts, person in self.history:
            bbox = person.get("bbox", [])
            bbox_h = (bbox[3] - bbox[1]) if len(bbox) >= 4 and all(math.isfinite(v) for v in bbox[:4]) else None
            posture = person.get("posture")
            posture_conf = person.get("posture_confidence")

            if bbox_h is not None:
                bbox_heights.append(bbox_h)
            if posture:
                postures.append(posture)

            frames_summary.append({
                "bbox_height": bbox_h,
                "posture": posture,
                "posture_confidence": posture_conf,
                "position": person.get("position"),
            })

        # Compute trends
        trends = {}
        if len(bbox_heights) >= 2:
            first_h = bbox_heights[0]
            last_h = bbox_heights[-1]
            min_h = min(bbox_heights)
            max_h = max(bbox_heights)
            trends["bbox_height_start"] = first_h
            trends["bbox_height_current"] = last_h
            trends["bbox_height_change"] = last_h - first_h  # >0 = taller (standing)
            trends["bbox_height_min"] = min_h
            trends["bbox_height_max"] = max_h
            trends["bbox_height_range"] = max_h - min_h  # larger = more squatting

        if len(postures) >= 2:
            trends["posture_first"] = postures[0]
            trends["posture_current"] = postures[-1]
            trends["posture_unique"] = list(set(postures))

        return {
            "count": self.count,
            "track_id": self.track_id,
            "frame_count": len(self.history),
            "frames": frames_summary,
            "trends": trends,
        }
