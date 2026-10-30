"""Deterministic simulated telemetry demo. Does not import ROS or a robot SDK."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
from telemetry_recorder_core import Recorder
from telemetry_recorder_store import ClipStore


def run(directory):
    now = [100.0]
    sources = {
        "imu": {"topic": "/SIMULATED/state/imu", "stale_s": .5},
        "joints": {"topic": "/SIMULATED/state/joints", "stale_s": .5},
        "battery": {"topic": "/SIMULATED/state/battery", "stale_s": 3},
        "motion_events": {"topic": "/SIMULATED/safety/motion_events", "event_stream": True},
    }
    recorder = Recorder(sources, clock=lambda: now[0], wall=lambda: 1793062800 + now[0] - 100)
    store = ClipStore(str(Path(directory).resolve()))
    store.prepare()
    recorder.start()
    key = None
    for step in range(321):
        t = step / 20
        now[0] = 100 + t
        # A deliberate 1.5s IMU gap demonstrates data coverage reporting.
        if not 8 <= t < 9.5:
            recorder.ingest_json("imu", json.dumps({"rpy": [.04 * math.sin(t), .02 * math.cos(t), 0]}))
        if step % 2 == 0:
            recorder.ingest_json("joints", json.dumps({"joints": [{"idx": 0, "q": .1,
                                          "dq": 0, "tau": 0, "temp": [42 + t / 10]}]}))
        if step % 20 == 0:
            recorder.ingest_json("battery", json.dumps({"soc": 80, "current": 100}))
        if step == 200:
            recorder.ingest_json("motion_events", '{"type":"DEMO_MARKER","reason":"simulated only"}')
            key = recorder.capture("SIMULATED DEMO — not a robot test", 10, 5)["clip_id"]
    docs = recorder.drain_ready()
    result = store.write(docs[0])
    recorder.stop()
    assert result["clip_id"] == key
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, help="Dedicated local demo directory")
    args = parser.parse_args()
    print(json.dumps(run(args.output_dir), ensure_ascii=False, indent=2))
