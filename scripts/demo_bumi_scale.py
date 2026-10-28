#!/usr/bin/env python3
"""Run the real quick-scale workflow with synthetic joints, without a robot."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import tempfile
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "noetix" / "bumi"))
from payload_mass import PayloadMassPlugin
from payload_mass_sampling import SharedSampler


class SyntheticArm:
    def __init__(self):
        self._action_lock, self._lock = threading.RLock(), threading.Lock()
        self._payload_owner = None
        self._high_ctrl = self
        self.mode, self.elbow, self.mass = 2, 0., 0.

    def get_mode(self):
        return self.mode

    def is_moving(self):
        return False

    def _preempt_servo(self, reason):
        return False

    def _do_teaching_action(self, action, args):
        self.mode, self.elbow = 23, -1.2
        return {"command_sent": True, "workmode": 23, "confirmed_started": True}

    def _do_stop_playback(self, reason):
        self.mode, self.elbow = 2, 0.
        return {"confirmed": True}

    def frame(self, seq):
        q = [0.] * 21; q[3] = self.elbow
        return {"motor_ids": list(range(21)), "q": q, "dq": [0.] * 21,
                "tau": [1+2*self.mass, -.5-self.mass, .2, .7+.6*self.mass] + [0.] * 17,
                "temperature": [30.] * 21, "errors": [0] * 21,
                "quaternion_xyzw": [0, 0, 0, 1], "gyro": [0, 0, 0],
                "workmode": self.mode, "source_seq": seq,
                "source_timestamp": None, "adapter": "SYNTHETIC_ONLY"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=None, help="directory for simulated trials and result JSON")
    args = parser.parse_args()
    root = Path(args.output or tempfile.mkdtemp(prefix="bumi-scale-demo-"))
    root.mkdir(parents=True, exist_ok=True)
    arm, sampler, halted = SyntheticArm(), SharedSampler(), threading.Event()
    events, last = [], [None]
    def publish(result):
        item = {"simulation_only": True, "state": result["state"], "mass_kg": result.get("mass_kg"),
                "stable": result.get("stable", False), "valid": result.get("valid", False)}
        key = (item["state"], item["mass_kg"], item["stable"])
        if key != last[0]:
            events.append(item); print(json.dumps(item), flush=True); last[0] = key
    card = PayloadMassPlugin({"setup_mode": "quick", "recording_id": 12, "data_dir": str(root / "trials")},
                             "simulation", sampler=sampler, loco_plugin=arm, publish=publish, notify=lambda *a: None)
    def feed():
        seq = 0
        while not halted.is_set():
            seq += 1; sampler.push(arm.frame(seq)); halted.wait(.02)
    worker = threading.Thread(target=feed, daemon=True); worker.start()
    def wait_for(predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with card._lock:
                result = dict(card._last)
            if result["state"] == "error":
                raise RuntimeError(result)
            if predicate(result):
                return result
            halted.wait(.02)
        raise TimeoutError(card._last)
    try:
        while not sampler.diagnostics()["received_count"]:
            halted.wait(.01)
        card.dispatch("prepare", {})
        wait_for(lambda r: r["state"] == "ready")
        card.dispatch("weigh", {})
        arm.mass = .2
        wait_for(lambda r: r.get("sampling_stable") and r.get("trial_file"))
        calibration = card.dispatch("calibrate", {"known_mass_kg": .2})
        if calibration["state"] != "calibrated":
            raise RuntimeError(calibration)
        wait_for(lambda r: r.get("valid") and r.get("mass_kg") == .2)
        for mass in (.25, .3, .35):
            arm.mass = mass; halted.wait(.3)
        measured = wait_for(lambda r: r.get("valid") and r.get("mass_kg") == .35)
        arm.mass = 0
        empty = wait_for(lambda r: r.get("valid") and r.get("mass_kg") == 0)
        card.dispatch("stop", {})
        home = wait_for(lambda r: r.get("home_confirmed"))
        report = {"simulation_only": True, "reference_mass_kg": .2, "unknown_mass_kg": measured["mass_kg"],
                  "unloaded_mass_kg": empty["mass_kg"], "home_confirmed": home["home_confirmed"], "events": events}
        (root / "result.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))
        print(json.dumps({"simulation_only": True, "report": str(root / "result.json")}))
    finally:
        card.stop(); halted.set(); worker.join(1)


if __name__ == "__main__":
    main()
