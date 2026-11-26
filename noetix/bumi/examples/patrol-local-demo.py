"""Run the real local patrol card with explicitly synthetic input adapters."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from patrol_card import PatrolPlugin, PatrolTelemetry


class SyntheticCapture:
    def dispatch(self, action, args):
        if action == "info":
            return {"ok": True, "latest_frame_age_s": 0.0}
        if action == "capture_photo":
            return {"ok": True, "file_path": "synthetic:fixed-passage-frame",
                    "frame_captured_at": datetime.now(timezone.utc).isoformat(),
                    "timestamp_source": "synthetic_demo"}
        raise ValueError("unexpected capture action")


def main():
    parser = argparse.ArgumentParser(description="Synthetic local fixed-point patrol")
    parser.add_argument("--scenario", choices=["normal", "deviation", "low-battery", "raw-error"], default="normal")
    parser.add_argument("--report-dir", default="output/patrol-local-demo")
    options = parser.parse_args()
    telemetry = PatrolTelemetry()
    telemetry.receive("battery", {"soc": 5 if options.scenario == "low-battery" else 80, "alarm": 0})
    telemetry.receive("motion", {
        "fresh": True, "workmode": {"code": 2},
        "joint_states": [{"motor_id": i, "velocity": 0, "error": 1 if options.scenario == "raw-error" else 0}
                         for i in range(21)],
    })
    card = PatrolPlugin({"report_dir": options.report_dir}, "local", None,
                        SyntheticCapture(), telemetry=telemetry)
    started = card.dispatch("begin", {"run_id": "synthetic-demo"})
    output = {"environment": "synthetic_local_demo", "begin": started}
    if started["state"] == "active":
        inspection = card.dispatch("inspect", {"point_id": "fixed-passage"})
        decision = "deviation" if options.scenario == "deviation" else "normal"
        output["record"] = card.dispatch("record", {
            "inspection_id": inspection["inspection_id"], "check_id": "clear-walkway",
            "decision": decision, "assessment": "合成示例，通道有箱子占用。" if decision == "deviation"
            else "合成示例，指定通道连续可见且畅通。",
        })
        output["finish"] = card.dispatch("finish", {})
    print(json.dumps(output, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
