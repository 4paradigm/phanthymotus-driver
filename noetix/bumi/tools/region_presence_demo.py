"""Offline acceptance using synthetic VOP frames; no ROS, model or robot."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from common.region_presence import PresenceMonitor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    monitor, rows, events = PresenceMonitor(), [], []
    cases = [(0, 0, "初始化无人"), (1.5, 0, "确认无人"),
             (2, 1, "目标出现，开始确认"), (2.7, 1, "确认出现"),
             (3, 0, "短暂漏检"), (3.3, 1, "目标恢复，不重复出现事件"),
             (4, 0, "开始确认离开"), (5.5, 0, "确认离开"),
             (9, None, "断流，不能当作无人"),
             (10, 1, "数据恢复，重新确认"), (10.7, 1, "恢复后初始化在场，不宣称新来客")]
    for index, (now, count, title) in enumerate(cases):
        if count is None:
            emitted = monitor.poll(now)
        else:
            objects = [{"name": "person", "confidence": .9, "position": [0., 0.]} for _ in range(count)]
            emitted = monitor.ingest({"timestamp": 1000+index, "count": count, "objects": objects}, now)
        events.extend(emitted)
        rows.append({"scenario": title, "at_s": now, "events": [e["event"] for e in emitted],
                     "observation": monitor.snapshot(now)})
    expected = ["initialized", "appeared", "disappeared", "unavailable", "initialized"]
    passed = [e["event"] for e in events] == expected and rows[8]["observation"]["present"] is None
    report = {"passed": passed, "synthetic_vop_data": True, "robot_connected": False,
              "physical_motion_requested": False, "events": events, "steps": rows}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# region_presence 离线演示", "", "合成 VOP 数据；未连接机器人，未运行识别模型。", "",
             "| 情况 | 查询状态 | present | 事件 |", "| --- | --- | --- | --- |"]
    for row in rows:
        obs = row["observation"]
        lines.append(f"| {row['scenario']} | {obs['state']} | {obs['present']} | {', '.join(row['events']) or '—'} |")
    lines.extend(["", f"验收：{'通过' if passed else '失败'}。", ""])
    (args.output_dir/"report.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k not in ("events", "steps")}, ensure_ascii=False))
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
