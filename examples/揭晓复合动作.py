#!/usr/bin/env python3
"""G1 颁奖复合动作示例 —— 「登台揭晓」

把 灯光 + 语音 + 手臂动作 串成一段复合动作。纯标准库，无第三方依赖。

安全设计：
  * 默认 --dry-run：只打印将要下发的 MCP 调用，绝不动机器人。
  * 加 --confirm 才真正执行；执行前先做只读健康检查（arm info）。
  * 机器人端运行（MCP 监听 127.0.0.1:15701）。真机执行必须人工旁站、空旷场地、急停在手。

用法：
    python3 揭晓复合动作.py                    # 彩排，只打印
    python3 揭晓复合动作.py --confirm          # 真机执行（谨慎）
    MCP_URL=http://127.0.0.1:15701/mcp python3 揭晓复合动作.py
"""
import argparse
import json
import os
import sys
import time
import urllib.request

MCP_URL = os.environ.get("MCP_URL", "http://127.0.0.1:15701/mcp")

GOLD = [
    {"r": 255, "g": 170, "b": 0, "duration_sec": 0.25},
    {"r": 0, "g": 0, "b": 0, "duration_sec": 0.15},
]
THEME = {"r": 180, "g": 140, "b": 40}


def mcp_call(name, arguments, dry_run=True, _id=[0]):
    _id[0] += 1
    payload = {
        "jsonrpc": "2.0",
        "id": _id[0],
        "method": "tools/call",
        "params": {"name": name, "arguments": arguments},
    }
    line = f"[{name}] {json.dumps(arguments, ensure_ascii=False)}"
    if dry_run:
        print("DRY ", line)
        return None
    print("CALL", line)
    req = urllib.request.Request(
        MCP_URL, data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=30) as resp:
        body = resp.read().decode()
    print("  ->", body[:400])
    return body


def reveal(name, award, pitch, dry_run=True):
    """一个获奖人的完整揭晓复合动作。"""
    tts = lambda text: mcp_call("tts", {"action": "speak", "text": text, "voice": 0}, dry_run)
    arm = lambda aid: mcp_call("arm", {"action": "execute", "action_id": aid}, dry_run)

    # 1) 悬念灯效：金色跑马灯，循环 3 轮
    mcp_call(
        "led",
        {"action": "set_sequence", "sequence": GOLD, "repeat_count": 3, "end_behavior": "hold"},
        dry_run,
    )
    # 2) 悬念台词（机器人口吻）
    tts(pitch)
    # 3) 揭晓亮点：双臂高举亮相（hands up = 15）
    arm(15)
    # 4) 念出获奖人
    tts(f"恭喜 {name}，获得{award}！")
    # 5) 庆祝：鼓掌（clap = 17）
    arm(17)
    # 6) 灯光回到主题色
    mcp_call("led", {"action": "set_once", **THEME}, dry_run)
    # 7) 手臂复位（release arm = 99）
    arm(99)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--confirm", action="store_true", help="真正执行（默认仅彩排打印）")
    ap.add_argument("--name", default="张三")
    ap.add_argument("--award", default="最佳创意奖")
    ap.add_argument(
        "--pitch",
        default="经过我零点三秒的高速运算，本届最佳创意奖得主是——",
    )
    args = ap.parse_args()

    dry_run = not args.confirm
    if dry_run:
        print("== 彩排模式（不会动机器人）。加 --confirm 才会执行。 ==\n")

    # 执行前只读健康检查
    mcp_call("arm", {"action": "info"}, dry_run)

    reveal(args.name, args.award, args.pitch, dry_run)

    if not dry_run:
        print("\n== 完成。如有异常立即：loco stop_move / 机身后退 / 遥控器急停 ==")


if __name__ == "__main__":
    sys.exit(main())
