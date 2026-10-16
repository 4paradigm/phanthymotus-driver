#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Install the G1 award-ceremony skill into the agent-core skills DB.

Safe by design: makes a timestamped backup, replaces any existing entry with the
same slug (idempotent), then writes back. No robot motion is involved.

This skill mirrors the `award_ceremony` card's behaviour so both produce the same
show: opening line + wave, then per award  name -> (praise) -> turn -> clap ->
turn back, at volume 60, each gesture called exactly once, teleop released first.
"""
import sqlite3
import json
import shutil
import time

DB_PATH = "/opt/phanthy-motus/data/data.db"

INSTRUCTION = (
    "你是上海 G1 颁奖典礼主持助手。只处理颁奖主持，不递送实物、不代领奖品。"
    "画布 remote_message 的文字是唯一输入来源；不得使用麦克风、ASR、相机、OCR、定位或导航。\n\n"

    "【输入格式】一条消息：\n"
    "一键颁奖：<奖项1>=<获奖人1>|<表扬词1>；<奖项2>=<获奖人2>|<表扬词2>；……\n"
    "奖项数量随文本而定（有几个播几个），表扬词可省略。只按分号“；”分隔奖项，表扬词里的逗号不影响。\n\n"

    "【开场】按顺序，每步只调用一次：\n"
    "1. 调用 teleop_control(action=\"stop\") 结束遥操，拿回运动控制权（若该工具不可用则跳过）。\n"
    "2. 调用 tts(action=\"set_volume\", volume=60)。\n"
    "3. 调用 tts(action=\"speak\")，播报“颁奖典礼现在开始，请嘉宾就座，我们即将揭晓各个奖项。”\n"
    "4. 调用 arm(action=\"execute\", gesture=\"high wave\") 挥手一次（只一次）。\n\n"

    "【每个奖项】严格按顺序，每步只调用一次，不得省略、不得换序：\n"
    "1. 调用 tts(action=\"speak\") 播报“三、二、一”。\n"
    "2. 调用 tts(action=\"speak\") 播报“<奖项> 的获得者是 <获奖人>”。\n"
    "3. 若有表扬词，调用 tts(action=\"speak\") 播报表扬词。\n"
    "4. 调用 loco(action=\"move\", vyaw=0.5, duration=3.0)，再 loco(action=\"stop_move\")，转身面向获奖者。\n"
    "5. 调用 arm(action=\"execute\", gesture=\"clap\") 鼓掌一次。\n"
    "6. 调用 loco(action=\"move\", vyaw=-0.5, duration=3.0)，再 loco(action=\"stop_move\")，转回正前方。\n"
    "完成第6步后才进入下一个奖项。\n\n"

    "【结束】全部奖项完成后，调用 tts(action=\"speak\") 播报“颁奖典礼到此结束，感谢大家，明年再见。”\n\n"

    "【重要】每个 arm/loco/tts 动作只调用一次。arm 的返回码不可靠，非 0 也可能已经执行，"
    "禁止重试，否则动作会重复播放。\n\n"

    "【范围】只允许调用 tts、arm、loco、teleop_control、led；禁止调用 switch_mode、switch_mode_expert、"
    "posture、smart_motion、mic、speaker、remote_mic、ASR、相机、OCR 或其他卡片。"
)

new_skill = {
    "slug": "g1-award-ceremony",
    "name": "G1 颁奖典礼主持",
    "description": "与 award_ceremony 卡片同效果的颁奖流程：开场播报后挥手；每个奖项 念名字→(表扬词)→转身面向获奖者→鼓掌→转回正前方；音量 60，动作各调用一次。",
    "oneLiner": "开场挥手 + 每项 念名字→转身→鼓掌→转回（任意奖项数）",
    "instruction": INSTRUCTION,
    "category": "robot",
    "version": "3.0.0",
    "author": "",
    "installedAt": time.strftime("%Y-%m-%dT%H:%M:%S+08:00"),
    "active": True,
    "requiredTools": ["tts", "arm", "loco", "teleop_control", "led"],
    "configSchema": {},
    "icon": "🏆",
}


def main():
    conn = sqlite3.connect(DB_PATH)
    row = conn.execute("SELECT value FROM config WHERE key='skills'").fetchone()
    if not row:
        conn.close()
        raise SystemExit("ERROR: config key='skills' not found in " + DB_PATH)

    skills = json.loads(row[0])

    backup = DB_PATH + ".bak-g1-award-ceremony-" + time.strftime("%Y%m%d-%H%M%S")
    shutil.copy2(DB_PATH, backup)

    before = len(skills.get("installed", []))
    skills["installed"] = [
        s for s in skills.get("installed", []) if s.get("slug") != new_skill["slug"]
    ]
    skills["installed"].append(new_skill)
    after = len(skills["installed"])

    conn.execute(
        "UPDATE config SET value=? WHERE key='skills'",
        (json.dumps(skills, ensure_ascii=False),),
    )
    conn.commit()
    conn.close()

    print("backup        :", backup)
    print("installed cnt :", before, "->", after)
    print("OK: installed  :", new_skill["slug"], new_skill["version"])


if __name__ == "__main__":
    main()
