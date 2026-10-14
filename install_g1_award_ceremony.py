#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Install the G1 award-ceremony skill into the agent-core skills DB.

Safe by design: makes a timestamped backup, replaces any existing entry with the
same slug (idempotent), then writes back. No robot motion is involved.
"""
import sqlite3
import json
import shutil
import time

DB_PATH = "/opt/phanthy-motus/data/data.db"

new_skill = {
    "slug": "g1-award-ceremony",
    "name": "G1 颁奖典礼主持",
    "description": "由主持人在画布发送文字指令，G1 播报颁奖开场、逐个揭晓奖项与获奖人并致庆祝语；经现场安全确认后可挥手庆祝、切换灯光主题色。不递送实物、不代领。",
    "oneLiner": "颁奖开场、揭晓播报与庆祝动作",
    "instruction": (
        "你是上海 G1 颁奖典礼主持助手。只处理颁奖主持，不记录或提交结果，不递送实物、不代领奖品。"
        "画布 remote_message 的文字是唯一输入来源；不得使用麦克风、ASR、相机、OCR、任何传感器、"
        "姿态/模式读取、定位或导航。\n\n"
        "可处理的指令：\n"
        "1. “开始颁奖”：调用 tts(action=\"speak\") 播报“颁奖典礼现在开始，请嘉宾就座，我们即将揭晓各个奖项。”"
        "不自行添加奖项、人员、时间或其他事实。\n"
        "2. “揭晓：<奖项>，<获奖人>”：先调用 tts 播报倒计时“三、二、一”，再播报“<奖项> 的获得者是 <获奖人>”。"
        "仅当操作员在本轮明确说明“机器人已稳定站立、周围无遮挡，可以动作”后，才可额外调用 "
        "arm(action=\"execute\", gesture=\"face wave\") 一次；只有确认后才执行挥手，未确认时只播报语音。"
        "动作失败时不要重试，仍完成语音播报。\n"
        "3. “庆祝：<奖项>”：调用 tts 播报“恭喜获奖！”，并在本轮已安全确认时额外挥手一次。\n"
        "4. “结束颁奖”：调用 tts 播报“颁奖典礼到此结束，感谢大家，明年再见。”不生成总结或名单。\n"
        "5. “主题色：<金/蓝/红>”：仅在本轮已安全确认时调用 led 设置对应主题色一次；失败不重试，不影响语音。\n\n"
        "安全与范围：只允许调用 tts、上述单次 arm 挥手和单次 led；禁止调用 loco、switch_mode、posture、"
        "smart_motion、mic、speaker、remote_mic、ASR、相机、OCR 或其他卡片。不得行走、转向、下蹲或改变站立模式。"
        "任何动作前必须先获得本轮明确的安全确认。"
    ),
    "category": "robot",
    "version": "1.0.0",
    "author": "",
    "installedAt": time.strftime("%Y-%m-%dT%H:%M:%S+08:00"),
    "active": True,
    "requiredTools": ["tts", "arm", "led"],
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
    print("OK: installed  :", new_skill["slug"])


if __name__ == "__main__":
    main()
