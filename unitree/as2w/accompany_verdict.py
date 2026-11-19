"""Offline verdict for the accompany-mode reconnaissance evidence.

Why this file exists
--------------------
``accompany_probe.py`` runs on the robot and dumps raw JSON
(``step1.json`` services/modes, ``step2.json`` watch, ``step3.json``
probe-ids).  Interpreting those dumps by hand against the verdict table in
``ACCOMPANY_FINDINGS.md`` §5 is error-prone, so this script does it
mechanically.  It runs on any laptop: pure stdlib, no DDS, no SDK.

Usage:

    python3 accompany_verdict.py /tmp/step1.json /tmp/step2.json
    python3 accompany_verdict.py /tmp/step3.json --json /tmp/verdict.json

All decision constants (the 14 documented modes, the known fsm names, the
suspicious-name regex) are imported from ``accompany_probe`` so the probe and
the verdict can never drift apart.

The verdict table it implements (ACCOMPANY_FINDINGS.md §5):

| evidence                                   | verdict          | next step                        |
|--------------------------------------------|------------------|----------------------------------|
| suspicious rt/api/* service in step 1      | H3               | read-only api probing of service |
| accompany-like name in CheckMode output    | H3 (cheaper)     | SelectMode(<name>) to verify     |
| step 2 ``mode`` outside the 14-entry table | independent mode | reverse-lookup via switcher      |
| step 2 ``fsm_name`` never seen before      | H1               | fill config.yaml fsm_names       |
| step 2 shows nothing new on both channels  | invisible        | only H2 / vendor channel remain  |
| step 3 api id returns 0                    | H2 candidate     | reproduce on the robot 3x first  |
"""

import argparse
import json
import re
import sys

from accompany_probe import (  # noqa: WPS433 - same-folder module
    MODE_NAMES,
    _SUSPECT,
    api_services,
    is_new_fsm_name,
    suspicious,
    undocumented_mode,
)

#: A step-3 return code starting with "4" means "unknown id / no permission"
#: per the SDK convention; anything else that is not 0 is reported verbatim.
_CODE_OK = 0


# ------------------------------------------------------------ pure helpers

def _checkmode_text(data):
    """Serialise the CheckMode payload so name matching works on any shape."""
    if data is None:
        return ""
    return json.dumps(data, ensure_ascii=False, default=str)


def _suspicious_tokens(text):
    """Distinct words in ``text`` that match the suspicious-name regex."""
    return sorted({
        token for token in re.findall(r"[A-Za-z_]+", text or "")
        if _SUSPECT.search(token)
    })


def verdict_step1(data):
    """Interpret the ``--services --modes`` dump (H3 evidence)."""
    findings = []
    services = data.get("services")
    if not services and data.get("endpoints"):
        services = api_services(data.get("endpoints"))
    for name in suspicious(services or {}):
        hosted = bool((services or {}).get(name, {}).get("request"))
        suffix = "" if hosted else "（仅见 /response 端，机器人未必 host 该服务）"
        findings.append({
            "step": 1,
            "signal": f"可疑服务 rt/api/{name}/*{suffix}",
            "verdict": "H3 成立",
            "next": f"对服务 {name} 做 CheckMode 式只读 api 探测（从 api id 1001 起），"
                    "找到只读接口后再谈写入",
        })
    text = _checkmode_text((data.get("motion_switcher") or {}).get("data"))
    hits = _suspicious_tokens(text)
    if hits:
        findings.append({
            "step": 1,
            "signal": f"motion_switcher.CheckMode 返回里出现伴随相关名称 {hits}",
            "verdict": "H3 成立（更省事）",
            "next": "用 SelectMode(<该名称>) 验证能否切换；SelectMode 是写操作，"
                    "验证时机器人应有人看护",
        })
    return findings


def verdict_step2(data):
    """Interpret the ``--watch`` dump (H1 / independent-mode evidence)."""
    watch = data.get("watch") or {}
    findings = []
    modes = watch.get("modes") or {}
    odd_modes = sorted((key for key in modes if undocumented_mode(key)),
                       key=lambda k: (not str(k).lstrip("-").isdigit(), str(k)))
    for key in odd_modes:
        findings.append({
            "step": 2,
            "signal": f"mode={key} 超出官方 {len(MODE_NAMES)} 项枚举"
                      f"（观测帧数 {modes[key]}）",
            "verdict": "伴随是一个独立 mode，几乎可以确定厂商未公开",
            "next": "用 motion_switcher（CheckMode/SelectMode）反查该 mode 的名称；"
                    "§3.3 表明切族入口就是它",
        })
    fsm_names = watch.get("fsm_names") or {}
    new_names = sorted(name for name in fsm_names if is_new_fsm_name(name))
    pairs = watch.get("pairs") or {}
    for name in new_names:
        mode_values = sorted({
            pair.split(":", 1)[0] for pair in pairs
            if pair.rsplit(":", 1)[-1].strip().upper() == str(name).strip().upper()
        })
        suffix = f"，同帧出现的 mode={mode_values}" if mode_values else ""
        findings.append({
            "step": 2,
            "signal": f"出现本仓库从未见过的 fsm_name={name}"
                      f"（观测帧数 {fsm_names[name]}）{suffix}",
            "verdict": "H1 成立：伴随态可观测",
            "next": f"把 {name} 填进 config.yaml 的 accompany.fsm_names，"
                    "卡片即从 inferred 升为 confirmed——但这只是「看见」，"
                    "不解决「控制」",
        })
    if not findings:
        findings.append({
            "step": 2,
            "signal": "全部 mode 均在官方枚举内且 fsm_name 均为已知名"
                      f"（共 {len(modes)} 个 mode、{len(fsm_names)} 个 fsm_name）",
            "verdict": "伴随态对 mode / fsm_name 两通道均不可见",
            "next": "只剩 H2（--probe-ids 空号探测，必须三道安全开关齐全）"
                    "与厂商渠道（提工单/索要接口文档）",
        })
    return findings


def verdict_step3(data):
    """Interpret the ``--probe-ids`` dump (H2 evidence). DANGEROUS input."""
    results = (data.get("probe") or {})
    findings = []
    for id_text, entry in sorted(results.items(), key=lambda kv: str(kv[0])):
        code = (entry or {}).get("code")
        if code == _CODE_OK or str(code) == str(_CODE_OK):
            findings.append({
                "step": 3,
                "signal": f"api id {id_text} 对空参数返回 code=0",
                "verdict": "H2 候选：该 id 存在且被服务端接受",
                "next": "红线：先别写进卡。需观察机器人当时是否动了，"
                        "并在真机复现 3 次以上确认语义后才能落卡",
            })
        else:
            findings.append({
                "step": 3,
                "signal": f"api id {id_text} 返回 code={code}",
                "verdict": "该 id 不存在或无权限，排除",
                "next": "无需跟进；如为 4xxx 属预期失败",
            })
    return findings


def verdict(data):
    """All findings for a merged evidence dict, in report order."""
    findings = []
    if any(key in data for key in ("services", "endpoints", "motion_switcher")):
        findings.extend(verdict_step1(data))
    if "watch" in data:
        findings.extend(verdict_step2(data))
    if "probe" in data:
        findings.extend(verdict_step3(data))
    return findings


def overall(findings):
    """One-line bottom line, mirroring the doc's escalation order."""
    verdicts = [item["verdict"] for item in findings]
    for prefix in ("H3", "伴随是一个独立 mode", "H1", "H2"):
        for item in findings:
            if item["verdict"].startswith(prefix):
                return f"{item['verdict']} → {item['next']}"
    for item in findings:
        if item["verdict"].startswith("伴随态对"):
            return f"{item['verdict']} → {item['next']}"
    if findings:
        return "；".join(verdicts)
    return "取证文件为空或不包含可判读的数据（需要 step1/step2/step3 之一）"


def merge_files(paths):
    """Merge several probe JSON dumps into one dict (later files win)."""
    merged = {}
    for path in paths:
        with open(path, "r", encoding="utf-8") as handle:
            merged.update(json.load(handle))
    return merged


# --------------------------------------------------------------------- main

def build_parser():
    parser = argparse.ArgumentParser(
        description="As2-W 伴随模式取证判读（离线，纯标准库）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("json_files", nargs="+", metavar="EVIDENCE_JSON",
                        help="accompany_probe.py 产出的 JSON，可给多个")
    parser.add_argument("--json", default="", metavar="PATH",
                        help="把判读结果写入 JSON")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    data = merge_files(args.json_files)
    findings = verdict(data)
    print(f"[verdict] 读取 {len(args.json_files)} 个取证文件："
          f"{', '.join(args.json_files)}")
    if not findings:
        print("[verdict] 没有找到可判读的数据（endpoints/services/"
              "motion_switcher/watch/probe 均缺失）")
    for item in findings:
        print(f"[verdict] 🟢 步骤{item['step']}：{item['signal']}")
        print(f"          结论：{item['verdict']}")
        print(f"          下一步：{item['next']}")
    bottom = overall(findings)
    print("[verdict] -------- 总结论 --------")
    print(f"[verdict] {bottom}")
    if args.json:
        payload = {
            "evidence_files": args.json_files,
            "findings": findings,
            "overall": bottom,
        }
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
        print(f"[verdict] 判读已写入 {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
