#!/usr/bin/env python3
"""Read-only Mac/Ubuntu MCP acceptance. Optional photos; never enables or moves."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import subprocess
import time
import urllib.request


def rpc(endpoint, method, params=None):
    request = urllib.request.Request(endpoint, data=json.dumps({"jsonrpc":"2.0","id":1,
        "method":method,"params":params or {}}).encode(), headers={"Content-Type":"application/json"})
    with urllib.request.urlopen(request, timeout=8) as response:
        body = json.load(response)
    if "error" in body:
        raise RuntimeError(body["error"]["message"])
    result = body["result"]
    if result.get("isError"):
        raise RuntimeError(result["content"][0]["text"])
    return result


def call(endpoint, name, action, **args):
    result = rpc(endpoint,"tools/call",{"name":name,"arguments":{"action":action,**args}})
    return json.loads(result["content"][0]["text"])


def finish_photo(endpoint, result):
    if "action_id" not in result:
        return result
    deadline = time.monotonic()+20
    while time.monotonic() < deadline:
        info = call(endpoint,"piper_photo","info")
        completed = info.get("last_action") or {}
        if completed.get("action_id") == result["action_id"] and completed.get("status") != "running":
            if completed["status"] != "completed":
                raise RuntimeError(str(completed))
            return completed["result"]
        time.sleep(.2)
    raise RuntimeError("照片完成状态超时，请单独停止 photo 并检查 info")


def host_check(command):
    try:
        proc = subprocess.run(command,capture_output=True,text=True,timeout=5)
        return {"returncode":proc.returncode,"output":(proc.stdout+proc.stderr)[-4000:]}
    except (OSError,subprocess.TimeoutExpired) as exc:
        return {"error":str(exc)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint",default="http://localhost:15741/mcp")
    parser.add_argument("--output",type=Path,default=Path("piper-diagnosis.json"))
    parser.add_argument("--photos",type=int,choices=range(4),default=0)
    args = parser.parse_args()
    report = {"time":datetime.now(timezone.utc).isoformat(),"architecture":platform.machine(),
              "can":host_check(["ip","-details","link","show","can0"]),
              "containers":host_check(["docker","ps","--format","{{.Names}} {{.Image}}"]),
              "control_commands_requested":0,"photos":[]}
    errors = {}
    def check(key, operation):
        try:
            report[key] = operation()
            return report[key]
        except Exception as exc:
            errors[key] = str(exc)
            return None

    check("server", lambda: rpc(args.endpoint,"initialize"))
    def check_cards():
        cards = [t["name"] for t in rpc(args.endpoint,"tools/list")["tools"]]
        expected = {"arm_control", "piper_photo"}
        if set(cards) != expected:
            raise RuntimeError("控制卡与相机未完整注册；请核对本版本工具清单")
        return cards
    check("cards", check_cards)
    check("before", lambda: call(args.endpoint,"arm_control","read"))
    for index in range(args.photos):
        photo = check(f"photo_{index+1}", lambda: finish_photo(args.endpoint,call(args.endpoint,"piper_photo","capture")))
        if photo is not None:
            report["photos"].append(photo)
    check("after", lambda: call(args.endpoint,"arm_control","read"))
    def check_unchanged():
        if not report.get("before") or not report.get("after"):
            raise RuntimeError("缺少新鲜状态，无法确认只读期间模式和使能未变化")
        for field in ("control_mode","enabled","teach_status"):
            if report["before"][field] != report["after"][field]:
                raise RuntimeError(f"只读验收期间 {field} 发生变化，需现场核查")
        if report["after"]["motion_enabled"]:
            raise RuntimeError("只读验收要求 PIPER_MOTION_ENABLED=0")
        return True
    check("unchanged", check_unchanged)
    report.update(passed=not errors, errors=errors)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(f"{'PASS' if report['passed'] else 'FAIL'}: {args.output}")
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
