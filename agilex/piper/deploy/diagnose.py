#!/usr/bin/env python3
"""Production PiPER passive health check. Optional photos; no arm controls."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import subprocess
import time
import urllib.request
import xml.etree.ElementTree as ET


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


def call(endpoint, name, action=None, **args):
    arguments = args if action is None else {"action": action, **args}
    result = rpc(endpoint,"tools/call",{"name":name,"arguments":arguments})
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


def diagnose(endpoint, photos=0):
    """Use production Info/resource queries; never Start, reconnect or Stop.

    A deployment gate set to 1 is reported, not changed or treated as evidence
    that this diagnostic submitted a control. This is not motion acceptance.
    """
    report = {"time":datetime.now(timezone.utc).isoformat(),"architecture":platform.machine(),
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

    check("server", lambda: rpc(endpoint,"initialize"))
    def check_cards():
        cards = [t["name"] for t in rpc(endpoint,"tools/list")["tools"]]
        expected = {"arm_control", "model", "piper_photo", "piper_camera_live"}
        if set(cards) != expected:
            raise RuntimeError(f"Production tools mismatch: expected {sorted(expected)}, received {sorted(cards)}")
        return cards
    cards = check("cards", check_cards)
    def snapshot(key):
        info = report.get(key)
        state = info.get("snapshot") if isinstance(info, dict) else None
        if not isinstance(state, dict) or state.get("connected") is not True or state.get("fresh") is not True:
            raise RuntimeError(f"{key}: feedback is disconnected, stale or unavailable; no reconnect was requested")
        if not isinstance(state.get("enabled"), (list, tuple)) or len(state["enabled"]) != 6:
            raise RuntimeError(f"{key}: six motor-enable feedback values are unavailable")
        if not isinstance(state.get("faults"), dict) or "teach_status" not in state["faults"]:
            raise RuntimeError(f"{key}: controller feedback is unavailable")
        return state
    def check_model():
        model = call(endpoint, "model")
        robot = ET.fromstring(model["urdf"])
        names = {joint.attrib.get("name") for joint in robot.findall("joint")}
        if robot.tag != "robot" or not {f"joint{i}" for i in range(1, 7)} <= names:
            raise RuntimeError("PiPER six-joint URDF is unavailable")
        return {"robot": robot.attrib.get("name"), "six_axis_joints": sorted(n for n in names if n in {f"joint{i}" for i in range(1, 7)})}
    # Reject a mismatched interface before asking it to perform any operation.
    if cards is not None:
        check("before", lambda: call(endpoint,"arm_control","info"))
        before = check("feedback", lambda: snapshot("before"))
        if before is not None:
            interface = before.get("can_interface")
            if isinstance(interface, str) and interface:
                report["can"] = host_check(["ip","-details","link","show",interface])
        check("model", check_model)
        check("camera", lambda: call(endpoint, "piper_camera_live", "info"))
        for index in range(photos):
            photo = check(f"photo_{index+1}", lambda: finish_photo(endpoint,
                         call(endpoint,"piper_photo","capture", capture_confirmed=True)))
            if photo is not None:
                report["photos"].append(photo)
        check("after", lambda: call(endpoint,"arm_control","info"))
    def check_unchanged():
        before, after = snapshot("before"), snapshot("after")
        for field in ("control_mode","enabled","motion_enabled"):
            if before.get(field) != after.get(field):
                raise RuntimeError(f"{field} changed during the passive check; inspect the actual robot")
        if before["faults"]["teach_status"] != after["faults"]["teach_status"]:
            raise RuntimeError("teach_status changed during the passive check; inspect the actual robot")
        return True
    if cards is not None:
        check("unchanged", check_unchanged)
    report.update(passed=not errors, errors=errors)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint",default="http://localhost:15741/mcp")
    parser.add_argument("--output",type=Path,default=Path("piper-diagnosis.json"))
    parser.add_argument("--photos",type=int,choices=range(4),default=0,
                        help="Explicitly authorize 0-3 photo captures; default only queries Info and model")
    # Firmware querying is not a public production action; do not invoke
    # teaching exit (a control) just to obtain a firmware string.
    args = parser.parse_args()
    report = diagnose(args.endpoint, args.photos)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(f"{'PASS' if report['passed'] else 'FAIL'}: {args.output}")
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
