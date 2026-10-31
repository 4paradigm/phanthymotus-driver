import json
import time
import urllib.request
import ssl
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = 15760
TIANYI_MCP_URL = "http://localhost:15707/mcp"
TOOL_NAME = "battery_guard"
AGENT_CORE_URL = "https://localhost:15678" 
DRIVER_ID = "battery_guard_driver"
SERVER_NAME = "battery_guard"

def _rpc(url: str, method: str, params: dict) -> dict:
    payload = json.dumps({"jsonrpc": "2.0", "id": int(time.time() * 1000), "method": method, "params": params}).encode()
    req = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=5.0) as res:
        return json.load(res).get("result", {})

def _call_tool(url: str, name: str, arguments: dict) -> dict:
    result = _rpc(url, "tools/call", {"name": name, "arguments": arguments})
    content = result.get("content") or []
    text = content[0].get("text", "{}") if content else "{}"
    try: return json.loads(text)
    except: return {"raw": text}

def _run_guard(args: dict) -> dict:
    try:
        # 接收大模型传入的任务所需最低电量
        required = float(args.get("required_battery", 20.0))
        
        level = None
        source = "unknown"
        
        # 优先从 battery 卡片读取真实电量
        try:
            battery_info = _call_tool(TIANYI_MCP_URL, "battery", {"action": "read"})
            if "master_power" in battery_info:
                level = float(battery_info["master_power"])
                source = "battery_card"
        except Exception as e:
            print(f"读取 battery 卡片失败: {e}")

        # 兜底从 power_board 读取
        if level is None:
            try:
                power_info = _call_tool(TIANYI_MCP_URL, "power_board", {"action": "read"})
                if "battery" in power_info and "power" in power_info["battery"]:
                    level = float(power_info["battery"]["power"])
                    source = "power_board_card"
            except Exception as e:
                print(f"读取 power_board 卡片失败: {e}")

        # 彻底无数据的安全拦截
        if level is None:
            msg = f"警告，底层未上报电量数据，安全起见禁止执行任务。"
            _call_tool(TIANYI_MCP_URL, "tts", {"action": "speak", "text": msg, "force": True})
            return {"state": "error", "error_code": "NO_BATTERY_DATA", "message": msg, "block_subsequent_actions": True}

        print(f"✅ 真实电量: {level}% (来源: {source})，任务所需: {required}%")

        # 动态对比逻辑
        if level < required:
            text = f"警告，当前电量 {level:.0f}%，低于该任务所需的安全线 {required:.0f}%，任务拦截，请先充电。"
            _call_tool(TIANYI_MCP_URL, "tts", {"action": "speak", "text": text, "force": True})
            return {
                "state": "error", 
                "error_code": "INSUFFICIENT_BATTERY", 
                "battery_level": level, 
                "required_battery": required,
                "message": text, 
                "block_subsequent_actions": True
            }
        else:
            text = f"电量充足（当前 {level:.0f}%，满足所需 {required:.0f}%），任务放行。"
            _call_tool(TIANYI_MCP_URL, "tts", {"action": "speak", "text": text})
            return {"state": "success", "battery_level": level, "message": text}
            
    except Exception as e:
        return {"state": "error", "error_code": "SYSTEM_ERROR", "message": f"检查失败: {e}"}

# 注意这里增加了 required_battery
TOOL = {
    "name": TOOL_NAME, "type": "actuator",
    "description": "长任务执行前的电量安全防线。动态评估所需电量，不足则拦截。",
    "inputSchema": {
        "type": "object", 
        "properties": {
            "action": {"type": "string", "enum": ["run"]},
            "required_battery": {"type": "number", "description": "任务所需的最低电量，如短途15，长途35"}
        },
        "required": ["action"]
    }
}

class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        rpc = json.loads(self.rfile.read(length))
        result = {}
        if rpc.get("method") == "initialize":
            result = {"protocolVersion": "2024-11-05", "capabilities": {}, "serverInfo": {"name": TOOL_NAME}}
        elif rpc.get("method") == "tools/list":
            result = {"tools": [TOOL]}
        elif rpc.get("method") == "tools/call":
            args = rpc.get("params", {}).get("arguments", {})
            result = {"content": [{"type": "text", "text": json.dumps(_run_guard(args), ensure_ascii=False)}]}
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({"jsonrpc": "2.0", "id": rpc.get("id"), "result": result}).encode())

def _registration_loop():
    payload = json.dumps({"id": DRIVER_ID, "name": SERVER_NAME, "url": f"http://localhost:{PORT}/mcp", "transport": "http", "category": "driver"}).encode()
    context = ssl._create_unverified_context()
    while True:
        try:
            request = urllib.request.Request(f"{AGENT_CORE_URL}/api/mcp", data=payload, headers={"Content-Type": "application/json"}, method="POST")
            with urllib.request.urlopen(request, timeout=5, context=context): pass
            print("[register] 注册成功！")
            time.sleep(30)
        except Exception as error:
            print(f"[register] 注册失败: {error}; 5秒后重试", flush=True)
            time.sleep(5)

if __name__ == "__main__":
    threading.Thread(target=_registration_loop, daemon=True, name="register").start()
    print(f"[{TOOL_NAME}] 监听 on {PORT}...")
    ThreadingHTTPServer(("", PORT), Handler).serve_forever()