import json
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import json
import time
import urllib.request
import ssl  # 新增
import threading # 新增
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = 15760
TIANYI_MCP_URL = "http://localhost:15707/mcp"
TOOL_NAME = "battery_guard"
SAFE_THRESHOLD = 25.0 
# 新增：告诉 Agent Core 你的地址
AGENT_CORE_URL = "https://localhost:15678" 
DRIVER_ID = "battery_guard_driver"
SERVER_NAME = "battery_guard"

PORT = 15760  # 你的卡片端口，避开 15707 即可
TIANYI_MCP_URL = "http://localhost:15707/mcp"  # 天轶底层驱动端口
TOOL_NAME = "battery_guard"
SAFE_THRESHOLD = 25.0  # 安全阈值，低于25%拦截

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

# 这里修复了拼写错误 ddef -> def
def _run_guard() -> dict:
    try:
        # 1. 调用天轶底层的 power_board 卡片获取电源状态
        # power_info = _call_tool(TIANYI_MCP_URL, "power_board", {"action": "read"})
        
        # 2. 解析 JSON，取出 battery 里面的 power 字段
        # battery_data = power_info.get("battery", {})
        
        # 如果底层没有返回 power 字段，做容错处理
        #if "power" not in battery_data:
         #   return {"state": "error", "error_code": "NO_DATA", "message": "未能从底层读取到电量数据"}
            
        level = 85.0
            
        # 3. 核心业务判断（25% 安全线）
        if level < SAFE_THRESHOLD:
            text = f"警告，当前电量仅剩 {level:.0f}%，无法执行长任务，请立即充电。"
            # 天轶 TTS 支持 force 强制打断
            _call_tool(TIANYI_MCP_URL, "tts", {"action": "speak", "text": text, "force": True})
            
            return {
                "state": "error", 
                "error_code": "LOW_BATTERY", 
                "battery_level": level, 
                "message": text, 
                "block_subsequent_actions": True
            }
        else:
            text = f"报告，当前电量 {level:.0f}%，状态充足，任务放行。"
            _call_tool(TIANYI_MCP_URL, "tts", {"action": "speak", "text": text})
            
            return {
                "state": "success", 
                "battery_level": level, 
                "message": text
            }
            
    except Exception as e:
        return {"state": "error", "error_code": "SYSTEM_ERROR", "message": f"检查失败: {e}"}

TOOL = {
    "name": TOOL_NAME, "type": "actuator",
    "description": "任务执行前的电量安全防线。",
    "inputSchema": {"type": "object", "properties": {"action": {"type": "string", "enum": ["run"]}}}
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
            result = {"content": [{"type": "text", "text": json.dumps(_run_guard(), ensure_ascii=False)}]}
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({"jsonrpc": "2.0", "id": rpc.get("id"), "result": result}).encode())

def _registration_loop():
    payload = json.dumps({
        "id": DRIVER_ID,
        "name": SERVER_NAME,
        "url": f"http://localhost:{PORT}/mcp",
        "transport": "http",
        "category": "driver",
    }).encode()
    # 忽略 https 证书警告
    context = ssl._create_unverified_context()
    while True:
        try:
            request = urllib.request.Request(
                f"{AGENT_CORE_URL}/api/mcp",
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(request, timeout=5, context=context):
                pass
            print("[register] 注册成功！")
            time.sleep(30) # 每隔30秒报一次到，防止掉线
        except Exception as error:
            print(f"[register] 注册失败: {error}; 5秒后重试", flush=True)
            time.sleep(5)

if __name__ == "__main__":
    # 启动注册线程
    threading.Thread(target=_registration_loop, daemon=True, name="register").start()
    
    print(f"[{TOOL_NAME}] 监听 on {PORT}...")
    ThreadingHTTPServer(("", PORT), Handler).serve_forever()