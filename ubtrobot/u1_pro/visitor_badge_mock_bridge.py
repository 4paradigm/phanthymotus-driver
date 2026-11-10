#!/usr/bin/env python3
"""访客贴桥接服务的模拟版：不连范式+、不连打印机，用来联调 visitor_badge 卡片。

它实现的接口就是真桥接服务要实现的接口（见 README.md「访客贴打印」）：
    GET  /health
    GET  /visitors?keyword=...
    POST /print      {"visitor_id": "...", "reprint": false}
    GET  /jobs/<job_id>

运行：
    python3 visitor_badge_mock_bridge.py --port 8765 --token test-token
然后在卡片配置里填 bridge_url=http://<这台电脑IP>:8765、token=test-token。
"打印"只是在终端打一行日志，约 1 秒后任务变成 done。
"""

from __future__ import annotations

import argparse
import json
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

SAMPLE_VISITORS = [
    {"id": "V1001", "name": "王磊", "company": "星河科技", "host": "张三", "visit_time": "10:00", "phone_tail": "1234"},
    {"id": "V1002", "name": "李娜", "company": "云帆咨询", "host": "李臻", "visit_time": "14:30", "phone_tail": "5678"},
    {"id": "V1003", "name": "李娜", "company": "北辰物业", "host": "行政部", "visit_time": "15:00", "phone_tail": "9012"},
]


class MockBridge:
    def __init__(self, token: str = "", print_seconds: float = 1.0, printer_ready: bool = True):
        self.token = token
        self.print_seconds = print_seconds
        self.printer_ready = printer_ready
        self.visitors = [dict(v) for v in SAMPLE_VISITORS]
        self.printed: set[str] = set()
        self.jobs: dict[str, dict] = {}
        self.print_log: list[str] = []
        self.lock = threading.Lock()

    def handler(self):
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def reply(self, status: int, payload: dict):
                body = json.dumps(payload, ensure_ascii=False).encode()
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def authorized(self) -> bool:
                if bridge.token and self.headers.get("Authorization") != f"Bearer {bridge.token}":
                    self.reply(401, {"error": "unauthorized"})
                    return False
                return True

            def do_GET(self):
                if not self.authorized():
                    return
                url = urlparse(self.path)
                if url.path == "/health":
                    self.reply(200, {
                        "printer": {"ready": bridge.printer_ready, "name": "模拟标签打印机",
                                    "message": "" if bridge.printer_ready else "打印机缺纸"},
                        "visitor_system": {"ready": True, "message": ""},
                    })
                elif url.path == "/visitors":
                    keyword = (parse_qs(url.query).get("keyword") or [""])[0].strip()
                    with bridge.lock:
                        hits = [dict(v, printed=v["id"] in bridge.printed) for v in bridge.visitors
                                if keyword and (keyword in v["name"] or keyword == v["phone_tail"])]
                    self.reply(200, {"visitors": hits})
                elif url.path.startswith("/jobs/"):
                    job_id = url.path[len("/jobs/"):]
                    with bridge.lock:
                        job = bridge.jobs.get(job_id)
                        if job and job["status"] == "printing" and time.monotonic() >= job["ready_at"]:
                            job["status"] = "done"
                    if not job:
                        self.reply(404, {"error": "没有这个打印任务", "code": "job_not_found"})
                    else:
                        self.reply(200, {"job_id": job_id, "status": job["status"], "error": job.get("error", "")})
                else:
                    self.reply(404, {"error": "not found"})

            def do_POST(self):
                if not self.authorized():
                    return
                if urlparse(self.path).path != "/print":
                    self.reply(404, {"error": "not found"})
                    return
                try:
                    body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                except json.JSONDecodeError:
                    self.reply(400, {"error": "请求不是 JSON", "code": "bad_request"})
                    return
                visitor_id = str(body.get("visitor_id", ""))
                with bridge.lock:
                    visitor = next((v for v in bridge.visitors if v["id"] == visitor_id), None)
                    if visitor is None:
                        self.reply(404, {"error": "范式+ 里今天没有这个访客", "code": "visitor_not_found"})
                        return
                    if visitor_id in bridge.printed and not body.get("reprint"):
                        self.reply(200, {"status": "already_printed"})
                        return
                    if not bridge.printer_ready:
                        self.reply(503, {"error": "打印机缺纸，请前台同事换纸", "code": "printer_not_ready"})
                        return
                    job_id = uuid.uuid4().hex[:12]
                    bridge.jobs[job_id] = {"status": "printing", "ready_at": time.monotonic() + bridge.print_seconds}
                    bridge.printed.add(visitor_id)
                    bridge.print_log.append(visitor_id)
                print(f"[mock-bridge] 打印访客贴：{visitor['name']}（{visitor['company']}）→ 被访人 {visitor['host']}", flush=True)
                self.reply(200, {"job_id": job_id, "status": "printing"})

        return Handler

    def serve(self, host: str, port: int) -> ThreadingHTTPServer:
        return ThreadingHTTPServer((host, port), self.handler())


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--token", default="")
    args = parser.parse_args()
    server = MockBridge(args.token).serve(args.host, args.port)
    print(f"[mock-bridge] 监听 {args.host}:{args.port}，示例访客：王磊、李娜（两位）", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
