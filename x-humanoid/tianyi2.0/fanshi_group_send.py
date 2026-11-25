"""Canvas actuator for a configured Fanshi+ group; loaded by the Tianyi bundle."""
import base64
from collections import OrderedDict, deque
import copy
import json
import os
from pathlib import Path
import re
import ssl
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, HTTPRedirectHandler, HTTPSHandler, build_opener
from uuid import uuid4


def _reply(status, message, **extra):
    result = {"ok": status in ("accepted", "queued", "sending", "ready", "idle", "configured"),
              "status": status, "message": message, **extra}
    if not result["ok"]:
        result.update(state="error", error=message)
    return result


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def _post_json(url, payload, timeout=15, context=None):
    handlers = [_NoRedirect()]
    if context is not None:
        handlers.append(HTTPSHandler(context=context))
    opener = build_opener(*handlers)
    request = Request(url, data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                      headers={"Content-Type": "application/json"}, method="POST")
    try:
        with opener.open(request, timeout=timeout) as response:
            return response.status, response.read(65_537)
    except HTTPError as exc:
        with exc:
            return exc.code, exc.read(65_537)


def _interpret(status, raw):
    if status >= 500 or status == 408:
        return _reply("unknown", "平台响应不确定，请先核对群聊，不要自动重发。")
    if not 200 <= status < 300:
        return _reply("failed", f"HTTP {status}，请检查平台权限、地址或限流。")
    try:
        if len(raw) > 65_536:
            raise ValueError()
        data = json.loads(raw)
        if not isinstance(data, dict):
            raise ValueError()
        codes = [data[key] for key in ("errcode", "errorcode") if key in data]
        if not codes or any(type(code) is not int for code in codes) or len(set(codes)) != 1:
            raise ValueError()
        detail = data.get("data")
        if detail is not None and not isinstance(detail, dict):
            raise ValueError()
    except (ValueError, UnicodeError):
        return _reply("unknown", "平台响应格式异常，请先核对群内是否收到。")
    failures = (detail or {}).get("fail")
    if codes[0] != 0 or failures:
        # Keep numeric per-group diagnostics; never echo a request or Webhook.
        failed_groups = {}
        if isinstance(failures, dict):
            for group, value in list(failures.items())[:32]:
                if re.fullmatch(r"[0-9]{1,20}", str(group)):
                    if type(value) is int:
                        failed_groups[str(group)] = value
                    elif isinstance(value, str) and re.fullmatch(r"-?[0-9]{1,12}", value):
                        failed_groups[str(group)] = int(value)
                    else:
                        failed_groups[str(group)] = "detail_unavailable"
        message = ("平台拒绝请求，请结合 platform_code 排查。"
                   if codes[0] != 0 else
                   "平台返回群投递失败，请查看 failed_groups 错误码；不能仅凭此认定为权限问题。")
        return _reply("failed", message, platform_code=codes[0],
                      has_group_failures=bool(failures), failed_groups=failed_groups)
    return _reply("accepted", "平台接受消息且无失败群；最终到达及图片展示以群内为准。")


def _split_payload(payload):
    """Plan requests before sending; no mixed request or fallback retry."""
    body = payload["message"]["body"]
    has_text = any(item["type"] == "TEXT" for item in body)
    has_image = any(item["type"] == "IMAGE" for item in body)
    if not (has_text and has_image):
        return [("image" if has_image else "text", payload)]
    requests = []
    for kind, items in (
        ("text", [item for item in body if item["type"] != "IMAGE"]),
        ("image", [item for item in body if item["type"] == "IMAGE"]),
    ):
        request = copy.deepcopy(payload)
        request["message"]["body"] = copy.deepcopy(items)
        requests.append((kind, request))
    return requests


def _combine_results(parts):
    accepted = sum(part["status"] == "accepted" for part in parts)
    if len(parts) == 1:
        result = {key: value for key, value in parts[0].items()
                  if key not in ("part", "attempted")}
    elif accepted == len(parts):
        result = _reply("accepted", "文字和图片已分别被平台接受，群内将显示为两条消息。")
    elif any(part["status"] == "unknown" for part in parts):
        result = _reply("unknown", "部分结果未确认；查看 parts 和群聊，勿整组重发。")
    elif accepted:
        result = _reply("partial", "仅部分消息被平台接受；查看 parts，勿重复发送已成功部分。")
    elif parts[0]["status"] == "cancelled":
        result = _reply("cancelled", "发送前卡片停止或配置改变，本组消息未发送。")
    else:
        result = _reply("failed", "发送失败，后续部分未发送；请查看 parts 的具体错误。")
    result.update(parts=parts, accepted_parts=accepted, total_parts=len(parts),
                  request_count=sum(part["attempted"] for part in parts))
    return result


def _complete(action_id, result):
    """Report ACP completion to the local Core; never relax Webhook TLS checks."""
    endpoint = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    parsed = urlsplit(endpoint)
    if parsed.scheme not in ("http", "https") or parsed.hostname not in ("localhost", "127.0.0.1", "::1"):
        raise ValueError("ACP requires the local Agent Core endpoint")
    context = None
    if parsed.scheme == "https":
        # The repository's loopback ACP endpoint uses a self-signed certificate.
        context = ssl.create_default_context()
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
    status = "completed" if result["status"] == "accepted" else "error"
    if result["status"] == "cancelled":
        status = "cancelled"
    code, _ = _post_json(endpoint + "/api/acp/complete", {
        "action_id": action_id, "status": status, "result": result,
        "tool": FanshiGroupSendPlugin.PREFIX, "ts": time.time(),
    }, timeout=5, context=context)
    if not 200 <= code < 300:
        raise ValueError("ACP rejected completion")


class FanshiGroupSendPlugin:
    PREFIX = "fanshi_group_send"

    def __init__(self, plugin_config, namespace):
        self._lock = threading.RLock()
        self._settings = None
        self._active = False
        self._generation = 0
        self._busy = False
        self._recent = deque()
        self._jobs = OrderedDict()
        self._worker_thread = None
        self._image_roots = (Path("/opt/phanthy-motus/data/images"),
                             Path(os.environ.get("FILE_INTAKE_DIR", "/work/uploads")))

    def get_tool(self):
        tool = {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": (
                "向预先配置的范式+群发送文字和可选单张图片、@一个成员。"
                "同时提供文字和图片时，依次发送两条消息，用同一 action_id 跟踪。"
                "全部 accepted 才表示整组被接受；partial 表示部分成功，勿整组重发。"
                "无需原消息 ID。send 返回 queued 只表示已排队；等待完成回执，"
                "或用 result 查询 action_id；accepted 才表示平台接受。"
                "unknown 时先核对群聊，不自动重发。图片使用拍照工具返回的真实 path。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["send", "result", "config"]},
                    "text": {"type": "string", "description": "文字；文字和图片至少提供一种"},
                    "image_path": {"type": "string", "format": "file", "uploadTo": "mcp",
                                   "accept": ".jpg,.jpeg,.png", "description": "驱动可读取的图片路径"},
                    "at_userid": {"type": "string", "description": "可选：一个真实成员 userid"},
                    "action_id": {"type": "string", "description": "result 查询使用 send 返回的 action_id"},
                },
                "required": ["action"],
                "x-action-params": {
                    "send": {"params": ["text", "image_path", "at_userid"], "description": "发送群消息"},
                    "result": {"params": ["action_id"], "description": "查询本次发送结果"},
                    "config": {"params": ["webhook_url", "group_id"], "description": "配置范式+群消息发送"},
                },
                "x-completion": {"actions": ["send"], "timeout": 45},
            },
            "configSchema": {
                "type": "object",
                "properties": {
                    "webhook_url": {"type": "string", "scope": "shared", "format": "password",
                                    "x-sensitive": True, "description": "完整 HTTPS Webhook"},
                    "group_id": {"type": "string", "scope": "shared", "pattern": "^[0-9]+$",
                                 "x-sensitive": True, "description": "目标群数字 ID，禁止留空"},
                },
                "required": ["webhook_url", "group_id"],
            },
        }
        # Expose config parameters to action-specific callers without defaults.
        action_settings = copy.deepcopy(tool["configSchema"]["properties"])
        for field in action_settings.values():
            field.pop("default", None)
        tool["inputSchema"]["properties"].update(action_settings)
        return tool

    def start(self):
        # Bundle lazy-start does not have canvas configuration; dispatch arms it.
        pass

    def stop(self):
        with self._lock:
            self._active = False
            self._generation += 1

    @staticmethod
    def _validate_config(args):
        url = args.get("webhook_url", "")
        group = args.get("group_id", "")
        if not isinstance(url, str) or not url.strip() or any(ord(c) < 32 for c in url):
            raise ValueError()
        url = url.strip()
        parsed = urlsplit(url)
        if parsed.scheme != "https" or not parsed.hostname or parsed.username or parsed.password or parsed.fragment:
            raise ValueError()
        if isinstance(group, bool) or not re.fullmatch(r"[0-9]{1,20}", str(group)) or int(group) <= 0:
            raise ValueError()
        return url, int(group)

    def _image_base64(self, raw_path):
        if not isinstance(raw_path, str) or not raw_path.startswith("/"):
            raise ValueError("图片必须是驱动可读取的绝对路径。")
        alias = "/work/resource/images/"
        if raw_path.startswith(alias):
            raw_path = "/opt/phanthy-motus/data/images/" + raw_path[len(alias):]
        path = Path(raw_path).resolve(strict=True)
        if not any(path.is_relative_to(root.resolve()) for root in self._image_roots):
            raise ValueError("图片必须位于驱动图片目录或文件上传目录。")
        if not path.is_file() or path.suffix.lower() not in (".png", ".jpg", ".jpeg"):
            raise ValueError("仅支持 PNG/JPEG 图片。")
        with path.open("rb") as stream:
            raw = stream.read(750_001)
        if not (raw.startswith(b"\x89PNG\r\n\x1a\n") or raw.startswith(b"\xff\xd8\xff")):
            raise ValueError("文件内容不是 PNG/JPEG。")
        encoded = base64.b64encode(raw).decode("ascii")
        if len(encoded) >= 1_000_000:
            raise ValueError("图片 Base64 必须小于 1,000,000 字节，请先缩小图片。")
        return encoded

    def _payload(self, group, args):
        text, path, userid = (args.get(k) or "" for k in ("text", "image_path", "at_userid"))
        if not all(isinstance(v, str) for v in (text, path, userid)):
            raise ValueError("文字、图片路径和 userid 必须为字符串。")
        body = []
        if text.strip():
            body.append({"type": "TEXT", "content": text})
        if path:
            body.append({"type": "IMAGE", "content": self._image_base64(path)})
        if not body:
            raise ValueError("文字和图片至少填写一种。")
        if userid.strip():
            body.append({"type": "AT", "atuserids": [userid.strip()], "atall": False})
        return {"message": {"header": {"toid": [group]}, "body": body}}

    def _send_parts(self, action_id, settings, generation, payload):
        planned = _split_payload(payload)
        parts = []
        for kind, request in planned:
            with self._lock:
                cancelled = not self._active or generation != self._generation
            if cancelled:
                outcome = _reply("cancelled", "该部分提交前卡片已停止或配置改变。")
                attempted = False
            else:
                attempted = True
                try:
                    outcome = _interpret(*_post_json(settings[0], request))
                except (OSError, URLError, ValueError):
                    outcome = _reply("unknown", "网络错误或超时；未自动重试，请先核对群聊。")
                except Exception:
                    outcome = _reply("unknown", "发送结果未确认；未自动重试，请先核对群聊。")
            parts.append({**outcome, "part": kind, "attempted": attempted})
            with self._lock:
                self._jobs[action_id]["parts"] = copy.deepcopy(parts)
            if outcome["status"] != "accepted":
                break
        for kind, _ in planned[len(parts):]:
            parts.append({**_reply("not_sent", "前一部分未确认成功，本部分未发送。"),
                          "part": kind, "attempted": False})
        return _combine_results(parts)

    def _worker(self, action_id, settings, generation, payload):
        with self._lock:
            cancelled = not self._active or generation != self._generation
            if not cancelled:
                self._jobs[action_id]["status"] = "sending"
        if cancelled:
            result = _reply("cancelled", "提交前卡片已停止或配置改变，没有发送本条消息。")
        else:
            try:
                result = self._send_parts(action_id, settings, generation, payload)
            except (OSError, URLError, ValueError):
                result = _reply("unknown", "网络错误或超时；未自动重试，请先核对群聊。")
            except Exception:
                result = _reply("unknown", "发送过程异常；请先核对群聊与平台日志。")
        result["action_id"] = action_id
        with self._lock:
            self._jobs[action_id] = result
            self._busy = False
        print(f"[fanshi_group_send] action_id={action_id} status={result['status']}", flush=True)
        try:
            _complete(action_id, result)
        except Exception:
            print(f"[fanshi_group_send] action_id={action_id} ACP_callback_failed; use result", flush=True)

    def dispatch(self, action, args):
        if action == "stop":
            self.stop()
            return _reply("idle", "已停止新发送；已提交的平台消息无法撤回。", state="idle")
        if action in ("config", "start", "info", "result"):
            with self._lock:
                if action == "config":
                    try:
                        settings = self._validate_config(args)
                    except (ValueError, TypeError):
                        self._settings = None
                        self._active = False
                        self._generation += 1
                        return _reply("failed", "配置无效：需要完整 HTTPS Webhook 和正整数群 ID。", adapter_ok=False)
                    if settings != self._settings:
                        self._settings = settings
                        self._generation += 1
                        self._active = False
                    return _reply("configured", "配置已校验，启动后可发送。", adapter_ok=True)
                if action == "start":
                    if self._settings is None:
                        return _reply("failed", "请先保存 Webhook 和目标群配置。")
                    self._active = True
                    return _reply("ready", "已就绪，启动操作没有发送消息。", state="ready")
                if action == "info":
                    return {"state": "ready" if self._active else "idle", "busy": self._busy,
                            "configured": self._settings is not None, "topic_in": [], "topic_out": []}
                action_id = args.get("action_id", "")
                if not isinstance(action_id, str) or action_id not in self._jobs:
                    return _reply("unknown", "该发送记录不存在或已过期；先查看群聊，不要自动重发。")
                return copy.deepcopy(self._jobs[action_id])
        if action != "send":
            return _reply("failed", "不支持的动作。")
        with self._lock:
            if not self._active or self._settings is None:
                return _reply("failed", "请先保存配置并启动画布项目。")
            settings, generation = self._settings, self._generation
        try:
            payload = self._payload(settings[1], args)
        except ValueError as exc:
            return _reply("failed", str(exc))
        except OSError:
            return _reply("failed", "图片不存在或不可读，请使用驱动中的实际路径。")
        request_cost = len(_split_payload(payload))
        with self._lock:
            if not self._active or generation != self._generation:
                return _reply("failed", "准备发送期间卡片停止或配置变化，请核对后重新操作。")
            if self._busy:
                return _reply("failed", "上一条消息正在发送，请先查询其结果。")
            now = time.monotonic()
            while self._recent and now - self._recent[0] >= 60:
                self._recent.popleft()
            if len(self._recent) + request_cost > 30:
                return _reply("failed", "达到本进程每分钟 30 次请求的限制；图文一组占两次。")
            # Reserve the entire group so the second request cannot exceed the cap.
            # Unused reservations expire normally; failures never trigger retries.
            self._recent.extend([now] * request_cost)
            action_id = "fanshi_" + uuid4().hex
            queued = _reply("queued", "已排队，等待完成回执或使用 result 查询；尚未确认平台接受。",
                            state="running", action_id=action_id)
            self._jobs[action_id] = dict(queued)
            while len(self._jobs) > 32:
                self._jobs.popitem(last=False)
            self._busy = True
            self._worker_thread = threading.Thread(target=self._worker,
                args=(action_id, settings, generation, payload), daemon=True)
            try:
                self._worker_thread.start()
            except RuntimeError:
                self._busy = False
                self._jobs[action_id] = _reply("failed", "发送线程未启动，未发送消息。", action_id=action_id)
                return dict(self._jobs[action_id])
            return queued
