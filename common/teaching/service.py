"""Teaching orchestration. Adapters retain ownership of robot safety and motion.

An asset is a reference to a complete, checksum-bound recording, not a learned
policy. Trials and user effect confirmation authorize publication, never future
motion. Each run goes through the adapter's fresh preview/confirmation gate.
"""
import copy
import json
import os
from pathlib import Path
import re
import stat
import tempfile
import threading
import time
from uuid import uuid4

from common.vendor_runtime import action_schema, tool

PROTOCOL = "motus.workbench/1"
ASSET_SCHEMA = "motus.teaching.asset/1"


class TeachingPlugin:
    def __init__(self, adapter, directory, name):
        self.adapter, self.directory, self.name = adapter, Path(directory), name
        self.lock = threading.RLock()
        self.draft = None
        self.pending = None
        self.last_run = None
        self.closed = False
        self.observer = None
        self.record_started = None

    def start(self):
        self.closed = False

    def stop(self):
        self.closed = True
        self.adapter.stop()
        if self.observer:
            self.observer.join(4)

    def get_tools(self):
        fields = {
            "instance_id": {"type": "string", "x-ui-hidden": True},
            "_trace_id": {"type": "string", "x-ui-hidden": True},
            "draft_id": {"type": "string", "description": "当前示教草稿 ID"},
            "filename": {"type": "string", "description": "已有完整录制文件"},
            "asset_id": {"type": "string", "description": "已验证的示教资产 ID"},
            "execute": {"type": "boolean", "default": False},
            "preview_id": {"type": "string"},
            "workspace_clear": {"type": "boolean", "description": "用户确认本次活动空间已清空"},
            "take_control": {"type": "boolean", "description": "用户确认本次接管机器人"},
            "effect_confirmed": {"type": "boolean", "description": "用户已观察并确认本次试运行的效果"},
            **{key: {"type": "string"} for key in ("name", "description", "conditions", "phrase")},
        }
        confirmation = ["execute", "preview_id", "workspace_clear", "take_control"]
        actions = {
            "start": ([], "准备示教扩展，不连接或移动机器人"),
            "info": ([], "读取示教进度和运行完成结果"),
            "ui": ([], "打开网页示教工作台"),
            "new": (["filename"], "创建示教草稿，或重新验证一条已有录制"),
            "check": ([], "用户请求检查反馈和设备准备条件，不移动机器人"),
            "begin": (["draft_id", *confirmation], "预览或确认开始遥操"),
            "record": (["draft_id"], "录制机器人实际执行的遥操目标"),
            "finish": (["draft_id"], "结束录制及遥操，等待完整落盘"),
            "inspect": (["draft_id"], "预览录制并比较当前姿态与回放起点"),
            "trial": (["draft_id", *confirmation], "预览或确认试运行当前录制"),
            "save": (["draft_id", "name", "description", "conditions", "phrase", "effect_confirmed"], "试运行成功且用户确认效果后保存技能"),
            "run": (["asset_id", *confirmation], "执行已保存的技能；先预览，再取得用户本次确认，最后提交执行。running 不是完成"),
            "stop": ([], "请求原控制器停止，读取 info 确认完成结果"),
        }
        schema = action_schema(actions, fields)
        schema["additionalProperties"] = False
        schema["x-resource"] = getattr(self.adapter, "resources", "arm")
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop"}}
        schema["x-completion"] = {"actions": ["run"], "timeout": getattr(self.adapter, "completion_timeout", 610)}
        schema["x-workbench"] = {"protocol": PROTOCOL, "kind": "skill-authoring",
                                 "title": "录制技能 · " + self.adapter.title, "entry": "ui", "actions": list(actions)}
        schema["properties"]["action"]["oneOf"] = [
            {"const": a, "title": {"ui": "录制技能", "run": "运行技能"}.get(a, a)} for a in actions]
        # The full wire contract includes authoring. Only execution/lifecycle are
        # useful as LLM/canvas actions; authoring is handled by the workbench.
        schema["x-action-params"] = {a: schema["x-action-params"][a] for a in ("start", "info", "run", "stop")}
        return [tool(self.name, "actuator", self.adapter.title + "：基于已验证示教录制的技能；保留原有速度、标定和设备绑定。", schema)]

    def _validate(self, action, args):
        schema = self.get_tools()[0]["inputSchema"]
        if action not in schema["properties"]["action"]["enum"]:
            raise ValueError("unknown_teaching_action")
        for key, value in args.items():
            if key == "_tool_name":
                continue
            prop = schema["properties"].get(key)
            if prop is None or (prop["type"] == "boolean" and type(value) is not bool) or (
                    prop["type"] == "string" and (not isinstance(value, str) or len(value) > 4000)):
                raise ValueError("invalid_teaching_argument_" + key)

    def _current(self, args):
        if not self.draft or args.get("draft_id") != self.draft["id"]:
            raise ValueError("teaching_draft_changed_reload")
        return self.draft

    def _idle(self):
        if self.pending or self.adapter.info()["busy"]:
            raise RuntimeError("robot_busy_wait_for_completion")

    def _recording(self, draft):
        if not draft.get("recording"):
            raise ValueError("complete_recording_required")
        recording = self.adapter.inspect(draft["recording"]["filename"])
        if recording["sha256"] != draft["recording"]["sha256"]:
            raise ValueError("recording_changed_record_again")
        return recording

    def _observe(self, action_id, purpose, draft_id):
        # Completion comes from the underlying controller, never elapsed duration.
        try:
            outcome = self.adapter.wait(action_id)
            if outcome.get("action_id") != action_id or outcome.get("status") not in {"completed", "cancelled", "error"}:
                raise RuntimeError("invalid_motion_completion")
        except Exception as exc:
            outcome = {"action_id": action_id, "status": "error", "result": {"reason": str(exc)}}
            try:
                self.adapter.stop()
            except Exception:
                pass
        with self.lock:
            if purpose == "trial" and self.draft and self.draft["id"] == draft_id:
                self.draft["trial"] = outcome
            elif purpose == "run":
                self.last_run = outcome
            self.pending = None
        # The adapter's motion owner delivers ACP for this same action_id. Do
        # not send a second event; Core correlates by ID, not by card name.

    def _play(self, recording, args, purpose, draft_id=None):
        if self.pending:
            raise RuntimeError("robot_busy_wait_for_completion")
        result = self.adapter.play(recording["filename"], args)
        if result.get("state") == "running":
            self.pending = {"action_id": result["action_id"], "purpose": purpose}
            if purpose == "trial":
                self.draft["trial"] = {"status": "running", "action_id": result["action_id"]}
            self.observer = threading.Thread(target=self._observe,
                args=(result["action_id"], purpose, draft_id), name="teaching-completion", daemon=True)
            self.observer.start()
        return result

    def _read_asset(self, asset_id):
        if not isinstance(asset_id, str) or not re.fullmatch(r"[a-f0-9]{32}", asset_id):
            raise ValueError("invalid_teaching_asset_id")
        fd = os.open(self.directory / (asset_id + ".json"), os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd, "r", encoding="utf-8") as stream:
            info = os.fstat(stream.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_size > 32768:
                raise ValueError("invalid_teaching_asset")
            asset = json.load(stream)
        if asset.get("schema") != ASSET_SCHEMA or asset.get("id") != asset_id or asset.get("adapter") != self.name:
            raise ValueError("incompatible_teaching_asset")
        recording = self.adapter.inspect(asset["recording"]["filename"])
        if (recording["sha256"] != asset["recording"]["sha256"] or
                recording["binding"] != asset["recording"]["binding"]):
            raise ValueError("skill_recording_or_binding_changed")
        return asset, recording

    def _save(self, draft, args):
        self._idle()
        recording = self._recording(draft)
        if not draft.get("trial") or draft["trial"].get("status") != "completed":
            raise ValueError("successful_trial_required")
        if args.get("effect_confirmed") is not True:
            raise ValueError("user_effect_confirmation_required")
        metadata = {}
        for key, limit in (("name", 100), ("description", 1000), ("conditions", 1000), ("phrase", 200)):
            value = args.get(key, "").strip()
            if key in ("description", "phrase") and not value:
                value = metadata["name"]
            if not value or len(value) > limit:
                raise ValueError("required_skill_field_" + key)
            metadata[key] = value
        if draft.get("saved"):
            if draft["saved"]["metadata"] != metadata:
                raise ValueError("skill_already_saved_create_new_draft")
            return {"skill": draft["saved"]["skill"], "asset_id": draft["id"]}
        asset_id = draft["id"]
        skill = {
            "slug": "taught-" + asset_id, "name": metadata["name"],
            "oneLiner": metadata["phrase"], "description": metadata["description"],
            "category": "robotics", "version": "1.0.0", "requiredTools": [self.name],
            "instruction": (
                f"这是 {self.adapter.title} 的已示教录制技能，资产 ID：{asset_id}。\n"
                f"用户描述：{metadata['description']}\n使用条件（每次执行前向用户核实）：{metadata['conditions']}\n"
                f"使用当前设备的 {self.name} 工具 run 动作，asset_id={asset_id}，先 execute=false 预览。\n"
                "向用户说明活动空间、双臂接管及起始姿态要求；取得本次确认后，带返回的 preview_id、"
                "execute=true、workspace_clear=true、take_control=true 再次 run。不得复用旧确认。\n"
                "预览不通过时报告原因，不得改录制或绕过检查。"
                + self.adapter.replay_instruction +
                "设备/标定绑定由驱动强制检查。running 只表示已提交，等待 ACP 完成或通过 info 核对相同 action_id 的终态。"
                "停止使用 stop 并等待完成结果。动作完成不等于任务效果成功，按需让用户确认。"
                "本技能保持录制时环境和物体布置，不具有视觉泛化或物体定位能力。"
            ),
        }
        asset = {"schema": ASSET_SCHEMA, "id": asset_id, "adapter": self.name,
                 "recording": {k: recording[k] for k in ("filename", "sha256", "binding")},
                 "metadata": metadata, "trial": draft["trial"], "effect_confirmed": True,
                 "created_at": time.time(), "skill": skill}
        self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        fd, path = tempfile.mkstemp(prefix=".asset-", dir=self.directory)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(asset, stream, ensure_ascii=False, allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(path, self.directory / (asset_id + ".json"))
        finally:
            if os.path.exists(path):
                os.unlink(path)
        draft["saved"] = asset
        return {"skill": skill, "asset_id": asset_id}

    def dispatch(self, action, args):
        self._validate(action, args)
        # Stop must remain reachable while file validation or a slow read is in progress.
        if action == "stop":
            return self.adapter.stop()
        if action == "ui":
            return {"protocol": PROTOCOL, "html": Path(__file__).with_name("workbench.html").read_text(encoding="utf-8")}
        with self.lock:
            if action == "info":
                status = self.adapter.info()
                if (status.get("recording") or {}).get("recording") and self.record_started is not None:
                    status["recording"]["elapsed_seconds"] = max(0, time.monotonic() - self.record_started)
                return {**status, "title": self.adapter.title,
                        "presentation": copy.deepcopy(self.adapter.presentation),
                        "busy": bool(self.pending) or status["busy"],
                        "draft": copy.deepcopy({k: v for k, v in (self.draft or {}).items() if k != "saved"}),
                        "saved": copy.deepcopy((self.draft or {}).get("saved", {}).get("metadata")),
                        "last_run": copy.deepcopy(self.last_run)}
            if self.closed:
                raise RuntimeError("teaching_service_closed")
            if action == "start":
                return {"state": "ready", "executed": False}
            if action == "check":
                return self.adapter.check()
            if action == "new":
                if args.get("filename"):
                    self._idle()
                else:
                    status = self.adapter.info()
                    if self.pending or not status.get("can_prepare", not status["busy"]):
                        raise RuntimeError("robot_busy_wait_for_completion")
                recording = self.adapter.inspect(args["filename"]) if args.get("filename") else None
                self.draft = {"id": uuid4().hex, "recording": recording, "trial": None}
                self.record_started = None
                return copy.deepcopy(self.draft)
            if action == "run":
                _, recording = self._read_asset(args.get("asset_id"))
                return self._play(recording, args, "run")
            draft = self._current(args)
            if action == "save":
                return self._save(draft, args)
            if action == "inspect":
                return {"recording": self._recording(draft), "start": self.adapter.start_difference(draft["recording"]["filename"])}
            if draft.get("saved"):
                raise ValueError("skill_already_saved_create_new_draft")
            if action == "begin":
                if draft.get("recording"):
                    raise ValueError("create_new_draft_to_record_again")
                result = self.adapter.begin(args)
                if result.get("state") == "running":
                    draft["teleop_id"] = result["action_id"]
                return result
            if action == "record":
                status = self.adapter.info()
                if self.adapter.presentation.get("session_start") == "controller" and not draft.get("teleop_id"):
                    if not status.get("can_record") or not status.get("session_id"):
                        raise ValueError("live_mapped_teleop_required")
                    draft["teleop_id"] = status["session_id"]
                if not draft.get("teleop_id") or status.get("session_id") != draft["teleop_id"]:
                    raise ValueError("start_teaching_teleop_first")
                result = self.adapter.record()
                self.record_started = time.monotonic()
                draft["trial"] = None
                return result
            if action == "finish":
                result = self.adapter.finish()
                if result.get("complete"):
                    draft["recording"] = self.adapter.inspect(result["filename"])
                    draft["trial"] = None
                return result
            if action == "trial":
                recording = self._recording(draft)
                return self._play(recording, args, "trial", draft["id"])
        raise ValueError("unknown_teaching_action")
