"""Teaching orchestration. Adapters retain ownership of robot safety and motion.

An asset is a reference to a complete, checksum-bound recording, not a learned
policy. Trials and user effect confirmation authorize publication, never future
motion. Each run goes through the adapter's fresh preview/confirmation gate.
"""
import copy
import hashlib
import http.client
import json
import os
from pathlib import Path
import re
import socket
import ssl
import stat
import tempfile
import threading
import time
from uuid import uuid4
from urllib.parse import urlsplit

from common.vendor_runtime import action_schema, tool
from .package import pack, read_package, SCHEMA as PACKAGE_SCHEMA

PROTOCOL = "motus.workbench/1"
ASSET_SCHEMA = "motus.teaching.asset/1"


def read_execution_policy(tool, instance_id, device_binding, asset_id, asset_binding):
    """Read this asset's saved policy from the same-host Core; never grant it."""
    url = urlsplit(os.environ.get('AGENT_CORE_URL', 'https://localhost:15678'))
    host = url.hostname
    # The host-network Compose fragment maps this service alias to loopback.
    if host == 'phanthy-motus' and socket.gethostbyname(host) == '127.0.0.1':
        host = '127.0.0.1'
    if (host not in ('localhost', '127.0.0.1', '::1') or url.username or url.password
            or url.path not in ('', '/') or url.query or url.fragment or url.scheme not in ('http', 'https')):
        raise ValueError('execution_policy_requires_local_core')
    kwargs = {'timeout': 3}
    if url.scheme == 'https':
        kwargs['context'] = ssl._create_unverified_context()  # host loopback, self-signed Core
    cls = http.client.HTTPSConnection if url.scheme == 'https' else http.client.HTTPConnection
    connection = cls(host, url.port, **kwargs)
    try:
        body = dict(tool=tool, instance_id=instance_id, device_binding=device_binding,
                    asset_id=asset_id, asset_binding=asset_binding)
        connection.request('POST', '/api/skills/package/execution/read', json.dumps(body),
                           {'Content-Type': 'application/json'})
        response = connection.getresponse()
        raw = response.read(4097)
        if response.status != 200 or len(raw) > 4096:
            raise ValueError('execution_policy_unavailable')
        payload = json.loads(raw)
        policy = payload.get('data') if isinstance(payload, dict) else None
        if (not isinstance(policy, dict) or type(policy.get('enabled')) is not bool
                or policy.get('source') not in ('manual', 'skill')
                or type(policy.get('binding_changed')) is not bool):
            raise ValueError('invalid_execution_policy')
        return policy
    finally:
        connection.close()


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
        self.boot_id = uuid4().hex

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
            "asset_id": {"type": "string", "description": "技能指令中的资产 ID；预览和正式执行都必须传同一个 ID"},
            "execute": {"type": "boolean", "default": False, "description": "正式执行必须为 true；先 preview，按 confirmation_required 判断是否需要人工确认"},
            "preview_id": {"type": "string", "description": "本次 preview 返回的完整编号，正式执行必填；不得省略或自行编造"},
            "workspace_clear": {"type": "boolean", "description": "用户确认本次活动空间已清空"},
            "take_control": {"type": "boolean", "description": "用户确认本次接管机器人"},
            "effect_confirmed": {"type": "boolean", "description": "用户已观察并确认本次试运行的效果"},
            "package": {"type": "string", "format": "file", "accept": ".motus-skill,.zip",
                        "uploadTo": "mcp", "description": "选择动作包，上传至当前设备后导入；不会执行动作"},
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
            "run": (["asset_id", *confirmation], "执行已保存技能；先 preview。confirmation_required=false 时直接执行，否则检查本次用户确认。running 不是完成"),
            "preview": (["asset_id"], "只读预览并检查此技能的自动执行设置；confirmation_required=false 时直接按返回参数执行，不再询问"),
            "stop": ([], "请求原控制器停止，读取 info 确认完成结果"),
        }
        if hasattr(self.adapter, "compatibility"):
            actions.update({"package_definition": (["asset_id"], "读取当前版本的技能调用说明，不执行动作"),
                            "export": (["asset_id"], "在设备保存完整动作包并返回文件路径，不执行动作"),
                            "import": (["package"], "检查上传的动作包并创建待验证草稿；随后打开录制技能继续试运行")})
            actions.update({"draft_list": ([], "列出本机待验证导入"),
                            "draft_open": (["draft_id"], "继续验证导入草稿，不执行动作")})
        persistent = hasattr(self.adapter, 'execution_binding')
        if persistent:
            actions['execution_binding'] = (['asset_id'], '读取设备及已验证动作的绑定，不执行动作')
        schema = action_schema(actions, fields)
        schema["additionalProperties"] = False
        schema["x-resource"] = getattr(self.adapter, "resources", "arm")
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop"}}
        schema["x-stop"] = {"pending_flags": {"busy": True, "authority_released": False}}
        schema["x-completion"] = {"actions": ["run"], "timeout": getattr(self.adapter, "completion_timeout", 610)}
        schema["x-workbench"] = {"protocol": PROTOCOL, "kind": "skill-authoring",
                                 "title": "录制技能 · " + self.adapter.title, "entry": "ui", "actions": list(actions)}
        if hasattr(self.adapter, "compatibility"):
            schema["x-skill-package"] = {"schema": PACKAGE_SCHEMA, "persistent_execution": persistent}
        schema["properties"]["action"]["oneOf"] = [
            {"const": a, "title": {"ui": "录制技能", "run": "运行技能", "import": "导入动作包",
                                    "export": "导出动作包"}.get(a, a)} for a in actions]
        # Recording authoring stays in the workbench; file import/export also
        # use the ordinary canvas/LLM argument form.
        public = ["start", "info", "preview", "run", "stop"]
        if hasattr(self.adapter, "compatibility"):
            public += ["import", "export"]
        schema["x-action-params"] = {a: schema["x-action-params"][a] for a in public}
        schema["x-action-params"]["preview"]["required"] = ["asset_id"]
        schema["x-action-params"]["run"]["required"] = ["asset_id", "execute", "preview_id"]
        if "import" in public:
            schema["x-action-params"]["import"]["required"] = ["package"]
            schema["x-action-params"]["export"]["required"] = ["asset_id"]
        definition = tool(self.name, "actuator", self.adapter.title + "：基于已验证示教录制的技能；保留原有速度、标定和设备绑定。", schema)
        return [definition]

    def _validate(self, action, args):
        schema = self.get_tools()[0]["inputSchema"]
        if action not in schema["properties"]["action"]["enum"]:
            raise ValueError("unknown_teaching_action")
        for key, value in args.items():
            if key == "_tool_name":
                continue
            prop = schema["properties"].get(key)
            if prop is None or (prop["type"] == "boolean" and type(value) is not bool) or (
                    prop["type"] == "integer" and type(value) is not int) or (
                    prop["type"] == "string" and (not isinstance(value, str) or len(value) > 4000)):
                raise ValueError("invalid_teaching_argument_" + key)

    def _current(self, args):
        if not self.draft or args.get("draft_id") != self.draft["id"]:
            raise ValueError("teaching_draft_changed_reload")
        return self.draft

    def _idle(self):
        if self.pending or self.adapter.info()["busy"]:
            raise RuntimeError("robot_busy_wait_for_completion")

    def _execution_binding(self, asset_id=''):
        def digest(value):
            return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
        result = {'device_binding': digest(self.adapter.execution_binding())}
        if asset_id:
            asset, recording = self._read_asset(asset_id)
            if not asset.get('effect_confirmed') or asset.get('trial', {}).get('status') != 'completed':
                raise ValueError('successful_trial_required')
            result.update(asset_id=asset_id, asset_binding=digest({
                'sha256': recording['sha256'], 'binding': recording['binding'], 'metadata': asset['metadata']}))
        return result

    def _execution_policy(self, args):
        if not hasattr(self.adapter, 'execution_binding'):
            return {'enabled': False, 'source': 'manual', 'binding_changed': False, 'supported': False}
        binding = self._execution_binding(args.get('asset_id', ''))
        policy = read_execution_policy(self.name, args.get('instance_id', ''), **binding)
        return {**policy, 'supported': True}

    def _persist_draft(self):
        if not self.draft or not self.draft.get('imported'):
            return
        directory = self.directory / 'drafts'
        directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        path = directory / (self.draft['id']+'.json')
        if self.draft.get('saved'):
            path.unlink(missing_ok=True)
            return
        value = copy.deepcopy(self.draft)
        value['recording'] = {k: value['recording'][k] for k in ('filename', 'sha256', 'binding')}
        value['boot_id'] = self.boot_id
        fd, temp = tempfile.mkstemp(prefix='.draft-', dir=directory)
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(value, stream, ensure_ascii=False, allow_nan=False)
                stream.flush(); os.fsync(stream.fileno())
            os.replace(temp, path)
        finally:
            if os.path.exists(temp): os.unlink(temp)

    def _load_draft(self, ident):
        if not isinstance(ident, str) or not re.fullmatch('[a-f0-9]{32}', ident):
            raise ValueError('invalid_draft_id')
        path = self.directory / 'drafts' / (ident+'.json')
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd) as stream:
            info = os.fstat(stream.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_size > 32768:
                raise ValueError('invalid_import_draft')
            draft = json.load(stream)
        if draft.get('id') != ident or not draft.get('imported'):
            raise ValueError('invalid_import_draft')
        if draft.get('boot_id') != self.boot_id or (draft.get('trial') or {}).get('status') == 'running':
            draft['trial'] = None
        return draft

    def _validated_package(self, manifest, data):
        # Only this device's saved, effect-confirmed receipt can skip a trial.
        # Imported claims never carry validation authority.
        for path in self.directory.glob('*.json'):
            try:
                asset, recording = self._read_asset(path.stem)
                if (asset.get('effect_confirmed') and asset.get('trial', {}).get('status') == 'completed'
                        and asset['metadata'] == manifest['skill']
                        and self.adapter.export_recording(recording) == data
                        and all(manifest['compatibility'].get(k) == v
                                for k, v in self.adapter.compatibility().items()
                                if k not in ('driver_version', 'labels'))):
                    return {'skill': self._skill_definition(asset['id'], asset['metadata']),
                            'asset_id': asset['id'], 'reused': True, 'requires_local_trial': False}
            except (ValueError, OSError, KeyError):
                continue

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
                try:
                    self._persist_draft()
                except OSError as exc:
                    self.draft['trial'] = {'action_id': action_id, 'status': 'error',
                                           'result': {'reason': 'draft_persistence_failed: '+str(exc)}}
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

    def _run(self, args):
        if not args.get("asset_id"):
            raise ValueError("missing_asset_id: 预览和正式执行均须传技能指令中的 asset_id；没有执行动作")
        context = self.adapter.execution_context() if hasattr(self.adapter, 'execution_context') else None
        _, recording = self._read_asset(args["asset_id"])
        policy = self._execution_policy(args)
        automatic = policy.get('enabled') is True
        if args.get("execute") is True:
            if not args.get("preview_id"):
                raise ValueError("missing_preview_id: 请保留 asset_id 并补齐本次预览的 preview_id；没有执行动作")
            if automatic:
                args = dict(args, workspace_clear=True, take_control=True,
                            _execution_context=context)
            elif args.get("workspace_clear") is not True or args.get("take_control") is not True:
                raise ValueError("user_confirmation_required: 请先询问用户本次路径、环境和双臂接管；不得自行将确认字段设为 true")
        result = self._play(recording, args, "run")
        if result.get("executed") is False:
            result.update(asset_id=args["asset_id"], confirmation_required=not automatic,
                          execution_policy=policy,
                          next_step=("已允许自动执行，按 execution_arguments 执行，无需重复询问；预览不是动作完成" if automatic else
                                     "询问用户本次活动空间、使用条件及双臂接管，得到确认后再执行；也可在技能详情设置自动执行"),
                          execution_arguments={"asset_id": args["asset_id"], "execute": True,
                              "preview_id": result["preview_id"], **({} if automatic else {"workspace_clear": False, "take_control": False})})
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
        skill = self._skill_definition(asset_id, metadata)
        asset = {"schema": ASSET_SCHEMA, "id": asset_id, "adapter": self.name,
                 "recording": {k: recording[k] for k in ("filename", "sha256", "binding")},
                 "metadata": metadata, "trial": draft["trial"], "effect_confirmed": True,
                 "created_at": time.time(), "skill": skill}
        if 'execution_parameters' in draft['trial'].get('result', {}):
            asset['execution_parameters'] = dict(draft['trial']['result']['execution_parameters'])
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
        self._persist_draft()
        return {"skill": skill, "asset_id": asset_id}

    def _skill_definition(self, asset_id, metadata):
        skill = {
            "slug": "taught-" + asset_id, "name": metadata["name"],
            "oneLiner": metadata["phrase"], "description": metadata["description"],
            "category": "robotics", "version": "1.0.0", "requiredTools": [self.name],
            "instruction": (
                f"这是 {self.adapter.title} 的已示教录制技能，资产 ID：{asset_id}。\n"
                f"用户描述：{metadata['description']}\n使用条件：{metadata['conditions']}\n"
                f"1. 调用当前设备的 {self.name}.preview，参数：" + json.dumps({"asset_id": asset_id}) + "。\n"
                "2. 预览成功没有执行动作。若 confirmation_required=false，驱动已核验此技能在当前卡片、设备和录制上的自动执行设置，"
                "用户要求执行时直接使用返回的 execution_arguments，不再追问。若为 true，表示没有自动执行权限，不表示用户尚未在对话中确认。"
                "先检查当前尚未执行的同一次请求：用户若已明确确认本次使用条件、移动路径、活动空间和双臂接管，不再重复询问；缺少确认时只询问缺少的条件并等待答复。"
                "用户可在示教保存、导入启用或技能详情设置此技能的自动执行，停止项目和重启后仍保留。不得替用户修改自动执行设置。"
                "仅在 confirmation_required=true 时，用户只说技能名称或‘抬手’不等于已确认环境与接管。未取得确认不得填写 true；已经明确确认时应据实填写 true，不得原样照抄预览中的 false。未执行不得播报动作已完成。\n"
                f"3. confirmation_required=false 时直接调用 {self.name}.run 并原样传入 execution_arguments。仅当需要逐次确认时，取得确认后必须一次传齐：" + json.dumps({"asset_id": asset_id,
                    "execute": True, "preview_id": "替换为本次预览返回的完整 preview_id", "workspace_clear": True,
                    "take_control": True}, ensure_ascii=False) + "。每次都带 asset_id，不能只传 preview_id。\n"
                "4. 缺少参数只补齐该参数，不得换动作或猜测起点；等待用户答复导致预览过期时重新预览。重新预览不会抹掉用户对同一次尚未执行请求的明确确认。"
                "若动作已执行、停止/重启、模式/标定/环境改变或授权被撤销，不得复用此前确认。user_confirmation_required 表示当前调用参数未确认；先核对用户答复和实际参数，不要重复发送两个 false。\n"
                "预览不通过时报告原因，不得改录制或绕过检查。"
                + self.adapter.replay_instruction +
                "设备/标定绑定由驱动强制检查。running 只表示已提交，等待 ACP 完成或通过 info 核对相同 action_id 的终态。"
                "若 trajectory_completed=true 且 stop_error 存在，录制轨迹已结束，但停止交权当时未确认；"
                "调用 info 核对同一 action_id 和 stop.operation_id 对应的 release，按实际状态报告，不得自动重放。"
                "只报告回执给出的错误原因，不得猜测碰撞、遮挡或障碍物；交权未确认不等于轨迹没有执行。"
                "停止使用 stop 并等待完成结果。动作完成不等于任务效果成功，按需让用户确认。"
                "本技能保持录制时环境和物体布置，不具有视觉泛化或物体定位能力。"
            ),
        }
        if hasattr(self.adapter, "compatibility"):
            skill["compatibility"] = self.adapter.compatibility()
            skill["attachment"] = {"schema": PACKAGE_SCHEMA, "tool": self.name, "asset_id": asset_id}
        return skill

    def dispatch(self, action, args):
        self._validate(action, args)
        # Stop must remain reachable while file validation or a slow read is in progress.
        if action == "stop":
            return self.adapter.stop()
        if action == "ui":
            return {"protocol": PROTOCOL, "html": Path(__file__).with_name("workbench.html").read_text(encoding="utf-8")}
        with self.lock:
            if action == 'execution_binding':
                return self._execution_binding(args.get('asset_id', ''))
            if action == 'draft_list':
                drafts = []
                for path in (self.directory / 'drafts').glob('*.json'):
                    try:
                        draft = self._load_draft(path.stem)
                        drafts.append({'id': draft['id'], 'name': draft['imported']['metadata']['name'],
                                       'compatibility': draft['imported']['compatibility']})
                    except (ValueError, KeyError, OSError):
                        continue
                return {'drafts': drafts}
            if action == 'draft_open':
                self._idle()
                if self.draft and self.draft.get('recording') and not self.draft.get('saved') and not self.draft.get('imported'):
                    raise ValueError('finish_or_discard_current_draft_before_import')
                self._persist_draft()
                draft = self._load_draft(args.get('draft_id'))
                draft['recording'] = self._recording(draft)
                self.draft = draft
                return {'draft': copy.deepcopy(draft)}
            if action == "package_definition":
                asset, _ = self._read_asset(args.get("asset_id"))
                return {"skill": self._skill_definition(asset["id"], asset["metadata"])}
            if action == "export":
                asset, recording = self._read_asset(args.get("asset_id"))
                manifest, data = pack(asset["metadata"], self.adapter.compatibility(),
                                      self.adapter.export_recording(recording), self.adapter.compatibility()["recording_schema"])
                directory = self.directory / 'exports'
                directory.mkdir(parents=True, exist_ok=True, mode=0o700)
                path = directory / (asset['id'] + '.motus-skill')
                fd, temp = tempfile.mkstemp(prefix='.package-', dir=directory)
                try:
                    with os.fdopen(fd, 'wb') as stream:
                        stream.write(data); stream.flush(); os.fsync(stream.fileno())
                    os.replace(temp, path)
                finally:
                    if os.path.exists(temp): os.unlink(temp)
                return {'path': str(path.resolve()), 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
                        'manifest': manifest, 'executed': False,
                        'message': '动作包已保存在设备。此路径属于设备容器，网页不会自动下载或上传技能广场。'}
            if action == "import":
                self._idle()
                if self.draft and self.draft.get("recording") and not self.draft.get("saved") and not self.draft.get('imported'):
                    raise ValueError("finish_or_discard_current_draft_before_import")
                manifest, data = read_package(args.get("package"))
                reused = self._validated_package(manifest, data)
                if reused:
                    # Re-enter the same save/import path in the host. The
                    # already-verified device asset is reused, never the
                    # untrusted validation claimed by an incoming package.
                    asset, recording = self._read_asset(reused['asset_id'])
                    self._persist_draft()
                    self.draft = {'id': asset['id'], 'recording': recording, 'trial': asset['trial'],
                                  'saved': asset}
                    return {**reused, 'executed': False,
                            'next_step': '已复用本机验证记录。选择「录制技能」打开工作台，保存到当前技能库。'}
                self._persist_draft()
                recording = self.adapter.import_recording(manifest, data)
                self.draft = {"id": uuid4().hex, "recording": recording, "trial": None,
                              "imported": {"metadata": manifest["skill"], "compatibility": manifest["compatibility"]}}
                self.record_started = None
                self._persist_draft()
                return {"draft": copy.deepcopy(self.draft), "requires_local_trial": True, "installed": False,
                        "executed": False, "next_step": "选择「录制技能」打开工作台，检查并试运行；导入尚未执行动作。"}
            if action == "info":
                status = self.adapter.info()
                execution_policy = {'supported': hasattr(self.adapter, 'execution_binding')}
                compatibility = None
                if hasattr(self.adapter, "compatibility"):
                    try:
                        compatibility = self.adapter.compatibility()
                    except (ValueError, KeyError):
                        # Missing calibration must leave the check page usable.
                        # Export/save still require the full compatibility data.
                        pass
                if (status.get("recording") or {}).get("recording") and self.record_started is not None:
                    status["recording"]["elapsed_seconds"] = max(0, time.monotonic() - self.record_started)
                return {**status, "title": self.adapter.title,
                        "execution_policy": execution_policy,
                        "presentation": copy.deepcopy(self.adapter.presentation),
                        "compatibility": compatibility,
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
                self._persist_draft()
                self.draft = {"id": uuid4().hex, "recording": recording, "trial": None}
                self.record_started = None
                return copy.deepcopy(self.draft)
            if action in ("preview", "run"):
                return self._run(dict(args, execute=False) if action == "preview" else args)
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
                if self.adapter.presentation.get('passive_recording'):
                    result = self.adapter.record(args)
                    self.record_started = time.monotonic()
                    draft['trial'] = None
                    return result
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
