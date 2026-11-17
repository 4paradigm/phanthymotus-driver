"""A remote simulation session, exposed through the standard workbench bridge."""

from __future__ import annotations

import threading
import json
import uuid
import os
from pathlib import Path

from .client import MotionClient, ServiceError


ACTIONS = {
    "inspect": ("session_id", "mode", "width", "height", "azimuth", "elevation", "distance", "lookat", "offset", "jpeg_quality", "detail"),
    "catalog": (),
    "simulation_list": ("view_id",),
    "simulation_attach": ("session_id", "view_id"),
    "simulation_detach": ("session_id", "view_id"),
    "simulation_claim": ("session_id", "view_id", "control_epoch", "takeover"),
    "simulation_heartbeat": ("session_id", "view_id", "control_epoch"),
    "simulation_release": ("session_id", "view_id", "control_epoch"),
    "simulation_create": ("profile_id", "model_id", "control_mode", "view_id", "request_id"),
    "simulation_inspect": ("session_id", "detail"), "simulation_run": ("session_id",),
    "simulation_pause": ("session_id",), "simulation_step": ("session_id", "steps"),
    "simulation_reset": ("session_id",), "simulation_close": ("session_id",),
    "simulation_perturb": ("session_id", "body_id", "force", "torque", "duration_s"),
    "simulation_record": ("session_id",), "simulation_record_stop": ("session_id",),
    "simulation_replay": ("session_id", "offset", "limit"),
    "simulation_render": ("session_id", "width", "height", "azimuth", "elevation", "distance", "lookat", "offset", "jpeg_quality", "detail"),
}

for _action, _params in list(ACTIONS.items()):
    if _action.startswith("simulation_") or _action == "inspect":
        ACTIONS[_action] = tuple(dict.fromkeys((*_params, "view_id")))
    if _action in {"simulation_run", "simulation_pause", "simulation_step", "simulation_reset",
                   "simulation_close", "simulation_perturb", "simulation_record", "simulation_record_stop"}:
        ACTIONS[_action] += ("control_epoch",)
ENVIRONMENT_ACTIONS = {"env_targets": (), "env_check": ("target_id",),
                       "env_install": ("target_id", "recipe_id", "mode"),
                       "env_cancel": ("job_id",), "env_connect": ("job_id",)}


class SimulationPlugin:
    NAME = "simulation"

    def __init__(self, config=None):
        self._cfg = dict(config or {})
        endpoint = self._cfg.get("endpoint") or os.environ.get("MOTION_SERVICE_URL", "")
        key = self._cfg.get("api_key") or os.environ.get("MOTION_API_TOKEN", "")
        self._state_dir = Path(self._cfg.get("state_dir") or os.environ.get("SIMULATION_WORKBENCH_STATE_DIR")
                               or "~/.local/state/phanthymotus/simulation").expanduser()
        self._binding_path = self._state_dir / "environment.json"
        self._environment_job = ""
        try:
            self._environment = MotionClient(self._cfg.get("environment_endpoint") or os.environ.get("MOTION_ENVIRONMENT_URL", ""),
                                             self._cfg.get("environment_token") or os.environ.get("MOTION_ENVIRONMENT_TOKEN", ""))
        except (ServiceError, ValueError, TypeError):
            self._environment = MotionClient()
        try:
            binding = json.loads(self._binding_path.read_text())
            if binding.get("environment_job") and self._environment.configured:
                endpoint, key = binding["endpoint"], binding["api_key"]
                self._environment_job = binding["environment_job"]
        except (OSError, ValueError, KeyError, TypeError):
            pass
        self._configuration_error = ""
        try:
            self._client = MotionClient(endpoint, key)
        except (ServiceError, ValueError, TypeError) as exc:
            self._client = MotionClient()
            self._configuration_error = str(exc)
        self._cfg.update(endpoint=self._client.endpoint, api_key=key)
        self._lock = threading.RLock()
        self._sessions = {}
        self._view_sessions = {}
        self._grants = {}
        self._requests = {}
        self._seen_sessions = set()

    def start(self):
        """Opening the bundle never starts physics."""

    def _current(self, instance, view=None):
        if view:
            return self._view_sessions.setdefault((instance, view), self._sessions.get(instance))
        return self._sessions.get(instance)

    def _pause_bound(self, client, instance=None, view=None):
        keys = [key for key in self._grants if (instance is None or key[0] == instance)
                and (view is None or key[1] == view)]
        managed = set()
        for key in keys:
            grant = self._grants[key]
            managed.add(grant["id"])
            try:
                client.call("simulation_pause", session_id=grant["id"], view_id=key[1],
                            control_epoch=grant["epoch"])
            except ServiceError as exc:
                if "控制权已变化" not in str(exc):
                    raise
                self._grants.pop(key, None)
        # Legacy clients have no view grant. Never send their unguarded stop
        # into a managed session: the service rejects it as an additional fence.
        if not view:
            for key, sid in self._sessions.items():
                if (instance is None or key == instance) and sid not in managed and not any(
                        value == sid for value in self._view_sessions.values()):
                    client.call("simulation_pause", session_id=sid)

    def stop(self):
        with self._lock:
            self._pause_bound(self._client)

    def get_tools(self):
        names = ["ui", "info", "start", "stop", "config", "check", *ACTIONS, *ENVIRONMENT_ACTIONS]
        props = {"action": {"type": "string", "enum": names}}
        for name in {p for params in {**ACTIONS, **ENVIRONMENT_ACTIONS}.values() for p in params}:
            props[name] = {"type": "integer" if name in {"steps", "body_id", "offset", "limit", "width", "height", "jpeg_quality", "control_epoch", "cursor"}
                           else "number" if name in {"duration_s", "azimuth", "elevation", "distance"}
                           else "array" if name in {"force", "torque", "lookat"} else "string"}
            if name in {"force", "torque", "lookat"}:
                props[name].update(items={"type": "number"}, minItems=3, maxItems=3)
        props["takeover"] = {"type": "boolean"}
        props["job_id"] = {"type": "string"}
        props["cursor"] = {"type": "integer"}
        props["target_id"] = {"type": "string"}
        props["mode"]["enum"] = ["frame", "state", "env_status", "reuse", "isolated"]
        props["detail"]["enum"] = ["full", "compact"]
        props["jpeg_quality"].update(minimum=40, maximum=95)
        return [{"name": self.NAME, "type": "actuator", "multiInstance": True,
                 "description": "仿真调试：MuJoCo 会话、策略验证、诊断和录制回放。",
                 "inputSchema": {"type": "object", "properties": props, "required": ["action"],
                                 "x-workbench": {"protocol": "motus.workbench/1", "kind": "simulation",
                                                 "title": "仿真调试", "entry": "ui", "actions": names,
                                                 "config": ["endpoint", "api_key", "environment_job"]},
                                 "x-action-params": {
                                     "ui": {"params": [], "description": "打开仿真调试"},
                                     "inspect": {"params": ["mode"], "description": "只读获取当前仿真画面或状态"},
                                 }},
                 "configSchema": {"type": "object", "properties": {
                     "environment_job": {"type": "string", "title": "已接入环境任务", "scope": "shared"},
                     "endpoint": {"type": "string", "title": "服务器地址", "scope": "shared"},
                     "api_key": {"type": "string", "title": "访问凭证", "format": "password",
                                 "scope": "shared", "x-sensitive": True},
                 }, "required": []}}]

    def _info(self, instance, view=None):
        with self._lock:
            return {"state": "ready" if self._client.configured else "unconfigured", "busy": False,
                    "session_id": self._current(instance, view),
                    "config_values": {"endpoint": self._client.endpoint, "environment_job": self._environment_job},
                    "environment_available": self._environment.configured,
                    "config_secrets": {"api_key": self._client.has_key},
                    "message": self._configuration_error}

    def _assert_switchable(self):
        bound = set(self._sessions.values()) | {sid for sid in self._view_sessions.values() if sid}
        if not (bound or self._seen_sessions or self._grants):
            return
        sessions = self._client.call("simulation_list").get("sessions")
        if not isinstance(sessions, list) or any(not isinstance(item, dict) or not isinstance(item.get("id"), str) for item in sessions):
            raise ServiceError("服务器返回了无效会话列表")
        local_views = {view for _, view in self._view_sessions} | {view for _, view in self._grants}
        for session in sessions:
            if session.get("state") == "closed":
                continue
            control = session.get("control")
            if isinstance(control, dict):
                if control.get("owner_view_id") in local_views:
                    raise ServiceError("仍有窗口在控制仿真，请先释放控制后切换服务器")
            elif session["id"] in bound:
                raise ServiceError("请先关闭当前仿真会话，再切换服务器连接")

    def _set_connection(self, client, key, job_id=""):
        changed = client.endpoint != self._client.endpoint
        if changed:
            self._assert_switchable()
        self._state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        if job_id:
            temporary = self._binding_path.with_name("environment." + uuid.uuid4().hex + ".tmp")
            try:
                fd = os.open(temporary, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
                with os.fdopen(fd, "w") as stream:
                    json.dump({"environment_job": job_id, "endpoint": client.endpoint, "api_key": key}, stream)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, self._binding_path)
            finally:
                temporary.unlink(missing_ok=True)
        else:
            self._binding_path.unlink(missing_ok=True)
        if changed:
            self._sessions.clear()
            self._view_sessions = dict.fromkeys(self._view_sessions)
            self._grants.clear()
            self._seen_sessions.clear()
        self._cfg.update(endpoint=client.endpoint, api_key=key)
        self._client, self._environment_job = client, job_id
        self._configuration_error = ""

    def _connect_environment(self, job_id, instance, view):
        if not isinstance(job_id, str) or not job_id:
            raise ServiceError("请先检查或安装环境")
        value = self._environment.call("connection", job_id=job_id)
        connection = value.get("connection", value)
        client = MotionClient(connection.get("endpoint", ""), connection.get("api_key", ""))
        if not client.configured:
            raise ServiceError("环境尚未通过检查")
        client.health()
        self._set_connection(client, connection["api_key"], job_id)
        return self._info(instance, view)

    def dispatch(self, name, args):
        args = args or {}
        action = args.get("action") or name
        instance = str(args.get("instance_id") or "default")
        view = args.get("view_id")
        try:
            if action == "ui":
                return {"protocol": "motus.workbench/1", "html": Path(__file__).with_name("workbench.html").read_text()}
            if action in {"info", "start"}:
                return self._info(instance, view)
            if action == "config":
                with self._lock:
                    job_id = args.get("environment_job", self._environment_job)
                    if job_id:
                        return self._info(instance, view) if job_id == self._environment_job else self._connect_environment(job_id, instance, view)
                    endpoint = args.get("endpoint", self._client.endpoint)
                    key = args.get("api_key", self._cfg.get("api_key", ""))
                    # The workbench host reapplies saved configuration on each
                    # read. Preserve the validated client when nothing changed.
                    if (endpoint == self._client.endpoint
                            and key == self._cfg.get("api_key", "")
                            and not self._configuration_error and not self._environment_job):
                        return self._info(instance, view)
                    client = MotionClient(endpoint, key)
                    self._set_connection(client, key)
                return self._info(instance, view)
            with self._lock:
                client = self._client
                current = self._current(instance, view)
                if action in ENVIRONMENT_ACTIONS:
                    values = {key: args[key] for key in ENVIRONMENT_ACTIONS[action] if key in args}
                    if action == "env_connect":
                        return self._connect_environment(values.get("job_id"), instance, view)
                    return self._environment.call(action.removeprefix("env_"), **values)
                if action == "inspect" and args.get("mode") == "env_status":
                    return self._environment.call("status", **{key: args[key] for key in ("job_id", "cursor", "target_id") if key in args})
                if action == "check":
                    return client.health()
                if action == "stop":
                    if not current:
                        return self._info(instance, view)
                    if view:
                        if type(args.get("control_epoch")) is int:
                            client.call("simulation_pause", session_id=current, view_id=view,
                                        control_epoch=args["control_epoch"])
                    else:
                        self._pause_bound(client, instance)
                    return self._info(instance, view)
                if action not in ACTIONS:
                    raise ServiceError("不支持的仿真操作")
                if action == "simulation_detach":
                    if view:
                        self._view_sessions[(instance, view)] = None
                        self._grants.pop((instance, view), None)
                        if self._sessions.get(instance) == current and not any(
                                sid == current for (bound_instance, _), sid in self._view_sessions.items()
                                if bound_instance == instance):
                            self._sessions.pop(instance, None)
                    else:
                        self._sessions.pop(instance, None)
                    return {"session": None}
                if action == "simulation_create" and view and isinstance(args.get("request_id"), str):
                    request_key = (instance, view, args["request_id"])
                    endpoint = self._requests.setdefault(request_key, client.endpoint)
                    if endpoint != client.endpoint:
                        raise ServiceError("此创建请求属于之前的服务器，请重新发起仿真")
                if action == "simulation_create" and current:
                    if view:
                        result = client.call("simulation_inspect", session_id=current, view_id=view)
                        session = result.get("session") or {}
                        if all(session.get(k) == args.get(k) for k in ("profile_id", "model_id")):
                            return result
                    raise ServiceError("请先离开当前仿真会话")
                if action == "simulation_list":
                    result = client.call(action, **({"view_id": view} if view else {}))
                    sessions = result.get("sessions")
                    if not isinstance(sessions, list) or any(
                        not isinstance(item, dict) or not isinstance(item.get("id"), str)
                        for item in sessions
                    ):
                        raise ServiceError("服务器返回了无效会话列表")
                    active = {item["id"] for item in sessions if item.get("state") != "closed"}
                    self._sessions = {key: value for key, value in self._sessions.items() if value in active}
                    self._view_sessions = {key: value if value in active else None for key, value in self._view_sessions.items()}
                    self._grants = {key: grant for key, grant in self._grants.items() if grant["id"] in active}
                    return result
                if action == "simulation_attach":
                    if current and current != args.get("session_id"):
                        raise ServiceError("请先离开当前仿真会话")
                    session_id = args.get("session_id")
                    if not isinstance(session_id, str) or not 1 <= len(session_id) <= 96:
                        raise ServiceError("无效仿真会话标识")
                    if not view and session_id in self._sessions.values():
                        raise ServiceError("此会话已由另一张卡片接入")
                    result = client.call("simulation_inspect", session_id=session_id, **({"view_id": view} if view else {}))
                    session = result.get("session") or {}
                    if session.get("id") != session_id or session.get("state") == "closed":
                        raise ServiceError("仿真会话已不可用")
                    self._seen_sessions.add(session_id)
                    if view:
                        self._view_sessions[(instance, view)] = session_id
                    else:
                        self._sessions[instance] = session_id
                    return result
                values = {key: args[key] for key in ACTIONS[action] if key in args}
                if "session_id" in ACTIONS[action]:
                    supplied = values.get("session_id")
                    if not current:
                        raise ServiceError("请先创建仿真会话")
                    if supplied and supplied != current:
                        raise ServiceError("仿真会话与当前卡片不匹配")
                    values["session_id"] = current
                if action == "inspect":
                    # The host's standard inspect action is read-only. Never
                    # let a mode select a physics or lifecycle operation.
                    mode = values.pop("mode", "frame")
                    if mode not in {"frame", "state"}:
                        raise ServiceError("无效的只读查看模式")
                    action = "simulation_render" if mode == "frame" else "simulation_inspect"
                    if mode == "state":
                        values = {key: value for key, value in values.items()
                                  if key in {"session_id", "detail", "view_id"}}
                if action == "simulation_create" and view:
                    values["origin"] = "simulation"
                result = client.call(action, **values)
                if action == "simulation_create":
                    session_id = (result.get("session") or {}).get("id")
                    if not session_id:
                        raise ServiceError("服务器未返回仿真会话标识")
                    self._sessions[instance] = session_id
                    self._seen_sessions.add(session_id)
                    if view:
                        self._view_sessions[(instance, view)] = session_id
                elif action == "simulation_close":
                    self._sessions = {key: sid for key, sid in self._sessions.items() if sid != current}
                    self._view_sessions = {key: sid if sid != current else None for key, sid in self._view_sessions.items()}
                    self._grants = {key: grant for key, grant in self._grants.items() if grant["id"] != current}
                control = result.get("control")
                if view and control and action != "simulation_close":
                    if control.get("is_controller"):
                        self._grants[(instance, view)] = {"id": result["session"]["id"], "epoch": control["epoch"]}
                    else:
                        self._grants.pop((instance, view), None)
                return result
        except (ServiceError, ValueError, TypeError, OSError) as exc:
            return {"state": "error", "message": str(exc), "error": str(exc)}


def build_plugins(config, namespace, ros2=None):
    options = (config.get("plugins") or {}).get("simulation") or {}
    if not options.get("enabled", True):
        return []
    return [SimulationPlugin(options)]
