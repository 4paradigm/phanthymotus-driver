"""Historical direct operator adapter using the one-use preview machinery.

Installed only by status_card.split_status_card for the retained legacy
two-card interface and its regression fixtures (test_piper_entrypoint and
test_piper_direct_control). Production device.build_plugins uses
IntentControlPlugin and does not install these hooks. Keep this module because
status_card imports it and historical consumers still exercise it.

No permission is latched globally. The confirmed invocation grants only its
own worker thread access; monitoring and other callers receive no grant.
"""
GUARDED = {"switch_mode", "enable", "move_joint", "jog_positive", "jog_negative",
           "gripper_open", "gripper_close", "gripper_set_width"}
PERMISSION_ACTIONS = {"allow_motion", "block_motion"}
INTERNAL_PARAMS = {"execute", "preview_id", "workspace_clear", "take_control", "allow_limit_adjustment"}


def direct_schema(definition):
    schema = definition["inputSchema"]
    action = schema["properties"]["action"]
    action["enum"] = [v for v in action["enum"] if v not in PERMISSION_ACTIONS]
    action["oneOf"] = [v for v in action.get("oneOf", []) if v["const"] not in PERMISSION_ACTIONS]
    for name in PERMISSION_ACTIONS:
        schema.get("x-action-params", {}).pop(name, None)
    for name in GUARDED:
        rule = schema.get("x-action-params", {}).get(name)
        if rule:
            rule["params"] = [p for p in rule["params"] if p not in INTERNAL_PARAMS]
    schema.pop("x-confirmation", None)
    definition["description"] = "直接控制：检查模式、使能、反馈和限位，满足条件后执行所选动作。"


def install_direct_control(runtime):
    original_dispatch = runtime.dispatch
    original_sync = runtime.dispatch_sync
    original_permission = runtime.require_motion_permission
    original_snapshot = runtime.read_monitor_snapshot

    def permission():
        if getattr(runtime.worker_context, "direct_operator_action", False):
            if runtime.closed or "arm_control" in runtime.stopped_cards or runtime.cancel.is_set():
                raise RuntimeError("控制卡已停止或动作已取消")
            return
        return original_permission()

    def sync(action, args):
        previous = getattr(runtime.worker_context, "direct_operator_action", False)
        runtime.worker_context.direct_operator_action = (
            args.get("_tool_name") == "arm_control"
            and (action in GUARDED or action == "disable" and args.get("supported") is True))
        try:
            return original_sync(action, args)
        finally:
            runtime.worker_context.direct_operator_action = previous

    def dispatch(action, args):
        if args.get("_tool_name") != "arm_control":
            return original_dispatch(action, args)
        if action in PERMISSION_ACTIONS:
            raise ValueError("无需单独请求控制，请直接选择动作；停止动作请使用停止选项")
        # Explicit legacy preview/execute calls retain their old semantics.
        # A current card invocation has no execute parameter and runs once.
        if action not in GUARDED or "execute" in args:
            return original_dispatch(action, args)
        preview = original_dispatch(action, {**args, "execute": False})
        if not isinstance(preview, dict) or preview.get("allowed") is False or not preview.get("preview_id"):
            return preview
        # Direct operator invocation replaces the old UI confirmation dialog.
        # Physical support/other-controller fields are still supplied by the
        # operator. Do not automatically authorize corrective enable motion.
        execution = {**args, "execute": True, "preview_id": preview["preview_id"],
                     "workspace_clear": True, "take_control": True}
        return original_dispatch(action, execution)

    def snapshot():
        return {**original_snapshot(), "control_policy": "direct_operator"}

    runtime.require_motion_permission = permission
    runtime.dispatch_sync = sync
    runtime.dispatch = dispatch
    runtime.read_monitor_snapshot = snapshot
