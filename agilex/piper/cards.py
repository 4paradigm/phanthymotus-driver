"""Ten focused PiPER contracts; Chinese labels are presentation metadata."""
from common.vendor_runtime import action_schema, tool


TITLES = {
    "piper_status": "机器人状态", "piper_standby": "切换待机",
    "piper_can": "切换电脑控制", "piper_enable": "电机使能",
    "piper_disable": "电机失能", "piper_joint": "单轴运动",
    "piper_cartesian": "末端空间移动",
    "piper_gripper": "夹爪开合", "piper_photo": "相机与照片",
    "piper_permission": "操作许可", "piper_stop": "停止动作",
}


def flag(title, hidden=False):
    return {"type": "boolean", "enum": [False, True], "default": False,
            "title": title, "oneOf": [{"const": False, "title": "未确认"},
                                      {"const": True, "title": "已确认"}],
            **({"x-ui-hidden": True} if hidden else {})}


def choice(title, pairs, default):
    return {"type": "string", "enum": [v for v, _ in pairs], "default": default,
            "title": title, "oneOf": [{"const": v, "title": label} for v, label in pairs]}


def build_tools(topic):
    lifecycle = {"start": ([], "启动卡片；不连接、不使能、不移动"),
                 "stop": ([], "停止本卡片；不失能"), "info": ([], "查看实际完成结果")}
    hidden = {"execute": flag("执行已确认的计划", True),
              "preview_id": {"type": "string", "title": "预览编号", "x-ui-hidden": True},
              "workspace_clear": flag("现场空间清空且可及时停止", True),
              "take_control": flag("已确认本次接管控制", True)}
    confirm_motion = {"workspace_clear": True, "take_control": True}
    result = []

    def add(name, description, actions, props=None, confirmations=None, kind="actuator"):
        schema = action_schema({**actions, **lifecycle}, props or {})
        schema["additionalProperties"] = False
        schema["x-manual-execution"] = True
        schema["properties"]["action"].update(title="操作", default=next(iter(actions)),
            oneOf=[{"const": a, "title": text} for a, (_, text) in {**actions, **lifecycle}.items()])
        if confirmations:
            schema["x-confirmation"] = confirmations
        if name not in {"piper_status", "piper_permission"}:
            schema["x-resource"] = ["arm", "camera"] if name == "piper_photo" else "arm"
            schema["x-is-dangerous"] = name != "piper_photo"
            schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop"}}
        definition = tool(name, kind, description, schema,
                          topic_out=[{"topic": topic, "format": "image/jpeg"}] if name == "piper_photo" else None)
        definition["title"] = TITLES[name]
        result.append(definition)

    add("piper_status", "模式、使能、故障分别反馈；只读取，不发送运动指令。",
        {"read": ([], "读取状态"), "firmware": ([], "查询固件"),
         "reconnect": ([], "重新连接（关闭许可并清除预览）")}, kind="resource")
    for name, label in [("piper_standby", "待机"), ("piper_can", "电脑控制（CAN）")]:
        props = {key: hidden[key] for key in ["execute", "preview_id"]}
        params = []
        if name == "piper_can":
            props["other_controllers_stopped"] = flag("其他控制程序已停止")
            params.append("other_controllers_stopped")
        add(name, f"仅切换到{label}；不使能、不回零。当前驱动要求六轴全部失能。",
            {"switch": (params, f"切换到{label}")}, props,
            {"switch": {"preview": {"action": "switch", "execute": False},
                        "execute": {"execute": True}, "message": "确认本次模式切换；以设备反馈为准。"}})
    object_prop = choice("操作对象", [("arm", "机械臂六轴"), ("gripper", "夹爪")], "arm")
    add("piper_enable", "使能当前选择的电机；已经使能时返回已就绪。需要边界修正时单独展示并确认。",
        {"enable": (["object", "speed_pct"], "使能电机")},
        {**hidden, "object": object_prop,
         "speed_pct": {"type": "integer", "minimum": 1, "maximum": 10, "default": 5, "title": "使能调整速度上限（%）", "x-visible-when": {"object": "arm"}},
         "allow_limit_adjustment": flag("允许本次预览中的小幅边界修正", True)},
        {"enable": {"preview": {"action": "enable", "execute": False},
                    "execute": {"execute": True, **confirm_motion, "allow_limit_adjustment": True},
                    "message": "确认其他控制程序已停止、现场空间清空、底座固定、可及时停止。若计划显示边界修正，本次会产生所示小幅运动。"}})
    add("piper_disable", "解除保持力。确认机械臂或夹持物已支撑，再执行；已经失能时不重复发指令。",
        {"disable": (["object", "supported"], "失能电机")},
        {"object": object_prop, "supported": flag("机械臂已可靠支撑 / 夹爪空载或夹持物已支撑")})
    add("piper_joint", "只控制一个关节的绝对角度；自动预览，再确认执行。",
        {"move": (["joint", "target_deg", "speed_deg_s", "speed_pct"], "移动所选关节")},
        {**hidden, "joint": {"type": "integer", "minimum": 1, "maximum": 6, "default": 1, "title": "关节编号（1–6）"},
         "target_deg": {"type": "number", "minimum": -180, "maximum": 180, "title": "目标角度（°）"},
         "speed_deg_s": {"type": "number", "minimum": .5, "maximum": 5, "default": 1, "title": "关节速度（°/秒）"},
         "speed_pct": {"type": "integer", "minimum": 1, "maximum": 10, "default": 5, "title": "SDK 速度上限（%）"}},
        {"move": {"preview": {"action": "move", "execute": False}, "execute": {"execute": True, **confirm_motion},
                  "message": "确认其他控制程序已停止、预览目标正确、活动范围无人无障碍，且可及时停止。"}})
    add("piper_cartesian", "保持末端姿态，沿基座 X/Y/Z 轴小幅移动；关节插补，先预览再确认。不是避障规划。",
        {"read": ([], "读取末端位置"),
         "move": (["axis", "distance_mm", "speed_deg_s", "speed_pct"], "沿基座坐标轴移动")},
        {**hidden, "axis": choice("基座坐标轴", [("x", "X 轴"), ("y", "Y 轴"), ("z", "Z 轴")], "z"),
         "distance_mm": {"type": "number", "minimum": -20, "maximum": 20, "default": 5, "title": "带正负号的移动距离（mm）"},
         "speed_deg_s": {"type": "number", "minimum": .5, "maximum": 5, "default": 1, "title": "关节速度上限（°/秒）"},
         "speed_pct": {"type": "integer", "minimum": 1, "maximum": 10, "default": 5, "title": "SDK 速度上限（%）"}},
        {"move": {"preview": {"action": "move", "execute": False},
                  "execute": {"execute": True, **confirm_motion},
                  "message": "确认所选机械臂、基座坐标轴与目标正确；其他控制程序已停止，整个机械臂、相机和夹爪活动空间清空，可及时停止。"}})
    add("piper_gripper", "只负责开合；使能和失能请用独立电机卡片。张开到上限，闭合到下限。",
        {"read": ([], "读取夹爪"), "open": (["torque_nm"], "张开至最大开度"),
         "close": (["torque_nm"], "闭合至最小开度"),
         "set_width": (["width_mm", "torque_nm"], "指定开度")},
        {**hidden, "width_mm": {"type": "number", "minimum": 0, "maximum": 100, "title": "目标开度（mm）"},
         "torque_nm": {"type": "number", "minimum": .001, "maximum": 1, "default": .5, "title": "夹爪力矩（N·m）"}},
        {a: {"preview": {"action": a, "execute": False}, "execute": {"execute": True, **confirm_motion},
             "message": "确认其他控制程序已停止、开度与力矩正确；手指离开夹持区、夹爪空载或已安排本次夹持。"}
         for a in ["open", "close", "set_width"]})
    add("piper_photo", "拍摄当前 RGB 一张并显示；文件导入保留来源标记，不移动机械臂。",
        {"capture": ([], "拍摄当前画面"), "import_photo": (["filename"], "导入已有照片")},
        {"capture_confirmed": flag("确认本次拍摄", True),
         "filename": {"type": "string", "title": "imports 目录内的照片文件名"}},
        {"capture": {"execute": {"capture_confirmed": True}, "message": "确认允许本次拍摄当前相机画面一张。"}})
    add("piper_permission", "本次操作的许可开关；不会使能。关闭许可请求取消控制并保持，重连/重启后关闭。",
        {"allow": ([], "开启本次控制许可"), "block": ([], "关闭本次控制许可")})
    # Keep permission changes out of the model's generated action catalog.
    result[-1]["inputSchema"]["x-action-params"] = {k: v for k, v in result[-1]["inputSchema"]["x-action-params"].items() if k in lifecycle}
    add("piper_stop", "随时请求取消选定动作并尽力保持；不失能，不是硬件急停。",
        {"halt": (["target"], "停止动作")},
        {"target": choice("停止范围", [("all", "全部动作"), ("arm", "机械臂动作"), ("gripper", "夹爪动作")], "all")})
    for definition in result:
        name = definition["name"]
        internal_params = {
            "piper_standby": {"switch": ["execute", "preview_id"]},
            "piper_can": {"switch": ["execute", "preview_id"]},
            "piper_enable": {"enable": ["execute", "preview_id", "workspace_clear", "take_control", "allow_limit_adjustment"]},
            "piper_joint": {"move": ["execute", "preview_id", "workspace_clear", "take_control"]},
            "piper_cartesian": {"move": ["execute", "preview_id", "workspace_clear", "take_control"]},
            "piper_gripper": {a: ["execute", "preview_id", "workspace_clear", "take_control"] for a in ["open", "close", "set_width"]},
            "piper_photo": {"capture": ["capture_confirmed"]},
        }
        for action, params in internal_params.get(name, {}).items():
            definition["inputSchema"]["x-action-params"][action]["params"] += params
        completions = {"piper_enable": ["enable"], "piper_joint": ["move"], "piper_cartesian": ["move"],
                       "piper_gripper": ["open", "close", "set_width"], "piper_photo": ["capture", "import_photo"]}
        if definition["name"] in completions:
            definition["inputSchema"]["x-completion"] = {"actions": completions[definition["name"]], "timeout": 60}
    return result
