"""plugins — Go1 设备插件(每功能一个模块)。

每个插件遵循统一契约:
    __init__(cfg, namespace, bridge, hl)   # cfg=该插件配置段, bridge=RosBridge, hl=Go1HighLevel
    get_tool() -> dict   或   get_tools() -> list[dict]   # MCP tool schema
    start()  / stop()
    dispatch(action, args) -> dict | None                 # 必须处理 start/stop/info
sensor 插件用 bridge.add_sensor 发数据到 ROS2 topic;actuator 在 dispatch 里调 hl/adapter。
所有 dispatch 返回 plain dict,由 main.py 的 HTTP handler 包成 MCP content。
"""
