"""RealMan 二指夹爪桥接：订阅 gripper 驱动的 DDS 命令，经 rm75 SDK 连接转发给控制箱。

复用 RM75SDKClient 独占的 SDK 连接，避免与控制箱 8080 端口产生第二个客户端。
wire format（flat）：{"command": "hand_follow_pos", "hand_pos": [pos]}，pos 范围 0~1000。
"""

from __future__ import annotations

import json

HAND_DOF = 6  # SDK 灵巧手数组固定 6 指，夹爪只占用第 1 指


class GripperBridgePlugin:
    """无 MCP 工具的插件：只做 DDS 订阅转发，get_tools() 返回空列表以隐藏于 tools/list。"""

    def __init__(self, client, config, ros2):
        from rclpy.node import Node
        from std_msgs.msg import String

        self.client = client
        topics = config.get("gripper_bridge", {})
        self.command_topic = topics.get("command", "/realman_gripper/command")
        self.state_topic = topics.get("state", "/realman_gripper/state")

        node = Node("rm75_gripper_bridge", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(node)
        self.node = node
        node.create_subscription(String, self.command_topic, self._on_command, 10)
        self.state_pub = node.create_publisher(String, self.state_topic, 10)

    def get_tools(self):
        return []

    def start(self):
        print(f"[gripper-bridge] listening {self.command_topic} -> SDK rm_set_hand_follow_pos", flush=True)

    def stop(self):
        pass

    def _on_command(self, msg):
        try:
            envelope = json.loads(msg.data)
            if envelope.get("command") != "hand_follow_pos":
                raise ValueError(f"unexpected command: {envelope.get('command')}")
            hand_pos = [int(value) for value in envelope.get("hand_pos", [])]
            if len(hand_pos) != 1:
                raise ValueError(f"expected exactly 1 hand_pos element, got {len(hand_pos)}")
            padded = hand_pos + [0] * (HAND_DOF - len(hand_pos))
            code = self.client.command("rm_set_hand_follow_pos", padded, False)
            print(f"[gripper-bridge] sent hand_follow_pos {hand_pos} (code={code})", flush=True)
            self._publish_state({"state": "accepted", "return_code": code, "hand_pos": hand_pos})
        except Exception as exc:
            print(f"[gripper-bridge] rejected: {exc}", flush=True)
            self._publish_state({"state": "error", "detail": str(exc)})

    def _publish_state(self, data):
        from std_msgs.msg import String

        out = String()
        out.data = json.dumps(data, ensure_ascii=False)
        self.state_pub.publish(out)
