#!/usr/bin/env python3
"""
q5_pub_client.py — Q5 MPC/SDK 控制脚本

用法:
    # MPC 控制
    python q5_pub_client.py --type mpc --cmd start     # 启动 MPC 算法
    python q5_pub_client.py --type mpc --cmd query     # 查询 MPC 状态 (1=已启动)
    python q5_pub_client.py --type mpc --cmd reset --mode 0   # 抬手 (0=抬, 1=放)
    python q5_pub_client.py --type mpc --cmd reset --mode 1   # 放手
    python q5_pub_client.py --type mpc --cmd stop      # 停止 MPC 算法

    # SDK 控制
    python q5_pub_client.py --type sdk --cmd start     # 启动 SDK
    python q5_pub_client.py --type sdk --cmd stop      # 停止 SDK

环境变量:
    ROS_DOMAIN_ID — ROS2 domain (默认 211)
"""

import argparse
import json
import os
import sys
import time


def call_service(node, service_type, service_name, request=None, timeout=5.0):
    """同步调用 ROS2 service."""
    import rclpy
    client = node.create_client(service_type, service_name)
    if not client.wait_for_service(timeout_sec=timeout):
        print(f"ERROR: service {service_name} not available")
        return None
    if request is None:
        request = service_type.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    node.destroy_client(client)
    if future.result() is None:
        print(f"ERROR: service {service_name} call timed out")
        return None
    return future.result()


def cmd_mpc_start(node):
    """启动 MPC 算法 — 调用 MPC 启动服务."""
    from std_srvs.srv import Trigger
    result = call_service(node, Trigger, "/mpc/start")
    if result and result.success:
        print("MPC started successfully")
        return True
    print("MPC start failed")
    return False


def cmd_mpc_query(node):
    """查询 MPC 状态."""
    from std_srvs.srv import Trigger
    result = call_service(node, Trigger, "/mpc/status")
    if result:
        status = 1 if result.success else 0
        print(f"MPC status: {status} ({'running' if status else 'stopped'})")
        return status
    return 0


def cmd_mpc_reset(node, mode: int):
    """Reset MPC — mode 0=抬手, mode 1=放手."""
    from std_srvs.srv import Trigger

    # 先查询 MPC 是否启动
    status = cmd_mpc_query(node)
    if not status:
        print("WARN: MPC not running, attempting to start...")
        if not cmd_mpc_start(node):
            return False
        time.sleep(2)

    # 发布 reset 指令
    result = call_service(node, Trigger, "/mpc/reset")
    if result and result.success:
        mode_str = "抬手" if mode == 0 else "放手"
        print(f"MPC reset: {mode_str} (mode={mode})")
        return True
    print("MPC reset failed")
    return False


def cmd_mpc_stop(node):
    """停止 MPC 算法."""
    from std_srvs.srv import Trigger
    result = call_service(node, Trigger, "/mpc/stop")
    if result and result.success:
        print("MPC stopped successfully")
        return True
    print("MPC stop failed")
    return False


def cmd_sdk_start(node):
    """启动 SDK."""
    from std_srvs.srv import Trigger
    result = call_service(node, Trigger, "/sdk/start")
    if result and result.success:
        print("SDK started successfully")
        return True
    print("SDK start failed")
    return False


def cmd_sdk_stop(node):
    """停止 SDK."""
    from std_srvs.srv import Trigger
    result = call_service(node, Trigger, "/sdk/stop")
    if result and result.success:
        print("SDK stopped successfully")
        return True
    print("SDK stop failed")
    return False


def main():
    parser = argparse.ArgumentParser(description="Q5 MPC/SDK control script")
    parser.add_argument("--type", choices=["mpc", "sdk"], required=True,
                        help="控制类型: mpc 或 sdk")
    parser.add_argument("--cmd", choices=["start", "stop", "query", "reset"],
                        required=True, help="控制命令")
    parser.add_argument("--mode", type=int, choices=[0, 1], default=0,
                        help="Reset 模式: 0=抬手(默认), 1=放手")
    args = parser.parse_args()

    domain_id = int(os.environ.get("ROS_DOMAIN_ID", "211"))

    import rclpy
    from rclpy.node import Node

    rclpy.init(domain_id=domain_id)
    node = Node("q5_pub_client", start_parameter_services=False)

    try:
        if args.type == "mpc":
            if args.cmd == "start":
                success = cmd_mpc_start(node)
            elif args.cmd == "query":
                cmd_mpc_query(node)
                success = True
            elif args.cmd == "reset":
                success = cmd_mpc_reset(node, args.mode)
            elif args.cmd == "stop":
                success = cmd_mpc_stop(node)
        elif args.type == "sdk":
            if args.cmd == "start":
                success = cmd_sdk_start(node)
            elif args.cmd == "stop":
                success = cmd_sdk_stop(node)
            else:
                print(f"ERROR: SDK does not support cmd '{args.cmd}'")
                success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
