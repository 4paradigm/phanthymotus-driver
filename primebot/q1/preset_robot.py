"""Q1 motion transport: vendor DDS configuration never leaks into Motus DDS."""
import math
import os
import time
from pathlib import Path

from preset_postures import MODE_NAMES, PresetController

SERVICE_PREFIX = "/aimdk_5Fmsgs/srv/"
SOURCE = "motus_q1_preset_postures"


def upright_quaternion(msg):
    if msg.orientation_covariance[0] == -1:
        return False
    q = [getattr(msg.orientation, name) for name in ("w", "x", "y", "z")]
    if not all(math.isfinite(v) for v in q):
        return False
    norm = math.hypot(*q)
    if norm < 1e-8:
        return False
    w, x, y, z = [v / norm for v in q]
    return 1 - 2 * (x * x + y * y) > .85


class RobotTransport:
    def __init__(self, domain, cancelled, deadline):
        from ament_index_python.packages import get_package_share_directory
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from aimdk_msgs.srv import GetMcAction, SetMcAction, SetMcPresetMotion, SetMcMotion
        from sensor_msgs.msg import Imu

        profile = Path(get_package_share_directory("aimdk_msgs")) / "config/fastdds_profiles.xml"
        for key in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
            os.environ[key] = os.environ.get("Q1_FASTDDS_PROFILE", str(profile))
        self.rclpy = rclpy
        self.cancelled, self.deadline = cancelled, deadline
        rclpy.init(domain_id=domain)
        self.node = rclpy.create_node("q1_preset_request", start_parameter_services=False)
        self.types = {t.__name__: t for t in (GetMcAction, SetMcAction, SetMcPresetMotion, SetMcMotion)}
        self.clients = {name: self.node.create_client(t, SERVICE_PREFIX + name)
                        for name, t in self.types.items()}
        self.imu = None
        self.last_response = None
        self.node.create_subscription(Imu, "/aima/hal/imu/state", self._imu, qos_profile_sensor_data)

    def _imu(self, msg):
        self.imu = (time.monotonic(), upright_quaternion(msg))

    def check(self):
        if self.cancelled.is_set():
            raise RuntimeError("请求已取消，停止后续调度；已发送的硬件指令不保证停止")
        if time.monotonic() >= self.deadline:
            raise TimeoutError("请求期限已过，不发送后续命令")

    def pause(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.check()
            self.rclpy.spin_once(self.node, timeout_sec=min(.05, max(0, end - time.monotonic())))

    def services(self):
        return {name: client.service_is_ready() for name, client in self.clients.items()}

    def upright(self):
        end = time.monotonic() + 1
        while self.imu is None and time.monotonic() < end:
            self.pause(.05)
        return bool(self.imu and time.monotonic() - self.imu[0] <= .5 and self.imu[1])

    def rpc(self, name, request):
        client = self.clients[name]
        end = time.monotonic() + 3
        while not client.service_is_ready() and time.monotonic() < end:
            self.pause(.05)
        if not client.service_is_ready():
            raise RuntimeError(f"{name} 服务不可用；请检查开发模式和官方运控进程")
        self.check()
        future = client.call_async(request)
        try:
            end = time.monotonic() + 3
            while not future.done() and time.monotonic() < end:
                self.pause(.05)
            self.check()
            if not future.done():
                raise TimeoutError(f"{name} 回应超时，结果未知；不自动重试")
            response = future.result()
            if response is None:
                raise RuntimeError(f"{name} 返回空响应")
            return response
        finally:
            if not future.done():
                future.cancel()
                client.remove_pending_request(future)

    def query(self):
        request = self.types["GetMcAction"].Request()
        request.request.header.stamp = self.node.get_clock().now().to_msg()
        response = self.rpc("GetMcAction", request)
        if response.header.code != 0:
            raise RuntimeError(f"GetMcAction 查询失败：{response.header.code}")
        info = response.info
        return {"value": info.current_action.value, "description": info.action_desc,
                "status": info.status.value, "switch_ready": info.switch_ready, "source": info.source}

    def _request(self, name):
        request = self.types[name].Request()
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.source = SOURCE
        return request

    def _send(self, name, request):
        result = self.rpc(name, request).response
        self.last_response = {"code": result.header.code, "state": result.state.value,
                              "reason": result.state.reason, "task_id": result.task_id}
        return self.last_response

    def set_mode(self, target):
        request = self._request("SetMcAction")
        request.command.action.value = target
        request.command.action_desc = MODE_NAMES[target]
        return self._send("SetMcAction", request)

    def preset(self, number, interrupt):
        request = self._request("SetMcPresetMotion")
        request.motion.value = number
        request.area.value = 0
        request.interrupt = interrupt
        request.ani_path = ""
        return self._send("SetMcPresetMotion", request)

    def motion(self, name):
        request = self._request("SetMcMotion")
        request.type = request.MIMIC_QY
        request.operation = request.EXECUTION
        request.motion = name
        request.interrupt = False
        request.model = request.resource = ""
        return self._send("SetMcMotion", request)

    def close(self):
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def run_request(domain, action, output, cancelled, deadline):
    from common import logsafe
    logsafe.install(check_fd=False)
    transport = None
    try:
        transport = RobotTransport(domain, cancelled, deadline)
        controller = PresetController(transport, lambda data: output.send({"event": "progress", "data": data}))
        result = controller.execute(action)
        output.send({"event": "result", "data": result})
    except Exception as exc:
        output.send({"event": "result", "data": {"state": "error", "message": str(exc),
                                                   "execution_confirmed": False}})
    finally:
        if transport is not None:
            transport.close()
        output.close()
