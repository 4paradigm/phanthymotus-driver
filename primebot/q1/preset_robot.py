"""Q1 motion transport: vendor DDS configuration never leaks into Motus DDS."""
import math
import os
import time
from pathlib import Path

from preset_postures import MODE_NAMES, PresetController

SERVICE_PREFIX = "/aimdk_5Fmsgs/srv/"
SOURCE = "motus_q1_preset_postures"
FULL_IMU = "/aima/hal/imu/state"
TORSO_ORIENTATION = "/torso_imu_orientation"
MC_STATE = "/aima/mc/common/state"
FRESH_SECONDS = .5


def upright_quaternion(msg):
    if msg.orientation_covariance[0] == -1:
        return None
    return upright_orientation(msg.orientation)


def upright_orientation(orientation):
    q = [getattr(orientation, name) for name in ("w", "x", "y", "z")]
    if not all(math.isfinite(v) for v in q):
        return None
    norm = math.hypot(*q)
    if norm < 1e-8:
        return None
    w, x, y, z = [v / norm for v in q]
    return 1 - 2 * (x * x + y * y) > .85


def action_info(info):
    return {"value": info.current_action.value, "description": info.action_desc,
            "status": info.status.value, "switch_ready": info.switch_ready, "source": info.source}


def reconcile_action(rpc, common, now):
    """Keep the RPC and a fresh native state consistent; never invent readiness."""
    result = {**rpc, "rpc_switch_ready": rpc["switch_ready"], "switch_ready_source": "GetMcAction"}
    if common is not None and 0 <= now - common[0] <= FRESH_SECONDS:
        state = common[1]
        consistent = (rpc["value"], rpc["status"]) == (state["action"]["value"], state["action"]["status"])
        result.update(switch_ready=state["action"]["switch_ready"], switch_ready_source=MC_STATE,
                      state_consistent=consistent,
                      telemetry_age_s=round(now - common[0], 3),
                      is_fallen=state["is_fallen"], is_lifted=state["is_lifted"],
                      is_moving=state["is_moving"])
        if not consistent:
            result["switch_ready"] = False
    return result


def select_posture(samples, now):
    for topic in (FULL_IMU, TORSO_ORIENTATION):
        sample = samples.get(topic)
        if sample and 0 <= now - sample[0] <= FRESH_SECONDS and sample[1] is not None:
            return {"upright": sample[1], "source": topic, "age_s": round(now - sample[0], 3)}
    return {"upright": None, "source": None, "age_s": None,
            "reason": "未收到新鲜有效的姿态数据；不能据此判断未站立"}


class RobotTransport:
    def __init__(self, domain, cancelled, deadline):
        from ament_index_python.packages import get_package_share_directory
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from aimdk_msgs.srv import GetMcAction, SetMcAction, SetMcPresetMotion, SetMcMotion
        from aimdk_msgs.msg import McCommonState
        from sensor_msgs.msg import Imu
        from nav_msgs.msg import Odometry

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
        self.imu = {}
        self.common = None
        self.matched = set()
        self.last_response = None
        self.node.create_subscription(Imu, FULL_IMU, self._imu, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, TORSO_ORIENTATION, self._torso, qos_profile_sensor_data)
        self.node.create_subscription(McCommonState, MC_STATE, self._common, qos_profile_sensor_data)

    def _imu(self, msg):
        self.imu[FULL_IMU] = (time.monotonic(), upright_quaternion(msg))

    def _torso(self, msg):
        self.imu[TORSO_ORIENTATION] = (time.monotonic(), upright_orientation(msg.pose.pose.orientation))

    def _common(self, msg):
        self.common = (time.monotonic(), {"action": action_info(msg.action_info),
                        "is_fallen": msg.is_fallen, "is_lifted": msg.lift_state.is_lifted,
                        "is_moving": msg.is_moving})

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

    def posture(self):
        end = time.monotonic() + 1
        while True:
            now = time.monotonic()
            result = select_posture(self.imu, now)
            if result["upright"] is not None or now >= end:
                return result
            self.pause(.05)

    def upright(self):
        return self.posture()["upright"] is True

    def rpc(self, name, request):
        client = self.clients[name]
        end = time.monotonic() + 8
        while not client.service_is_ready() and time.monotonic() < end:
            self.pause(.05)
        if not client.service_is_ready():
            self.matched.discard(name)
            raise RuntimeError(f"{name} 服务发现超时；可能是 DDS 连接或官方运控服务暂不可用")
        if name not in self.matched:
            self.pause(.2)  # Official SDK: endpoint matching can lag discovery.
            self.matched.add(name)
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
        # Only a read-only query may be retried. A motion RPC is always sent once.
        for attempt in range(2):
            try:
                response = self.rpc("GetMcAction", request)
                break
            except TimeoutError:
                if attempt:
                    raise
                self.pause(.2)
        if response.header.code != 0:
            raise RuntimeError(f"GetMcAction 查询失败：{response.header.code}")
        return reconcile_action(action_info(response.info), self.common, time.monotonic())

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


def run_worker(domain, channel, cancelled):
    from common import logsafe
    logsafe.install(check_fd=False)
    transport = None
    try:
        transport = RobotTransport(domain, cancelled, float("inf"))
        # Creating a participant is not the same as discovering the robot.
        # Keep cold-start discovery outside any queued user motion request.
        discovery_end = time.monotonic() + 20
        while (not all(transport.services().values())
               and time.monotonic() < discovery_end and transport.rclpy.ok()):
            transport.rclpy.spin_once(transport.node, timeout_sec=.05)
        # Discovery can precede completion of the RPC writer/reader matching.
        # Warm all clients before accepting any motion, without sending one.
        matching_end = time.monotonic() + 1
        while time.monotonic() < matching_end and transport.rclpy.ok():
            transport.rclpy.spin_once(transport.node, timeout_sec=.05)
        channel.send({"event": "initialized"})
        while True:
            if channel.poll():
                request = channel.recv()
                if request is None:
                    break
                action, transport.deadline = request
                controller = PresetController(
                    transport, lambda data: channel.send({"event": "progress", "data": data}))
                channel.send({"event": "result", "data": controller.execute(action)})
            # Continue discovery and receive fresh state between explicit requests.
            transport.rclpy.spin_once(transport.node, timeout_sec=.05)
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        channel.send({"event": "result", "data": {"state": "error", "message": str(exc),
                                                "execution_confirmed": False}})
    finally:
        if transport is not None:
            transport.close()
        channel.close()
