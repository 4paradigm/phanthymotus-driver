from __future__ import annotations

#!/usr/bin/env python3
"""
ros_bridge.py — 把 rclpy(ROS2)隔离在这一个文件里。

设计目的:除本文件外,**任何驱动模块都不得在顶层 import rclpy**。
  • 有 rclpy(机器狗镜像)   → 真发 ROS2 topic(std_msgs/String 装 JSON)。
  • 无 rclpy(开发 Mac 等)  → 发布变 no-op,用普通线程定时器跑 producer。
这样整套驱动代码能在没装 ROS2 的机器上直接 import / 跑 / 测。

sensor 插件统一用法:
    node = bridge.add_sensor(name, topic, hz, producer)   # producer() -> dict | None
    ...                                                     # node.last 缓存最近一次数据(测试可读)
actuator 插件不需要 bridge(直接在 dispatch 里调 go1_hl / adapter)。

生命周期(由 main.py 调):
    bridge = RosBridge(force_mode)      # auto|real|fake
    bridge.start()                      # real: rclpy.init() + 建 executor
    ... 构造插件(内部 add_sensor) ...
    bridge.spin_background()            # real: 后台 spin;fake: 启动各 sensor 线程
    bridge.shutdown()
"""
import json
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String
    _HAVE_RCLPY = True
except Exception:
    _HAVE_RCLPY = False


def have_rclpy() -> bool:
    return _HAVE_RCLPY


# ── 真实模式:rclpy Node(每个 sensor 一个 node + timer) ─────────────────────
if _HAVE_RCLPY:
    # 低延迟 QoS:best-effort、只留最近、易失(与 go2 一致)
    _LOW_LAT_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=200,
        durability=DurabilityPolicy.VOLATILE,
    )

    class _RealSensor(Node):
        def __init__(self, name, topic, hz, producer):
            super().__init__(name)
            self._topic = topic
            self._producer = producer
            self.last = None
            self._pub = self.create_publisher(String, topic, _LOW_LAT_QOS)
            self.create_timer(1.0 / max(float(hz), 0.001), self.tick)

        def tick(self):
            try:
                data = self._producer()
                if data is None:
                    return
                self.last = data
                msg = String(); msg.data = json.dumps(data)
                self._pub.publish(msg)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"[{self._topic}] publish error: {e}")


# ── 离线模式:线程定时器 + no-op 发布 ────────────────────────────────────────
class _FakeSensor:
    """无 rclpy 时的替身:同样的 add_sensor 契约,tick() 缓存最近数据供测试/离线读取。"""

    def __init__(self, name, topic, hz, producer):
        self.name = name
        self._topic = topic
        self._hz = float(hz)
        self._producer = producer
        self.last = None
        self._running = False
        self._thread = None
        self._err_shown = False

    def tick(self):
        try:
            data = self._producer()
            self._err_shown = False
            if data is None:
                return
            self.last = data
        except Exception as e:  # noqa: BLE001
            if not self._err_shown:   # 去重:同一错误只打一次,不刷屏
                print(f"[ros_bridge/fake] {self._topic} producer error: {e}", flush=True)
                self._err_shown = True

    def _loop(self):
        dt = 1.0 / max(self._hz, 0.001)
        while self._running:
            t0 = time.monotonic()
            self.tick()
            s = dt - (time.monotonic() - t0)
            if s > 0:
                time.sleep(s)

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name=self.name)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1)


class RosBridge:
    def __init__(self, force_mode: str = "auto"):
        mode = (force_mode or "auto").lower()
        if mode == "fake":
            self.real = False
        elif mode == "real":
            if not _HAVE_RCLPY:
                raise RuntimeError("force_mode=real 但 rclpy 不可用")
            self.real = True
        else:  # auto
            self.real = _HAVE_RCLPY
        self._sensors = []
        self._executor = None
        self._spin_thread = None
        self._started = False

    def start(self) -> None:
        if self.real:
            rclpy.init()
            from rclpy.executors import MultiThreadedExecutor
            self._executor = MultiThreadedExecutor()
        self._started = True
        print(f"[ros_bridge] mode={'real(rclpy)' if self.real else 'fake(offline)'}", flush=True)

    def add_sensor(self, name: str, topic: str, hz: float, producer):
        """注册一个周期性 sensor。producer()->dict|None,返回值发到 topic(JSON)。"""
        if self.real:
            node = _RealSensor(name, topic, hz, producer)
            if self._executor is not None:
                self._executor.add_node(node)
        else:
            node = _FakeSensor(name, topic, hz, producer)
        self._sensors.append(node)
        return node

    def spin_background(self) -> None:
        if self.real:
            def _spin():
                while rclpy.ok():
                    self._executor.spin_once(timeout_sec=0.1)
            self._spin_thread = threading.Thread(target=_spin, daemon=True, name="ros_spin")
            self._spin_thread.start()
        else:
            for s in self._sensors:
                s.start()

    def shutdown(self) -> None:
        if self.real:
            try:
                if self._executor is not None:
                    self._executor.shutdown()
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass
        else:
            for s in self._sensors:
                s.stop()
