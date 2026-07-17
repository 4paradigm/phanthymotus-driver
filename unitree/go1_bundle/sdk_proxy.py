"""
sdk_proxy.py — Go1HighSdkClient 的子进程代理（go1_bundle）。

主进程线程数多（ROS2 executor、camera 收流、MCP HTTP server 线程池等），
robot_interface C 扩展的 50Hz UDP 收发循环持续争 GIL，导致其他线程响应变慢。
把 Go1HighSdkClient 整体搬到独立 spawn 子进程，主进程通过两个 Queue 与之通信，
彻底消除 robot_interface 对主进程 GIL 的影响。

对外接口与 Go1HighSdkClient 完全相同，所有卡片文件零改动。
"""

import multiprocessing
import threading


# ── 子进程入口 ─────────────────────────────────────────────────────────────────

def _sdk_worker(cmd_q: multiprocessing.Queue, result_q: multiprocessing.Queue,
                network_iface: str, target_ip: str, target_port: int, local_port: int):
    """子进程：持有 Go1HighSdkClient，处理来自主进程的命令队列。"""
    from go1_sdk_client import Go1HighSdkClient

    client = Go1HighSdkClient(
        network_iface=network_iface,
        target_ip=target_ip,
        target_port=target_port,
        local_port=local_port,
    )
    client.start()

    # 向主进程发送就绪信号，携带 available 状态
    result_q.put({"available": client.available})
    print(f"[SdkWorker] ready ({'live' if client.available else 'STUB'})", flush=True)

    while True:
        try:
            cmd = cmd_q.get()
        except Exception:
            break

        if cmd is None:
            # None 哨兵：退出
            break

        name = cmd.get("cmd")
        args = cmd.get("args", [])
        kwargs = cmd.get("kwargs", {})

        try:
            if name == "snapshot":
                result_q.put({"result": client.snapshot()})

            elif name == "diagnostics":
                result_q.put({"result": client.diagnostics()})

            elif name == "move":
                vx, vy, vyaw = args[0], args[1], args[2]
                gait = args[3] if len(args) > 3 else kwargs.get("gait", None)
                result_q.put({"result": client.move(vx, vy, vyaw, gait=gait)})

            elif name == "stop_move":
                client.stop_move()
                result_q.put({"result": None})

            elif name == "set_posture":
                mode = args[0]
                euler = args[1] if len(args) > 1 else kwargs.get("euler", (0.0, 0.0, 0.0))
                body_height = args[2] if len(args) > 2 else kwargs.get("body_height", 0.0)
                foot_raise = args[3] if len(args) > 3 else kwargs.get("foot_raise", 0.0)
                speed_level = args[4] if len(args) > 4 else kwargs.get("speed_level", 0)
                result_q.put({"result": client.set_posture(
                    mode, euler=euler, body_height=body_height,
                    foot_raise=foot_raise, speed_level=speed_level,
                )})

            elif name == "set_gait":
                result_q.put({"result": client.set_gait(args[0])})

            elif name == "desired_gait":
                result_q.put({"result": client.desired_gait()})

            else:
                result_q.put({"error": f"unknown command: {name}"})

        except Exception as e:  # noqa: BLE001
            print(f"[SdkWorker] error handling '{name}': {e}", flush=True)
            result_q.put({"error": str(e)})

    client.stop()
    print("[SdkWorker] stopped", flush=True)


# ── 主进程侧薄壳 ───────────────────────────────────────────────────────────────

class SdkProxy:
    """主进程侧代理：接口与 Go1HighSdkClient 完全相同，内部走 subprocess Queue IPC。

    所有卡片传入的 client 替换为本对象后，无需任何改动。
    """

    def __init__(self, network_iface: str = "",
                 target_ip: str = "192.168.123.161",
                 target_port: int = 8082,
                 local_port: int = 8090):
        ctx = multiprocessing.get_context("spawn")
        self._cmd_q = ctx.Queue()
        self._result_q = ctx.Queue()
        self._lock = threading.Lock()  # 串行化所有 put+get，避免乱序
        self._stopped = False

        self._proc = ctx.Process(
            target=_sdk_worker,
            args=(self._cmd_q, self._result_q, network_iface, target_ip, target_port, local_port),
            daemon=True,
            name="go1_sdk_worker",
        )
        self._proc.start()

        # 等待子进程就绪信号（最多 15s，robot_interface 初始化可能需要时间）
        try:
            ready = self._result_q.get(timeout=15.0)
            self.available = ready.get("available", False)
        except Exception as e:
            print(f"[SdkProxy] subprocess did not become ready: {e}", flush=True)
            self.available = False

    # start() 是空操作——子进程在 __init__ 里已经启动
    def start(self) -> None:
        pass

    def stop(self) -> None:
        with self._lock:
            if self._stopped:
                return
            self._stopped = True
        try:
            self._cmd_q.put(None)
            self._proc.join(timeout=3)
        except Exception:
            pass

    # ── 内部调用辅助 ────────────────────────────────────────────────────────────

    def _call(self, cmd: str, args=None, kwargs=None, timeout: float = 5.0):
        with self._lock:
            if self._stopped:
                return None
            self._cmd_q.put({"cmd": cmd, "args": args or [], "kwargs": kwargs or {}})
            try:
                r = self._result_q.get(timeout=timeout)
            except Exception as e:
                print(f"[SdkProxy] timeout waiting for '{cmd}': {e}", flush=True)
                return None
            if "error" in r:
                print(f"[SdkProxy] '{cmd}' error: {r['error']}", flush=True)
                return None
            return r["result"]

    # ── 公共接口（与 Go1HighSdkClient 完全同名同签名）─────────────────────────

    def snapshot(self) -> dict:
        result = self._call("snapshot", timeout=2.0)
        return result if result is not None else {"fresh": False}

    def diagnostics(self) -> dict:
        result = self._call("diagnostics", timeout=2.0)
        return result if result is not None else {}

    def move(self, vx=0.0, vy=0.0, vyaw=0.0, gait=None):
        # move 是控制卡 50ms 节奏重发的热路径，超时要短
        return self._call("move", args=[vx, vy, vyaw, gait], timeout=1.0)

    def stop_move(self):
        self._call("stop_move", timeout=1.0)

    def set_posture(self, mode, euler=(0.0, 0.0, 0.0), body_height=0.0,
                    foot_raise=0.0, speed_level=0):
        return self._call("set_posture",
                          args=[mode, list(euler), body_height, foot_raise, speed_level],
                          timeout=2.0)

    def set_gait(self, gait) -> int:
        result = self._call("set_gait", args=[int(gait)], timeout=2.0)
        return result if result is not None else int(gait)

    def desired_gait(self) -> int:
        result = self._call("desired_gait", timeout=2.0)
        return result if result is not None else 1