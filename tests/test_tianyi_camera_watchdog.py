"""天轶头部相机：收不到帧时要自己重启主机的 orbbec 服务。

2026-10-15 实测的一次故障，形状是「所有健康指标都正常，只有画面没了」：

    14:34:18  厂商 Orbbec 节点（component_container，ppid=start_orbbec_camera.sh）
              在 CPU 0 上死转，饿死 RCU grace-period kthread 与 timer softirq
    14:34:39  rcu_preempt self-detected stall on CPU 6（5251 jiffies，HZ=250 → 21 秒）
    之后      相机一帧不出，而 systemd 报 active/running、NRestarts=0、日志零报错、
              lsusb 有设备、我们的订阅也正常挂着（Subscription count: 2）
    19:03     systemctl restart orbbec_head → 立刻 70 帧/6s

唯一能发现它的办法是量帧率，所以这里加一个看门狗。两条边界各有代价，用例分别钉住：
重启得太积极 → 每次抖动都掐一次流；重启得不够 → 画面一直黑着等人来发现。

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_tianyi_camera_watchdog.py -q
"""

from __future__ import annotations

import ast
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT / "x-humanoid" / "tianyi2.0"


def load_camera_class():
    """只把 CameraPlugin 这一个类取出来执行 —— device.py 整体导入要 rclpy。"""
    tree = ast.parse((BUNDLE / "device.py").read_text(encoding="utf-8"))
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == "CameraPlugin")
    module = ast.Module(
        body=[ast.ImportFrom(module="__future__",
                             names=[ast.alias(name="annotations")], level=0), cls],
        type_ignores=[])
    ns = {"threading": threading, "time": time, "Node": object}
    exec(compile(ast.fix_missing_locations(module), str(BUNDLE / "device.py"), "exec"), ns)
    return ns["CameraPlugin"]


CameraPlugin = load_camera_class()


def make_plugin(*, grace=0.3, stall=0.3, poll=0.02):
    p = CameraPlugin.__new__(CameraPlugin)
    p._running = False
    p._latest_frame = None
    p._frame_lock = threading.Lock()
    p._last_frame_at = 0.0
    p._armed_at = 0.0
    p._watchdog_thread = None
    p._watchdog_restarts = 0
    p._watchdog_gave_up = False
    p._FIRST_FRAME_GRACE_S = grace
    p._FRAME_STALL_S = stall
    p._WATCHDOG_POLL_S = poll

    p.restarts = []
    p._restart_orbbec_service = lambda: (p.restarts.append(time.time()), None)[1]
    return p


def arm(p):
    """等价于 start() 里与看门狗有关的那几行，不碰 ROS。"""
    p._running = True
    p._armed_at = time.time()
    p._last_frame_at = 0.0
    p._watchdog_restarts = 0
    p._watchdog_gave_up = False
    t = threading.Thread(target=p._watchdog_loop, daemon=True)
    t.start()
    p._watchdog_thread = t
    return t


def wait_for(pred, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.01)
    return False


# ── 有帧：绝不能重启 ─────────────────────────────────────────────────────────

def test_frames_arriving_never_restarts_the_service():
    """画面正常时重启服务 = 主动把流掐掉。这条边界比「发现得慢」严重得多。"""
    p = make_plugin(grace=0.3, stall=0.3, poll=0.02)
    arm(p)
    # 模拟 30fps 的回调持续喂帧
    stop = threading.Event()

    def feed():
        while not stop.is_set():
            p._on_image_grab(object())
            time.sleep(0.01)

    t = threading.Thread(target=feed, daemon=True)
    t.start()
    try:
        time.sleep(1.2)          # 远超 stall 阈值
    finally:
        stop.set()
        t.join(timeout=1)
    p._running = False
    assert p.restarts == [], f"有帧却重启了 {len(p.restarts)} 次"


def test_first_frame_grace_is_not_the_stall_threshold():
    """arm 之后服务可能正在重启（device init 实测 1.6s）+ DDS 发现，
    这段时间没有帧是正常的，不能按断流阈值判。"""
    p = make_plugin(grace=1.0, stall=0.1, poll=0.02)
    arm(p)
    time.sleep(0.5)              # 已经超过 stall(0.1) 但还在 grace(1.0) 里
    assert p.restarts == [], "首帧宽限被当成断流阈值用了"
    assert wait_for(lambda: len(p.restarts) == 1), "宽限到点后仍然没有重启"
    p._running = False


# ── 无帧：重启一次，且不连环重启 ─────────────────────────────────────────────

def test_no_frames_restarts_once_then_waits_for_a_full_grace():
    """重启之后要重新给满首帧宽限 —— 否则下一次轮询立刻又判一次超时。"""
    p = make_plugin(grace=0.4, stall=0.1, poll=0.02)
    arm(p)
    assert wait_for(lambda: len(p.restarts) >= 1), "无帧却没有重启"
    first = p.restarts[0]
    assert wait_for(lambda: len(p.restarts) >= 2, timeout=3), "第二次重启没来"
    gap = p.restarts[1] - first
    p._running = False
    assert gap >= 0.35, f"两次重启只隔了 {gap:.2f}s，没有重新走首帧宽限"


def test_restarts_are_capped_and_the_giving_up_is_announced(capsys):
    """重启两次还不回来就不是这条路能修的了。继续重启只会每 15 秒掉一次流，
    并且把真实原因埋在刷屏里 —— 所以要停手，而且要说出来。"""
    p = make_plugin(grace=0.1, stall=0.1, poll=0.02)
    arm(p)
    assert wait_for(lambda: p._watchdog_gave_up, timeout=5), "没有放弃，可能在连环重启"
    n_at_giveup = len(p.restarts)
    time.sleep(0.5)
    p._running = False
    assert n_at_giveup == CameraPlugin._MAX_WATCHDOG_RESTARTS, n_at_giveup
    assert len(p.restarts) == n_at_giveup, "放弃之后还在重启"
    out = capsys.readouterr().out
    assert "看门狗放弃" in out and "dmesg" in out, out


# ── 停掉的卡片不许碰主机服务 ─────────────────────────────────────────────────

def test_a_stopped_card_never_restarts_the_host_service():
    """`stop()` 之后看门狗必须退出。停掉的卡片去重启主机服务，会把别的
    使用者（快照卡、厂商自己的工具）的流一起掐掉。"""
    p = make_plugin(grace=0.1, stall=0.1, poll=0.02)
    t = arm(p)
    assert wait_for(lambda: len(p.restarts) >= 1)
    p._running = False           # 等价于 stop()
    assert wait_for(lambda: not t.is_alive(), timeout=2), "看门狗线程没有随 stop 退出"
    n = len(p.restarts)
    time.sleep(0.4)
    assert len(p.restarts) == n, "卡片停掉之后看门狗还在重启主机服务"


def test_watchdog_reference_is_ignored_before_the_first_arm():
    """从没 arm 过（_armed_at == 0）时不能把 0 当成「很久以前」而立刻重启。"""
    p = make_plugin(grace=0.1, stall=0.1, poll=0.02)
    p._running = True
    t = threading.Thread(target=p._watchdog_loop, daemon=True)
    t.start()
    time.sleep(0.3)
    p._running = False
    assert p.restarts == [], "没 arm 过就重启了主机服务"


# ── 源码层面的不变量 ────────────────────────────────────────────────────────

def test_restart_helper_is_unconditional_unlike_ensure():
    """`_ensure_orbbec_service` 是「active 就不碰」，看门狗要的恰恰是
    「它看着 active 但其实死了」。两者不能复用同一个函数。"""
    src = (BUNDLE / "device.py").read_text(encoding="utf-8")
    cam = src[src.index("class CameraPlugin"):src.index("class CameraSnapshotPlugin")]
    assert "systemctl\", \"restart\", \"orbbec_head.service\"" in cam.replace("'", '"')
    assert "_restart_orbbec_service" in cam
    # 看门狗不得去重建订阅（start 的注释解释了为什么重建不可靠）
    wd = cam[cam.index("def _watchdog_loop"):]
    for forbidden in ("create_subscription", "destroy_subscription", "destroy_node"):
        assert forbidden not in wd, forbidden
