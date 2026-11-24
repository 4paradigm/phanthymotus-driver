"""Read-only reconnaissance for the As2-W accompany mode.

Why this file exists
--------------------
The shipped SDK exposes no accompany/follow entry point (all 21 sport api ids
were enumerated, plus every other service in the bundle), yet the robot clearly
runs the behaviour: the manual says short-press ``M`` on the companion remote
and the head LED turns solid purple. So the behaviour exists in firmware and
the question is only how to reach it from software.

Three things can be learned without touching the robot's actuators:

1. ``--services``  Enumerate the DDS endpoints the robot actually hosts. Every
   Unitree RPC service is a topic pair ``rt/api/<service>/request`` and
   ``rt/api/<service>/response`` (see ``unitree_sdk2py/core/channel_name.py``),
   so a subscription-endpoint dump reveals undocumented services. This is the
   only way to find a service that the SDK does not wrap.
2. ``--modes``     Ask the ``motion_switcher`` service which motion modes are
   available (``CheckMode``, api 1001). Read-only.
3. ``--watch``     Record ``mode`` from ``rt/lf/sportmodestate`` and ``fsm_name``
   from ``GetState`` while a human presses ``M``. This is the calibration run:
   it either names the accompany state or proves it is invisible from both
   channels.

The only command that sends a *command* is ``--probe-ids``, which sends nothing
but an empty parameter to api ids that the SDK does not define. It refuses to
run without three explicit flags; see ``_guard_armed``.

Run inside the container on the robot:

    python3 /work/accompany_probe.py --services --modes
    python3 /work/accompany_probe.py --watch 60 --iface eth0
    python3 /work/accompany_probe.py --probe-ids 1022 --armed \\
        --robot-on-stand --remote-in-hand
"""

import argparse
import json
import re
import sys
import threading
import time


def _install_logsafe():
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except (ImportError, TypeError):
        pass


def _safe_payload(value, limit=200):
    """Render an untrusted remote payload without control characters.

    An undocumented vendor endpoint can answer with anything, including NUL and
    terminal escape sequences. Undicipped bytes would corrupt the container log
    stream, so escape to ASCII and cap the length. The raw value still lands in
    the evidence JSON, which is read offline rather than streamed.
    """
    text = value if isinstance(value, str) else str(value)
    return text.encode("unicode_escape").decode("ascii")[:limit]


# ---------------------------------------------------------------- constants

#: ``mode`` field of ``rt/lf/sportmodestate``, from the vendor example
#: ``unitree_sdk2_python/example/as2/sport/as2_sport_state.py``. An index outside
#: this tuple is a mode the vendor never documented, which is exactly what an
#: undocumented accompany state would look like.
MODE_NAMES = (
    "PASSIVE",
    "STAND_DOWN",
    "STAND_UP",
    "DEFAULT_MODE",
    "RUNNING_MODE",
    "CLIMB_MODE",
    "LEFT_SIDE_GAIT",
    "RIGHT_SIDE_GAIT",
    "HANDSTAND",
    "BIPED_STAND",
    "FRONT_FLIP",
    "BACK_FLIP",
    "RECOVERY",
    "BASE_HEIGHT_CTRL",
)

#: Sport api ids the SDK registers. Any id in 1001..1041 outside this set is a
#: hole in the vendor's public table: possible undocumented command.
SPORT_API_IDS = (
    1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008, 1009,
    1011, 1013, 1015, 1016, 1017, 1018, 1019, 1020, 1021,
    1034, 1040, 1041,
)

_API_TOPIC = re.compile(r"^rt/api/(?P<service>[A-Za-z0-9_]+)/(?P<side>request|response)$")
_SUSPECT = re.compile(r"accompany|follow|iss|escort|track|guide|lead", re.I)

#: Values ``GetState`` has been observed to return in this repository
#: (``device.py`` treats both the plain and the ``AI_`` prefixed family), kept
#: here so the summary can flag a *new* name instead of burying it.
KNOWN_FSM_NAMES = frozenset({
    "PASSIVE", "DAMPING", "AI_DAMPING", "STAND_DOWN", "AI_STAND_DOWN",
    "STAND_UP", "AI_STAND_UP", "BALANCE_STAND", "AI_BALANCE_STAND",
    "RECOVERY_STAND", "AI_RECOVERY_STAND", "FREE_WALK", "AI_FREE_WALK",
    "WALK", "AI_WALK", "RUN", "AI_RUN", "REGULAR_WALK", "REGULAR_RUN",
    "LYING", "FALL", "FALLEN", "AI_FALL", "AI_FALLEN", "SQUAT",
})


# ------------------------------------------------------------ pure helpers

def api_services(topic_names):
    """Map ``rt/api/<service>/<side>`` topic names to the services they imply.

    Returns ``{service: {"request": bool, "response": bool}}``. A robot-hosted
    service always *subscribes* to its ``/request`` topic, so that side being
    present is the signal; ``/response`` only tells us a client is connected.
    """
    services = {}
    for name in topic_names or ():
        match = _API_TOPIC.match(str(name))
        if not match:
            continue
        entry = services.setdefault(match.group("service"),
                                    {"request": False, "response": False})
        entry[match.group("side")] = True
    return services


def suspicious(services):
    """Services whose name hints at following behaviour, sorted for stability."""
    return sorted(name for name in services if _SUSPECT.search(name))


def mode_name(index):
    """Name for a ``mode`` value, or a marker that the value is undocumented."""
    try:
        value = int(index)
    except (TypeError, ValueError):
        return f"<non-integer {index!r}>"
    if 0 <= value < len(MODE_NAMES):
        return MODE_NAMES[value]
    return f"UNDOCUMENTED_{value}"


def undocumented_mode(index):
    """True when ``mode`` falls outside the vendor's 14-entry table."""
    try:
        value = int(index)
    except (TypeError, ValueError):
        return True
    return not 0 <= value < len(MODE_NAMES)


def sport_api_gaps():
    """Sport api ids in the 1001..1041 range that the SDK does not register."""
    defined = set(SPORT_API_IDS)
    return tuple(value for value in range(1001, 1042) if value not in defined)


def is_new_fsm_name(name):
    """True for an ``fsm_name`` this repository has never seen before."""
    return bool(name) and str(name).strip().upper() not in KNOWN_FSM_NAMES


def _guard_armed(armed, robot_on_stand, remote_in_hand):
    """Refuse to send anything unless all three safety statements are given."""
    missing = []
    if not armed:
        missing.append("--armed")
    if not robot_on_stand:
        missing.append("--robot-on-stand")
    if not remote_in_hand:
        missing.append("--remote-in-hand")
    if missing:
        raise RuntimeError(
            "probe-ids 会向固件发送未公开的 api id，可能触发未知动作。"
            f"缺少安全声明：{', '.join(missing)}。"
            "请把机器人架起（轮子离地）、遥控器握在手里、周围无人后再重试。")


# ------------------------------------------------------------ DDS plumbing

def _participant():
    """Reuse the SDK's participant so discovery runs on the same interface."""
    from unitree_sdk2py.core.channel import ChannelFactory  # noqa: WPS433
    factory = ChannelFactory()
    return getattr(factory, "_ChannelFactory__participant", None)


def _endpoint_topics(builtin_topic, wait_s=3.0):
    """Collect ``topic_name`` from a builtin endpoint reader."""
    from cyclonedds.builtin import BuiltinDataReader  # noqa: WPS433
    participant = _participant()
    if participant is None:
        from cyclonedds.domain import DomainParticipant  # noqa: WPS433
        participant = DomainParticipant(0)
    reader = BuiltinDataReader(participant, builtin_topic)
    seen = {}
    deadline = time.monotonic() + max(0.0, wait_s)
    while time.monotonic() < deadline:
        for sample in reader.take(N=256):
            name = getattr(sample, "topic_name", None)
            if name:
                seen[name] = getattr(sample, "type_name", "") or ""
        time.sleep(0.2)
    return seen


def discover_endpoints(wait_s=3.0):
    """Topic name -> type name for every subscription and publication nearby."""
    from cyclonedds.builtin import (  # noqa: WPS433
        BuiltinTopicDcpsPublication, BuiltinTopicDcpsSubscription)

    found = {}
    for topic in (BuiltinTopicDcpsSubscription, BuiltinTopicDcpsPublication):
        try:
            found.update(_endpoint_topics(topic, wait_s=wait_s))
        except Exception as exc:  # pragma: no cover - depends on live DDS
            print(f"[probe] 端点发现失败（{topic}）：{exc}", file=sys.stderr)
    return found


# ------------------------------------------------------------------ command

def cmd_services(args, findings):
    endpoints = discover_endpoints(wait_s=args.wait)
    services = api_services(endpoints)
    order = sorted(services)
    print(f"[probe] 邻近 DDS 端点 {len(endpoints)} 个，其中 rt/api/* 服务 {len(order)} 个")
    for name in order:
        side = services[name]
        mark = "  <== 名称可疑" if _SUSPECT.search(name) else ""
        print(f"  {'S' if side['request'] else '-'}"
              f"{'R' if side['response'] else '-'}  {name}{mark}")
    suspects = suspicious(services)
    if suspects:
        print(f"[probe] 🟢 发现可疑服务：{suspects} "
              "—— 优先对它们做 CheckMode 式只读调用")
    else:
        print("[probe] 未发现名称含 accompany/follow/iss 的服务")
    others = sorted(name for name in endpoints
                    if name.startswith("rt/") and not _API_TOPIC.match(name))
    if others:
        print(f"[probe] 其它 rt/* 话题 {len(others)} 个（前 20）：{others[:20]}")
    findings["endpoints"] = endpoints
    findings["services"] = services
    return 0 if order else 2


def cmd_modes(args, findings):
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (  # noqa: WPS433
        MotionSwitcherClient)
    client = MotionSwitcherClient()
    client.SetTimeout(5.0)
    client.Init()
    code, data = client.CheckMode()
    print(f"[probe] motion_switcher.CheckMode -> code={code}")
    if data is not None:
        # The response comes from the robot, not from us: escape and cap it
        # instead of dumping an unbounded remote blob into the log stream.
        print(f"[probe] 当前/可用模式："
              f"{_safe_payload(json.dumps(data, ensure_ascii=False), 300)}")
    findings["motion_switcher"] = {"code": code, "data": data}
    return 0 if code == 0 else 3


def watch_snapshot(latest_mode, latest_name, elapsed, code, mode_counts):
    """Build one time-correlated watch observation.

    ``latest_mode`` must be the *most recent* callback value, never the most
    frequent one. A short accompany transition after a long baseline is a
    handful of frames against hundreds, so a cumulative winner (``max(counts)``)
    would keep pairing the new ``fsm_name`` with the stale baseline mode and
    write a false ``mode:fsm_name`` mapping into the evidence JSON.

    ``mode_counts`` is kept only for the crop report; it never feeds pairs.
    """
    return {
        "at_s": round(float(elapsed), 3),
        "mode": latest_mode,
        "fsm_name": latest_name or None,
        "code": code,
    }


def watch_mode_lines(modes):
    """Render the mode tally even when no DDS frame ever arrived.

    ``latest["mode"]`` starts as ``None`` on a degraded link. ``None`` cannot
    be compared with ``int`` in ``sorted()`` and does not support ``<3``
    formatting, so the original report crashed before writing the evidence
    JSON it was meant to diagnose the outage with.
    """
    lines = []
    for value in sorted(modes, key=lambda v: (v is not None, v or 0)):
        shown = "?" if value is None else str(value)
        name = "(no frame)" if value is None else mode_name(value)
        lines.append(f"  mode={shown:<3} {name:<20} 帧数 {modes[value]}")
    return lines


def cmd_watch(args, findings):
    """Record mode/fsm_name transitions; the human presses M during the window."""
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_  # noqa: WPS433
    from unitree_sdk2py.core.channel import ChannelSubscriber  # noqa: WPS433
    from unitree_sdk2py.as2.sport.sport_client import SportClient  # noqa: WPS433

    pairs = {}
    modes = {}
    names = {}
    samples = []
    # The subscriber callback and the poll loop run on different threads.
    # Snapshot mode and timestamp together: a fresh mode with an old stamp
    # would produce a misleading mode_age in the evidence.
    latest = {"mode": None, "at_s": 0.0}
    latest_lock = threading.Lock()

    def on_state(msg):
        # Called on the DDS thread: mode and timestamp must move together.
        with latest_lock:
            latest["mode"] = int(getattr(msg, "mode", -1))
            latest["at_s"] = time.monotonic()

    subscriber = ChannelSubscriber("rt/lf/sportmodestate", SportModeState_)
    subscriber.Init(on_state, 10)

    sport = SportClient()
    sport.SetTimeout(2.0)
    sport.Init()

    print(f"[probe] 开始观测 {args.watch:.0f} s。**现在请按顺序操作**：")
    print("        1) 先保持不动 3 s（记录基线）")
    print("        2) 短按伴随遥控器 M 键进入伴随模式，保持 8 s")
    print("        3) 拨一下摇杆退出，再保持 5 s")
    started = time.monotonic()
    last_mode = None
    last_name = None
    while time.monotonic() - started < args.watch:
        state_map = {}
        code = 0
        try:
            code = sport.GetState(state_map)
        except Exception as exc:
            print(f"[probe] GetState 失败：{exc}", file=sys.stderr)
        name = str(state_map.get("fsm_name", "") or "").strip().upper()
        elapsed = time.monotonic() - started
        # Snapshot both fields under one lock so the row is coherent.
        with latest_lock:
            with_mode = latest["mode"]
            mode_at_s = latest["at_s"]
        observation = watch_snapshot(with_mode, name, elapsed, code, modes)
        samples.append(observation)
        modes[with_mode] = modes.get(with_mode, 0) + 1
        if name:
            # GetState can report an FSM name while the SportModeState feed is
            # degraded, so the name tally must not depend on with_mode: folding
            # it into the pair count would discard valid evidence and let the
            # verdict fall through to "both channels invisible".
            names[name] = names.get(name, 0) + 1
        if with_mode != last_mode or name != last_name:
            label = mode_name(with_mode) if with_mode is not None else "?"
            mode_age = elapsed - mode_at_s
            print(f"  [{elapsed:6.2f}s] mode={with_mode} ({label})  "
                  f"fsm_name={name or '(空)'}  code={code}  "
                  f"mode_age={mode_age:.2f}s")
            if with_mode is not None and name:
                pairs[f"{with_mode}:{name}"] = pairs.get(f"{with_mode}:{name}", 0) + 1
            last_mode, last_name = with_mode, name
        time.sleep(0.25)

    print("[probe] -------- 本次观测到 --------")
    for line in watch_mode_lines(modes):
        print(line)
    for name in sorted(names):
        flag = "  <== 本仓库从未见过" if is_new_fsm_name(name) else ""
        print(f"  fsm_name={name}{flag}")
    # None is the "no frame arrived" bucket, not a mode value: a degraded link
    # must not be reported as a vendor-secret mode.
    odd = [value for value in modes
           if value is not None and undocumented_mode(value)]
    if odd:
        print(f"[probe] 🟢 出现官方枚举之外的 mode 值 {odd}："
              "几乎可以确定是厂商未公开的模式，优先怀疑就是伴随态")
    else:
        print("[probe] ⚠️ 全部 mode 均在官方 14 项枚举内，"
              "说明伴随模式没有独立的 mode 值；"
              "若 fsm_name 也未变化，则伴随态对这两个通道均不可见")
    findings["watch"] = {
        "modes": {str(k): v for k, v in modes.items()},
        "mode_names": {str(k): mode_name(k) for k in modes},
        "fsm_names": names,
        "pairs": pairs,
        # Time-correlated observations replace the old cumulative-winner logic
        # as the authoritative evidence. Each sample pairs the freshest mode
        # value with the fsm_name read at that instant, so a brief transition
        # is never paired with the baseline mode that dominated the counts.
        "samples": samples,
    }
    return 0


def cmd_probe(args, findings):
    """Send an empty parameter to undocumented sport api ids. DANGEROUS."""
    try:
        _guard_armed(args.armed, args.robot_on_stand, args.remote_in_hand)
    except RuntimeError as exc:
        print(f"[probe] 拒绝执行：{exc}", file=sys.stderr)
        return 4

    from unitree_sdk2py.as2.sport.sport_client import SportClient  # noqa: WPS433
    gaps = set(sport_api_gaps())
    requested = []
    for token in args.probe_ids.split(","):
        token = token.strip()
        if not token:
            continue
        if not token.isdigit():
            print(f"[probe] 忽略非法 id {token!r}", file=sys.stderr)
            continue
        value = int(token)
        if value not in gaps:
            print(f"[probe] 跳过 {value}：该 id 在 SDK 中有定义，"
                  f"本次只探测未公开的 id", file=sys.stderr)
            continue
        requested.append(value)
    if not requested:
        print(f"[probe] 没有可探测的 id。未公开的 id 有：{list(sport_api_gaps())}",
              file=sys.stderr)
        return 4

    print("[probe] ⚠️  即将发送空参数到未公开 api id。确认：机器人已架起、"
          "遥控器在手中、周围无人、随时可断电。3 秒后开始。")
    time.sleep(3.0)
    sport = SportClient()
    sport.SetTimeout(2.0)
    sport.Init()
    results = {}
    for value in requested:
        try:
            sport._RegistApi(value, 0)
            code, data = sport._Call(value, "{}")
        except Exception as exc:
            code, data = f"exception: {exc}", None
        results[str(value)] = {"code": code, "data": data}
        print(f"  id={value} -> code={code} data={_safe_payload(data)}")
        time.sleep(1.0)
    findings["probe"] = results
    print("[probe] 判读方法：code=0 说明该 id 存在且接受了空参数，"
          "需要立刻观察机器人是否动了；返回 4xxx 说明不存在或无权限。")
    return 0


# --------------------------------------------------------------------- main

def build_parser():
    parser = argparse.ArgumentParser(
        description="As2-W 伴随模式取证工具（默认只读）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--iface", default="", help="网卡名，例如 eth0")
    parser.add_argument("--wait", type=float, default=3.0,
                        help="端点发现等待秒数")
    parser.add_argument("--services", action="store_true",
                        help="枚举 rt/api/* 服务（只读）")
    parser.add_argument("--modes", action="store_true",
                        help="调用 motion_switcher.CheckMode（只读）")
    parser.add_argument("--watch", type=float, default=0.0, metavar="SECONDS",
                        help="观测 mode/fsm_name 变化（只读）")
    parser.add_argument("--probe-ids", default="", metavar="ID[,ID]",
                        help="向未公开 api id 发送空参数（危险，需三道开关）")
    parser.add_argument("--armed", action="store_true", help="我确认要发送指令")
    parser.add_argument("--robot-on-stand", action="store_true",
                        help="机器人已架起，轮子离地")
    parser.add_argument("--remote-in-hand", action="store_true",
                        help="遥控器在手中，随时可接管")
    parser.add_argument("--json", default="", metavar="PATH",
                        help="把取证结果写入 JSON")
    return parser


def main(argv=None):
    _install_logsafe()
    parser = build_parser()
    args = parser.parse_args(argv)
    if not any((args.services, args.modes, args.watch, args.probe_ids)):
        args.services = True
        args.modes = True

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: WPS433
    ChannelFactoryInitialize(0, args.iface or None)

    findings = {"at": time.strftime("%Y-%m-%dT%H:%M:%S"), "iface": args.iface}
    status = 0
    if args.services:
        status = max(status, cmd_services(args, findings))
    if args.modes:
        try:
            status = max(status, cmd_modes(args, findings))
        except Exception as exc:
            print(f"[probe] motion_switcher 不可用：{exc}", file=sys.stderr)
    if args.watch:
        try:
            status = max(status, cmd_watch(args, findings))
        except Exception as exc:
            print(f"[probe] 观测失败：{exc}", file=sys.stderr)
    if args.probe_ids:
        status = max(status, cmd_probe(args, findings))

    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(findings, handle, ensure_ascii=False, indent=2,
                      default=str)
        print(f"[probe] 结果已写入 {args.json}")
    return status


if __name__ == "__main__":
    sys.exit(main())
