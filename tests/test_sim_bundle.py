"""Packaging: the bundle assembles, and its four descriptions agree.

`driver.yaml`'s card list is hand-synced with `config.yaml`'s plugin switches,
exactly as the other bundles do it — which means it drifts unless something
checks. The marketplace listing is built from `driver.yaml`, so drift shows up as
a card that either never appears or appears and does nothing.

This file also runs `scripts/check_service_yml.py` from pytest. The repo has no
CI at all, so a lint nobody runs is a lint that does not exist; invoking it here
puts the DDS isolation contract into the one suite people do run.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_sim_bundle.py -q
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

BUNDLE = ROOT / "simulator" / "generic"

from common.vendor_runtime import DriverBundle  # noqa: E402
from simulator.generic.plugins import build_plugins  # noqa: E402


def load(name: str) -> dict:
    with (BUNDLE / name).open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)


@pytest.fixture(scope="module")
def config() -> dict:
    return load("config.yaml")


@pytest.fixture(scope="module")
def bundle(config):
    built = DriverBundle(build_plugins(config, "sim", None))
    yield built
    built.stop_all()


# ── the four descriptions agree ──────────────────────────────────────────────

def test_driver_yaml_cards_match_the_tools_actually_served(config, bundle):
    declared = {card["name"] for card in load("driver.yaml")["cards"]}
    served = {tool["name"] for tool in bundle.get_all_tools()}

    assert declared == served, (
        f"declared but not served: {sorted(declared - served)}; "
        f"served but not declared: {sorted(served - declared)}")


def test_driver_yaml_card_types_match_the_tool_types(bundle):
    """A `sensor` listed as an `actuator` changes whether the barrier exempts it
    and whether a `viewer` peer can reach it."""
    declared = {card["name"]: card["type"] for card in load("driver.yaml")["cards"]}
    for tool in bundle.get_all_tools():
        assert declared[tool["name"]] == tool["type"], f"{tool['name']} type mismatch"


def test_config_plugin_switches_cover_every_card(config):
    declared = {card["name"] for card in load("driver.yaml")["cards"]}
    assert set(config["plugins"]) == declared


def test_ports_agree_across_config_driver_yaml_and_dockerfile(config):
    driver = load("driver.yaml")
    dockerfile = (BUNDLE / "Dockerfile").read_text(encoding="utf-8")

    assert config["mcp_port"] == driver["port"] == 15711
    assert driver["mcp_url"] == "http://localhost:15711/mcp"
    assert "EXPOSE 15711" in dockerfile


def test_the_bundle_sits_two_levels_deep():
    """Three repo-wide checks glob `*/*/` — check_service_yml.py, the Dockerfile
    COPY test and the image tag in build.sh. A one-level bundle is skipped by all
    three *and still builds*, because build.sh also globs `*/driver.yaml`. That
    combination is the worst case: it looks compliant."""
    assert (BUNDLE / "driver.yaml").relative_to(ROOT).parts[:2] == ("simulator", "generic")
    assert load("driver.yaml")["hardware_provider"] == "simulator"


def test_namespace_is_not_the_hostname(config):
    """`vendor_runtime.resolve_namespace` falls back to socket.gethostname(),
    which on a rig already running a real driver would put the simulator's topics
    on top of the robot's."""
    assert config["ros_namespace"] == "sim"


# ── deployment contract ──────────────────────────────────────────────────────

def test_service_yml_passes_the_dds_isolation_lint():
    """The repo has no CI, so a lint nobody runs is a lint that does not exist."""
    result = subprocess.run([sys.executable, str(ROOT / "scripts" / "check_service_yml.py"),
                             "simulator/generic"],
                            cwd=ROOT, capture_output=True, text=True)

    assert result.returncode == 0, result.stdout + result.stderr


def test_service_yml_claims_no_hardware():
    """A simulator that reaches outside its container has stopped being one."""
    service = load("deploy/service.yml")["simulator-generic"]

    assert "privileged" not in service
    assert "device_cgroup_rules" not in service
    assert not any(str(v).startswith("/dev") for v in service.get("volumes", []))


def test_service_yml_records_what_a_run_was_measured_against():
    """A score with no configuration attached is noise."""
    environment = load("deploy/service.yml")["simulator-generic"]["environment"]
    keys = {entry.split("=", 1)[0] for entry in environment}

    assert {"SIM_TIER", "SIM_LLM_MODEL", "IMAGE_TAG", "GIT_SHA"} <= keys


def test_scenarios_can_be_added_without_rebuilding_the_image():
    service = load("deploy/service.yml")["simulator-generic"]
    mounts = [str(v) for v in service["volumes"]]

    assert any("/opt/phanthy-motus/data/sim" in mount for mount in mounts)
    assert any(entry.startswith("SIM_SCENARIO_DIR=") for entry in service["environment"])


def test_dockerfile_stays_thin():
    """Every one of these exists in a vendor bundle and none of it is needed
    here — there is no hardware to talk to."""
    dockerfile = (BUNDLE / "Dockerfile").read_text(encoding="utf-8").lower()
    for unwanted in ("colcon", "build-essential", "cmake", "espeak", "alsa-utils",
                     "sshpass", "opencv", "cyclonedds"):
        line = next((l for l in dockerfile.splitlines()
                     if unwanted in l and not l.strip().startswith("#")), None)
        assert line is None, f"Dockerfile pulls in {unwanted}: {line}"


# ── it actually boots ────────────────────────────────────────────────────────

def test_build_plugins_works_without_ros(bundle):
    """Everything above the publisher is ROS-free, so the whole schema and
    dispatch surface can be exercised on a laptop."""
    tools = bundle.get_all_tools()

    assert len(tools) == 15
    assert {"controlled_spatial", "tts", "loco", "spatial_map",
            "sim_scenario", "sim_report"} <= {t["name"] for t in tools}


def test_the_default_scenario_is_the_real_beijing_hall(bundle):
    """默认载**真图**，不是合成世界。

    默认原先是 `exhibition_tour` —— 用矩形拼出来的世界，航点叫「一号展区」那些。
    容器一重建就回到它，而人看着 agent 往「一号展区」导航，会以为是真图上的展位名
    不对、或者地图打包错了。Orin6 上就是这么被问到的。
    """
    state = bundle.dispatch("sim_scenario", {"action": "read"})

    assert state["scenario"] == "beijing_2f_tour"
    # 真图的点位是从驱动的 controlled_spatial.db 导出来的 P 系列，不是手写的名字。
    assert all(name.startswith("P") for name in state["waypoints"]), state["waypoints"]


def test_scenario_pois_become_map_tags(bundle):
    """真机上导览前先把展区打好点，之后每段走 navigate_to_tag —— 场景载入时
    POI 就变成地图上的 tag，导航卡和 map 卡读的是同一份世界状态。"""
    tags = bundle.dispatch("controlled_spatial", {"action": "list_tags"})["tags"]
    loaded = bundle.dispatch("sim_scenario", {"action": "read"})

    # 比的是**两边一致**，不是某一串具体的名字 —— 钉死名字的话，换一次默认场景
    # 这条就红了，而它要守的东西一点没变。
    assert [t["name"] for t in tags] == loaded["waypoints"]
    assert tags, "载入的场景没有产出任何 tag"


def test_every_card_answers_info_or_declines_cleanly(bundle):
    """start-project is strict: one card that fails start rolls the whole project
    back and the robot comes up with nothing running.

    `resource` cards are exempt, and not by choice: `DriverBundle.dispatch` does
    not pop an `action` for that kind, so a resource tool is called by its own
    name and cannot tell a lifecycle verb from a request for the resource. They
    are checked below for returning their payload instead.
    """
    from common import lifecycle

    for tool in bundle.get_all_tools():
        if tool["type"] == "resource":
            continue
        result = bundle.dispatch(tool["name"], {"action": "info"})
        assert isinstance(result, dict)
        assert "state" in result or lifecycle.is_declined(result), f"{tool['name']}: {result}"


def test_resource_cards_return_their_resource(bundle):
    model = bundle.dispatch("model", {})
    report = bundle.dispatch("sim_report", {})

    assert model["joint_names"] == ["head_yaw", "head_pitch"]
    for name in model["joint_names"]:
        # The skeleton renderer matches by name; one mismatch draws nothing and
        # reports nothing about why.
        assert f'<joint name="{name}"' in model["urdf"]
    assert report["scenario"] == "beijing_2f_tour"


@pytest.mark.parametrize("slug", ["beijing_2f_tour", "exhibition_tour"])
def test_every_shipped_scenario_loads_with_no_warnings(bundle, slug):
    """两个都要能干净载入。合成的那个没有被删 —— 打断/恢复那几条断言是照它写的，
    它只是不再是默认。"""
    result = bundle.dispatch("sim_scenario", {"action": "load", "scenario": slug})

    assert result["warnings"] == [], result["warnings"]


# ── 不声明地图的重置 ─────────────────────────────────────────────────────────

def test_resetting_a_map_built_world_does_not_look_up_a_scenario(config):
    """**从地图资产建出来的场景，它的 slug 是地图名，不是场景名。**

    `do_reset(map=...)` 用 `Scenario.from_dict({...}, slug=map)` 造它。所以随后一次
    不带地图的 `reset` 要是拿这个 slug 去查场景表，必然查不到 —— 报出来是
    `unknown scenario: bj-2f`，一个看着像「用例写错了地图」的错误，而其实是重置自己
    走错了路。

    Orin6 上一个**不声明地图**的基准测试用例就撞上了：跑在当前世界上是新的默认，
    而「当前世界」恰恰就是这种从资产建出来的场景。
    """
    from simulator.generic.cards_scenario import SimScenarioCard
    from simulator.generic.clock import FakeClock
    from simulator.generic.backend import LocalBackend
    from simulator.generic.world import VirtualWorld

    world = VirtualWorld(LocalBackend(), FakeClock(), {})
    card = SimScenarioCard(world, config, "sim",
                           scenario_dirs=[BUNDLE / "simulator" / "generic" / "scenarios"]
                           if (BUNDLE / "simulator").exists() else None)

    built = card.do_reset(map="bj-2f", owner="t")
    assert "error" not in built, built

    again = card.do_reset(owner="t")

    assert "error" not in again, again
    assert again.get("loaded") == "bj-2f"


# ── 热加一张地图 ─────────────────────────────────────────────────────────────

def _tiny_map(path, name):
    """一张最小的可用地图。格式照 `simulator/generic/maps/bj-2f.json`。"""
    import base64
    import json
    import zlib
    grid = bytes(20 * 20)                      # 全 0 = 全部可通行
    path.write_text(json.dumps({
        "name": name, "resolution": 0.1, "origin": [0.0, 0.0],
        "width": 20, "height": 20,
        "data": base64.b64encode(zlib.compress(grid)).decode(),
        "pois": [{"name": "门口", "x": 0.5, "y": 0.5, "yaw": 0.0}],
    }), encoding="utf-8")


def test_a_map_dropped_into_the_mount_is_reachable_through_the_agents_own_path(
        tmp_path, monkeypatch, config):
    """**加一张地图不该要重建镜像。**

    `SimScenarioCard.maps()` 的文档一直写着「丢一份进 bind-mount 的目录，刷新画布就能
    选到」，但两个默认目录都在镜像里，而 `build_plugins` 也没传 `map_dirs` —— 场景能
    热加，地图不能，两者在 UI 上却长得一样。

    这条走的是 **agent 自己那条路**：导航卡的 `list_maps` / `load_map`，和真机上技能的
    第 0 步同名。只测 `maps()` 能扫到是不够的 —— 中间还隔着 `set_map_hooks` 那一层接线，
    而那正是「扫得到却载不进」会发生的地方。
    """
    from simulator.generic.cards_motion import ControlledSpatialCard
    from simulator.generic.cards_scenario import SimScenarioCard

    external = tmp_path / "maps"
    external.mkdir()
    _tiny_map(external / "site-b.json", "site-b")

    # **走 `build_plugins`，不手工塞 `map_dirs`。** 直接给卡片传目录的话，旧代码也能
    # 过 —— 而旧代码缺的正是 `build_plugins` 里那一句没传。绕过被测的那一环去测它，
    # 等于没测。
    monkeypatch.setenv("SIM_MAP_DIR", str(external))
    cards = build_plugins(config, "sim", None)
    try:
        nav = next(c for c in cards if isinstance(c, ControlledSpatialCard))
        scenario = next(c for c in cards if isinstance(c, SimScenarioCard))

        assert "site-b" in nav.do_list_maps()["maps"]

        loaded = nav.do_load_map(map_name="site-b")

        assert "error" not in loaded, loaded
        assert scenario.active_map == "site-b"
    finally:
        for card in cards:
            try:
                card.stop()
            except Exception:
                pass


def test_map_dirs_puts_the_mounted_one_last_so_it_wins(monkeypatch):
    """和场景那条同一个规矩：镜像里那份在前，挂进来的在后 —— 同名时挂进来的赢。"""
    from simulator.generic.plugins import map_dirs

    monkeypatch.setenv("SIM_MAP_DIR", "/mnt/somewhere/maps")

    dirs = [str(d) for d in map_dirs({})]

    assert dirs[-1] == "/mnt/somewhere/maps"
    assert any(d.endswith("generic/maps") for d in dirs)


def test_config_can_replace_the_map_dirs_outright():
    from simulator.generic.plugins import map_dirs

    dirs = map_dirs({"scenario": {"map_dirs": ["/only/here"]}})

    assert [str(d) for d in dirs] == ["/only/here"]


def test_a_map_built_scenario_reports_the_maps_own_waypoints(tmp_path):
    """`summary()` 读的必须是 `waypoints()`，不是 `self.pois`。

    从地图资产建出来的场景自己一个 POI 都没声明 —— 航点在资产里，`waypoints()` 会
    回落过去。读 `self.pois` 的话，载入成功的返回里写着 `waypoints: []`，而世界其实
    已经拿到了点位。同一个类对「航点」给出两个答案，送出去的是错的那个。
    """
    from simulator.generic.backend import LocalBackend
    from simulator.generic.cards_scenario import SimScenarioCard
    from simulator.generic.clock import FakeClock
    from simulator.generic.world import VirtualWorld

    external = tmp_path / "maps"
    external.mkdir()
    _tiny_map(external / "site-c.json", "site-c")

    world = VirtualWorld(LocalBackend(), FakeClock(), {})
    card = SimScenarioCard(world, {}, "sim", map_dirs=[external])

    loaded = card.switch_map("site-c")

    assert loaded.get("waypoints") == ["门口"]


# ── 世界锁不能活过它的持有者 ─────────────────────────────────────────────────

def _scenario_card(tmp_path):
    from simulator.generic.backend import LocalBackend
    from simulator.generic.cards_scenario import SimScenarioCard
    from simulator.generic.clock import FakeClock
    from simulator.generic.world import VirtualWorld

    external = tmp_path / "maps"
    external.mkdir()
    _tiny_map(external / "site-a.json", "site-a")
    _tiny_map(external / "site-b.json", "site-b")
    world = VirtualWorld(LocalBackend(), FakeClock(), {})
    return SimScenarioCard(world, {}, "sim", map_dirs=[external])


def test_a_running_benchmark_still_keeps_others_out(tmp_path):
    """这把锁存在的理由没变：被测的 agent 能调到 `reset`，也就是能重置正在测它的
    那次测量，而且重置之后什么痕迹都不剩。"""
    card = _scenario_card(tmp_path)
    card.do_reset(map="site-a", owner="benchmark")

    refused = card.switch_map("site-b")

    assert "error" in refused and refused["owner"] == "benchmark"


def test_an_abandoned_lock_expires_instead_of_bricking_the_world(tmp_path, monkeypatch):
    """**锁不能活过它的持有者。**

    原先唯一的释放路径是带着对的 owner 显式 abort —— agent-core 重启、跑动被杀、
    容器被重建，任何一种都让世界永久锁死。而症状不是一条报错：agent 读到「地图加载
    不了」，很合理地告诉访客展厅在维护，然后 finish()。Orin6 上真发生过，每条日志
    都正常。
    """
    import simulator.generic.cards_scenario as mod

    card = _scenario_card(tmp_path)
    card.do_reset(map="site-a", owner="benchmark")
    # 持有者走了：没有人再续租。
    monkeypatch.setattr(mod.time, "time", lambda: card._owner_seen + mod.OWNER_TTL + 1)

    loaded = card.switch_map("site-b")

    assert "error" not in loaded, loaded
    assert card.active_map == "site-b"


def test_polling_the_report_renews_the_lock(tmp_path, monkeypatch):
    """跑动每两秒问一次事实，那一下就是心跳 —— 否则一次超过 TTL 的长跑会被自己的锁
    过期，而它明明还在跑。"""
    import simulator.generic.cards_scenario as mod
    from simulator.generic.cards_scenario import SimReportCard

    card = _scenario_card(tmp_path)
    card.do_reset(map="site-a", owner="benchmark")
    report = SimReportCard(card.world, {}, "sim", scenario_card=card)

    later = card._owner_seen + mod.OWNER_TTL - 1
    monkeypatch.setattr(mod.time, "time", lambda: later)
    report.report()                                   # 续租
    monkeypatch.setattr(mod.time, "time", lambda: later + mod.OWNER_TTL - 1)

    assert "error" in card.switch_map("site-b")       # 还在跑，仍然拒绝


# ── 世界要听得见真实 tts ─────────────────────────────────────────────────────

def test_the_speaker_declares_the_stream_half_of_the_canvas_contract():
    """**`topic_in` 是这张卡存在的理由。**

    这里原先有一张 `speaker`，被删掉了，因为它照着 `voice_play` 那种调用式的卡抄，
    却用了 speaker 的名字 —— 没有 `topic_in`，画布上什么都连不进去。真机上的每一张
    speaker 都是订阅 `audio/pcm-16k` 的。
    """
    from simulator.generic.cards_audio import SpeakerCard
    from simulator.generic.backend import LocalBackend
    from simulator.generic.clock import FakeClock
    from simulator.generic.world import VirtualWorld

    card = SpeakerCard(VirtualWorld(LocalBackend(), FakeClock(), {}), {}, "sim")
    definition = card.get_tool()

    assert definition["topic_in"] == [{"format": "audio/pcm-16k"}]
    # 嘴由**合成**的那张卡占（tts）。这里再占一次，barrier 会让它和自己的上游串行 ——
    # 而它正在播，恰恰是因为 tts 正在说。
    assert "x-resource" not in definition["inputSchema"]


def test_audio_on_the_stream_becomes_speak_events_in_the_world():
    """左栏能看到 agent 调了 tts，右栏却一句话都没有 —— 差的就是这一步。"""
    from simulator.generic.cards_audio import SpeakerCard
    from simulator.generic.backend import LocalBackend
    from simulator.generic.clock import FakeClock
    from simulator.generic.world import VirtualWorld

    world = VirtualWorld(LocalBackend(), FakeClock(), {})
    card = SpeakerCard(world, {}, "sim")
    card._input_topic = "/perception/tts_audio"

    card._on_step(10.0, 0.1)          # 世界的时钟走到 10 秒
    card._on_audio(None)              # 第一块音频
    card._on_step(10.2, 0.1)
    card._on_audio(None)              # 还在播
    card._on_step(12.0, 0.1)          # 静了 1.8 秒 > QUIET_SECONDS

    kinds = [e["event"] for e in world.events() if e["event"].startswith("speak")]
    assert kinds == ["speak_start", "speak_end"]


def test_one_sentence_is_not_logged_as_several():
    """PCM 流里没有「说完了」这个标记，只能按静默判断 —— 门槛太短，一句话会被拆成
    好几段，而「到了再讲」那条判定会看到一串莫名其妙的短播报。"""
    from simulator.generic.cards_audio import SpeakerCard
    from simulator.generic.backend import LocalBackend
    from simulator.generic.clock import FakeClock
    from simulator.generic.world import VirtualWorld

    world = VirtualWorld(LocalBackend(), FakeClock(), {})
    card = SpeakerCard(world, {}, "sim")

    for tick in range(10):            # 每 0.2 秒一块，中间的间隔都小于门槛
        card._on_step(tick * 0.2, 0.2)
        card._on_audio(None)
    card._on_step(5.0, 0.2)

    starts = [e for e in world.events() if e["event"] == "speak_start"]
    assert len(starts) == 1


def _speaker():
    from simulator.generic.cards_audio import SpeakerCard
    from simulator.generic.backend import LocalBackend
    from simulator.generic.clock import FakeClock
    from simulator.generic.world import VirtualWorld

    return SpeakerCard(VirtualWorld(LocalBackend(), FakeClock(), {}), {}, "sim")


def test_start_carries_the_topic_the_canvas_resolved():
    """**`Card.dispatch` 把 `start` 当生命周期动词拦下来，自己调无参的 `self.start()`。**

    第一版把它写成了 `do_start(input_topic=...)` —— 永远不会被调到。结果是卡片启动
    成功、`state: running`、`input_topic` 始终为空：一张**聋着的**卡，从外面看不出
    任何异常，而「世界真的做了什么」那一列就一直是空的。Orin6 上就是这么现形的。
    """
    card = _speaker()

    card.dispatch("start", {"input_topic": "/perception/tts"})

    assert card.do_read()["input_topic"] == "/perception/tts"


def test_info_reports_the_topic_it_actually_bound():
    """agent-core 用 `info().topic_in[].topic` 判断一张卡接上了什么
    （`api/config.py::_bound_inputs`）。不报的话，聋着的卡和正常的卡在它眼里一样。"""
    card = _speaker()

    deaf = card.info()
    card.dispatch("start", {"input_topic": "/perception/tts"})
    bound = card.info()

    assert [t.get("topic") for t in deaf["topic_in"] if t.get("topic")] == []
    assert [t["topic"] for t in bound["topic_in"]] == ["/perception/tts"]


def test_a_speaker_with_no_ros_simply_does_not_subscribe():
    """pytest 和离线重放都没有 ROS。安静地不订阅，而不是把整张卡带崩。"""
    card = _speaker()

    card.dispatch("start", {"input_topic": "/perception/tts_audio"})

    assert card.do_read()["state"] == "running"
