"""Wire the bundle together.

One `VirtualWorld`, constructed once and handed to every card — the same shape
`x-humanoid/tianyi2.0/main.py` uses to thread its `slamtec_client` through eight
plugins.

That injection point is the answer to the `if self.mock:` pattern in
`chasing/qianjiao_p200_pro`, which sprays a flag through thirteen call sites
because it has no substitutable client to swap. The simulator is not a *mode* of
a driver; it is a different object behind the same method surface.
"""

from __future__ import annotations

import os
from pathlib import Path

from simulator.generic.backend import LocalBackend
from simulator.generic.cards_audio import TtsCard
from simulator.generic.cards_motion import (
    ArmCard,
    ControlledSpatialCard,
    LedCard,
    LocoCard,
    SwitchModeCard,
)
from simulator.generic.cards_scenario import SimReportCard, SimScenarioCard
from simulator.generic.cards_sensors import (
    BatteryCard,
    ImuCard,
    LaserScanCard,
    MapCard,
    ModelCard,
    OdomCard,
)
from simulator.generic.clock import RealClock
from simulator.generic.world import VirtualWorld

BUNDLE_DIR = Path(__file__).resolve().parent

CARD_TYPES = {
    "odom": OdomCard, "imu": ImuCard, "laser_scan": LaserScanCard, "battery": BatteryCard,
    "map": MapCard, "model": ModelCard,
    "loco": LocoCard, "controlled_spatial": ControlledSpatialCard,
    "switch_mode": SwitchModeCard, "arm": ArmCard, "led": LedCard,
    "tts": TtsCard,
    "sim_scenario": SimScenarioCard, "sim_report": SimReportCard,
}


def scenario_dirs(config: dict) -> list[Path]:
    """Baked-in first, bind-mounted second — later wins, so a rig gets a new
    scenario by dropping a YAML in a directory rather than rebuilding an image."""
    configured = (config.get("scenario") or {}).get("dirs")
    if configured:
        return [Path(entry) for entry in configured]
    external = os.environ.get("SIM_SCENARIO_DIR", "/opt/phanthy-motus/data/sim/scenarios")
    return [BUNDLE_DIR / "scenarios", Path(external)]


def build_world(config: dict, clock=None) -> VirtualWorld:
    world = VirtualWorld(LocalBackend(), clock or RealClock(), config.get("world") or {})
    world.reset({
        "grid": None,
        "spawn": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "motion": (config.get("world") or {}).get("motion") or {},
        "dof": int((config.get("embodiment") or {}).get("dof", 0)),
    })
    return world


def build_plugins(config: dict, namespace: str, ros2=None) -> list:
    """`vendor_runtime.run_driver` calls this once at boot."""
    world = build_world(config)
    enabled = config.get("plugins") or {}
    cards: list = []

    scenario_card = None
    report_card = None
    spatial_card = None
    map_card = None

    for name, card_cls in CARD_TYPES.items():
        if not (enabled.get(name) or {}).get("enabled", True):
            continue
        if card_cls is SimScenarioCard:
            scenario_card = SimScenarioCard(world, config, namespace, ros2,
                                            scenario_dirs=scenario_dirs(config))
            cards.append(scenario_card)
            continue
        if card_cls is SimReportCard:
            continue                                   # constructed last; needs the scenario card
        card = card_cls(world, config, namespace, ros2)
        cards.append(card)
        if isinstance(card, ControlledSpatialCard):
            spatial_card = card
        elif isinstance(card, MapCard):
            map_card = card

    if scenario_card is not None:
        # Tags live in the world, so the navigator and the map both read the
        # live set rather than each holding a copy that can go stale.
        if map_card is not None:
            map_card.set_waypoints_provider(world.tags)
        if spatial_card is not None:
            # 「载入地图」在这套东西里就是「载入场景」——一张地图一个场景。
            spatial_card.set_map_hooks(
                lambda name: scenario_card.do_load(scenario=name),
                lambda: sorted(scenario_card.refresh()))
        scenario_card.set_injector(_ros_injector(config, ros2))
        if (enabled.get("sim_report") or {}).get("enabled", True):
            report_card = SimReportCard(world, config, namespace, ros2, scenario_card=scenario_card)
            cards.append(report_card)

    default = (config.get("scenario") or {}).get("default")
    if default and scenario_card is not None:
        result = scenario_card.dispatch("load", {"scenario": default})
        for warning in result.get("warnings") or []:
            print(f"[sim] scenario warning: {warning}", flush=True)

    world.start()
    return cards


def _ros_injector(config: dict, ros2):
    """Publish an injected event onto `/remote_control/message`.

    The same topic a real remote control uses, which is the point: an injected
    task travels the path it would in the field, and agent-core needs no change
    to receive it. Without ROS the scenario card falls back to recording the
    event in the log, which is what the pytest runner wants.
    """
    if ros2 is None:
        return None
    topic = (config.get("scenario") or {}).get("inject_topic", "/remote_control/message")
    state: dict = {}

    def publish(text: str, kind: str) -> None:
        import json

        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        if "pub" not in state:
            node = Node("sim_injector", context=ros2.ctx_core)
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
            state["pub"] = node.create_publisher(String, topic, qos)
            ros2.executor_core.add_node(node)
            state["node"] = node
        message = String()
        message.data = json.dumps({"text": text, "kind": kind, "source": "simulator"},
                                  ensure_ascii=False)
        state["pub"].publish(message)

    return publish
