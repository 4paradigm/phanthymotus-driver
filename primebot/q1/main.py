"""Motus MCP entry point for the PrimeBot Q1 Explorer."""
from common.vendor_runtime import run_driver
from battery import BatteryCard
from preset_postures import PresetPosturesCard


def build_plugins(config, namespace, ros):
    domain = config["ros"]["robot_domain_id"]
    return [BatteryCard(namespace, ros, domain),
            PresetPosturesCard(namespace, ros, domain)]


if __name__ == "__main__":
    run_driver(__file__, "primebot-q1-driver", "primebot-q1-device-bundle", build_plugins)
