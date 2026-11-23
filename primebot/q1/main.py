"""Motus MCP entry point for the PrimeBot Q1 Explorer."""
from common.vendor_runtime import run_driver
from battery import BatteryCard


def build_plugins(config, namespace, ros):
    return [BatteryCard(namespace, ros, config["ros"]["robot_domain_id"])]


if __name__ == "__main__":
    run_driver(__file__, "primebot-q1-driver", "primebot-q1-device-bundle", build_plugins)
