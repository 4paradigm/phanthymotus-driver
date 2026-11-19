"""Self-check our marketplace metadata, bundle config and file layout."""
from pathlib import Path

import yaml


def test_driver_metadata_and_config_consistent():
    root = Path(__file__).resolve().parents[1]
    driver = yaml.safe_load((root / "driver.yaml").read_text(encoding="utf-8"))
    cfg = yaml.safe_load((root / "config.yaml").read_text(encoding="utf-8"))
    compose = yaml.safe_load((root / "deploy/service.yml").read_text(encoding="utf-8"))
    assert driver["category"] == "driver"
    assert driver["hardware_provider"] == "samsung"
    assert driver["port"] == cfg["mcp_port"] == 15750
    assert driver["mcp_url"] == "http://localhost:15750/mcp"
    assert 15751 == cfg["ingress_port"] != driver["port"]
    cards = {card["name"]: card["type"] for card in driver["cards"]}
    assert cards == {"heart_rate": "sensor", "device_state": "sensor"}
    assert {name for name, x in cfg["plugins"].items() if x.get("enabled")} == set(cards)
    service = compose["samsung-watch7"]
    assert service["network_mode"] == "host"
    assert service["image"] == "__IMAGE__"
    assert "ROS_DOMAIN_ID=42" in service["environment"]
    assert "FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml" in service["environment"]
    assert all((root / name).is_file() for name in [
        "main.py", "device.py", "sensors.py", "transport.py",
        "protocol_v1.py", "protocol_v2.py", "Dockerfile", "requirements.txt"])
