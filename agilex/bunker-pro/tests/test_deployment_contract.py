from pathlib import Path

import yaml


DRIVER_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = DRIVER_DIR.parents[1]


def test_mcp_port_is_unique_and_consistent():
    metadata = yaml.safe_load((DRIVER_DIR / "driver.yaml").read_text())
    config = yaml.safe_load((DRIVER_DIR / "config.yaml").read_text())
    dockerfile = (DRIVER_DIR / "Dockerfile").read_text()
    port = metadata["port"]

    assert port == config["mcp_port"] == 15744
    assert metadata["mcp_url"] == f"http://localhost:{port}/mcp"
    assert f"EXPOSE {port}" in dockerfile
    for path in REPO_ROOT.rglob("driver.yaml"):
        if path == DRIVER_DIR / "driver.yaml":
            continue
        other = yaml.safe_load(path.read_text()) or {}
        assert other.get("port") != port, f"{path} already uses port {port}"


def test_runtime_image_has_deployment_fragment_and_plain_ros_logs():
    dockerfile = (DRIVER_DIR / "Dockerfile").read_text()
    assert "COPY deploy/ /deploy/" in dockerfile
    assert "RCUTILS_COLORIZED_OUTPUT=0" in dockerfile
    assert "COPY . /work/agilex/bunker-pro" not in dockerfile

    bootstrap = (DRIVER_DIR / "deploy" / "host-can-bootstrap.sh").read_text()
    assert "--address=\"$bus_address\"" in bootstrap
    assert "RestartUnit ss phanthy-bunker-can.service replace" in bootstrap
    assert "\nsystemctl " not in bootstrap
