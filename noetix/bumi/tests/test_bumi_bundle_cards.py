"""Regression test: Bumi bundle enabled cards must be importable with default config.

Catches Dockerfile COPY omissions and deleted modules that config.yaml still enables.
Run: cd noetix/bumi && python3 -m pytest tests/test_bumi_bundle_cards.py -v
"""

import importlib
import sys
from pathlib import Path

import yaml

BUMI_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BUMI_DIR))

try:
    import rclpy
    _HAS_RCLPY = True
except ImportError:
    _HAS_RCLPY = False


def _load_config():
    config_path = BUMI_DIR / "config.yaml"
    with open(config_path) as f:
        return yaml.safe_load(f)


def test_enabled_card_files_exist():
    """Verify every enabled card .py file exists (catches Dockerfile COPY misses).

    Bumi bundles many cards inside device.py; only check cards that are
    dynamically imported as separate modules.
    """
    cfg = _load_config()
    plugins = cfg.get("plugins") or {}
    enabled_cards = [name for name, conf in plugins.items()
                     if isinstance(conf, dict) and conf.get("enabled")]

    # Cards implemented as separate .py files (dynamically imported by main.py)
    # Cards like state/loco/mic/speaker/camera/motion_state/vision_capture
    # are implemented inside device.py and should NOT be checked here.
    standalone_cards = [c for c in enabled_cards if c in (
        "squat_counter", "demo_echo",
    )]

    missing = []
    for card in standalone_cards:
        py_file = BUMI_DIR / f"{card}.py"
        if not py_file.is_file():
            missing.append(f"{card}.py not found at {py_file}")

    if "squat_counter" in enabled_cards:
        if not (BUMI_DIR / "squat_count.py").is_file():
            missing.append("squat_count.py not found (required by squat_counter)")

    assert not missing, (
        f"Enabled standalone card files missing (likely missing from Dockerfile COPY):\n"
        + "\n".join(missing)
    )


def test_enabled_cards_importable():
    """Verify every enabled card can be imported (requires rclpy/ROS2 runtime)."""
    if not _HAS_RCLPY:
        import pytest
        pytest.skip("rclpy not available; import check skipped")

    cfg = _load_config()
    plugins = cfg.get("plugins") or {}
    enabled_cards = [name for name, conf in plugins.items()
                     if isinstance(conf, dict) and conf.get("enabled")]

    missing = []
    for card in enabled_cards:
        try:
            importlib.import_module(card)
        except ModuleNotFoundError as e:
            missing.append(f"{card}: {e}")

    assert not missing, (
        f"Enabled cards failed to import:\n"
        + "\n".join(missing)
    )
