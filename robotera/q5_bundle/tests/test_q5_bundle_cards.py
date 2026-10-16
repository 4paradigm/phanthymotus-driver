"""Regression test: every enabled Q5 bundle card file must exist on disk.

Catches Dockerfile COPY omissions that used to silently skip capabilities.
Import checks are skipped when rclpy is unavailable (e.g. local dev machine).
Run: cd robotera/q5_bundle && python3 -m pytest tests/test_q5_bundle_cards.py -v
"""

import importlib
import sys
from pathlib import Path

import yaml

BUNDLE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BUNDLE_DIR))

try:
    import rclpy
    _HAS_RCLPY = True
except ImportError:
    _HAS_RCLPY = False


def _load_config():
    config_path = BUNDLE_DIR / "config.yaml"
    with open(config_path) as f:
        return yaml.safe_load(f)


def test_enabled_card_files_exist():
    """Verify every enabled card .py file exists (catches Dockerfile COPY misses)."""
    cfg = _load_config()
    plugins = cfg.get("plugins") or {}
    enabled_cards = [name for name, conf in plugins.items()
                     if isinstance(conf, dict) and conf.get("enabled")]

    missing = []
    for card in enabled_cards:
        py_file = BUNDLE_DIR / f"{card}.py"
        if not py_file.is_file():
            missing.append(f"{card}.py not found at {py_file}")

    # squat_counter depends on squat_count.py
    if "squat_counter" in enabled_cards:
        if not (BUNDLE_DIR / "squat_count.py").is_file():
            missing.append("squat_count.py not found (required by squat_counter)")

    assert not missing, (
        f"Enabled card files missing (likely missing from Dockerfile COPY):\n"
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


def test_enabled_cards_have_make_plugin():
    """Verify enabled cards expose make_plugin or a standard plugin class."""
    if not _HAS_RCLPY:
        import pytest
        pytest.skip("rclpy not available; class check skipped")

    cfg = _load_config()
    plugins = cfg.get("plugins") or {}
    enabled_cards = [name for name, conf in plugins.items()
                     if isinstance(conf, dict) and conf.get("enabled")]

    broken = []
    for card in enabled_cards:
        mod = importlib.import_module(card)
        if not hasattr(mod, "make_plugin") and not any(
            hasattr(mod, cls) for cls in ("Plugin", "BasePlugin", "Card")
        ):
            broken.append(card)

    assert not broken, (
        f"Enabled cards missing plugin factory/class:\n"
        + "\n".join(broken)
    )
