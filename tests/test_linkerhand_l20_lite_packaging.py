"""SDK-free staging and deployment compatibility checks; no serial opens."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand/l20_lite"


def module(name, file):
    spec = importlib.util.spec_from_file_location(name, DRIVER / file)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def test_build_context_contains_native_runtime_without_private_configuration(tmp_path):
    builder = module("native_builder", "build_context.py")
    target = builder.stage_context(tmp_path / "build")
    assert (target / "glove_protocol.py").is_file()
    assert (target / "glove_thumb.py").is_file()
    assert (target / "common/vendor_runtime.py").is_file()
    assert (target / "resource/l20lite_left_skeleton.urdf").is_file()
    assert not list(target.rglob("*.dll"))
    assert not list(target.rglob(".env"))
    with pytest.raises(ValueError, match="empty"):
        builder.stage_context(target)


def test_context_excludes_local_environment_variants(tmp_path, monkeypatch):
    builder = module("native_builder_private", "build_context.py")
    fake = tmp_path / "repo/linkerhand/l20_lite"
    fake.mkdir(parents=True)
    for name in builder.FILES:
        (fake / name).write_text("fixture")
    for folder in [fake / "deploy", fake / "resource", fake.parents[1] / "common"]:
        folder.mkdir()
        for name in [".env", ".env.local", "private.env", ".env.example"]:
            (folder / name).write_text("fixture")
    monkeypatch.setattr(builder, "DRIVER", fake)
    staged = builder.stage_context(tmp_path / "staged")
    assert (staged / "deploy/.env.example").exists()
    assert not list(staged.rglob(".env")) and not list(staged.rglob("private.env"))


def test_preflight_does_not_open_uart_and_detects_alias_conflict(monkeypatch, tmp_path):
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace())
    checker = module("native_preflight", "check_runtime.py")

    def forbidden(*args, **kwargs):
        raise AssertionError("preflight must not open UART")

    monkeypatch.setattr(checker.os, "open", forbidden)
    assert checker.check(files_only=True)["ok"]
    alias = tmp_path / "same-device"
    alias.symlink_to("/dev/null")
    assert not checker.check("/dev/null", str(alias))["ok"]


def test_hand_only_deployment_has_no_required_glove_or_sdk_mount():
    fragment = (DRIVER / "deploy/service.yml").read_text()
    service = yaml.safe_load(fragment)["linkerhand-l20-lite"]
    assert ":?" not in fragment
    assert "/vendor/hex" not in fragment
    assert any("/dev:/dev" == value for value in service["volumes"])
    assert (
        yaml.safe_load((DRIVER / "config.yaml").read_text())["serial_port"]
        == "/dev/ttyUSB0"
    )


@pytest.mark.parametrize("fragment", ["service.yml", "glove-overlay.yml"])
def test_each_deployment_fragment_bounds_docker_logs(fragment):
    data = yaml.safe_load((DRIVER / "deploy" / fragment).read_text())
    service = data.get("services", data)["linkerhand-l20-lite"]
    assert service["logging"] == {
        "driver": "local", "options": {"max-size": "10m", "max-file": "3"},
    }
