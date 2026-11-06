"""Native teach deployment and fail-before-write checks, without real mounts."""

from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from dobot.cra import native_mounts
from dobot.cra.native_mounts import readable_profile, writable_controller_directory
from dobot.cra.trajectory import TrajectoryStore
from test_dobot_teach import recorded_store, recording


ROOT = Path(__file__).resolve().parents[1]


def mount_line(directory, filesystem="cifs", options="rw", super_options="rw"):
    target = str(directory).replace("\\", "\\134").replace(" ", "\\040")
    return f"40 20 0:45 / {target} {options},relatime - {filesystem} //controller/trajectory {super_options}\n"


def test_native_overlay_maps_host_paths_to_the_exact_runtime_paths():
    base = yaml.safe_load((ROOT / "dobot/cra/deploy/service.yml").read_text())["dobot-cra"]
    overlay = yaml.safe_load((ROOT / "dobot/cra/deploy/native-teach.compose.yml").read_text())["services"]["dobot-cra"]
    volumes = {volume["target"]: volume for volume in overlay["volumes"]}
    env = overlay["environment"]
    profile = volumes[env["DOBOT_TEACH_CSV_PROFILE"]]
    share = volumes[env["DOBOT_TEACH_CONTROLLER_DIR"]]
    assert "DOBOT_TEACH_PROFILE_HOST_FILE:?" in profile["source"]
    assert "DOBOT_TEACH_CONTROLLER_HOST_DIR:?" in share["source"]
    assert profile["read_only"] is True and share["read_only"] is False
    assert all(volume["type"] == "bind" and volume["bind"]["create_host_path"] is False
               for volume in volumes.values())
    # The shipped base service has no host prerequisites for native playback.
    assert "DOBOT_TEACH_CSV_PROFILE=" in base["environment"]
    assert "DOBOT_TEACH_CONTROLLER_DIR=" in base["environment"]
    assert "COPY deploy/ /deploy/" in (ROOT / "dobot/cra/Dockerfile").read_text()


@pytest.mark.parametrize("kind", ["missing", "directory", "unreadable", "invalid_json", "not_object", "relative"])
def test_profile_path_failures_are_actionable(tmp_path, kind):
    path = tmp_path / "profile.json"
    if kind == "directory": path.mkdir()
    elif kind == "unreadable":
        path.write_text("{}")
        path.chmod(0o200)
    elif kind == "invalid_json": path.write_text("not JSON")
    elif kind == "not_object": path.write_text("[]")
    elif kind == "relative": path = Path("profile.json")
    try:
        with pytest.raises(ValueError):
            readable_profile(path)
    finally:
        if kind == "unreadable": path.chmod(0o600)


@pytest.mark.parametrize("filesystem", ["cifs", "smb3", "nfs", "nfs4"])
def test_existing_network_share_and_nested_directory_pass_preflight(tmp_path, monkeypatch, filesystem):
    share = tmp_path / "mounted share"
    directory = share / "process" / "trajectory"
    directory.mkdir(parents=True)
    monkeypatch.setattr(native_mounts, "_mountinfo_text", lambda: mount_line(share, filesystem))
    assert writable_controller_directory(directory, probe=True) == directory
    assert not list(directory.iterdir())  # write/readback probe was removed


@pytest.mark.parametrize("fault", ["ordinary_directory", "local_bind", "read_only_bind", "read_only_share", "mode", "missing"])
def test_invalid_controller_mount_is_rejected_before_export(tmp_path, monkeypatch, fault):
    store = recorded_store(tmp_path)
    store.require_network_mount = True
    record, _ = recording(store)
    directory = store.controller_root
    text = mount_line(directory)
    if fault == "ordinary_directory": text = mount_line("/", "ext4")
    elif fault == "local_bind": text = mount_line(directory, "ext4")
    elif fault == "read_only_bind": text = mount_line(directory, options="ro")
    elif fault == "read_only_share": text = mount_line(directory, super_options="ro")
    elif fault == "mode": directory.chmod(0o555)
    elif fault == "missing": directory.rmdir()
    monkeypatch.setattr(native_mounts, "_mountinfo_text", lambda: text)
    try:
        if fault == "missing":
            assert store.export(record["trajectory_id"])["native_state"] == "transfer_required"
        else:
            with pytest.raises(RuntimeError, match="share|writable"):
                store.export(record["trajectory_id"])
        assert not directory.exists() or not list(directory.iterdir())
    finally:
        if fault == "mode": directory.chmod(0o755)


def test_nested_local_mount_cannot_hide_inside_a_network_share(tmp_path, monkeypatch):
    directory = tmp_path / "local-child"
    directory.mkdir()
    text = mount_line(tmp_path) + mount_line(directory, "ext4")
    monkeypatch.setattr(native_mounts, "_mountinfo_text", lambda: text)
    with pytest.raises(RuntimeError, match="local directory"):
        writable_controller_directory(directory)


def test_runtime_native_export_defaults_to_requiring_a_network_share(tmp_path, monkeypatch):
    source = recorded_store(tmp_path)
    record, _ = recording(source)
    store = TrajectoryStore(source.root, str(source.controller_root), source.profile_path)
    monkeypatch.setattr(native_mounts, "_mountinfo_text", lambda: mount_line("/", "ext4"))
    with pytest.raises(RuntimeError, match="local directory"):
        store.playback(record["trajectory_id"], "127.0.0.1")


def test_native_export_succeeds_after_verified_network_mount_and_profile(tmp_path, monkeypatch):
    store = recorded_store(tmp_path)
    store.require_network_mount = True
    record, _ = recording(store)
    monkeypatch.setattr(native_mounts, "_mountinfo_text", lambda: mount_line(store.controller_root))
    exported = store.export(record["trajectory_id"])
    assert exported["native_state"] == "exported"
    assert (store.controller_root / exported["controller_file"]).is_file()


def test_native_profile_missing_never_writes_to_share(tmp_path, monkeypatch):
    store = recorded_store(tmp_path)
    record, _ = recording(store)
    Path(store.profile_path).unlink()
    with pytest.raises(ValueError, match="cannot be read"):
        store.export(record["trajectory_id"])
    assert not list(store.controller_root.iterdir())


def test_host_preflight_uses_host_variable_names_and_removes_probe(tmp_path, monkeypatch, capsys):
    store = recorded_store(tmp_path)
    monkeypatch.setenv("DOBOT_TEACH_PROFILE_HOST_FILE", store.profile_path)
    monkeypatch.setenv("DOBOT_TEACH_CONTROLLER_HOST_DIR", str(store.controller_root))
    monkeypatch.setattr(sys, "argv", ["native_mounts.py"])
    monkeypatch.setattr(native_mounts, "_mountinfo_text", lambda: mount_line(store.controller_root))
    native_mounts.main()
    assert "verified" in capsys.readouterr().out
    assert not list(store.controller_root.iterdir())


def test_host_preflight_fails_for_missing_configuration(monkeypatch, capsys):
    monkeypatch.delenv("DOBOT_TEACH_PROFILE_HOST_FILE", raising=False)
    monkeypatch.delenv("DOBOT_TEACH_CONTROLLER_HOST_DIR", raising=False)
    monkeypatch.setattr(sys, "argv", ["native_mounts.py"])
    with pytest.raises(SystemExit) as error:
        native_mounts.main()
    assert error.value.code == 1
    assert "preflight failed" in capsys.readouterr().err


def test_preflight_script_runs_without_site_packages_or_robot_sdk():
    result = subprocess.run([sys.executable, "-S", str(ROOT / "dobot/cra/native_mounts.py"), "--help"],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0
    assert "--controller-dir" in result.stdout
