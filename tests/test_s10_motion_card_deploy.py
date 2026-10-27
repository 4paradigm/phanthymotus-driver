"""Reproducible packaging/provisioning contracts; no robot or network I/O."""
import importlib.util
import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / 'deep_robotics/s10_pro'
spec = importlib.util.spec_from_file_location('s10_prepare_init', DRIVER / 'deploy/init_prepare_policy.py')
setup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(setup)


def test_new_modules_are_in_image_and_timed_card_is_in_manifest():
    dockerfile = (DRIVER / 'Dockerfile').read_text()
    assert 'canvas_posture.py' in dockerfile and 'stationary_feedback.py' in dockerfile
    cards = yaml.safe_load((DRIVER / 'driver.yaml').read_text())['cards']
    assert {'motion', 'timed_move'} <= {c['name'] for c in cards}


def test_supervised_overlay_mounts_identity_and_policy_read_only():
    service = yaml.safe_load((DRIVER / 'deploy/service.supervised.yml').read_text())['deep-robotics-s10-pro']
    assert all(mount.endswith(':ro') for mount in service['volumes'])
    assert any('/run/s10/prepare-machine-id:ro' in m for m in service['volumes'])
    assert 'S10_OPERATOR_AUTHORIZATION_FILE=/run/s10/operator-authorization.json' in service['environment']


def test_provision_defaults_to_dry_run_without_reading_machine_id(monkeypatch, capsys):
    monkeypatch.setattr(setup.Path, 'read_text', lambda *a, **k: pytest.fail('dry run read host identity'))
    setup.main([])
    assert 'Dry run' in capsys.readouterr().out


def test_apply_requires_all_site_confirmations():
    with pytest.raises(SystemExit):
        setup.main(['--apply'])


def test_provision_protects_files_and_never_overwrites_existing_policy(tmp_path, monkeypatch):
    monkeypatch.setattr(setup.os, 'geteuid', lambda: 0)
    real_lstat = setup.Path.lstat
    class RootOwner:
        def __init__(self, info): self.info = info
        def __getattr__(self, name): return 0 if name == 'st_uid' else getattr(self.info, name)
    monkeypatch.setattr(setup.Path, 'lstat', lambda self, *a, **k: RootOwner(real_lstat(self, *a, **k)))
    machine = 'a' * 32
    paths = setup.provision(tmp_path / 's10', machine)
    before = [p.read_bytes() for p in paths]
    assert all(p.stat().st_mode & 0o777 == 0o600 for p in paths)
    policy = json.loads(paths[1].read_text())
    assert policy['activation'] == 'explicit_prepare'
    assert policy['allowed_actions'] == ['stand', 'lie', 'move_mode', 'forward_30cm', 'backward_30cm']
    assert machine not in paths[1].read_text()
    with pytest.raises(FileExistsError):
        setup.provision(tmp_path / 's10', machine)
    assert [p.read_bytes() for p in paths] == before


@pytest.mark.parametrize('machine,duration', [('bad', 3600), ('a'*32, 3601), ('a'*32, True)])
def test_policy_rejects_invalid_identity_and_session_duration(machine, duration):
    with pytest.raises(ValueError):
        setup.make_policy(machine, duration_s=duration)
