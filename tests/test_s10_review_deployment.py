"""Deployment regressions raised by PR #451 review; no robot I/O."""
import ast
from pathlib import Path

import pytest
import yaml

from deep_robotics.s10_pro.bridge import reader_environment

DRIVER = Path(__file__).resolve().parents[1] / 'deep_robotics/s10_pro'


def deployed_environment():
    service = yaml.safe_load((DRIVER / 'deploy/service.yml').read_text())['deep-robotics-s10-pro']
    return service, dict(item.split('=', 1) for item in service['environment'])


def test_deployed_config_selects_mounted_vendor_profile(tmp_path):
    service, inherited = deployed_environment()
    config = yaml.safe_load((DRIVER / 'config.yaml').read_text())
    selected = config['reader']['vendor_profile']
    assert any(mount.split(':')[1] == selected and mount.endswith(':ro')
               for mount in service['volumes'])
    assert inherited['S10_REQUIRE_VENDOR_PROFILE'] == '1'
    # Represent the container mount without requiring /opt on the test host.
    profile = tmp_path / 'vendor.xml'
    profile.write_text('<profiles/>')
    config['reader']['vendor_profile'] = str(profile)
    env = reader_environment(config, inherited)
    assert env['FASTRTPS_DEFAULT_PROFILES_FILE'] == str(profile)
    assert env['ROS_DOMAIN_ID'] == '0'
    assert env['RCUTILS_LOGGING_USE_STDOUT'] == '0'


@pytest.mark.parametrize('profile', ['', None, '/missing/s10-vendor.xml'])
def test_deployment_rejects_missing_vendor_profile(profile):
    _, inherited = deployed_environment()
    with pytest.raises(ValueError):
        reader_environment({'reader': {'vendor_profile': profile}}, inherited)


def test_reader_preserves_standard_descriptors():
    tree = ast.parse((DRIVER / 'reader.py').read_text())
    assert not any(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                   and node.func.attr in ('dup2', 'dup') for node in ast.walk(tree))
    # A caller's conflicting logging preference cannot contaminate NDJSON.
    assert reader_environment({}, {'RCUTILS_LOGGING_USE_STDOUT': '1'})[
        'RCUTILS_LOGGING_USE_STDOUT'] == '0'


def test_interface_sources_are_text_and_runtime_excludes_build_tools():
    assert not list((DRIVER / 'vendor').rglob('*.zip'))
    sources = DRIVER / 'vendor/deep-robotics-msg-a0d1a29eec5c4db5a9107595bb51e3be8122b86c'
    assert (sources / 'LICENSE').is_file()
    assert [line.split('#')[0].strip() for line in
            (sources / 'msg/NavCmdValue.msg').read_text().splitlines()[:3]] == [
        'float32 x_vel', 'float32 y_vel', 'float32 yaw_vel']
    dockerfile = (DRIVER / 'Dockerfile').read_text()
    runtime = dockerfile.rsplit('FROM ${ROS_BASE_IMAGE}', 1)[1]
    installs = runtime.split('COPY --from=interfaces', 1)[0]
    assert 'python3-colcon' not in installs and 'rosidl-default-generators' not in installs
    assert 'python3-pytest' not in installs
    assert 'python3-yaml' in installs
    assert not any(line.strip() and not line.startswith('#')
                   for line in (DRIVER / 'requirements.txt').read_text().splitlines())
