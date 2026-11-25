"""Executed over authenticated SSH from the driver image, on the target host.

The payload contains image-owned source, not commands or arbitrary destinations.
Controller upgrades are staged for next boot; a live motor owner is never restarted.
"""
import base64
import hashlib
import json
import os
from pathlib import Path
import pwd
import re
import shlex
import subprocess
import sys
import tempfile

FILES = {'config.yaml', 'host/install.py', 'host/install_controller.py',
         'host/controller_boot.py', 'host/install_speaker.py', 'host/speaker_relay.py',
         'host/rosbridge_loopback.py', 'host/rosbridge.launch', 'host/runtime_manager.py',
         'host/battery_relay.py', 'host/head_camera.launch', 'host/head_jpeg.py', 'host/head_depth.py', 'host/rgbd_geometry.py', 'host/head_imu.py', 'host/head_rgbd.py', 'host/body_mount.py'}
ROOTS = {'host': Path('/opt/phanthy-motus/kuavo5w-host'),
         'controller': Path('/opt/phanthy-motus/kuavo5w-controller'),
         'chassis': Path('/opt/phanthy-motus/kuavo5w-speaker')}
OWNED = {'host': ('rosbridge_loopback.py', 'rosbridge.launch', 'runtime_manager.py',
                 'battery_relay.py', 'head_camera.launch', 'head_jpeg.py', 'head_depth.py', 'rgbd_geometry.py', 'head_imu.py', 'head_rgbd.py', 'body_mount.py'),
         'controller': ('controller_boot.py',), 'chassis': ('speaker_relay.py',)}
UNIT_DIR = Path('/etc/systemd/system')
CONTROLLER_RUNTIME = Path('/run/kuavo5w-controller')


class Blocked(RuntimeError):
    pass


def run(command, timeout=30, check=True, **kwargs):
    result = subprocess.run(command, capture_output=True, timeout=timeout, **kwargs)
    if check and result.returncode:
        raise Blocked('operation_failed:' + Path(command[0]).name)
    return result


def safe(value, label):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_./:@-]+', value):
        raise Blocked('invalid_' + label)
    return value


def discover_workspace(role, target, home):
    if target.get('workspace'):
        candidates = [Path(safe(target['workspace'], 'workspace'))]
    else:
        name = 'kuavo_ros_application' if role == 'host' else 'kuavo-ros-opensource'
        candidates = [home / name]
        candidates += [path for path in Path('/home').glob('*/' + name) if path not in candidates]
    matches = [path for path in candidates if (path / 'devel/setup.bash').is_file()]
    if role == 'controller':
        matches = [p for p in matches if (p / 'src/kuavo_assets/models/biped_s63/urdf/biped_s63.urdf').is_file()
                   and (p / 'src/humanoid-control/humanoid_controllers').is_dir()]
    if len(matches) != 1:
        raise Blocked('missing_or_ambiguous_' + role + '_workspace')
    return Path(safe(str(matches[0]), 'workspace'))


def head_workspace(target, home):
    if target.get('head_workspace'):
        candidates = [Path(safe(target['head_workspace'], 'head_workspace'))]
    else:
        candidates = [home / 'kuavo_data_pilot', home / 'kuavo_ros_application']
    found = [path for path in candidates if (path / 'devel/setup.bash').is_file() and
             (path / 'src/OrbbecSDK_ROS1/launch/gemini_330_series.launch').is_file()]
    if not found:
        raise Blocked('missing_vendor_orbbec_workspace')
    # Preferred factory workspace precedes the older application overlay.
    return Path(safe(str(found[0]), 'head_workspace'))


def dependencies(role, workspace, install_missing):
    if not Path('/opt/ros/noetic/setup.bash').is_file():
        raise Blocked('missing_vendor_ros_noetic')
    source = 'source /opt/ros/noetic/setup.bash'
    if workspace:
        source += ' && source ' + shlex.quote(str(workspace / 'devel/setup.bash'))
    modules = {'rospy': 'ros-noetic-rospy'}
    if role == 'host':
        modules.update({'rosbridge_library': 'ros-noetic-rosbridge-server',
                        'rosapi': 'ros-noetic-rosapi', 'yaml': 'python3-yaml',
                        'cv2': 'python3-opencv', 'numpy': 'python3-numpy'})
    missing = set()
    for module, package in modules.items():
        if run(['/bin/bash', '-c', source + ' && python3 -c ' + shlex.quote('import ' + module)], check=False).returncode:
            missing.add(package)
    if role == 'chassis':
        for binary in ('pactl', 'paplay'):
            if run(['/bin/sh', '-c', 'command -v ' + binary], check=False).returncode:
                missing.add('pulseaudio-utils')
        if not Path('/usr/bin/pulseaudio').is_file():
            missing.add('pulseaudio')
    if missing:
        if not install_missing:
            raise Blocked('missing_runtime_packages')
        # Fixed allowlist above; no operator-provided package or shell argument.
        env = dict(os.environ, DEBIAN_FRONTEND='noninteractive')
        run(['apt-get', 'update'], timeout=100, env=env)
        run(['apt-get', 'install', '-y', '--no-install-recommends', *sorted(missing)], timeout=120, env=env)


def export_messages(workspace):
    candidates = [workspace / location / 'lib/python3/dist-packages/kuavo_msgs'
                  for location in ('devel', 'install')]
    found = next((p for p in candidates if (p / 'msg/__init__.py').is_file()
                  and (p / 'srv/__init__.py').is_file()), None)
    if found is None:
        raise Blocked('missing_controller_generated_kuavo_msgs')
    result, size = {}, 0
    # catkin_tools links devel exports into devel/.private. Resolve only inside
    # the discovered workspace, and enumerate the known genpy package layout
    # rather than recursively following arbitrary links or directory cycles.
    paths = [path for subdir in ('', 'msg', 'srv') for path in (found / subdir).glob('*.py')]
    for path in sorted(paths):
        if workspace.resolve() not in path.resolve().parents:
            raise Blocked('controller_message_link_outside_workspace')
        data = path.read_bytes()
        size += len(data)
        if size > 2_000_000:
            raise Blocked('oversized_controller_messages')
        result['kuavo_msgs/' + str(path.relative_to(found))] = base64.b64encode(data).decode()
    return result


def write_messages(directory, messages):
    if not isinstance(messages, dict) or not messages:
        raise Blocked('controller_message_export_unavailable')
    size = 0
    for name, encoded in messages.items():
        parts = Path(name).parts
        if (not isinstance(name, str) or not name.startswith('kuavo_msgs/') or
                not name.endswith('.py') or '..' in parts or Path(name).is_absolute()):
            raise Blocked('invalid_controller_message_path')
        try:
            data = base64.b64decode(encoded, validate=True)
        except (ValueError, TypeError):
            raise Blocked('invalid_controller_message_data') from None
        size += len(data)
        if size > 2_000_000:
            raise Blocked('oversized_controller_messages')
        path = directory / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    if not all((directory / 'kuavo_msgs' / part / '__init__.py').is_file() for part in ('msg', 'srv')):
        raise Blocked('incomplete_controller_messages')


def activate_controller(runtime=None):
    runtime = runtime or CONTROLLER_RUNTIME
    unit = 'kuavo5w-controller.service'
    result = run(['systemctl', 'show', unit, '--property=ActiveState,ExecMainStartTimestampMonotonic',
                  '--value']).stdout.decode().splitlines()
    # Do not assume property output order from different systemd versions.
    if 'active' in result or 'activating' in result:
        return 'already_running_not_restarted'
    if 'failed' in result or any(item.isdigit() and int(item) > 0 for item in result):
        raise Blocked('controller_already_attempted_this_boot_no_retry')
    runtime.mkdir(parents=True, exist_ok=True)
    marker = runtime / 'automatic-start-attempted'
    try:
        with marker.open('x'):
            pass
    except FileExistsError:
        raise Blocked('controller_already_attempted_this_boot_no_retry') from None
    run(['systemctl', 'start', '--no-block', unit])
    return 'initialization_requested_once'


def ensure_service(unit):
    # A stopped non-motor helper may be safely restarted. Do not restart an
    # already-active service or change its playback/camera session every poll.
    run(['systemctl', 'start', '--no-block', unit])


def ensure_vendor_base():
    # This factory service owns chassis cameras, lidar, IMU and odometry, not
    # just audio. Never guess launch files or replace proprietary firmware.
    loaded = run(['systemctl', 'show', 'jaten.service', '--property=LoadState', '--value'],
                 check=False).stdout.decode().strip()
    if loaded != 'loaded':
        raise Blocked('missing_vendor_jaten_service')
    ensure_service('jaten.service')


def install(request):
    role, target = request.get('role'), request.get('target')
    if role not in ROOTS or not isinstance(target, dict):
        raise Blocked('invalid_role')
    user = pwd.getpwnam(safe(target['user'], 'user'))
    home = Path(user.pw_dir)
    ros_ip = safe(target['ros_ip'], 'ros_ip')
    master = safe(request['master'], 'master')
    workspace = discover_workspace(role, target, home) if role != 'chassis' else None
    head = head_workspace(target, home) if role == 'host' else None
    if role == 'controller' and request.get('auto_initialize_and_pause') is not True:
        # Revoking the boot policy affects future boots; do not kill a live
        # controller or replay initialization to implement this configuration.
        if (UNIT_DIR / 'kuavo5w-controller.service').is_file():
            run(['systemctl', 'disable', 'kuavo5w-controller.service'])
        return {'ok': True, 'state': 'disabled', 'changed': False,
                'messages': export_messages(workspace),
                'controller_activation': 'automatic_boot_disabled_existing_process_untouched'}
    if role == 'controller' and not Path('/opt/drake/lib/libdrake.so').is_file():
        raise Blocked('missing_vendor_drake_runtime')
    messages = export_messages(workspace) if role == 'controller' else request.get('messages')
    files = request.get('files')
    if not isinstance(files, dict) or set(files) != FILES:
        raise Blocked('image_payload_incomplete')
    # Include settings and exact runtime overlay, not credentials, in identity.
    identity = {'role': role, 'target': target, 'master': master,
                'files': files, 'messages': messages if role == 'host' else None}
    digest = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    root = ROOTS[role]
    record = root / 'deployment.json'
    try:
        previous = json.loads(record.read_text()) if record.is_file() else {}
    except ValueError:
        previous = {}
    changed = previous.get('sha256') != digest
    if changed:
        dependencies(role, workspace, request.get('install_missing_packages') is True)
    # A deleted/corrupted owned file must repair even with an unchanged marker.
    for name in OWNED[role]:
        path = root / name
        if not path.is_file() or base64.b64encode(path.read_bytes()).decode() != files['host/' + name]:
            changed = True
    units = {'host': ('kuavo5w-runtime-manager.service', 'kuavo5w-rosbridge.service',
                     'kuavo5w-battery-relay.service', 'kuavo5w-head-camera.service',
                     'kuavo5w-head-jpeg-relay.service', 'kuavo5w-head-imu-relay.service'),
             'controller': ('kuavo5w-controller.service',), 'chassis': ('kuavo5w-speaker.service',)}[role]
    if not all((UNIT_DIR / unit).is_file() for unit in units):
        changed = True
    for unit, expected in previous.get('unit_sha256', {}).items():
        if unit not in units:
            continue
        path = UNIT_DIR / unit
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            changed = True
    with tempfile.TemporaryDirectory(prefix='kuavo5w-image-') as temporary:
        stage = Path(temporary)
        for name, encoded in files.items():
            data = base64.b64decode(encoded, validate=True)
            if len(data) > 300_000:
                raise Blocked('oversized_image_file')
            path = stage / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        if role == 'host':
            write_messages(stage / 'messages', messages)
            for name, encoded in messages.items():
                path = root / 'ros1_messages' / name
                if not path.is_file() or base64.b64encode(path.read_bytes()).decode() != encoded:
                    changed = True
        if changed:
            command = ['python3', str(stage / 'host' / {'host': 'install.py',
                       'controller': 'install_controller.py', 'chassis': 'install_speaker.py'}[role]),
                       '--master', master, '--ros-ip', ros_ip]
            if role in ('host', 'chassis'):
                command += ['--user', target['user']]
            if workspace:
                command += ['--workspace', str(workspace)]
            if role == 'host':
                command += ['--head-workspace', str(head), '--message-overlay', str(stage / 'messages')]
            elif role == 'controller':
                if request.get('auto_initialize_and_pause') is not True:
                    raise Blocked('automatic_initialization_not_authorized')
                command += ['--confirm-auto-initialize-and-pause']
            elif role == 'chassis':
                volume = target.get('volume', 50)
                if type(volume) is not int or not 0 <= volume <= 100:
                    raise Blocked('invalid_speaker_volume')
                command += ['--volume', str(volume)]
            run(command, timeout=90)
            record.write_text(json.dumps({'sha256': digest, 'unit_sha256': {
                unit: hashlib.sha256((UNIT_DIR / unit).read_bytes()).hexdigest()
                for unit in units}}))
    activation = None
    if role == 'controller':
        activation = activate_controller()
        if changed and activation == 'already_running_not_restarted':
            activation = 'already_running_updates_apply_next_boot'
    elif role == 'host':
        for unit in units[:3]:
            ensure_service(unit)
        # Head camera/relays remain on-demand through runtime-manager routes.
    else:
        ensure_service(units[0])
        ensure_vendor_base()
    observed = (*units, 'jaten.service') if role == 'chassis' else units
    status = {unit: run(['systemctl', 'is-active', unit], check=False).stdout.decode().strip()
              for unit in observed}
    required = units[:3] if role == 'host' else observed
    # Starting a unit is asynchronous; installed files alone are not evidence
    # that its runtime is ready. Optional head streams remain on-demand.
    state = 'ready' if all(status[unit] == 'active' for unit in required) else 'starting'
    result = {'ok': True, 'state': state, 'changed': changed, 'services': status}
    if workspace:
        result['workspace'] = str(workspace)
    if head:
        result['head_workspace'] = str(head)
    if role == 'controller':
        result.update(messages=messages, controller_activation=activation)
    return result


def main():
    raw = sys.stdin.buffer.read(4_000_001)
    try:
        if len(raw) > 4_000_000:
            raise Blocked('oversized_provisioning_request')
        request = json.loads(raw)
        if os.geteuid() != 0:
            password = request.pop('_sudo_password', None)
            source = globals().get('BOOTSTRAP_SOURCE')
            if source is None:
                raise Blocked('bootstrap_source_missing')
            root_command = 'BOOTSTRAP_SOURCE=%r;exec(compile(BOOTSTRAP_SOURCE,"bootstrap_remote.py","exec"))' % source
            command = ['sudo', '-n', 'python3', '-c', root_command]
            payload = json.dumps(request).encode()
            if run(['sudo', '-n', 'true'], check=False).returncode:
                if password is None:
                    raise Blocked('sudo_credentials_required')
                command = ['sudo', '-S', '-p', '', 'python3', '-c', root_command]
                payload = password.encode() + b'\n' + payload
            result = run(command, input=payload, timeout=220, check=False)
            if result.returncode:
                raise Blocked('sudo_authentication_or_execution_failed')
            sys.stdout.buffer.write(result.stdout)
            return
        request.pop('_sudo_password', None)
        response = install(request)
    except Blocked as exc:
        response = {'ok': False, 'error': str(exc)}
    except Exception as exc:
        response = {'ok': False, 'error': type(exc).__name__}
    print(json.dumps(response))


if __name__ == '__main__':
    main()
