#!/usr/bin/env python3
"""One-time host provisioning; driver startup then requests fixed services.

Run: sudo python3 host/install.py --user USER --workspace /path/to/catkin_ws
       --master http://192.168.26.1:11311 --ros-ip 192.168.26.12
No passwords are stored; does not start or enable the robot controller.
"""
import argparse
import os
from pathlib import Path
import pwd
import re
import shutil
import subprocess
import tempfile
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--user', required=True)
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--master', required=True)
    parser.add_argument('--ros-ip', required=True)
    parser.add_argument('--message-overlay', help='Optional directory containing controller-generated kuavo_msgs Python package; isolates its ROS1 wire version to bridge/rosapi')
    parser.add_argument('--head-workspace', help='Optional installed Orbbec ROS1 workspace; auto-detect known local workspace if absent')
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error('Run on the ROS1 host as root (sudo)')
    if not all(re.fullmatch(r'[a-zA-Z0-9_./:@-]+', value)
               for value in (args.user, args.workspace, args.master, args.ros_ip, args.head_workspace or 'none')):
        parser.error('Unsupported shell/systemd characters in arguments')
    user = pwd.getpwnam(args.user)
    setup = Path(args.workspace) / 'devel/setup.bash'
    if not setup.is_file():
        parser.error('Missing workspace devel/setup.bash')
    root = Path('/opt/phanthy-motus/kuavo5w-host')
    root.mkdir(parents=True, exist_ok=True)
    overlay_changed = False
    if args.message_overlay:
        package = Path(args.message_overlay).resolve() / 'kuavo_msgs'
        if not (package / 'msg/__init__.py').is_file() or not (package / 'srv/__init__.py').is_file():
            parser.error('Message overlay must contain generated kuavo_msgs/msg and kuavo_msgs/srv packages')
        overlay_changed = any(not (root / 'ros1_messages/kuavo_msgs' / path.relative_to(package)).is_file()
            or (root / 'ros1_messages/kuavo_msgs' / path.relative_to(package)).read_bytes() != path.read_bytes()
            for path in package.rglob('*.py'))
        shutil.copytree(package, root / 'ros1_messages/kuavo_msgs', dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    launch = root / 'head_camera.launch'
    head_launch_changed = launch.is_file() and launch.read_bytes() != (Path(__file__).parent / launch.name).read_bytes()
    for name in ('rosbridge_loopback.py', 'rosbridge.launch', 'battery_relay.py', 'runtime_manager.py',
                 'head_camera.launch', 'head_jpeg.py', 'head_depth.py'):
        shutil.copyfile(Path(__file__).parent / name, root / name)
        (root / name).chmod(0o644)
    config = Path('/opt/phanthy-motus/drivers/lejurobot-kuavo5-w/config.yaml')
    config.parent.mkdir(parents=True, exist_ok=True)
    if not config.exists():
        # Exclusive creation: never replace an operator's existing config.
        with config.open('x') as stream:
            stream.write((Path(__file__).parents[1] / 'config.yaml').read_text())
    else:
        # Add newly supported cards, preserving explicit switches/limits/URLs.
        operator = yaml.safe_load(config.read_text())
        if not isinstance(operator, dict) or not isinstance(operator.get('plugins', {}), dict):
            parser.error('Existing operator config is not a mapping')
        defaults = yaml.safe_load((Path(__file__).parents[1] / 'config.yaml').read_text())
        plugins = operator.setdefault('plugins', defaults['plugins'].copy())
        depth_new = 'camera_depth' not in plugins
        aliases = {'camera_rgb': 'camera_left', 'camera_rgb_right': 'camera_right'}
        for old, new in aliases.items():
            if old in plugins:
                plugins.setdefault(new, plugins.pop(old))
        plugins.pop('state', None)
        for name in ('battery', 'laser_scan', 'camera_left', 'camera_right', 'camera_head', 'camera_depth', 'speaker'):
            plugins.setdefault(name, defaults['plugins'][name])
        sensors = operator.setdefault('sensors', {})
        if 'auto_start' in sensors:
            sensors['auto_start'] = [aliases.get(name, name) for name in sensors['auto_start'] if name != 'state']
            if depth_new and plugins['camera_depth'].get('enabled', True) and 'camera_depth' not in sensors['auto_start']:
                sensors['auto_start'].append('camera_depth')
        with tempfile.NamedTemporaryFile(mode='w', dir=config.parent, delete=False) as stream:
            yaml.safe_dump(operator, stream, sort_keys=False)
            staged = Path(stream.name)
        staged.chmod(0o644)
        staged.replace(config)
    commands = {
        'kuavo5w-rosbridge': f'roslaunch --wait {root}/rosbridge.launch',
        'kuavo5w-battery-relay': f'python3 {root}/battery_relay.py',
    }
    head = None
    candidates = [Path(args.head_workspace)] if args.head_workspace else [
        Path(user.pw_dir) / 'kuavo_data_pilot', Path(args.workspace)]
    for candidate in candidates:
        if ((candidate / 'devel/setup.bash').is_file()
                and (candidate / 'src/OrbbecSDK_ROS1/launch/gemini_330_series.launch').is_file()):
            head = candidate / 'devel/setup.bash'
            commands['kuavo5w-head-camera'] = f'roslaunch --wait {root}/head_camera.launch'
            commands['kuavo5w-head-jpeg-relay'] = f'python3 {root}/head_jpeg.py'
            commands['kuavo5w-head-depth-relay'] = f'python3 {root}/head_depth.py'
            break
    if args.head_workspace and head is None:
        parser.error('Head workspace must contain built OrbbecSDK_ROS1 gemini_330_series.launch')
    for name, command in commands.items():
        unit = f'''[Unit]
Description=Kuavo 5-W {name}
After=network-online.target
Wants=network-online.target
[Service]
User={args.user}
WorkingDirectory={user.pw_dir}
Environment=ROS_MASTER_URI={args.master}
Environment=ROS_IP={args.ros_ip}
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source {head if name.startswith("kuavo5w-head-") else setup} && exec {command}'
Restart=on-failure
RestartSec=3
TimeoutStopSec=10
[Install]
WantedBy=multi-user.target
'''
        Path('/etc/systemd/system/' + name + '.service').write_text(unit)
    Path('/etc/systemd/system/kuavo5w-runtime-manager.service').write_text(f'''[Unit]
Description=Kuavo 5-W fixed-operation loopback startup manager
After=network.target
[Service]
ExecStart=/usr/bin/python3 {root}/runtime_manager.py
Restart=on-failure
RestartSec=3
NoNewPrivileges=true
ProtectSystem=strict
ProtectHome=true
PrivateTmp=true
[Install]
WantedBy=multi-user.target
''')
    subprocess.run(['systemctl', 'daemon-reload'], check=True)
    subprocess.run(['systemctl', 'enable', '--now', 'kuavo5w-runtime-manager.service'], check=True)
    # enable --now leaves an existing helper process running with old routes.
    subprocess.run(['systemctl', 'restart', 'kuavo5w-runtime-manager.service'], check=True)
    if overlay_changed:
        # Already-running bridge imports are cached; changing files alone would
        # leave a wrong sensorsData MD5 until the next reboot.
        subprocess.run(['systemctl', 'try-restart', 'kuavo5w-rosbridge.service'], check=True)
    if head is not None and head_launch_changed:
        # Reload a running RGB-only pipeline on upgrade; inactive cameras remain
        # inactive until a card requests them. Never touch robot controllers.
        subprocess.run(['systemctl', 'try-restart', 'kuavo5w-head-camera.service'], check=True)
    print('Host helper installed. Driver startup ensures rosbridge/battery; camera card start ensures installed RGB/depth services.')


if __name__ == '__main__':
    main()
