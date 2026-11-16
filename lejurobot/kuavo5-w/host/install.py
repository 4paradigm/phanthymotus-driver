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
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error('Run on the ROS1 host as root (sudo)')
    if not all(re.fullmatch(r'[a-zA-Z0-9_./:@-]+', value)
               for value in (args.user, args.workspace, args.master, args.ros_ip)):
        parser.error('Unsupported shell/systemd characters in arguments')
    user = pwd.getpwnam(args.user)
    setup = Path(args.workspace) / 'devel/setup.bash'
    if not setup.is_file():
        parser.error('Missing workspace devel/setup.bash')
    root = Path('/opt/phanthy-motus/kuavo5w-host')
    root.mkdir(parents=True, exist_ok=True)
    for name in ('rosbridge_loopback.py', 'rosbridge.launch', 'battery_relay.py', 'runtime_manager.py'):
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
        for name in ('battery', 'laser_scan', 'camera_rgb', 'camera_rgb_right'):
            plugins.setdefault(name, defaults['plugins'][name])
        with tempfile.NamedTemporaryFile(mode='w', dir=config.parent, delete=False) as stream:
            yaml.safe_dump(operator, stream, sort_keys=False)
            staged = Path(stream.name)
        staged.chmod(0o644)
        staged.replace(config)
    commands = {
        'kuavo5w-rosbridge': f'roslaunch --wait {root}/rosbridge.launch',
        'kuavo5w-battery-relay': f'python3 {root}/battery_relay.py',
    }
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
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source {setup} && exec {command}'
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
    print('Host helper installed. Driver startup can now ensure rosbridge and battery telemetry.')


if __name__ == '__main__':
    main()
