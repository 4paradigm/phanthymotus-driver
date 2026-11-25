#!/usr/bin/env python3
"""Explicit one-time lower-host provisioning of automatic init-then-pause.

Enables the next-boot service only. Never stops/replaces a running controller.
"""
import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path('/opt/phanthy-motus/kuavo5w-controller')
UNIT = 'kuavo5w-controller.service'


def service_text(workspace, master, ros_ip):
    for value in (workspace, master, ros_ip):
        if not re.fullmatch(r'[a-zA-Z0-9_./:@-]+', value):
            raise ValueError('Unsupported shell/systemd characters')
    return f'''[Unit]
Description=Kuavo 5-W commissioned boot initialization and software pause
Wants=network-online.target ocs2_h12pro_monitor.service
After=network-online.target ocs2_h12pro_monitor.service
[Service]
Type=simple
User=root
WorkingDirectory={workspace}
Environment=ROS_MASTER_URI={master}
Environment=ROS_IP={ros_ip}
Environment=ROS_HOSTNAME={ros_ip}
Environment=ROBOT_VERSION=63
Environment=KUAVO_CONTROL_SCHEME=multi
Environment=KUAVO_ROS_CONTROL_WS_PATH={workspace}
RuntimeDirectory=kuavo5w-controller
RuntimeDirectoryPreserve=yes
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source {workspace}/devel/setup.bash && exec /usr/bin/python3 {ROOT}/controller_boot.py --robot-version 63 --adopt-existing-paused'
# Never automatically restart motor control after a fault or hardware stop.
Restart=no
KillMode=mixed
KillSignal=SIGINT
TimeoutStopSec=30
[Install]
WantedBy=multi-user.target
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--master', required=True)
    parser.add_argument('--ros-ip', required=True)
    parser.add_argument('--confirm-auto-initialize-and-pause', action='store_true', required=True,
                        help='Operator authorizes motor initialization on every host boot')
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error('Run on the lower ROS1 controller host as root')
    workspace = Path(args.workspace)
    if not (workspace / 'devel/setup.bash').is_file() or not (
            workspace / 'src/kuavo_assets/models/biped_s63/urdf/biped_s63.urdf').is_file():
        parser.error('Missing commissioned lower-host version63 workspace')
    try:
        unit = service_text(args.workspace, args.master, args.ros_ip)
    except ValueError as exc:
        parser.error(str(exc))
    ROOT.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(Path(__file__).with_name('controller_boot.py'), ROOT / 'controller_boot.py')
    (ROOT / 'controller_boot.py').chmod(0o644)
    Path('/etc/systemd/system/' + UNIT).write_text(unit)
    subprocess.run(['systemctl', 'daemon-reload'], check=True)
    subprocess.run(['systemctl', 'enable', UNIT], check=True)
    print('Enabled for NEXT BOOT only. Existing controller left unchanged. No automatic resume.')


if __name__ == '__main__':
    main()
