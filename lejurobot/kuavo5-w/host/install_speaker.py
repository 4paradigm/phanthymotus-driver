#!/usr/bin/env python3
"""Image-owned provisioning on the independent chassis computer, not the Jetson.

sudo python3 install_speaker.py --user jt --master http://192.168.26.1:11311
Uses existing ROS Noetic and PulseAudio packages. Never installs an SDK.
"""
import argparse
import os
from pathlib import Path
import pwd
import re
import shutil
import subprocess


def ensure_user_audio(user_name, uid):
    subprocess.run(['loginctl', 'enable-linger', user_name], check=True)
    subprocess.run(['systemctl', 'start', f'user@{uid}.service'], check=True)
    # A fresh robot may never have opened a desktop session. Enable socket
    # activation explicitly rather than relying on this machine's prior login.
    subprocess.run(['runuser', '-u', user_name, '--', 'env',
                    f'XDG_RUNTIME_DIR=/run/user/{uid}',
                    f'DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/{uid}/bus',
                    'systemctl', '--user', 'enable', '--now', 'pulseaudio.socket'], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--user', required=True)
    parser.add_argument('--master', required=True)
    parser.add_argument('--ros-ip', default='192.168.26.22')
    parser.add_argument('--volume', type=int, default=50)
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error('run on the chassis with sudo')
    if not all(re.fullmatch(r'[A-Za-z0-9_./:@-]+', v) for v in (args.user, args.master, args.ros_ip)):
        parser.error('unsupported systemd characters')
    if not 0 <= args.volume <= 100:
        parser.error('volume must be from 0 to 100')
    user = pwd.getpwnam(args.user)
    if not Path('/opt/ros/noetic/setup.bash').is_file() or not all(shutil.which(p) for p in ('paplay', 'pactl')):
        parser.error('ROS Noetic and paplay/pactl must already be installed')
    root = Path('/opt/phanthy-motus/kuavo5w-speaker')
    root.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(Path(__file__).with_name('speaker_relay.py'), root / 'speaker_relay.py')
    unit = f'''[Unit]
Description=Kuavo 5-W chassis PCM speaker (no motor access)
Wants=network-online.target user@{user.pw_uid}.service
After=network-online.target user@{user.pw_uid}.service
[Service]
User={args.user}
Environment=ROS_MASTER_URI={args.master}
Environment=ROS_IP={args.ros_ip}
Environment=XDG_RUNTIME_DIR=/run/user/{user.pw_uid}
Environment=PULSE_SERVER=unix:/run/user/{user.pw_uid}/pulse/native
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && exec python3 {root}/speaker_relay.py --volume {args.volume}'
Restart=on-failure
RestartSec=3
TimeoutStopSec=5
NoNewPrivileges=true
[Install]
WantedBy=multi-user.target
'''
    Path('/etc/systemd/system/kuavo5w-speaker.service').write_text(unit)
    ensure_user_audio(args.user, user.pw_uid)
    subprocess.run(['systemctl', 'daemon-reload'], check=True)
    subprocess.run(['systemctl', 'enable', '--now', 'kuavo5w-speaker.service'], check=True)
    subprocess.run(['systemctl', 'restart', 'kuavo5w-speaker.service'], check=True)
    print('Chassis speaker service installed. Card start checks the verified analog sink.')


if __name__ == '__main__':
    main()
