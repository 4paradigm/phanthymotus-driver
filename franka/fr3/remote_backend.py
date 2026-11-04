"""SSH stdio transport with an application heartbeat; no listening TCP service.

Server is limited to configured robot IPs and launches only the native backend.
No action is retried here. A broken link expires the lease and signals the native
process to stop; physical stop still requires native confirmation/robot interlocks.
"""
import argparse
import ipaddress
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

HEARTBEAT = b"@FR3_PING\n"
LEASE_SECONDS = 2.0
MAX_LINE = 4096


def allowed_robot(address, allowed):
    address = str(ipaddress.ip_address(address))
    if address not in {str(ipaddress.ip_address(v)) for v in allowed.split(',') if v}:
        raise ValueError('Robot IP is not in the backend allowlist')
    return address


def relay(process, heartbeat_sender):
    """Forward bounded input, without blocking telemetry or heartbeat checks."""
    os.set_blocking(process.stdin.fileno(), False)
    sources = {sys.stdin.fileno(): 'input', process.stdout.fileno(): 'output',
               process.stderr.fileno(): 'error'}
    pending = b''
    queue = b''
    last_input = last_ping = time.monotonic()
    try:
        while process.poll() is None:
            now = time.monotonic()
            if heartbeat_sender and now-last_ping >= .25:
                queue += HEARTBEAT
                last_ping = now
            if not heartbeat_sender and now-last_input > LEASE_SECONDS:
                raise TimeoutError('Orin heartbeat lease expired')
            ready, writable, _ = select.select(list(sources),
                [process.stdin.fileno()] if queue else [], [], .05)
            for fd in ready:
                data = os.read(fd, 65536)
                kind = sources[fd]
                if not data:
                    if kind == 'input':
                        return  # finally requests native stop, never automatic resume
                    del sources[fd]
                    continue
                if kind == 'input':
                    pending += data
                    while b'\n' in pending:
                        line, pending = pending.split(b'\n', 1)
                        if len(line) > MAX_LINE:
                            raise ValueError('Command exceeds limit')
                        if not heartbeat_sender and line+b'\n' == HEARTBEAT:
                            last_input = time.monotonic()
                        else:
                            queue += line+b'\n'
                    if len(pending) > MAX_LINE or len(queue)>65536:
                        raise ValueError('Input backlog exceeds limit')
                else:
                    os.write(sys.stdout.fileno() if kind=='output' else sys.stderr.fileno(), data)
            if writable and queue:
                try:
                    n=os.write(process.stdin.fileno(), queue)
                    queue=queue[n:]
                except BlockingIOError:
                    pass
    finally:
        if process.poll() is None:
            if not heartbeat_sender:
                process.send_signal(signal.SIGINT)
            else:
                process.stdin.close()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                try: process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill();process.wait()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('mode',choices=['client','server'])
    parser.add_argument('robot_ip')
    parser.add_argument('--allowed',default='')
    parser.add_argument('--native',default='/usr/local/bin/franka-state-reader')
    args=parser.parse_args()
    address=str(ipaddress.ip_address(args.robot_ip))
    if args.mode=='server':
        allowed_robot(address,args.allowed)
        command=[args.native,address]
    else:
        host=os.environ['FRANKA_REMOTE_HOST']
        import re
        if not re.fullmatch(r'[a-z_][a-z0-9_-]*@[0-9.]+',host):
            raise ValueError('Remote host must be user@IPv4')
        base=os.environ['FRANKA_REMOTE_DIR']
        if not re.fullmatch(r'/[A-Za-z0-9_./-]+',base):raise ValueError('Invalid backend directory')
        command=['ssh','-T','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes',
            '-o','ConnectTimeout=5','-o','ServerAliveInterval=1','-o','ServerAliveCountMax=2',
            '-i',os.environ['FRANKA_REMOTE_KEY'],
            '-o','UserKnownHostsFile='+os.environ['FRANKA_REMOTE_KNOWN_HOSTS'],
            host,'fr3-backend',address]
    process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE,bufsize=0)
    relay(process,args.mode=='client')
    return process.returncode or 0

if __name__=='__main__':
    try:sys.exit(main())
    except Exception as error:
        print(str(error),file=sys.stderr);sys.exit(1)
