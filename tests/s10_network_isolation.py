"""Reject usable non-loopback networking before synthetic DDS publication.

Docker Desktop can expose down, addressless kernel tunnel devices even with
--network none. Those devices are allowed only when down and without routes.
"""
import socket
from pathlib import Path


def require_loopback_network(*, names=None, read_text=None):
    names = [name for _, name in socket.if_nameindex()] if names is None else names
    read_text = (lambda path: Path(path).read_text()) if read_text is None else read_text
    if 'lo' not in names:
        raise RuntimeError('Synthetic DDS requires a loopback interface')
    for name in names:
        flags = int(read_text(f'/sys/class/net/{name}/flags').strip(), 16)
        if name != 'lo' and flags & 1:
            raise RuntimeError('Synthetic DDS requires Docker --network none: non-loopback interface is up')
        if name == 'lo' and flags & 9 != 9:
            raise RuntimeError('Synthetic DDS requires an active loopback interface')
    for line in read_text('/proc/net/route').splitlines()[1:]:
        if line.split() and line.split()[0] != 'lo':
            raise RuntimeError('Synthetic DDS refuses non-loopback IPv4 routes')
    for line in read_text('/proc/net/ipv6_route').splitlines():
        if line.split() and line.split()[-1] != 'lo':
            raise RuntimeError('Synthetic DDS refuses non-loopback IPv6 routes')
    for line in read_text('/proc/net/if_inet6').splitlines():
        if line.split() and line.split()[-1] != 'lo':
            raise RuntimeError('Synthetic DDS refuses non-loopback IPv6 addresses')
