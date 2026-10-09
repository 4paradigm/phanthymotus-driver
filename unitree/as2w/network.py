"""Robot-network interface selection helpers."""
import ipaddress
import socket
import struct


def _interface_ipv4(name):
    """Return an interface IPv4 address when the host exposes one."""
    try:
        import fcntl
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            data = fcntl.ioctl(sock.fileno(), 0x8915, struct.pack("256s", name.encode()[:15]))
        return socket.inet_ntoa(data[20:24])
    except (AttributeError, OSError, struct.error, ValueError):
        return None


def _interface_up(name):
    try:
        with open(f"/sys/class/net/{name}/operstate", encoding="ascii") as stream:
            return stream.read().strip() == "up"
    except (OSError, UnicodeError):
        # Contract tests and non-Linux hosts may not expose sysfs; retain the
        # name-based fallback there rather than dropping every candidate.
        return True


def interface_candidates(requested="", configured="", robot_subnet="192.168.123.0/24"):
    """Use an explicit interface, or discover likely wired robot adapters."""
    if requested:
        return [requested]
    if configured:
        return [configured]
    try:
        names = [name for _, name in socket.if_nameindex()]
    except (AttributeError, OSError):
        names = []
    candidates = [name for name in names if name != "lo" and _interface_up(name) and (
        name.startswith("eth") or name.startswith("en") or name.startswith("usb")
    )]
    try:
        network = ipaddress.ip_network(robot_subnet, strict=False) if robot_subnet else None
    except ValueError:
        network = None
    if network:
        robot_candidates = [name for name in candidates
                            if (address := _interface_ipv4(name)) and
                            ipaddress.ip_address(address) in network]
        if robot_candidates:
            candidates = robot_candidates
    # Never return an empty sentinel here.  Passing ``None``/``""`` to
    # ChannelFactoryInitialize lets CycloneDDS choose its own interface and
    # can silently bind the robot participant to Wi-Fi.  The caller must keep
    # the driver in degraded mode when no eligible wired adapter exists.
    return sorted(candidates, key=lambda name: (not name.startswith("enx"), name))
