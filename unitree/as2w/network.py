"""Robot-network interface selection helpers."""
import socket


def interface_candidates(requested="", configured=""):
    """Use an explicit interface, or discover likely wired robot adapters."""
    if requested:
        return [requested]
    if configured:
        return [configured]
    try:
        names = [name for _, name in socket.if_nameindex()]
    except (AttributeError, OSError):
        names = []
    candidates = [name for name in names if name != "lo" and (
        name.startswith("eth") or name.startswith("en") or name.startswith("usb")
    )]
    return sorted(candidates, key=lambda name: (not name.startswith("enx"), name)) or [""]
