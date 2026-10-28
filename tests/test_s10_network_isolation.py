import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('s10_network_isolation', Path(__file__).with_name('s10_network_isolation.py'))
isolation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(isolation)


def fixture_files():
    return {'/sys/class/net/lo/flags': '0x9', '/sys/class/net/tunl0/flags': '0x80',
            '/proc/net/route': 'Iface Destination Gateway Flags\n',
            '/proc/net/ipv6_route': '', '/proc/net/if_inet6': ''}


def test_down_tunnel_devices_in_none_network_are_not_usable_interfaces():
    isolation.require_loopback_network(names=['lo', 'tunl0'], read_text=fixture_files().__getitem__)


@pytest.mark.parametrize('key,value', [
    ('/sys/class/net/tunl0/flags', '0x81'),
    ('/sys/class/net/lo/flags', '0x8'),
    ('/proc/net/route', 'Iface Destination\ntunl0 00000000\n'),
    ('/proc/net/ipv6_route', '00000000 00 00000000 tunl0\n'),
    ('/proc/net/if_inet6', 'fe80000000000000 02 40 20 80 tunl0\n'),
])
def test_usable_or_routed_non_loopback_network_is_rejected(key, value):
    files = fixture_files()
    files[key] = value
    with pytest.raises(RuntimeError):
        isolation.require_loopback_network(names=['lo', 'tunl0'], read_text=files.__getitem__)
