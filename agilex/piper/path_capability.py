"""Fail-closed MOVE P acceptance, bound to one adapter/firmware/source/config.

The official pinned V2 piper_ctrl_moveP.py uses mode 0x00 + EndPoseCtrl.
No acceptance is shipped. Metadata and monitor reads never query or command CAN.
"""
from functools import lru_cache
import hashlib
import json
from pathlib import Path


@lru_cache(maxsize=1)
def driver_digest():
    digest = hashlib.sha256()
    for source in sorted(Path(__file__).parent.glob('*.py')):
        digest.update(source.name.encode())
        digest.update(source.read_bytes())
    return digest.hexdigest()


def adapter_serial(interface):
    try:
        device = (Path('/sys/class/net') / interface / 'device').resolve(strict=True)
        for parent in (device, *device.parents):
            serial = parent / 'serial'
            if serial.is_file():
                return serial.read_text().strip()
    except OSError:
        pass
    return None


def configuration_digest(config):
    intent = config.get('intent_control', {})
    values = {k: v for k, v in intent.items() if k != 'point_to_point_acceptance'}
    for key in ('joint_limits_deg', 'gripper_limits_mm', 'can_interface'):
        values[key] = config.get(key)
    return hashlib.sha256(json.dumps(values, sort_keys=True, allow_nan=False).encode()).hexdigest()


def path_capability(config, *, serial=None, firmware=None):
    identity = {'device_serial': serial, 'firmware': firmware,
                'driver_sha256': driver_digest(), 'configuration_sha256': configuration_digest(config)}
    record = config.get('intent_control', {}).get('point_to_point_acceptance')
    reason = 'Point-to-point requires supervised hardware acceptance for this device, firmware and driver.'
    accepted = (isinstance(record, dict) and record.get('accepted') is True
                and isinstance(record.get('evidence'), str) and bool(record['evidence'].strip())
                and bool(serial) and bool(firmware)
                and all(record.get(key) == value for key, value in identity.items()))
    return {'available': bool(accepted), 'reason': None if accepted else reason, **identity}
