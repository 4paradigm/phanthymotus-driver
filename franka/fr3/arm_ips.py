"""Validate a physical left/right controller mapping."""
from hardware import canonical_ip


def mapping(value):
    if not isinstance(value, dict) or set(value) != {'left', 'right'}:
        raise ValueError('Arm IP mapping must contain exactly left and right')
    result = {side: canonical_ip(value[side]) for side in ('left', 'right')}
    if result['left'] == result['right']:
        raise ValueError('Left and right controller IPs must be distinct')
    return result
