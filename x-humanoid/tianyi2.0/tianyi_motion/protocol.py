"""Strict, authenticated domain-42 control/2 envelopes (no hardware access)."""
import hmac
import math
from motion_stream import sign, vector

SCHEMA = 'motus.control/2'
FIELDS = frozenset(('schema', 'boot_id', 'session_id', 'seq', 'source_seq',
    'mapping_epoch', 'generated_ns', 'valid_until_ns', 'mode', 'dof', 'values',
    'model_version', 'calibration_version', 'frame'))


def validate_descriptor(value, expected):
    """Validate a /2 action-space declaration against this Driver's capability.

    Core forwards the downstream declaration without negotiating rate/limits.
    Unknown extension fields are allowed, but cannot replace required fields.
    This is separate from the /1 ControlSink contract and per-frame validation.
    """
    def require(condition):
        if not condition:
            raise ValueError('invalid_control_descriptor')

    def number(item):
        try:
            return type(item) in (int, float) and math.isfinite(item)
        except OverflowError:
            return False

    require(isinstance(value, dict))
    mode = value.get('mode')
    require(mode in ('eef_pose', 'joint_position') and mode == expected['mode'])
    require(value.get('control_interface') == SCHEMA and value.get('schema') == SCHEMA)
    require(type(value.get('protocol_version')) is int and value['protocol_version'] == 2)
    require(type(value.get('dof')) is int and value['dof'] == 14)
    for name in ('model_version', 'calibration_version', 'frame'):
        require(isinstance(value.get(name), str) and bool(value[name])
                and value[name] == expected[name])
    units = value.get('units')
    require(isinstance(units, dict))
    require(all(units.get(name) == unit for name, unit in expected['units'].items()))
    groups = value.get('groups')
    require(isinstance(groups, list) and len(groups) == len(expected['groups']))
    for group, reference in zip(groups, expected['groups']):
        require(isinstance(group, dict))
        for name in ('offset', 'count'):
            require(type(group.get(name)) is int and group[name] == reference[name])
        for name in ('name', 'mode', 'unit', 'resource'):
            require(group.get(name) == reference[name])
    rate = value.get('rate')
    require(isinstance(rate, dict))
    for name in ('max_hz', 'expected_hz', 'watchdog_ms'):
        item = rate.get(name)
        require(number(item) and item > 0 and item == expected['rate'][name])
    require(rate['expected_hz'] <= rate['max_hz'])
    if mode == 'eef_pose':
        require(value.get('effector_ids') == expected['effector_ids'])
    else:
        names = value.get('joint_names')
        require(isinstance(names, list) and len(names) == 14
                and all(isinstance(name, str) and bool(name) for name in names))
        require(len(set(names)) == 14 and names == expected['joint_names'])
        limits = value.get('limits')
        require(isinstance(limits, dict))
        for name in ('lower', 'upper', 'max_velocity'):
            items = limits.get(name)
            require(isinstance(items, list) and len(items) == 14 and all(number(x) for x in items))
            require(items == expected['limits'][name])
        require(all(lo <= hi and velocity > 0 for lo, hi, velocity in zip(
            limits['lower'], limits['upper'], limits['max_velocity'])))


def validate(packet, lease, *, mode, now, model_version, calibration_version,
             frame, previous_seq=-1, previous_epoch=-1):
    if not isinstance(packet, dict) or set(packet) != FIELDS | {'mac'}:
        raise ValueError('invalid_command_fields')
    if not lease or any(packet[k] != lease[k] for k in ('boot_id', 'session_id')):
        raise ValueError('stale_session')
    body = {k: v for k, v in packet.items() if k != 'mac'}
    try:
        valid_mac = isinstance(packet['mac'], str) and hmac.compare_digest(
            sign(body, lease['secret']), packet['mac'])
    except (ValueError, TypeError, OverflowError):
        valid_mac = False
    if not valid_mac:
        raise ValueError('invalid_command_mac')
    if body['schema'] != SCHEMA or body['mode'] != mode or type(body['dof']) is not int or body['dof'] != 14:
        raise ValueError('invalid_control_interface')
    for name in ('seq', 'source_seq', 'mapping_epoch'):
        if type(body[name]) is not int or not 0 <= body[name] < 2**53:
            raise ValueError('invalid_' + name)
    if body['seq'] <= previous_seq:
        raise ValueError('stale_sequence')
    if body['mapping_epoch'] < previous_epoch:
        raise ValueError('stale_mapping_epoch')
    if (body['model_version'] != model_version or body['calibration_version'] != calibration_version
            or body['frame'] != frame):
        raise ValueError('control_calibration_mismatch')
    generated, until = body['generated_ns'], body['valid_until_ns']
    maximum = 300_000_000 if mode == 'eef_pose' else 100_000_000
    if type(generated) is not int or type(until) is not int or not 0 < until-generated <= maximum:
        raise ValueError('invalid_deadline')
    if not generated <= now < until:
        raise ValueError('command_expired')
    values = vector(body['values'], 14, 'values')
    if mode == 'eef_pose':
        for offset in (3, 10):
            if abs(sum(x*x for x in values[offset:offset+4])-1) > .002:
                raise ValueError('quaternion_not_unit')
    return {**body, 'values': values}


def envelope(lease, *, seq, source_seq, mapping_epoch, generated_ns,
             valid_until_ns, mode, values, model_version, calibration_version, frame):
    body = dict(schema=SCHEMA, boot_id=lease['boot_id'], session_id=lease['session_id'],
        seq=seq, source_seq=source_seq, mapping_epoch=mapping_epoch,
        generated_ns=generated_ns, valid_until_ns=valid_until_ns, mode=mode,
        dof=14, values=list(values), model_version=model_version,
        calibration_version=calibration_version, frame=frame)
    return {**body, 'mac': sign(body, lease['secret'])}
