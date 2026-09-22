"""Strict, authenticated domain-42 control/2 envelopes (no hardware access)."""
import hmac
from motion_stream import sign, vector

SCHEMA = 'motus.control/2'
FIELDS = frozenset(('schema', 'boot_id', 'session_id', 'seq', 'source_seq',
    'mapping_epoch', 'generated_ns', 'valid_until_ns', 'mode', 'dof', 'values',
    'model_version', 'calibration_version', 'frame'))


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
