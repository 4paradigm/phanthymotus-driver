"""Read status fields; mapping ST_DEVED is commissioned from this robot's feedback.

It is not inferred from request success and is not a vendor-documented enum map.
"""


def message_state(status):
    if not isinstance(status, dict) or status.get('result') != 'success' \
            or status.get('success') is not True or status.get('cmd') != 'status':
        raise ValueError('Not a successful, correlated teach-status response')
    message = status.get('message')
    if not isinstance(message, str) or len(message) > 300:
        raise ValueError('Missing or invalid teach-state message')
    fields = {}
    for part in message.split(';'):
        if '=' not in part:
            raise ValueError('Unknown teach-state message format')
        key, value = part.split('=', 1)
        if key not in ('state', 'sub', 'rec') or key in fields:
            raise ValueError('Unknown or duplicate teach-state message field')
        fields[key] = value
    if set(fields) != {'state', 'sub', 'rec'}:
        raise ValueError('Incomplete teach-state message')
    if fields['sub'] != '' or fields['rec'] != '0':
        raise ValueError('Teach substate/recording is active or unrecognized')
    return fields['state']


def inactive_value(status, config):
    pointer = config.get('teach_state_pointer')
    expected = config.get('teach_inactive_value')
    if pointer == '/message#state':
        if expected != 'ST_DEVED' or config.get('expected_working_mode') != 'developer_mode':
            raise ValueError('The observed status mapping requires developer_mode and ST_DEVED')
        if message_state(status) != 'ST_DEVED':
            raise ValueError('Robot is not in the commissioned developer state; teach may be active')
        return
    if not isinstance(pointer, str) or not pointer.startswith('/') or len(pointer) < 2:
        raise ValueError('Teach status field has not been commissioned')
    keys = [s.replace('~1', '/').replace('~0', '~') for s in pointer[1:].split('/')]
    if keys[-1] in ('result', 'success', 'cmd', 'message'):
        raise ValueError('A command receipt is not a teach-state field')
    if not (expected is False or type(expected) is int and expected == 0
            or type(expected) is str and expected.lower() in
            ('idle', 'inactive', 'disabled', 'off', 'none', 'false')):
        raise ValueError('Unverified inactive-state value')
    value = status
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            raise ValueError('Firmware has no commissioned teach-state field: ' + pointer)
        value = value[key]
    if type(value) is not type(expected) or value != expected:
        raise ValueError('Hand Guiding/teach is not confirmed inactive')
