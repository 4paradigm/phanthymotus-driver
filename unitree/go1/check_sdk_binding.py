"""Build-time check of the real pybind property, without robot communication."""
import json

FIELDS = ('TotalCount', 'SendCount', 'RecvCount', 'SendError',
          'FlagError', 'RecvCRCError', 'RecvLoseError')


def check_binding(sdk):
    # Ephemeral local port, loopback destination; do not Send, Recv, or start loops.
    udp = sdk.UDP(0xEE, 0, '127.0.0.1', 9)
    stats = udp.udpState  # Must convert the actual C++ instance into Python.
    if not isinstance(stats, sdk.UDPState):
        raise TypeError('udpState did not return a registered UDPState instance')
    counts = {name: getattr(stats, name) for name in FIELDS}
    if any(type(value) is not int or value != 0 for value in counts.values()):
        raise ValueError('Expected fresh integer UDP counters, got: ' + repr(counts))
    return counts


if __name__ == '__main__':
    import robot_interface
    print(json.dumps({'binding_property_read': 'passed',
                      'counts': check_binding(robot_interface)}))
