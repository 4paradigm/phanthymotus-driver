"""Read-only TRON2 values. Receipt age is not measurement age."""
import math

STATUS_FIELDS = ('imu', 'camera', 'motor', 'sw_version', 'status',
                 'tele_operation', 'working_mode', 'working_mode_valid',
                 'collision_enable', 'collision_enable_valid',
                 'compliant_enable', 'compliant_enable_valid')


def vector(value, count, field):
    if not isinstance(value, list) or len(value) != count or any(
            isinstance(x, bool) or not isinstance(x, (int, float))
            or not math.isfinite(x) for x in value):
        raise ValueError('Invalid joint array: ' + field)
    return list(value)


def joint_state(data):
    if not isinstance(data, dict) or data.get('result') != 'success':
        raise ValueError('Joint-state request did not succeed')
    q = data.get('q')
    if not isinstance(q, list) or not 1 <= len(q) <= 64:
        raise ValueError('Expected 1 to 64 joint positions')
    q = vector(q, len(q), 'q')
    result = {'joint_count': len(q), 'q': q, 'position_unit': 'rad',
              'positions_deg': [math.degrees(x) for x in q]}
    for field in ('dq', 'tau'):
        values = data.get(field)
        result[field] = [] if values is None or values == [] else vector(values, len(q), field)
    names = data.get('names')
    indexed = names is None or names == []
    if not indexed and (not isinstance(names, list) or len(names) != len(q)
            or any(not isinstance(n, str) or not n.strip() or len(n) > 128 for n in names)
            or len(set(names)) != len(names)):
        raise ValueError('Invalid joint names')
    result.update(names=[] if indexed else list(names),
                  labels=['joint_index_' + str(i) for i in range(len(q))]
                         if indexed else list(names), labels_are_indices=indexed,
                  name_source='index' if indexed else 'robot')
    stamp = data.get('timestamp')
    if stamp is not None and (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
            or not math.isfinite(stamp) or stamp < 0):
        raise ValueError('Invalid opaque robot timestamp')
    result['robot_timestamp'] = stamp
    # The SDK does not define this timestamp's clock/unit. Do not fabricate age.
    result['measurement_time_verified'] = False
    return result


def select_fields(data, fields):
    result = {}
    for key in fields:
        value = data.get(key)
        if value is None or isinstance(value, (bool, str, int, float)):
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError('Invalid status number: ' + key)
            if isinstance(value, str) and len(value) > 1024:
                raise ValueError('Status value is too long: ' + key)
            result[key] = value
        else:
            raise ValueError('Invalid status value: ' + key)
    return result


def configuration(accid):
    """Classify the reported identity, never a configured ROBOT_TYPE or pose.

    DACH/WF were observed on hardware; SF is documented but not field verified.
    This is a process-start observation, not physical assembly verification.
    """
    prefix = accid.split('_', 1)[0] if isinstance(accid, str) else ''
    modes = {'DACH': 'upper_limb', 'WF': 'wheel_foot', 'SF': 'sole_foot'}
    return {'state': 'identified' if prefix in modes else 'unknown',
            'mode': modes.get(prefix, 'unknown'), 'accid': accid,
            'source': 'notify_robot_info.accid',
            'field_verified': prefix in ('DACH', 'WF'),
            'fixed_for_process': True, 'enforces_card_restrictions': False}


def _bms_integer(value, nonnegative=False):
    if isinstance(value,bool):return None
    try:
        if isinstance(value,str) and len(value)>32:return None
        number=float(value)
        if not math.isfinite(number) or not number.is_integer() or (nonnegative and number<0):return None
        return int(number)
    except (TypeError,ValueError,OverflowError):return None

def battery_state(data):
    """SDK notify_robot_info.battery; firmware also returns numeric strings."""
    value = data.get('battery')
    if isinstance(value, str):
        if not value.strip() or len(value) > 32:
            raise ValueError('Invalid battery percentage')
        try:
            value = float(value)
        except ValueError:
            raise ValueError('Invalid battery percentage') from None
    if isinstance(value, bool) or not isinstance(value, (int, float)) \
            or not 0 <= value <= 100 or not math.isfinite(value):
        raise ValueError('Invalid battery percentage')
    return {'percentage': value, 'current': _bms_integer(data.get('bms_current')) if data.get('bms_current_valid') is True else None,
            'cycle': _bms_integer(data.get('bms_charging_cycle'), nonnegative=True), 'unit': '%',
            'source': 'notify_robot_info.data.battery'}


def battery_stream(sample):
    """Same measurement keys as Go2; unavailable values are null, never invented zero."""
    data=sample.get('data') if sample.get('available') else None
    if not data:return {'soc':None,'current':None,'cycle':None}
    soc=data['percentage']
    return {'soc':int(soc) if float(soc).is_integer() else soc,'current':data.get('current'),'cycle':data.get('cycle')}
