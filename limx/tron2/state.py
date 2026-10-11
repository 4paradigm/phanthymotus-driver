"""Read-only TRON2 values. Receipt age is not measurement age."""
import math

STATUS_FIELDS = ('imu', 'camera', 'motor', 'sw_version', 'status',
                 'tele_operation', 'working_mode', 'working_mode_valid',
                 'collision_enable', 'collision_enable_valid',
                 'compliant_enable', 'compliant_enable_valid')
BATTERY_FIELDS = ('battery', 'bms_voltage', 'bms_current', 'bms_current_valid',
                  'bms_current_updated_at', 'bms_charging_cycle',
                  'bms_temp1', 'bms_temp2', 'bms_temp3', 'bms_temp4', 'bms_temp5')


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


def battery_state(data):
    result = {'raw': select_fields(data, BATTERY_FIELDS)}
    raw = data.get('battery')
    try:
        percent = float(raw) if not isinstance(raw, bool) else None
    except (ValueError, TypeError):
        percent = None
    result['battery_percent'] = percent if percent is not None and math.isfinite(percent) \
        and 0 <= percent <= 100 else None
    # SDK does not specify BMS voltage/current/temperature scaling here.
    result['bms_units'] = 'vendor_raw_unverified'
    return result
