"""Read-only TRON2 values. Receipt age is not measurement age."""
import math

STATUS_FIELDS = ('imu', 'camera', 'motor', 'sw_version', 'status',
                 'tele_operation', 'working_mode', 'working_mode_valid',
                 'collision_enable', 'collision_enable_valid',
                 'compliant_enable', 'compliant_enable_valid')
BATTERY_FIELDS = ('soc', 'voltage', 'current', 'power', 'temperature', 'cycle')
BATTERY_UNITS = {'soc': '%', 'voltage': 'vendor_raw_unverified',
                 'current': 'vendor_raw_unverified', 'power': 'W',
                 'temperature': 'vendor_raw_unverified', 'cycle': 'count'}


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


def battery_number(raw):
    """Parse a finite vendor number without guessing its physical unit."""
    if isinstance(raw, bool) or not isinstance(raw, (str, int, float)):
        return None
    try:
        value = float(raw)
    except (ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def battery_state(data):
    soc = battery_number(data.get('battery'))
    voltage = battery_number(data.get('bms_voltage'))
    # Invalid or unverified current must not be presented as a real zero.
    current = battery_number(data.get('bms_current')) \
        if data.get('bms_current_valid') is True else None
    # User-facing current is magnitude only; preserve missing/invalid as null.
    current = abs(current) if current is not None else None
    soc_valid = soc is not None and 0 <= soc <= 100
    voltage_valid = voltage is not None and voltage >= 0
    # Keep fixed channel order; missing/invalid channels must not shift indices.
    temperatures = [battery_number(data.get('bms_temp' + str(i))) for i in range(1, 6)]
    cycle = battery_number(data.get('bms_charging_cycle'))
    cycle_valid = cycle is not None and cycle >= 0 and cycle.is_integer()
    updated = battery_number(data.get('bms_current_updated_at'))
    updated = updated if updated is not None and updated >= 0 else None
    return {'schema_version': 2,
            'soc': soc if soc_valid else None,
            'voltage': voltage if voltage_valid else None,
            'current': current,
            # Raw BMS scales are not documented. Do not invent watts from them.
            'power': None, 'power_status': 'unit_conversion_unverified',
            'temperature': temperatures,
            'cycle': int(cycle) if cycle_valid else None,
            'units': dict(BATTERY_UNITS),
            'validity': {'soc': soc_valid, 'voltage': voltage_valid,
                         'current': current is not None, 'power': False,
                         'temperature': [t is not None for t in temperatures],
                         'cycle': cycle_valid},
            # Opaque vendor marker: do not subtract it from the host clock.
            'current_updated_at_raw': updated}


def battery_stream(sample):
    """Flat measurements plus diagnostics, including when offline or stale."""
    data = sample['data'] if sample['available'] else battery_state({})
    return {**data, 'status': {key: sample[key] for key in (
        'connected', 'available', 'sample_status', 'received_at',
        'reception_age_seconds', 'measurement_time_verified')}}
