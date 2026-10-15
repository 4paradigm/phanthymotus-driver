"""Operator-selected return joints; capture reads only, never moves or resets."""
import json
import os
from pathlib import Path
import tempfile
import time

from agilex.piper.intent_motion import checked_state
from agilex.piper.user_motion import vector


def read_preset(directory, config, serial):
    path = Path(directory) / 'return-preset.json'
    if path.exists():
        try:
            record = json.loads(path.read_text())
            if (not serial or record['device_serial'] != serial
                    or record['can_interface'] != config.get('can_interface', 'can0')):
                raise ValueError('Saved position belongs to another arm connection; use Set position again.')
            joints = vector(record['joints_deg'], 6, 'saved position')
            return {**record, 'joints_deg': joints, 'source': 'saved_position'}
        except (OSError, KeyError, TypeError, ValueError) as exc:
            raise ValueError('Saved position is unavailable: ' + str(exc)) from exc
    value = config.get('intent_control', {}).get('preset_joints_deg')
    if value is None:
        raise ValueError('No return posture is configured. Use Set position while the arm is stationary.')
    return {'joints_deg': vector(value, 6, 'preset_joints_deg'), 'source': 'configuration'}


def write_preset(directory, record):
    """Atomic replace: invalid input or failed write leaves the old target intact."""
    vector(record['joints_deg'], 6, 'saved position')
    if not record.get('device_serial'):
        raise ValueError('USB-CAN identity is unavailable; no position was saved.')
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=directory, prefix='.return-preset-', delete=False) as stream:
            name = stream.name
            json.dump(record, stream, allow_nan=False, indent=2)
            stream.write('\n');stream.flush();os.fsync(stream.fileno())
        os.replace(name, directory / 'return-preset.json')
    finally:
        if name is not None and os.path.exists(name):
            os.unlink(name)


def capture_position(driver, *, interface, serial, clock=time.monotonic, sleep=time.sleep):
    if not serial:
        raise ValueError('USB-CAN identity is unavailable; no position was saved.')
    driver._check_cancelled()
    first = checked_state(driver, allow_teaching=True)
    deadline = clock() + .5
    while True:
        driver._check_cancelled()
        state = checked_state(driver, allow_teaching=True)
        if state.control_mode != first.control_mode or state.enabled != first.enabled:
            raise RuntimeError('Arm mode or motor state changed; no position was saved.')
        if max(abs(q-b) for q,b in zip(state.joints_deg, first.joints_deg)) > .2:
            raise RuntimeError('Keep the arm stationary for Set position; no position was saved.')
        if clock() >= deadline:
            driver._check_cancelled()
            return {'joints_deg': state.joints_deg, 'device_serial': serial,
                    'can_interface': interface, 'saved_at': time.time(),
                    'feedback_time': state.feedback_time}
        sleep(.05)
