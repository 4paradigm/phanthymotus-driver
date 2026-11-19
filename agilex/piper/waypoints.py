"""Versioned, connection-bound waypoint storage; never sends robot commands."""
from contextlib import contextmanager, nullcontext
from copy import deepcopy
import fcntl
import json
import math
import os
from pathlib import Path
import tempfile
import time
import unicodedata

from agilex.piper.user_motion import vector


def waypoint_name(value):
    if not isinstance(value, str):
        raise ValueError('Waypoint name must be text.')
    value = unicodedata.normalize('NFC', value.strip())
    if not value or len(value) > 80 or any(unicodedata.category(c).startswith('C') for c in value):
        raise ValueError('Waypoint name must contain 1–80 printable characters.')
    return value


def validate_record(record):
    if not isinstance(record, dict):
        raise ValueError('Invalid waypoint record.')
    result = deepcopy(record)
    result['joints_deg'] = list(vector(record['joints_deg'], 6, 'waypoint joints'))
    for key in ('device_serial', 'can_interface'):
        if not isinstance(record.get(key), str) or not record[key].strip():
            raise ValueError('Waypoint connection identity is unavailable.')
    for key in ('saved_at', 'feedback_time'):
        value = record.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('Invalid waypoint timestamp.')
    grip = record.get('gripper')
    if grip is not None:
        if not isinstance(grip, dict):
            raise ValueError('Invalid gripper observation.')
        vector([grip['width_mm'], grip['effort_nm'], grip['feedback_time']], 3, 'gripper observation')
        if type(grip['enabled']) is not bool or type(grip['error_code']) is not int:
            raise ValueError('Invalid gripper observation.')
    return result


class WaypointStore:
    MAX_POINTS = 256

    def __init__(self, directory):
        self.directory = Path(directory)
        self.path = self.directory / 'waypoints.json'

    def _load(self):
        try:
            with self.path.open(encoding='utf-8') as stream:
                data = json.load(stream)
        except FileNotFoundError:
            return {}
        except (ValueError, OSError) as exc:
            raise ValueError('Waypoint library is unreadable; preserve it and repair it before retrying.') from exc
        try:
            if type(data['version']) is not int or data['version'] != 1 or not isinstance(data['points'], dict):
                raise ValueError('Unsupported waypoint library version.')
            if len(data['points']) > self.MAX_POINTS:
                raise ValueError('Too many waypoints.')
            result = {}
            for name, record in data['points'].items():
                if waypoint_name(name) != name:
                    raise ValueError('Invalid stored waypoint name.')
                result[name] = validate_record(record)
            return result
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('Waypoint library is invalid; no data was changed.') from exc

    @contextmanager
    def _edit(self, check_cancelled=lambda: None):
        self.directory.mkdir(parents=True, exist_ok=True)
        with (self.directory / 'waypoints.lock').open('a') as lock:
            deadline = time.monotonic() + 2
            while True:
                check_cancelled()
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    if time.monotonic() >= deadline:
                        raise RuntimeError('Waypoint storage is busy; retry after the other writer finishes.')
                    time.sleep(.05)
            yield

    def _save(self, points, commit_guard=nullcontext):
        name = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.directory,
                                             prefix='.waypoints-', delete=False) as stream:
                name = stream.name
                json.dump({'version': 1, 'points': points}, stream, ensure_ascii=False, allow_nan=False, indent=2)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            # Slow writes and lock acquisition never hold the runtime lifecycle
            # lock. Only the final cancellation/state check and replace do.
            with commit_guard():
                os.replace(name, self.path)
        finally:
            if name is not None and os.path.exists(name):
                os.unlink(name)

    def list(self):
        return [{'name': name, **record} for name, record in sorted(self._load().items())]

    def read(self, name):
        name = waypoint_name(name)
        points = self._load()
        if name not in points:
            raise ValueError('Waypoint does not exist: ' + name)
        return {'name': name, **points[name]}

    def capture(self, name, record, *, overwrite=False, check_cancelled=lambda: None, commit_guard=nullcontext):
        name, record = waypoint_name(name), validate_record(record)
        with self._edit(check_cancelled):
            points = self._load()
            if name in points and not overwrite:
                raise ValueError('Waypoint already exists; choose another name or explicitly enable overwrite.')
            if name not in points and len(points) >= self.MAX_POINTS:
                raise ValueError('Waypoint library is full; delete a point first.')
            points[name] = record
            self._save(points, commit_guard)
        return {'name': name, **record}

    def delete(self, name):
        name = waypoint_name(name)
        with self._edit():
            points = self._load()
            if name not in points:
                raise ValueError('Waypoint does not exist: ' + name)
            del points[name]
            self._save(points)
        return {'deleted': True, 'name': name}
