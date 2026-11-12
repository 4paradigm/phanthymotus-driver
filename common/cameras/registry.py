"""Automatic, lazy backend loading and bounded discovery per backend."""
from __future__ import annotations

import importlib
import json
import multiprocessing as mp
from multiprocessing.connection import wait
import pkgutil
import time

from . import backends
from .interface import BackendUnavailable, validate_device


def backend_modules():
    return sorted(f'{backends.__name__}.{m.name}' for m in pkgutil.iter_modules(backends.__path__)
                  if not m.name.startswith('_'))


def _load(module_name):
    backend = importlib.import_module(module_name).BACKEND
    if backend.id != module_name.rsplit('.', 1)[-1] or not callable(backend.discover) or not callable(backend.open):
        raise BackendUnavailable(f'Invalid camera backend: {module_name}')
    return backend


def get_backend(backend_id):
    if not backend_id.isidentifier() or backend_id.startswith('_'):
        raise BackendUnavailable(f'Unknown camera backend: {backend_id}')
    try:
        return _load(f'{backends.__name__}.{backend_id}')
    except (ImportError, AttributeError) as exc:
        raise BackendUnavailable(f'Camera backend {backend_id} unavailable: {exc}') from exc


def _scan_backend(module_name, connection):
    from common import logsafe
    logsafe.install()
    try:
        backend = _load(module_name)
        devices = backend.discover()
        if not isinstance(devices, list):
            raise ValueError('Backend discovery must return a device list')
        for device in devices:
            validate_device(device, backend.id)
        json.dumps(devices)  # Never ship native SDK objects to the body driver.
        connection.send({'devices': devices, 'error': None})
    except Exception as exc:
        connection.send({'devices': [], 'error': str(exc)})
    finally:
        connection.close()


def discover_devices(timeout_s=8.):
    """Discover concurrently: one failed/hung SDK cannot hide other devices."""
    ctx = mp.get_context('spawn')
    handles, pending, devices, errors, failed = [], {}, [], [], []
    try:
        for module in backend_modules():
            receiver, sender = ctx.Pipe(duplex=False)
            proc = ctx.Process(target=_scan_backend, args=(module, sender), daemon=True)
            name = module.rsplit('.', 1)[-1]
            handles.append((proc, receiver))
            try:
                proc.start()
                pending[receiver] = (name, proc)
            except OSError as exc:
                errors.append(f'{name} discovery: {exc}')
                failed.append(name)
            finally:
                sender.close()
        deadline = time.monotonic() + timeout_s
        while pending and time.monotonic() < deadline:
            ready = wait(list(pending), timeout=max(0., deadline - time.monotonic()))
            for connection in ready:
                name, proc = pending.pop(connection)
                try:
                    result = connection.recv()
                except (EOFError, OSError):
                    proc.join(timeout=.05)
                    result = {'devices': [], 'error': f'worker exited ({proc.exitcode})'}
                if result['error']:
                    errors.append(f"{name} discovery: {result['error']}")
                    failed.append(name)
                else:
                    devices.extend(result['devices'])
        for name, proc in pending.values():
            errors.append(f'{name} discovery: timed out')
            failed.append(name)
    finally:
        for proc, connection in handles:
            if proc.pid is not None:
                proc.join(timeout=.1)
                if proc.is_alive():
                    proc.terminate()
                    proc.join(timeout=.2)
                if proc.is_alive():
                    proc.kill()
                    proc.join(timeout=.2)
            proc.close()
            connection.close()
    return {'devices': deduplicate_devices(devices),
            'errors': sorted(errors), 'failed_backends': sorted(failed)}


def deduplicate_devices(devices):
    # Prefer a native SDK owner to a generic UVC interface of the SAME known
    # physical device. Similar names/unknown identities are not evidence.
    devices = list({d['id']: d for d in devices}.values())
    native = {d['physical_id'] for d in devices
              if d['backend'] != 'uvc' and d.get('physical_id')}
    return sorted((d for d in devices if d['backend'] != 'uvc' or d.get('physical_id') not in native),
                  key=lambda d: d['id'])


def select_profiles(device, fps='auto', resolution='auto'):
    """Choose native streams; the current depth wire format requires 640x480."""
    profiles = device['profiles']
    choices = {'rgb': list(profiles.get('rgb', []))}
    if profiles.get('depth'):
        choices['depth'] = [p for p in profiles['depth']
                            if (p['width'], p['height']) == (640, 480)]
        if not choices['depth']:
            raise ValueError('Depth requires a native 640x480 stream')
    if resolution != 'auto':
        choices['rgb'] = [p for p in choices['rgb']
                          if f"{p['width']}x{p['height']}" == resolution]
    if any(not values for values in choices.values()):
        raise ValueError(f'Unsupported native resolution: {resolution}')
    rates = set.intersection(*({p['fps'] for p in values} for values in choices.values()))
    if fps != 'auto':
        rates &= {int(fps)}
    if not rates:
        raise ValueError('No supported common frame rate for the selected streams')
    rate = 15 if 15 in rates else min(rates, key=lambda n: (abs(n - 15), n))
    return {channel: min((p for p in values if p['fps'] == rate),
                         key=lambda p: (abs(p['width'] - 640) + abs(p['height'] - 480),
                                        p['format']))
            for channel, values in choices.items()}
