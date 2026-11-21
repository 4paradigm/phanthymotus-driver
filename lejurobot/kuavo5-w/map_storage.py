"""Host-persistent, verified snapshots of factory map files (no map mutations)."""
import base64
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import tempfile
import threading
import time

SUFFIXES = ('.pgm', '.yaml', '.pbstream', '.path')
CHUNK_SIZE = 1024 * 1024
MAX_FILE_SIZE = 512 * 1024 * 1024


def safe_name(value):
    if (not isinstance(value, str) or not value.strip() or len(value) > 80
            or value in ('.', '..') or any(c in value for c in '/\\')
            or any(ord(c) < 32 for c in value)):
        raise ValueError('invalid_map_name')
    return value


def migrate_database(source, destination):
    """One-time SQLite backup, atomically published without overwriting a DB."""
    source, destination = Path(source), Path(destination)
    if destination.exists() or not source.exists() or source == destination:
        return
    if source.is_symlink() or not source.is_file():
        raise RuntimeError('invalid_legacy_spatial_database')
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.spatial-migration-', dir=destination.parent)
    os.close(fd)
    temporary = Path(name)
    try:
        old = sqlite3.connect(source.resolve().as_uri()+'?mode=ro', uri=True)
        new = sqlite3.connect(temporary)
        try:
            old.backup(new)
            if new.execute('PRAGMA quick_check').fetchone()[0] != 'ok':
                raise RuntimeError('spatial_database_migration_failed')
        finally:
            new.close()
            old.close()
        with temporary.open('rb') as stream:
            os.fsync(stream.fileno())
        try:
            os.link(temporary, destination)  # Exclusive creation; keep source for rollback.
        except FileExistsError:
            pass
    finally:
        temporary.unlink(missing_ok=True)


class MapArchive:
    def __init__(self, client, root, closed, idle=lambda: True):
        self.client, self.root, self.closed, self.idle = client, Path(root), closed, idle
        self.wake = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.status = {'state': 'idle', 'directory': str(self.root), 'role': 'upper_board_backup'}

    def snapshot(self):
        with self.lock:
            return dict(self.status)

    def _status(self, **values):
        with self.lock:
            self.status = {**self.status, **values}

    def schedule(self):
        self.wake.set()
        with self.lock:
            if self.thread is None:
                self.thread = threading.Thread(target=self._run, daemon=True, name='map-archive')
                self.thread.start()

    def _run(self):
        while not self.closed.is_set():
            if self.wake.wait(60):
                self.wake.clear()
            if self.closed.is_set():
                return
            try:
                if not self.idle():
                    continue
                maps = self.client.maps()
                names = sorted({safe_name(Path(m['fileName']).stem) for m in maps})
                self._status(state='syncing', error=None)
                failures = {}
                for name in names:
                    if self.closed.is_set() or not self.idle():
                        break
                    try:
                        self.sync(name)
                    except Exception as exc:
                        failures[name] = type(exc).__name__
                else:
                    self._status(state='retrying' if failures else 'ready', maps=names,
                                 failures=failures, checked_at=time.time(), error=None)
            except Exception as exc:
                self._status(state='retrying', error=type(exc).__name__,
                             message='Map backup will retry in background; chassis originals are unchanged.')

    @staticmethod
    def validate(manifest, name):
        if manifest.get('map_name') != name or set(manifest.get('files', {})) != set(SUFFIXES):
            raise RuntimeError('invalid_map_archive_manifest')
        for item in manifest['files'].values():
            size, digest = item.get('size'), item.get('sha256')
            if (type(size) is not int or not 0 < size <= MAX_FILE_SIZE
                    or not isinstance(digest, str) or len(digest) != 64
                    or any(c not in '0123456789abcdef' for c in digest)):
                raise RuntimeError('invalid_map_archive_file')
        return hashlib.sha256(json.dumps(manifest, sort_keys=True).encode()).hexdigest()

    def archived(self, name):
        directory = self.root / safe_name(name)
        pointer = json.loads((directory / 'current.json').read_text())
        version = pointer['version']
        if (pointer['map_name'] != name or not isinstance(version, str) or len(version) != 64
                or any(c not in '0123456789abcdef' for c in version)):
            raise ValueError('invalid_map_archive_pointer')
        snapshot = directory / 'versions' / version
        manifest = json.loads((snapshot / 'manifest.json').read_text())
        if self.validate(manifest, name) != version:
            raise ValueError('invalid_map_archive_manifest')
        return snapshot, manifest

    def stage_upload(self, name, token, cancelled, deadline):
        directory, manifest = self.archived(name)
        identity = {'map_name': name, 'token': token}
        self.client.native('map_upload_begin', **identity)
        for suffix, item in manifest['files'].items():
            path = directory / (name + suffix)
            if path.is_symlink() or not path.is_file() or path.stat().st_size != item['size']:
                raise RuntimeError('invalid_upload_source')
            with path.open('rb') as stream:
                for offset in range(0, item['size'], CHUNK_SIZE):
                    if cancelled.is_set() or time.monotonic() >= deadline:
                        raise InterruptedError('map_upload_interrupted')
                    data = stream.read(CHUNK_SIZE)
                    if len(data) != min(CHUNK_SIZE, item['size'] - offset):
                        raise RuntimeError('invalid_upload_source')
                    self.client.native('map_upload_chunk', **identity, suffix=suffix,
                                       offset=offset, data=base64.b64encode(data).decode('ascii'))
        return manifest

    def sync(self, name):
        safe_name(name)
        manifest = self.client.native('map_archive_manifest', map_name=name)
        version = self.validate(manifest, name)
        directory = self.root / name
        versions = directory / 'versions'
        versions.mkdir(parents=True, exist_ok=True)
        final = versions / version
        if not final.exists():
            temporary = Path(tempfile.mkdtemp(prefix='.incoming-', dir=versions))
            try:
                for suffix in SUFFIXES:
                    item = manifest['files'][suffix]
                    digest = hashlib.sha256()
                    with (temporary / (name+suffix)).open('wb') as stream:
                        for offset in range(0, item['size'], CHUNK_SIZE):
                            if self.closed.is_set() or not self.idle():
                                raise InterruptedError('map_archive_interrupted')
                            result = self.client.native('map_archive_chunk', map_name=name, suffix=suffix,
                                offset=offset, expected=item)
                            data = base64.b64decode(result['data'], validate=True)
                            if len(data) != min(CHUNK_SIZE, item['size']-offset):
                                raise RuntimeError('map_archive_chunk_size_mismatch')
                            stream.write(data)
                            digest.update(data)
                        stream.flush()
                        os.fsync(stream.fileno())
                    if digest.hexdigest() != item['sha256']:
                        raise RuntimeError('map_archive_digest_mismatch')
                if self.client.native('map_archive_manifest', map_name=name) != manifest:
                    raise RuntimeError('map_changed_during_archive')
                (temporary / 'manifest.json').write_text(json.dumps(manifest, indent=2))
                temporary.rename(final)
            finally:
                if temporary.exists():
                    shutil.rmtree(temporary)
        # Verify even existing snapshots; never advertise a corrupted backup.
        for suffix, item in manifest['files'].items():
            path = final / (name+suffix)
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(CHUNK_SIZE), b''):
                    digest.update(chunk)
            if path.stat().st_size != item['size'] or digest.hexdigest() != item['sha256']:
                raise RuntimeError('local_map_archive_corrupted')
        pointer = {'version': version, 'map_name': name, 'directory': str(final)}
        fd, path = tempfile.mkstemp(prefix='.current-', dir=directory)
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(pointer, stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(path, directory / 'current.json')
        finally:
            Path(path).unlink(missing_ok=True)
        self._status(current_map=name, current_directory=str(final), last_synced_at=time.time())
