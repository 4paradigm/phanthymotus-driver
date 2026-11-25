"""Map transfers verify identity before installing or deleting files."""
import base64
import hashlib
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from host.navigation_rpc import map_archive_read, map_archive_write


def test_map_transfer_checks_identity_and_removes_deleted_files(tmp_path):
    name = 'test1'
    upload = {'map_name': name, 'token': 'a' * 32}
    delete = {'map_name': name, 'token': 'b' * 32}
    files = {'.path': b'{}', '.yaml': b'grid', '.pbstream': b'state', '.pgm': b'pixels'}
    manifest = {'map_name': name, 'files': {
        suffix: {'size': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
        for suffix, data in files.items()}}

    def run(operation, args):
        return map_archive_write(operation, args, tmp_path, lambda name: None)

    run('map_upload_begin', upload)
    for suffix, data in files.items():
        run('map_upload_chunk', {**upload, 'suffix': suffix, 'offset': 0,
                                 'data': base64.b64encode(data).decode()})
    bad = {**manifest, 'files': {**manifest['files'], '.pgm': {
        'size': 6, 'sha256': '0' * 64}}}
    with pytest.raises(ValueError, match='map_archive_digest_mismatch'):
        run('map_upload_commit', {**upload, 'manifest': bad})
    assert not (tmp_path / 'test1.pgm').exists()

    run('map_upload_commit', {**upload, 'manifest': manifest})
    archived = map_archive_read('map_archive_manifest', upload, tmp_path)

    def active(_):
        raise ValueError('cannot_modify_active_map')

    with pytest.raises(ValueError, match='cannot_modify_active_map'):
        map_archive_write('map_delete', {**delete, 'manifest': archived}, tmp_path, active)
    assert (tmp_path / 'test1.pgm').exists()
    with pytest.raises(ValueError, match='map_name_already_exists'):
        run('map_upload_begin', {**upload, 'token': 'c' * 32})
    run('map_delete', {**delete, 'manifest': archived})
    quarantine = tmp_path / '.motus-map-transfers' / ('deleted-' + delete['token'])
    for suffix in files:
        assert not (tmp_path / (name + suffix)).exists()
    assert not quarantine.exists()
