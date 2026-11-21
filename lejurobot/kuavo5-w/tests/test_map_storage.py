import base64
import copy
import importlib.util
import json
import os
from pathlib import Path
import sqlite3
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from controlled_spatial import ControlledSpatialPlugin, SpatialStore
from map_storage import MapArchive, migrate_database, SUFFIXES, CHUNK_SIZE
from test_controlled_spatial import Bundle, Client, wait

spec = importlib.util.spec_from_file_location('map_storage_rpc',
    Path(__file__).resolve().parents[1] / 'host/navigation_rpc.py')
rpc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rpc)


class MapStorageTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.remote = self.root/'chassis'
        self.remote.mkdir()
        self.name = '测试地图'
        for i, suffix in enumerate(SUFFIXES):
            p = self.remote/(self.name+suffix)
            p.write_bytes((b'pbstream-fixture' * (CHUNK_SIZE//15+2)) if suffix=='.pbstream'
                          else b'{}' if suffix=='.path' else b'fixture')
            os.utime(p, ns=(time.time_ns()+i*1000000,)*2)
        self.closed = threading.Event()
        self.calls = []
        def native(operation, **args):
            self.calls.append((operation, args))
            return rpc.map_archive_read(operation, args, self.remote)
        self.client = type('ReadOnlyClient', (), {'native': staticmethod(native)})()
        self.archive = MapArchive(self.client, self.root/'upper/maps/kuavo5w', self.closed)

    def current(self):
        p = self.archive.root/self.name/'current.json'
        return json.loads(p.read_text())

    def test_chunked_backup_preserves_all_files_and_reuses_identical_snapshot(self):
        self.archive.sync(self.name)
        current = self.current()
        version = Path(current['directory'])
        for suffix in SUFFIXES:
            self.assertEqual((version/(self.name+suffix)).read_bytes(),
                             (self.remote/(self.name+suffix)).read_bytes())
        self.assertGreater(len([c for c in self.calls if c[0]=='map_archive_chunk']), 4)
        self.calls.clear()
        self.archive.sync(self.name)
        self.assertEqual(self.current(), current)
        self.assertEqual([c[0] for c in self.calls], ['map_archive_manifest'])

    def test_corrupt_transfer_never_replaces_a_good_snapshot(self):
        self.archive.sync(self.name)
        previous = self.current()
        path = self.remote/(self.name+'.pgm')
        path.write_bytes(b'changed-map')
        os.utime(self.remote/(self.name+'.path'), ns=(time.time_ns()+10000000,)*2)
        original = self.client.native
        def corrupt(op, **args):
            result = original(op, **args)
            if op=='map_archive_chunk':
                result['data'] = base64.b64encode(b'x'*len(base64.b64decode(result['data']))).decode()
            return result
        self.client.native = corrupt
        with self.assertRaisesRegex(RuntimeError, 'digest_mismatch'):
            self.archive.sync(self.name)
        self.assertEqual(self.current(), previous)
        self.assertFalse(list((self.archive.root/self.name/'versions').glob('.incoming-*')))

    def test_remote_change_during_transfer_is_not_published(self):
        original = self.client.native
        calls = []
        def changed(op, **args):
            result = original(op, **args)
            if op=='map_archive_manifest':
                calls.append(True)
                if len(calls)==2:
                    result['files']['.pgm']['mtime_ns'] += 1
            return result
        self.client.native = changed
        with self.assertRaisesRegex(RuntimeError, 'changed_during_archive'):
            self.archive.sync(self.name)
        self.assertFalse((self.archive.root/self.name/'current.json').exists())

    def test_interrupted_backup_has_no_current_pointer_or_partial_snapshot(self):
        self.closed.set()
        with self.assertRaises(InterruptedError):
            self.archive.sync(self.name)
        self.assertFalse((self.archive.root/self.name/'current.json').exists())
        self.assertFalse(list((self.archive.root/self.name/'versions').iterdir()))

    def test_incomplete_symlink_and_unbounded_chunk_requests_are_rejected(self):
        for name in ('../map', '.', '..', '/tmp/map', ''):
            with self.assertRaises(ValueError):
                rpc.map_archive_read('map_archive_manifest', {'map_name':name}, self.remote)
        path = self.remote/(self.name+'.yaml')
        original = path.read_bytes()
        path.unlink();path.symlink_to(self.remote/(self.name+'.pgm'))
        with self.assertRaises(RuntimeError):
            self.archive.sync(self.name)
        path.unlink();path.write_bytes(original)
        os.utime(self.remote/(self.name+'.path'), ns=(time.time_ns()+10000000,)*2)
        for suffix, offset in (('.secret',0),('.pgm',-1),('.pgm',True),('.pgm',1)):
            with self.assertRaises(ValueError):
                rpc.map_archive_read('map_archive_chunk', {'map_name':self.name,
                    'suffix':suffix,'offset':offset}, self.remote)

    def test_background_backup_does_not_block_start_and_recovers_from_idle_check_failure(self):
        entered, release = threading.Event(), threading.Event()
        attempts = []
        def idle():
            attempts.append(True)
            if len(attempts) == 1:
                raise RuntimeError('transient state read failure')
            return True
        def maps():
            entered.set()
            release.wait(2)
            return [{'fileName':self.name+'.pgm'}]
        self.archive.idle = idle
        self.client.maps = maps
        try:
            self.archive.schedule()
            wait(lambda:self.archive.snapshot()['state']=='retrying')
            self.archive.schedule()
            self.assertTrue(entered.wait(2))
            self.assertFalse((self.archive.root/self.name/'current.json').exists())
            release.set()
            wait(lambda:self.archive.snapshot()['state']=='ready')
            self.assertTrue((self.archive.root/self.name/'current.json').is_file())
        finally:
            release.set()
            self.closed.set()
            self.archive.wake.set()
            if self.archive.thread:
                self.archive.thread.join(3)

    def test_database_migration_preserves_tags_pending_and_source_without_overwriting(self):
        source, target = self.root/'config/controlled_spatial.db', self.root/'data/kuavo5w/controlled_spatial.db'
        store = SpatialStore(source)
        identity = ('test','fingerprint')
        store.tag(identity, 'P1','tag',{'x':1,'y':2,'yaw':.3})
        pending = {'navigation':{'task_id':'old-task'},'uncertain':True}
        store.state('pending',pending)
        migrate_database(source, target)
        migrated = SpatialStore(target)
        self.assertEqual(migrated.tags(identity), store.tags(identity))
        self.assertEqual(migrated.state('pending'), pending)
        self.assertTrue(source.is_file())
        migrated.state('pending',{'new':True})
        migrate_database(source, target)
        self.assertEqual(migrated.state('pending'), {'new':True})
        migrated.close(); store.close()

    def test_production_default_migrates_legacy_database_into_standard_host_data(self):
        source = self.root/'config/controlled_spatial.db'
        store = SpatialStore(source)
        store.state('mapping', {'name':'in-progress','phase':'saving'})
        store.close()
        with patch.dict(os.environ, {'KUAVO_STATE_DIR':str(source.parent),
                                     'KUAVO_DATA_DIR':str(self.root/'data')}):
            bundle = Bundle()
            plugin = ControlledSpatialPlugin(bundle, client=Client())
            try:
                self.assertEqual(plugin.store.path,self.root/'data/kuavo5w/controlled_spatial.db')
                self.assertEqual(plugin.store.state('mapping')['phase'],'saving')
                self.assertEqual(plugin.archive.root,self.root/'data/maps/kuavo5w')
                self.assertFalse(plugin.archive.idle())
            finally:
                plugin.close()


class LoadEfficiencyTests(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.client=Client();self.bundle=Bundle()
        self.plugin=ControlledSpatialPlugin(self.bundle,{'db_path':self.folder.name+'/db'},self.client)
        self.bundle.spatial=self.plugin
        self.addCleanup(self.plugin.close)

    def load(self, name='factory'):
        before=len(self.bundle.tracker.events)
        result=self.plugin.dispatch('load_map', {'map_name':name})
        self.assertEqual(result['state'],'executing')
        wait(lambda:len(self.bundle.tracker.events)>before)
        return self.bundle.tracker.events[-1]

    def test_same_confirmed_map_skips_reloading_and_preserves_localization(self):
        self.assertEqual(self.load()[0],'completed')
        self.client.commands.clear()
        self.assertEqual(self.load()[0],'completed')
        self.assertFalse(self.client.commands)
        self.assertEqual(self.plugin.store.state('last_map_operation')['state'],'completed')

    def test_same_name_with_changed_content_or_missing_localization_forces_real_load(self):
        self.plugin.LOAD_LOCALIZATION_WAIT = .01
        self.load()
        for changed in ('identity','localization'):
            self.client.commands.clear()
            if changed=='identity':self.client.identity=('factory','changed-grid')
            else:self.client.feedback['state']['agvPosition']['positionInitialized']=False
            self.assertEqual(self.load()[0],'completed')
            self.assertIn(('ChangeCartoMap',{'mapName':'factory'}),self.client.commands)

    def test_redundant_mode_command_is_skipped_only_with_fresh_feedback(self):
        self.plugin._command('ChangeMode',{'mode':'AUTO'},threading.Event())
        self.assertFalse(self.client.commands)

    def test_operation_reports_per_stage_timing_without_fabricating_save_speed(self):
        status,result=self.load('other')
        self.assertEqual(status,'completed')
        timing=result['timing']
        self.assertEqual(timing['operation'],'load_map')
        self.assertGreaterEqual(timing['elapsed_s'],0)
        self.assertEqual([s['stage'] for s in timing['stages']],
            ['ChangeMode','ChangeCartoMap','ChangeRoadMap','ChangeMode'])
        self.assertTrue(all(s['elapsed_s']>=0 for s in timing['stages']))
