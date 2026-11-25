"""Orphan map records can be retired without claiming save/stop or moving."""
import copy
import importlib.util
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from test_controlled_spatial import Bundle, Client, wait
from controlled_spatial import ControlledSpatialPlugin

spec = importlib.util.spec_from_file_location('mapping_recovery_rpc',
    Path(__file__).resolve().parents[1] / 'host/navigation_rpc.py')
rpc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rpc)


class ReconciliationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.bundle, self.client = Bundle(), Client()
        self.plugin = ControlledSpatialPlugin(self.bundle, {'db_path': self.temp.name+'/state.db'}, self.client)
        self.bundle.spatial = self.plugin
        self.addCleanup(self.plugin.close)
        self.mapping = {'name': 'failed', 'token': 'mapping-test', 'phase': 'saving'}
        self.transition = {'operation': 'stop_mapping', 'stage': 'ChangeCartoMap',
                           'stage_started_at': time.time()-3600,
                           'chassis_boot_id': '00000000-0000-0000-0000-000000000002'}
        self.plugin.store.state('mapping', self.mapping)
        self.plugin._map_pending = copy.deepcopy(self.transition)
        self.plugin._persist()
        self.plugin._restored = True
        self.pending = self.plugin.store.state('pending')
        self.plugin.store.tag(('failed', 'mapping-test'), 'P0', '', {'x': 0, 'y': 0, 'yaw': 0})

    def call(self, **args):
        return self.plugin.dispatch('reconcile_mapping', {'map_name': 'failed', 'confirm_abandon': True, **args})

    def assert_kept(self):
        self.assertEqual(self.plugin.store.state('mapping'), self.mapping)
        self.assertEqual(self.plugin.store.state('pending'), self.pending)
        self.assertEqual(self.plugin._map_pending, self.transition)
        self.assertIsNone(self.plugin.store.state('last_mapping_reconciliation'))

    def test_rebooted_inactive_session_is_retired_atomically_with_audit_and_tags(self):
        result = self.call()
        self.assertEqual(result['state'], 'completed')
        self.assertTrue(result['files_and_tags_preserved'])
        self.assertFalse(result['motion_command_sent'])
        self.assertIsNone(self.plugin.store.state('mapping'))
        self.assertIsNone(self.plugin._map_pending)
        self.assertEqual(self.plugin.store.state('pending'), dict(self.pending, map_transition=None))
        evidence = self.plugin.store.state('last_mapping_reconciliation')
        self.assertEqual(evidence['previous_mapping'], self.mapping)
        self.assertEqual(evidence['previous_transition'], self.transition)
        self.assertEqual(len(self.plugin.store.tags(('failed', 'mapping-test'))), 1)
        self.assertEqual([c[0] for c in self.client.calls], ['map_runtime_status']*2)
        self.assertFalse(self.client.commands)
        self.assertFalse(self.bundle.pauses)
        self.assertTrue(self.plugin.base_preparation_required)

    def test_confirmation_and_exact_session_name_are_required(self):
        for args in ({'confirm_abandon': False}, {'map_name': 'different'}):
            self.assertEqual(self.call(**args)['state'], 'error')
            self.assert_kept()
        self.assertFalse(self.client.calls)

    def test_container_restart_in_same_chassis_boot_does_not_prove_save_finished(self):
        self.transition['chassis_boot_id'] = '00000000-0000-0000-0000-000000000001'
        self.plugin._map_pending = copy.deepcopy(self.transition)
        self.plugin._persist()
        self.pending = self.plugin.store.state('pending')
        self.assertIn('may_still_be_running', self.call()['error'])
        self.assert_kept()

    def test_legacy_record_requires_later_clock_consistent_chassis_boot(self):
        self.transition.pop('chassis_boot_id')
        self.plugin._map_pending = copy.deepcopy(self.transition)
        self.plugin._persist()
        result = self.call()
        self.assertEqual(result['state'], 'completed')
        self.assertEqual(result['basis'], 'legacy_command_predates_chassis_boot')

    def test_legacy_recent_command_or_clock_skew_cannot_be_abandoned(self):
        self.transition.pop('chassis_boot_id')
        self.transition['stage_started_at'] = time.time()
        self.plugin._map_pending = copy.deepcopy(self.transition)
        self.plugin._persist()
        self.pending = self.plugin.store.state('pending')
        self.assertIn('confirmed_later_chassis_boot', self.call()['error'])
        self.assert_kept()
        self.transition['stage_started_at'] -= 3600
        self.plugin._map_pending = copy.deepcopy(self.transition)
        self.plugin._persist()
        self.pending = self.plugin.store.state('pending')
        now = time.time()
        self.client.native = lambda *a, **kw: {'boot_id': '00000000-0000-0000-0000-000000000001',
                                              'uptime_s': 60, 'observed_at': now}
        with patch('controlled_spatial.time.time', return_value=now+600):
            self.assertEqual(self.call()['state'], 'error')
        self.assert_kept()

    def test_live_mapping_driving_and_native_tasks_are_rejected(self):
        variants = [('agvState', 'isMapping', True), ('state', 'driving', True),
                    ('state', 'edgeStates', [{}]), ('state', 'nodeStates', [{}]),
                    ('state', 'actionStates', [{}])]
        for group, key, value in variants:
            with self.subTest(key=key):
                original = copy.deepcopy(self.client.feedback)
                self.client.feedback[group][key] = value
                self.assertEqual(self.call()['error'], 'chassis_mapping_or_task_still_active')
                self.assert_kept()
                self.client.feedback = original

    def test_local_worker_and_other_control_owners_block_reconciliation(self):
        for key, value in (('_task', {'task_id': 'active'}), ('_uncertain', True),
                           ('_preparation', {'delivery_uncertain': True}), ('_keyboard_pending', True)):
            old = getattr(self.plugin, key)
            setattr(self.plugin, key, value)
            self.assertEqual(self.call()['state'], 'error')
            setattr(self.plugin, key, old)
            self.assert_kept()
        release = threading.Event()
        worker = threading.Thread(target=lambda: release.wait(2), name='kuavo-map-operation')
        self.plugin._workers.add(worker)
        worker.start()
        try:
            self.assertEqual(self.call()['state'], 'error')
            self.assert_kept()
        finally:
            release.set(); worker.join()
        self.assertFalse(self.client.calls)

    def test_stop_or_reconnect_during_reads_keeps_records(self):
        native = self.client.native
        def changed(operation, **args):
            self.bundle._stop_epoch += 1
            return native(operation, **args)
        self.client.native = changed
        self.assertEqual(self.call()['state'], 'error')
        self.assert_kept()

    def test_chassis_reboot_during_read_keeps_records(self):
        count = [0]
        def reboot(operation, **args):
            count[0] += 1
            return {'boot_id': '00000000-0000-0000-0000-%012d' % (count[0]+3)}
        self.client.native = reboot
        self.assertIn('rebooted_during', self.call()['error'])
        self.assert_kept()

    def test_changed_selected_map_keeps_records(self):
        maps = iter([{'mapId': 'factory'}, {'mapId': 'other'}])
        self.client.current_map = lambda: next(maps)
        self.assertIn('map_changed_during', self.call()['error'])
        self.assert_kept()

    def test_external_database_update_is_not_overwritten(self):
        native = self.client.native
        def changed(operation, **args):
            self.plugin.store.state('pending', dict(self.pending, uncertain=True))
            return native(operation, **args)
        self.client.native = changed
        self.assertIn('record_changed', self.call()['error'])
        self.assertTrue(self.plugin.store.state('pending')['uncertain'])
        self.assertEqual(self.plugin.store.state('mapping'), self.mapping)
        self.assertIsNone(self.plugin.store.state('last_mapping_reconciliation'))

    def test_info_does_not_claim_orphaned_worker_is_still_optimizing(self):
        result = self.plugin.dispatch('info', {})
        self.assertEqual(result['state'], 'recovery_required')
        self.assertEqual(result['recovery']['map_name'], 'failed')
        self.assertIn('no longer running', result['message'])
        self.assert_kept()

    def test_chassis_reboot_between_commands_never_sends_the_next_command(self):
        self.plugin._map_deadline = time.monotonic()+10
        with self.assertRaisesRegex(RuntimeError, 'boot_changed'):
            self.plugin._command('ChangeCartoMap', {'mapName': 'failed'}, threading.Event())
        self.assertFalse(self.client.commands)
        self.assert_kept()

    def test_failed_runtime_probe_does_not_erase_the_new_session_recovery_fence(self):
        self.plugin.store.state('mapping', None)
        self.plugin._map_pending = None
        self.plugin._persist()
        original = self.client.native
        def unavailable(operation, **args):
            if operation == 'map_runtime_status':
                raise TimeoutError('boot_probe_unavailable')
            return original(operation, **args)
        self.client.native = unavailable
        self.assertEqual(self.plugin.dispatch('start_mapping', {'map_name': 'new'})['state'], 'executing')
        wait(lambda: bool(self.bundle.tracker.events))
        self.assertEqual(self.bundle.tracker.events[-1][0], 'error')
        self.assertEqual(self.plugin.store.state('mapping')['phase'], 'starting')
        pending = self.plugin.store.state('pending')['map_transition']
        self.assertEqual(pending['operation'], 'start_mapping')
        self.assertIsInstance(pending['stage_started_at'], float)
        self.assertFalse(self.client.commands)


class PartialMapDeletionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root/'failed.pgm').write_bytes(b'')
        (self.root/'failed.yaml').write_bytes(b'partial-yaml')

    def manifest(self):
        return rpc.map_delete_manifest('failed', self.root)

    def delete(self, manifest, inactive=lambda name: None):
        return rpc.map_archive_write('map_delete', {'map_name': 'failed', 'token': 'a'*32,
                                                   'manifest': manifest}, self.root, inactive)

    def test_partial_map_can_be_deleted_but_cannot_be_archived(self):
        manifest = self.manifest()
        self.assertEqual(manifest['files']['.pgm']['size'], 0)
        with self.assertRaisesRegex(RuntimeError, 'not_complete'):
            rpc.map_archive_read('map_archive_manifest', {'map_name': 'failed'}, self.root)
        self.assertTrue(self.delete(manifest)['success'])
        self.assertFalse((self.root/'failed.pgm').exists())
        self.assertFalse((self.root/'failed.yaml').exists())

    def test_active_map_and_changed_files_still_cannot_be_deleted(self):
        manifest = self.manifest()
        def active(name):
            raise ValueError('cannot_modify_active_map')
        with self.assertRaisesRegex(ValueError, 'active_map'):
            self.delete(manifest, active)
        (self.root/'failed.pgm').write_bytes(b'new-write')
        with self.assertRaisesRegex(ValueError, 'changed_since_backup'):
            self.delete(manifest)
        self.assertTrue((self.root/'failed.pgm').exists())

    def test_symlink_and_unsafe_names_are_rejected(self):
        (self.root/'failed.path').symlink_to(self.root/'failed.yaml')
        with self.assertRaisesRegex(RuntimeError, 'invalid_map_delete_source'):
            self.manifest()
        for name in ('../map', '.', '..', ''):
            with self.assertRaises(ValueError):
                rpc.map_delete_manifest(name, self.root)

    def test_driver_delete_uses_partial_manifest_and_keeps_active_map_guard(self):
        bundle, client = Bundle(), Client()
        client.grid_maps.append({'fileName': 'failed.pgm'})
        native = client.native
        def call(operation, **args):
            if operation == 'map_delete_manifest':
                return self.manifest()
            if operation == 'map_delete':
                response = self.delete(args['manifest'])
                client.grid_maps = [m for m in client.grid_maps if m['fileName'] != 'failed.pgm']
                return response
            return native(operation, **args)
        client.native = call
        plugin = ControlledSpatialPlugin(bundle, {'db_path': str(self.root/'state.db')}, client)
        bundle.spatial = plugin
        self.addCleanup(plugin.close)
        result = plugin.dispatch('delete_map', {'map_name': 'failed', 'confirm_delete': True})
        self.assertEqual(result['state'], 'executing')
        wait(lambda: bool(bundle.tracker.events))
        self.assertEqual(bundle.tracker.events[-1][0], 'completed')
        self.assertFalse(client.commands)


if __name__ == '__main__':
    unittest.main()
