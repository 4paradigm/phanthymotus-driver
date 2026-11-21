"""Focused regression tests for map isolation, persistence and navigation guards."""
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controlled_spatial import ControlledSpatialPlugin, SpatialStore, filter_body_returns, safe_name
import audio


class SpawnLoggingTest(unittest.TestCase):
    def test_logging_is_installed_before_child_setup(self):
        from common import logsafe
        import base_servo
        import spatial_map

        class InitializationReached(Exception):
            pass

        entries = [
            (audio._worker, (None,) * 5),
            (spatial_map._publisher, (None,) * 2),
            (base_servo._receive, (None,) * 6),
        ]
        for entry, args in entries:
            with self.subTest(entry=entry.__module__):
                with patch.object(logsafe, 'install', side_effect=InitializationReached) as install:
                    with self.assertRaises(InitializationReached):
                        entry(*args)
                    install.assert_called_once_with(check_fd=False)


class SpatialTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.plugin = ControlledSpatialPlugin({'native_slam_pcd_dir': str(root / 'maps'),
                            'native_slam_db_path': str(root / 'controlled_spatial.db'),
                            'vendor_map_dir': str(root / 'native')}, 'test', None)
        self.plugin.vendor_dir.mkdir()
        self.plugin._call = Mock(return_value=SimpleNamespace(map_names=[], success=True))

    def tearDown(self):
        self.plugin.stop()
        self.tmp.cleanup()

    def test_start_is_ready_without_claiming_mapping_or_localization(self):
        p = self.plugin
        p._call.reset_mock()
        before = p.info()
        result = p.dispatch('start', {})
        self.assertEqual(result['state'], 'ready')
        self.assertEqual(result['data']['state'], 'idle')
        self.assertEqual(p.dispatch('info', {})['state'], 'idle')
        self.assertEqual(before['active_map'], result['data']['active_map'])
        self.assertFalse(p.mapping)
        self.assertIsNone(p.nav)
        p._call.assert_not_called()

    def test_start_preserves_detailed_mapping_status(self):
        self.plugin.mapping = True
        self.plugin.active_map = 'room'
        result = self.plugin.dispatch('start', {})
        self.assertEqual(result['state'], 'ready')
        self.assertEqual(result['data']['state'], 'mapping')
        self.assertEqual(result['data']['active_map'], 'room')
        self.assertEqual(self.plugin.dispatch('info', {})['state'], 'mapping')

    def test_rejects_paths_and_empty_names(self):
        for value in ['', '../other', '/tmp/map', 'a/b', 'a.b', 'x'*65]:
            with self.assertRaises(ValueError):
                safe_name(value)
        self.assertEqual(safe_name('测试_map-1'), '测试_map-1')

    def test_failed_vendor_operation_never_reports_saved(self):
        p = self.plugin
        p.mapping, p.active_map = True, 'room'
        p.pose, p.pose_received, p.localized_after = {'x': 1, 'y': 2, 'yaw': 0}, time.monotonic(), 0
        p.grid = SimpleNamespace()
        p._call.side_effect = RuntimeError('disk full')
        result = p.dispatch('stop_mapping', {})
        self.assertEqual(result['state'], 'error')
        self.assertTrue(p.mapping)
        self.assertEqual(p.store.maps(), [])

    def test_tags_persist_and_match_full_map_and_name(self):
        p = self.plugin
        p.store.tag('P10', 'room', {'x': 9, 'y': 2, 'yaw': 0}, '')
        p.store.tag('P1', 'other', {'x': 7, 'y': 2, 'yaw': 0}, '')
        p.active_map = 'room'
        p.pose, p.pose_received, p.localized_after = {'x': 1, 'y': 2, 'yaw': 0}, time.monotonic(), 0
        p.scan_received = time.monotonic()
        p._navigate = Mock()
        result = p.dispatch('navigate_to_tag', {'tag_name': 'P1'})
        self.assertEqual(result['state'], 'error')
        p._navigate.assert_not_called()
        second = SpatialStore(str(Path(self.tmp.name) / 'controlled_spatial.db'))
        self.assertEqual(second.tags('room')[0]['name'], 'P10')
        second.conn.close()

    def test_stale_pose_and_map_transitions_block_tagging(self):
        p = self.plugin
        p.active_map = 'room'
        p.pose, p.pose_received, p.localized_after = {'x': 1, 'y': 2, 'yaw': 0}, time.monotonic()-10, 0
        self.assertEqual(p.dispatch('tag_place', {'name': 'P1'})['state'], 'error')
        p.pose_received, p.localized_after = time.monotonic(), time.monotonic()+1
        self.assertEqual(p.dispatch('get_pose', {})['state'], 'error')
        self.assertEqual(p.store.tags('room'), [])

    def test_mapping_refuses_missing_scan_before_reset(self):
        p = self.plugin
        result = p.dispatch('start_mapping', {'map_name': 'room'})
        self.assertEqual(result['state'], 'error')
        self.assertIn('/scan', result['error'])
        self.assertEqual(p._call.call_count, 1)
        self.assertEqual(p._call.call_args.args, ('get_map_list',))
        self.assertFalse(p.mapping)

    def test_single_lidar_preserves_message_and_yields_to_merged_scan(self):
        p = self.plugin
        stamp = SimpleNamespace(sec=100, nanosec=0)
        msg = SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id='front_sensor'), ranges=[1.0])
        p.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
        p.scan_relay = Mock()
        try:
            p._single_scan(msg)
            p.scan_relay.publish.assert_called_once_with(msg)
            merged = SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id='base_laser_link'), ranges=[1.0])
            p._scan(merged)
            p._single_scan(msg)
            self.assertEqual(p.scan_relay.publish.call_count, 1)
            self.assertEqual(msg.header.frame_id, 'front_sensor')
        finally:
            p.node = None

    def test_single_lidar_rejects_stale_frames(self):
        p = self.plugin
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0), frame_id='front_sensor'), ranges=[1.0])
        p.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
        p.scan_relay = Mock()
        try:
            p._single_scan(msg)
            p.scan_relay.publish.assert_not_called()
        finally:
            p.node = None

    def test_body_filter_keeps_external_hits_at_the_same_range(self):
        import math
        import numpy as np

        scan = SimpleNamespace(header=object(), angle_min=0.0, angle_increment=math.pi,
                               range_min=.1, range_max=50., ranges=[.15, .15, float('inf')])
        transform = np.eye(4)
        transform[0, 3] = .22584
        result, count = filter_body_returns(scan, transform, [-.15, .15, -.14, .14])
        self.assertEqual(count, 1)
        self.assertEqual(scan.ranges[1], .15)
        self.assertEqual(result.ranges[0], .15)
        self.assertTrue(math.isnan(result.ranges[1]))
        self.assertTrue(math.isinf(result.ranges[2]))
        self.assertIs(result.header, scan.header)

    def test_unverified_loaded_pose_cannot_start_navigation(self):
        p = self.plugin
        p.config['require_tag_localization'] = True
        p.active_map = 'room'
        p.localized_after = 0
        p.pose_received = p.scan_received = time.monotonic()
        p.pose = {'x': .35, 'y': -2., 'yaw': 1.1}
        p._navigate = Mock()
        self.assertEqual(p.info()['state'], 'localizing')
        self.assertFalse(p.dispatch('get_pose', {})['localization_verified'])
        self.assertIn('relocalize_at_tag', p.dispatch('tag_place', {'name': 'P0'})['error'])
        self.assertEqual(p.store.tags('room'), [])
        result = p.dispatch('navigate_to_pose', {'x': 0., 'y': 0., 'yaw': 0.})
        self.assertIn('relocalize_at_tag', result['error'])
        p._navigate.assert_not_called()

    def test_tag_seed_checks_new_tf_for_agreement_before_enabling_navigation(self):
        p = self.plugin
        tag = {'name': 'P0', 'x': -.326, 'y': -.002, 'yaw': -.15}
        p.active_map = 'room'
        p.grid = object()
        p.scan_received = 10.
        p.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100_000_000_000, to_msg=lambda: object())))
        p.initial_pose_type = lambda: SimpleNamespace(header=SimpleNamespace(), pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(), orientation=SimpleNamespace()), covariance=[0.] * 36))
        p.initial_pose_pub = Mock()
        p.initial_pose_pub.get_subscription_count.return_value = 1
        try:
            for matches in (False, True):
                fake_now = 10.
                def spin(_):
                    nonlocal fake_now
                    fake_now += .2
                    p.pose = {'x': tag['x'], 'y': tag['y'] if matches else -2., 'yaw': tag['yaw']}
                    p.pose_source = 'tf:map->base_link'
                    p.pose_received = fake_now
                    p.pose_stamp = 100. + fake_now - 10.
                with patch('controlled_spatial.time.monotonic', lambda: fake_now), \
                     patch('controlled_spatial.time.sleep', spin):
                    if matches:
                        self.assertAlmostEqual(p._localize(tag)['y'], tag['y'])
                    else:
                        with self.assertRaisesRegex(RuntimeError, 'did not settle'):
                            p._localize(tag)
                    self.assertEqual(p.localization_verified, matches)
                sent = p.initial_pose_pub.publish.call_args.args[0]
                self.assertEqual(sent.header.frame_id, 'map')
                self.assertEqual(sent.pose.pose.position.x, tag['x'])
        finally:
            p.node = None

    def test_load_at_mapping_start_requires_no_tag(self):
        p = self.plugin
        p.config['require_origin_localization'] = True
        start = {'x': .04, 'y': -.1, 'yaw': .35}
        p.store.save_map('room', p.root / 'room', start)
        p._call.return_value.map_names = ['room', 'old']
        p._wait_until = Mock(return_value=True)
        p._localize = Mock(return_value=start)

        result = p.dispatch('load_map', {'map_name': 'room'})
        self.assertEqual(p.store.tags('room'), [])
        self.assertEqual(result['state'], 'localized')
        self.assertEqual(result['seed_source'], 'mapping_start_pose')
        p._localize.assert_called_once_with(start)

        result = p.dispatch('load_map', {'map_name': 'old'})
        self.assertEqual(result['state'], 'localizing')
        self.assertFalse(p.localization_verified)
        p._localize.assert_called_once()

        p._localize.side_effect = RuntimeError('SLAM pose did not settle')
        self.assertEqual(p.dispatch('load_map', {'map_name': 'old', **start})['state'], 'localizing')
        self.assertIsNone(p.store.map_start_pose('old'))

        p._localize.side_effect = None
        result = p.dispatch('load_map', {'map_name': 'old', **start})
        self.assertEqual(result['seed_source'], 'provided_start_pose')
        self.assertEqual(p.store.map_start_pose('old'), start)
        self.assertEqual(p.dispatch('load_map', {'map_name': 'old'})['seed_source'], 'mapping_start_pose')

    def test_native_archive_excludes_prefix_collisions(self):
        p = self.plugin
        p.mapping_start_pose = {'x': .02, 'y': .05, 'yaw': .3}
        for name in ['room.posegraph', 'room.data', 'room2.posegraph']:
            (p.vendor_dir/name).write_text(name)
        target = p._archive('room')
        self.assertEqual({f.name for f in target.iterdir()}, {'room.posegraph', 'room.data'})
        self.assertEqual(p.store.maps()[0]['name'], 'room')
        self.assertEqual(p.store.map_start_pose('room'), p.mapping_start_pose)

    def test_no_fake_save_without_native_files(self):
        with self.assertRaises(RuntimeError):
            self.plugin._archive('room')
        self.assertEqual(self.plugin.store.maps(), [])

    def test_rejects_overwrite_and_delete_of_active_map(self):
        p = self.plugin
        p._call.return_value.map_names = ['room']
        self.assertEqual(p.dispatch('start_mapping', {'map_name': 'room'})['state'], 'error')
        self.assertEqual(p._call.call_count, 1)
        p.active_map = 'room'
        self.assertEqual(p.dispatch('delete_map', {'map_name': 'room'})['state'], 'error')
        self.assertEqual(p._call.call_count, 1)

    def test_navigation_cancel_during_acceptance_keeps_goal_owned(self):
        p = self.plugin
        p.nav = {'action_id': 'one', 'handle': None, 'state': 'accepting'}
        self.assertEqual(p.stop_nav()['state'], 'canceling')
        self.assertEqual(p.nav['state'], 'canceling')
        with self.assertRaises(RuntimeError):
            p._transition()
        p.nav = None

    def test_navigation_timeout_never_claims_stopped(self):
        p = self.plugin
        handle = SimpleNamespace(accepted=True, cancel_goal_async=Mock())
        nav = {'action_id': 'one', 'state': 'navigating', 'handle': handle,
               'started': time.monotonic()-200, 'target': {'x': 0, 'y': 0, 'yaw': 0}}
        p.nav = nav
        p._watchdog()
        self.assertIs(p.nav, nav)
        self.assertTrue(nav['timeout'])
        handle.cancel_goal_async.assert_called_once()
        p._notify = Mock()
        p._finish(nav, 'completed', 'late success')
        self.assertEqual(p.last_navigation['status'], 'error')

    def test_stop_bypasses_operation_lock(self):
        p = self.plugin
        p.operations.acquire()
        try:
            self.assertEqual(p.dispatch('stop_nav', {})['state'], 'idle')
            self.assertEqual(p.dispatch('list_maps', {})['state'], 'busy')
        finally:
            p.operations.release()


class AudioDiscoveryTest(unittest.TestCase):
    @patch.object(audio, '_pulse_user', return_value={})
    @patch.object(audio.glob, 'glob', return_value=['/run/panther-host-tmp/pulse-new/native'])
    @patch.object(audio.subprocess, 'run')
    def test_auto_uses_reachable_host_socket(self, run, *_):
        run.side_effect = [SimpleNamespace(returncode=1, stderr='refused'),
                           SimpleNamespace(returncode=0, stderr='')]
        config = {'server': 'auto', 'uid': 1001, 'gid': 1002}
        result = audio._resolve_pulse(config)
        self.assertEqual(result['server'], 'unix:/run/panther-host-tmp/pulse-new/native')
        self.assertEqual(config['server'], 'auto')  # Repeat discovery on the next start.



class StoreCompatibilityTest(unittest.TestCase):
    def test_reads_reference_schema_with_extra_columns(self):
        import sqlite3
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / 'db')
            conn = sqlite3.connect(path)
            conn.executescript('''CREATE TABLE maps(name TEXT PRIMARY KEY, pcd_path TEXT NOT NULL,
                created_at REAL, visual_path TEXT);
                CREATE TABLE poi(id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT, description TEXT,
                    x REAL, y REAL, yaw REAL, map_name TEXT, created_at REAL,
                    UNIQUE(name,map_name));''')
            conn.close()
            store = SpatialStore(path)
            store.save_map('room', '/maps/room')
            store.tag('P1', 'room', {'x': 1, 'y': 2, 'yaw': 3}, 'desk')
            self.assertEqual(store.tags('room')[0]['description'], 'desk')
            store.conn.close()



class MapRenderingTest(unittest.TestCase):
    def test_grid_uses_cell_centers_and_rotated_origin(self):
        import numpy as np
        from spatial_map import grid_points
        points = grid_points([49, 50, -1, 100], 2, 2, 1.0, [10, 20, np.pi/2])
        np.testing.assert_allclose(points, [[9.5, 21.5, 0], [8.5, 21.5, 0]])
        self.assertEqual(points.dtype.str, '<f4')

    def test_map_display_budget_is_bounded(self):
        from spatial_map import grid_points
        points = grid_points([100] * 100000, 1000, 100, 0.05, [0, 0, 0])
        self.assertLessEqual(len(points), 78000)

    def test_snapshot_roundtrip_and_binary_mapping_contract(self):
        import struct
        import json
        import numpy as np
        from spatial_map import save_grid, SpatialMapPlugin
        with tempfile.TemporaryDirectory() as tmp:
            p = ControlledSpatialPlugin({'native_slam_pcd_dir': str(Path(tmp)/'maps'),
                'native_slam_db_path': str(Path(tmp)/'db')}, 'test', None)
            p.active_map = 'room'
            path = p.root/'room'
            path.mkdir()
            msg = SimpleNamespace(data=[0,100,-1,100], info=SimpleNamespace(width=2,height=2,
                resolution=0.5, origin=SimpleNamespace(position=SimpleNamespace(x=0,y=0),
                    orientation=SimpleNamespace(w=1,x=0,y=0,z=0))))
            save_grid(path/'occupancy.npz', msg)
            p.store.save_map('room', path)
            p.store.tag('P1', 'room', {'x': 1, 'y': 2, 'yaw': np.pi / 2}, 'desk')
            p.store.tag('P2', 'other', {'x': 10, 'y': 20, 'yaw': 0}, '')
            display = SpatialMapPlugin(p, 'test')
            payload = display._payload()
            x,y,yaw,flags,count = struct.unpack_from('<fffBI', payload)
            self.assertEqual(flags, 7)
            self.assertGreater(count, 2)
            offset = 17 + count*12
            length, = struct.unpack_from('<I', payload, offset)
            meta = json.loads(payload[offset+4:])
            self.assertEqual(length, len(payload)-offset-4)
            self.assertFalse(meta['robot']['pose_available'])
            self.assertEqual(meta['tags'][0]['name'], 'P1')
            self.assertEqual(meta['grid_points'], 2)
            points = np.frombuffer(payload, dtype='<f4', count=count * 3, offset=17).reshape(-1, 3)
            self.assertTrue(np.all(points[:2, 2] == 0))
            markers = points[2:]
            np.testing.assert_allclose(markers[:, 0].mean(), 1, atol=0.001)
            self.assertAlmostEqual(float(markers[:, 2].max()), 0.82, places=5)
            np.testing.assert_allclose(markers[markers[:, 1].argmax()], [1, 2.55, 0.14], atol=0.001)
            # A live grid must never change the saved-map rendering.
            p.grid = SimpleNamespace(data=[])
            self.assertEqual(display._payload(), payload)
            # Keep every marker visible when a dense grid fills the renderer budget.
            dense = SimpleNamespace(data=np.full(78000, 100), info=SimpleNamespace(
                width=780, height=100, resolution=0.5, origin=msg.info.origin))
            save_grid(path/'occupancy.npz', dense)
            dense_payload = display._payload()
            dense_count = struct.unpack_from('<I', dense_payload, 13)[0]
            self.assertEqual(dense_count, 78000 + len(markers))
            for index in range(2, 5):
                p.store.tag(f'P{index}', 'room', {'x': index, 'y': 2, 'yaw': 0}, '')
            dense_payload = display._payload()
            dense_count = struct.unpack_from('<I', dense_payload, 13)[0]
            self.assertEqual(dense_count, 80000)
            last_point = np.frombuffer(dense_payload, dtype='<f4', count=3, offset=17 + (dense_count - 1) * 12)
            np.testing.assert_allclose(last_point, [4.55, 2, 0.14], atol=0.001)
            p.stop()

    def test_corrupt_grid_dimensions_are_rejected(self):
        from spatial_map import grid_points
        with self.assertRaises(ValueError):
            grid_points([100], 2, 2, 0.1, [0,0,0])


if __name__ == '__main__':
    unittest.main()
