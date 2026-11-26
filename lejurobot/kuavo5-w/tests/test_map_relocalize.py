"""Relocalization relies on held chassis feedback, independent of upper ROS."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location('map_relocalize',
    Path(__file__).resolve().parents[1]/'host/map_relocalize.py')
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


class RelocalizationHoldTests(unittest.TestCase):
    def test_only_fresh_held_stationary_chassis_passes(self):
        for held, age, speed, expected in (
                (True, .1, 0., True), (False, .1, 0., False),
                (True, .6, 0., False), (True, -.1, 0., False),
                (True, .1, .02, False), (True, .1, float('nan'), False)):
            with self.subTest(held=held, age=age, speed=speed):
                odom = NS(header=NS(stamp=NS(to_sec=lambda: 10.-age)),
                    twist=NS(twist=NS(linear=NS(x=speed, y=0.), angular=NS(z=0.))))
                self.assertEqual(helper.stationary_hold(NS(soft_estop=held), odom, 10.), expected)

    def test_tf_warmup_uses_a_new_scan_at_its_actual_stamp(self):
        old = NS(header=NS(frame_id='laser', stamp=1))
        fresh = NS(header=NS(frame_id='laser', stamp=2))
        transform = Mock(side_effect=[LookupError(), (0., 0., 0.)])
        with patch.object(helper.time, 'monotonic', side_effect=[0., .1, .2]):
            scan, mounting = helper.scan_with_transform(Mock(side_effect=[old, fresh]), transform, (LookupError,))
        self.assertIs(scan, fresh)
        self.assertEqual(mounting, (0., 0., 0.))
        self.assertEqual(transform.call_args.args, ('base_link', 'laser', 2))

    def test_missing_tf_stays_unconfirmed(self):
        scan = NS(header=NS(frame_id='laser', stamp=1))
        with patch.object(helper.time, 'monotonic', side_effect=[0., .1, 4.]):
            self.assertEqual(helper.scan_with_transform(lambda: scan,
                Mock(side_effect=LookupError()), (LookupError,)), (None, None))

