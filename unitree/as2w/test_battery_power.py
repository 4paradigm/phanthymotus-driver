"""Offline synthetic tests; optional real JSONL via AS2W_BMS_REPLAY."""
import io
from contextlib import nullcontext
import json
from decimal import Decimal, localcontext
import os
from pathlib import Path
import threading
import tempfile
import types
import unittest
from unittest.mock import patch

import yaml

from unitree.as2w.battery_power import BatteryPower, PowerConfig, parse_bms


def bms(**changes):
    result = dict(bmsvoltage=[38800, 38820, 0], cell_vol=[3240]*12+[0]*28,
                  current=-2000, soc=50, soh=99)
    result.update(changes)
    return result


def reference_replay(path, config):
    """Integrate raw integer units independently of the production calculator.

    Decimal arithmetic retains the recorded nanosecond intervals. Invalid raw
    fields fail the replay fixture check instead of becoming expected results.
    """
    with open(path, encoding='utf-8') as stream:
        rows = [row for line in stream if (row := json.loads(line)).get('bms')]
    if len(rows) < 2:
        raise ValueError('replay requires at least two BMS samples')
    with localcontext() as context:
        context.prec = 40
        points = []
        for row in rows:
            raw = row['bms']
            volts, cells = raw['bmsvoltage'], raw['cell_vol']
            if (len(volts) != 3 or len(cells) != 40
                    or any(type(v) is not int for v in volts + cells)
                    or volts[0] <= 0 or volts[1] <= 0 or volts[2] != 0
                    or any(not config.cell_min_mv <= v <= config.cell_max_mv
                           for v in cells[:12])
                    or any(cells[12:])
                    or max(abs(volts[0] - volts[1]),
                           abs(volts[0] - sum(cells)),
                           abs(volts[1] - sum(cells)))
                    > config.voltage_tolerance_mv
                    or type(raw['current']) is not int
                    or not -(2**31) <= raw['current'] < 2**31
                    or any(type(raw[k]) is not int or not 0 <= raw[k] <= 100
                           for k in ('soc', 'soh'))):
                raise ValueError('invalid raw BMS replay sample')
            power = (Decimal(volts[0]) * max(0, -raw['current'])
                     / Decimal(1000000))
            points.append((Decimal(row['monotonic_ns']) / Decimal(10**9),
                           power, raw['current'] < 0, row['sample']))
        segments = []
        area = duration = Decimal(0)
        skipped = 0
        for left, right in zip(points, points[1:]):
            t0, p0, discharge0, seq0 = left
            t1, p1, discharge1, seq1 = right
            if t1 <= t0 or seq1 != seq0 + 1:
                raise ValueError('replay time/order is not continuous')
            dt = t1 - t0
            continuous = dt <= Decimal(str(config.max_gap_s))
            skipped += not continuous
            if continuous and discharge0 and discharge1:
                area += (p0 + p1) * dt / 2
                duration += dt
                segments.append((t0, t1, p0, p1))
            else:
                segments.clear()
        boundary = points[-1][0] - Decimal(str(config.window_s))
        window_area = window_duration = Decimal(0)
        for t0, t1, p0, p1 in segments:
            start = max(t0, boundary)
            if start >= t1:
                continue
            start_power = p0 + (p1 - p0) * (start - t0) / (t1 - t0)
            window_area += (start_power + p1) * (t1 - start) / 2
            window_duration += t1 - start
        return rows, dict(
            energy_wh=float(area / 3600), valid_seconds=float(duration),
            average_power_w=float(area / duration) if duration else None,
            window_seconds=float(window_duration),
            window_power_w=(float(window_area / window_duration)
                            if window_duration else None),
            skipped_intervals=skipped)


class TestBatteryPower(unittest.TestCase):
    def test_sdk_shape_units_and_zero(self):
        for msg in (bms(), types.SimpleNamespace(**bms())):
            parsed = parse_bms(msg, PowerConfig())
            self.assertEqual(38.8, parsed['voltage_v'])
            self.assertEqual(-2, parsed['current_a'])
            self.assertAlmostEqual(77.6, parsed['power_w'])
            self.assertEqual(99, parsed['soh_percent'])
        for current, status in ((2000, 'charging'), (0, 'idle')):
            p = parse_bms(bms(current=current), PowerConfig())
            self.assertEqual(status, p['status'])
            self.assertEqual(0, p['power_w'])

    def test_invalid_voltage_and_layout(self):
        for changes in (dict(bmsvoltage=None), dict(bmsvoltage=[]),
                        dict(bmsvoltage=[0, 38820, 0]), dict(
                            bmsvoltage=[38800, 0, 0]),
                        dict(bmsvoltage=[38800, 38820, 1]), dict(
                            bmsvoltage=[float('nan'), 38820, 0]),
                        dict(bmsvoltage=[38800, 42000, 0]), dict(
                            cell_vol=[3240]*40),
                        dict(cell_vol=[3240]*11+[0]*29), dict(cell_vol=None),
                        dict(cell_vol=[0]+[3240]*11+[0]*28),
                        dict(cell_vol=[65000]*12+[0]*28)):
            with self.subTest(changes=changes):
                p = parse_bms(bms(**changes), PowerConfig())
                self.assertEqual('invalid', p['status'])
                self.assertIsNone(p['power_w'])
                self.assertIsNone(p['voltage_v'])

    def test_invalid_current_and_percentages(self):
        for current in (None, True, '0', float('inf'), 2**31):
            self.assertIsNone(
                parse_bms(bms(current=current), PowerConfig())['power_w'])
        for value in (None, -1, 101, float('nan'), True):
            p = parse_bms(bms(soc=value, soh=value), PowerConfig())
            self.assertIsNone(p['soc'])
            self.assertIsNone(p['soh_percent'])
            self.assertEqual('discharging', p['status'])
        self.assertEqual(0, parse_bms(bms(soc=0, soh=0), PowerConfig())['soc'])

    def test_constant_irregular_and_identical_new_samples(self):
        c = BatteryPower()
        for n, t in enumerate((0, .05, .15, .2), 1):
            c.update(bms(), n, t, 1000+n)
            self.assertFalse(c.update(bms(current=-9000), n, t, 1000+n))
        self.assertAlmostEqual(77.6*.2/3600, c.energy_wh)
        self.assertEqual(3, len(c.segments))
        c = BatteryPower()
        for n, (t, current) in enumerate(
                ((0, -1000), (.1, -2000), (.25, -4000)), 1):
            c.update(bms(current=current), n, t, n)
        self.assertAlmostEqual((58.2*.1+116.4*.15)/3600, c.energy_wh)

    def test_older_sequence_cannot_be_reintegrated(self):
        c = BatteryPower()
        for n in (1, 2, 3):
            c.update(bms(), n, n*.05, n*50)
        energy = c.energy_wh
        for n in (1, 2, 3):
            self.assertFalse(c.update(bms(), n, n*.05, n*50))
        self.assertEqual(energy, c.energy_wh)
        self.assertEqual(3, c.sequence)

    def test_discontinuities_and_pause_preserve_energy(self):
        for change in (
            'sequence', 'gap', 'reverse', 'invalid', 'charging', 'pause',
                'stale'):
            with self.subTest(change=change):
                c = BatteryPower()
                c.update(bms(), 1, 0, 0)
                c.update(bms(), 2, .1, 100)
                energy = c.energy_wh
                if change == 'sequence':
                    c.update(bms(), 4, .2, 200)
                elif change == 'gap':
                    c.update(bms(), 3, 2, 2000)
                elif change == 'reverse':
                    c.update(bms(), 3, .05, 50)
                elif change in ('invalid', 'charging'):
                    c.update(bms(current=None if change ==
                             'invalid' else 1000), 3, .15, 150)
                    c.update(bms(), 4, .2, 200)
                elif change == 'pause':
                    c.reset()
                    c.update(bms(), 3, .2, 200)
                else:
                    self.assertEqual('stale', c.snapshot(2)['status'])
                    c.update(bms(), 3, 2.1, 2100)
                self.assertEqual(energy, c.energy_wh)
                self.assertEqual(0, len(c.segments))

    def feed(
            self, config=PowerConfig(capacity_wh=600),
            end=400, current=-2000):
        c = BatteryPower(config)
        for n in range(end*10+1):
            c.update(bms(current=current), n, n/10, n*100)
        return c

    def test_long_window_runtime_no_soh_correction(self):
        c = self.feed()
        s = c.snapshot(400)
        self.assertAlmostEqual(300, c.covered)
        self.assertAlmostEqual(77.6, s['average_power_w'])
        self.assertAlmostEqual(600*.5/77.6*60, s['remaining_runtime_minutes'])
        self.assertAlmostEqual(77.6*400/3600, s['discharged_energy_wh'])
        self.assertEqual(400000, s['timestamp_ms'])

    def test_partial_boundary(self):
        c = BatteryPower(PowerConfig(window_s=.15, min_observation_s=.1))
        c.update(bms(current=-1000), 1, 0, 0)
        c.update(bms(current=-2000), 2, .1, 100)
        c.update(bms(current=-3000), 3, .2, 200)
        self.assertAlmostEqual(.15, c.covered)
        self.assertAlmostEqual((67.9*.05+97*.1)/.15,
                               c.snapshot(.2)['average_power_w'])

    def test_min_observation_and_coverage(self):
        c = self.feed(end=179)
        self.assertEqual('warming_up', c.snapshot(179)['runtime_status'])
        c = self.feed(end=180)
        self.assertEqual('available', c.snapshot(180)['runtime_status'])
        # Small bounded storage retains only 50% of the configured window.
        c = self.feed(PowerConfig(capacity_wh=600, window_s=4,
                      min_observation_s=1, max_segments=20), end=10)
        self.assertAlmostEqual(2, c.covered)
        self.assertEqual('warming_up', c.snapshot(10)['runtime_status'])
        self.assertEqual(20, len(c.segments))

    def test_runtime_reasons_and_active_staleness(self):
        for capacity in (None, 0, -1, float('nan')):
            c = self.feed(PowerConfig(capacity_wh=capacity), end=180)
            self.assertEqual('capacity_unconfigured',
                             c.snapshot(180)['runtime_status'])
        c = self.feed(end=180, current=-1)
        self.assertEqual('low_power', c.snapshot(180)['runtime_status'])
        c = self.feed(end=180)
        c.update(bms(soc=255), 1801, 180.1, 180100)
        self.assertEqual('invalid_soc', c.snapshot(180.1)['runtime_status'])
        c.update(bms(current=1), 1802, 180.2, 180200)
        self.assertEqual('charging', c.snapshot(180.2)['runtime_status'])
        self.assertIsNone(c.snapshot(180.2)['remaining_runtime_minutes'])
        c = self.feed(end=180)
        energy = c.energy_wh
        s = c.snapshot(181.01)
        self.assertFalse(s['fresh'])
        self.assertEqual('stale', s['runtime_status'])
        self.assertIsNone(s['remaining_runtime_minutes'])
        self.assertIsNone(s['power_w'])
        self.assertEqual(energy, c.energy_wh)

    def test_coverage_threshold_and_capacity_zero_soc(self):
        for max_segments, expected in (
            (1800, 'available'),
                (1799, 'warming_up')):
            c = self.feed(
                PowerConfig(
                    capacity_wh=600, window_s=200,
                    min_observation_s=100,
                    max_segments=max_segments),
                end=200)
            self.assertEqual(expected, c.snapshot(200)['runtime_status'])
        c = self.feed(end=180)
        c.update(bms(soc=0), 1801, 180.1, 180100)
        self.assertEqual(0, c.snapshot(180.1)['remaining_runtime_minutes'])

    def test_config_validation(self):
        for kwargs in (
                dict(stale_s=0),
                dict(max_gap_s=2),
                dict(min_coverage=0),
                dict(max_segments=0),
                dict(min_power_w=float('nan')),
                dict(min_observation_s=301)):
            with self.assertRaises(ValueError):
                PowerConfig(**kwargs)

    @unittest.skipUnless(os.getenv('AS2W_BMS_REPLAY'),
                         'real JSONL path not supplied')
    def test_real_replay(self):
        c = BatteryPower()
        rows, reference = reference_replay(
            os.environ['AS2W_BMS_REPLAY'], c.config)
        for row in rows:
            c.update(row['bms'], row['sample'],
                     row['monotonic_ns']/1e9, row['timestamp_ms'])
        avg = c.energy_wh*3600/reference['valid_seconds']
        self.assertEqual(len(rows), rows[-1]['sample']-rows[0]['sample']+1)
        self.assertEqual(reference['skipped_intervals'],
                         c.skipped_intervals)
        self.assertAlmostEqual(reference['energy_wh'], c.energy_wh, places=9)
        self.assertAlmostEqual(reference['average_power_w'], avg, places=7)
        snapshot = c.snapshot(rows[-1]['monotonic_ns']/1e9)
        self.assertAlmostEqual(reference['window_power_w'],
                               snapshot['average_power_w'], places=7)
        self.assertAlmostEqual(reference['window_seconds'], c.covered,
                               places=7)
        self.assertIsNone(snapshot['remaining_runtime_minutes'])
        print(
            f'REAL REPLAY: samples={len(rows)}, '
            f'valid={reference["valid_seconds"]:.9f}s, '
            f'average={avg:.8f}W, energy={c.energy_wh:.9f}Wh')


class TestPowerCardIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from unitree.as2w.test_driver import _install_device_stubs, _load, ROOT
        _install_device_stubs()
        cls.device = _load('as2w_power_device_test', ROOT/'device.py')
        cls.main = _load('as2w_power_main_test', ROOT/'main.py')
        cls.root = ROOT

    def make_plugin(self, real_worker=False, power_enabled=True):
        d = self.device

        class Node:
            def __init__(self, *args): self.destroyed = False
            def create_publisher(
                    self, *args):
                return types.SimpleNamespace(publish=lambda m: None)

            def destroy_node(self): self.destroyed = True

        subscribers = []

        class Subscriber:
            def __init__(self, *args):
                self.closed = False
                subscribers.append(self)

            def Init(self, callback, *args): self.callback = callback
            def Close(self): self.closed = True
        worker = types.SimpleNamespace(
            start=lambda: None, join=lambda **kwargs: None,
            is_alive=lambda: False)
        worker_context = (nullcontext() if real_worker else patch.object(
            d.threading, 'Thread', return_value=worker))
        with patch('rclpy.node.Node', Node), patch.object(
                d, 'ChannelSubscriber', Subscriber), worker_context:
            plugin = d.StatePlugin(
                {},
                'offline', types.SimpleNamespace(add_node=lambda n: None),
                battery_power_config={
                    'enabled': power_enabled, 'capacity_wh': 600})
        plugin.test_subscribers = subscribers
        self.addCleanup(plugin.shutdown)
        return plugin

    def capacity_config_snapshot(self, override=None):
        config_path = self.root / 'config.yaml'
        with tempfile.TemporaryDirectory() as directory:
            if override is not None:
                config = yaml.safe_load(
                    config_path.read_text(encoding='utf-8'))
                config['plugins']['battery_power'].update(override)
                config_path = Path(directory) / 'config.yaml'
                config_path.write_text(
                    yaml.safe_dump(config), encoding='utf-8')
            with config_path.open(encoding='utf-8') as stream:
                with patch.dict(
                        os.environ, {'CONFIG_PATH': str(config_path)}), \
                        patch('builtins.open', return_value=stream) as opened:
                    config = self.main.load_config()
                    opened.assert_called_once_with(str(config_path))
        card = self.device._BatteryPowerCard(
            config['plugins']['battery_power'], 'offline')
        for sequence in range(1801):
            card.core.update(bms(), sequence, sequence / 10, sequence * 100)
        return card.core.config, card.core.snapshot(180)

    def test_driver_initialization_publishes_without_mcp_start(self):
        p = self.make_plugin(real_worker=True)
        node, card = p._state, p._power_card
        received = threading.Event()
        original = node._publish

        def publish(publisher, value):
            original(publisher, value)
            if publisher is card.publisher and value['fresh']:
                received.set()
        with patch.object(node, '_publish', side_effect=publish):
            node._on_bms(types.SimpleNamespace(**bms()))
            self.assertTrue(received.wait(2))
        self.assertFalse(card.logical_running)
        self.assertTrue(card.running)
        worker = node._publisher_thread
        p.shutdown()
        self.assertFalse(worker.is_alive())
        self.assertTrue(node._low.closed)
        self.assertTrue(node._bms.closed)
        self.assertTrue(node._sport.closed)
        self.assertTrue(node.node.destroyed)

    def test_always_on_window_reaches_runtime_and_survives_canvas_cycles(self):
        node, card, output = self.make_queue_node()
        clock = [0.0]
        with patch.object(
                self.device.time, 'monotonic',
                side_effect=lambda: clock[0]):
            for n in range(1851):
                clock[0] = n/10
                node._on_bms(types.SimpleNamespace(**bms()))
                node._process_battery_power()
                if n in (500, 1000, 1800):
                    energy, covered = card.core.energy_wh, card.core.covered
                    for _ in range(3):
                        card.start()
                        card.stop()
                    self.assertEqual(energy, card.core.energy_wh)
                    self.assertEqual(covered, card.core.covered)
            self.assertEqual('available', output[-1][1]['runtime_status'])
            self.assertAlmostEqual(185, card.core.covered)
            self.assertAlmostEqual(77.6*185/3600, card.core.energy_wh)
            self.assertFalse(card.logical_running)

    def test_other_card_stops_preserve_power_logical_state_and_window(self):
        plugin = self.make_plugin()
        node, card = plugin._state, plugin._power_card
        args = {'_tool_name': 'battery_power'}
        clock = [0.0]
        with patch.object(self.device.time, 'monotonic',
                          side_effect=lambda: clock[0]):
            self.assertEqual('running',
                             plugin.dispatch('start', args)['state'])
            for n in range(1851):
                clock[0] = n/10
                node._on_bms(types.SimpleNamespace(**bms()))
                node._process_battery_power()
            self.assertEqual('available', card.info()['diagnostics'][
                'snapshot']['runtime_status'])
            for name in (
                    'imu', 'joints', 'battery', 'joint_state', 'loco_state'):
                energy, covered = card.core.energy_wh, card.core.covered
                plugin.dispatch('stop', {'_tool_name': name})
                info = plugin.dispatch('info', args)
                self.assertEqual('running', info['state'])
                self.assertEqual('running', info['stream_state'])
                self.assertEqual(energy, card.core.energy_wh)
                self.assertEqual(covered, card.core.covered)
                self.assertIs(node, plugin._state)
                self.assertFalse(node._bms.closed)
                clock[0] += .1
                node._on_bms(types.SimpleNamespace(**bms()))
                node._process_battery_power()
                self.assertAlmostEqual(covered+.1, card.core.covered)
                self.assertAlmostEqual(energy+77.6*.1/3600,
                                       card.core.energy_wh)
            energy, covered = card.core.energy_wh, card.core.covered
            info = plugin.dispatch('stop', args)
            self.assertEqual('idle', info['state'])
            self.assertEqual('running', info['stream_state'])
            self.assertEqual(energy, card.core.energy_wh)
            self.assertEqual(covered, card.core.covered)
            clock[0] += .1
            node._on_bms(types.SimpleNamespace(**bms()))
            node._process_battery_power()
            self.assertAlmostEqual(covered+.1, card.core.covered)
            self.assertGreater(card.core.energy_wh, energy)
            plugin.shutdown()
            self.assertFalse(card.running)
            self.assertTrue(node._bms.closed)
            self.assertTrue(node.node.destroyed)

    def test_shared_sensor_stops_keep_both_battery_streams_alive(self):
        p = self.make_plugin()
        node = p._state
        for name in (
                'battery_power', 'battery', 'imu',
                'joint_state', 'loco_state'):
            self.assertEqual('idle', p.dispatch(
                'stop', {'_tool_name': name})['state'])
            self.assertIs(node, p._state)
            self.assertTrue(p._power_card.running)
            self.assertFalse(node._bms.closed)
        published = []
        with patch.object(
                node, '_publish',
                side_effect=lambda pub, data: published.append(pub)):
            node._on_bms(types.SimpleNamespace(**bms()))
            # Use the production shared loop; no Dashboard start was issued.
            with patch.object(
                    node._stop_event, 'wait',
                    side_effect=lambda _: node._stop_event.set()):
                node._publish_loop()
        self.assertIn(node.battery, published)
        self.assertIn(p._power_card.publisher, published)

    def test_detached_logical_card_still_handles_charge_invalid_gap_and_stale(
            self):
        node, card, output = self.make_queue_node()
        clock = [0.0]
        with patch.object(
                self.device.time, 'monotonic',
                side_effect=lambda: clock[0]):
            for n in range(2):
                clock[0] = n*.1
                node._on_bms(types.SimpleNamespace(**bms()))
                node._process_battery_power()
            energy = card.core.energy_wh
            card.stop()
            for now, current, status in ((.2, 1000, 'charging'),
                                         (.3, None, 'invalid')):
                clock[0] = now
                node._on_bms(types.SimpleNamespace(**bms(current=current)))
                node._process_battery_power()
                self.assertEqual(status, output[-1][1]['status'])
                self.assertIsNone(output[-1][1]['remaining_runtime_minutes'])
                self.assertEqual(energy, card.core.energy_wh)
            for now in (.4, .8):
                clock[0] = now
                node._on_bms(types.SimpleNamespace(**bms()))
                node._process_battery_power()
            self.assertEqual(1, card.core.skipped_intervals)
            self.assertEqual(0, card.core.covered)
            self.assertEqual(energy, card.core.energy_wh)
            clock[0] = 1.81
            node._process_battery_power()
            self.assertEqual('stale', output[-1][1]['status'])
            self.assertEqual(energy, card.core.energy_wh)

    def test_new_driver_instance_starts_new_energy_counter(self):
        first = self.make_plugin()
        first._power_card.core.update(bms(), 1, 0, 0)
        first._power_card.core.update(bms(), 2, .1, 100)
        self.assertGreater(first._power_card.core.energy_wh, 0)
        first.shutdown()
        second = self.make_plugin()
        self.assertTrue(second._power_card.running)
        self.assertEqual(0, second._power_card.core.energy_wh)
        self.assertEqual(0, second._power_card.core.covered)
        self.assertIsNot(first._state, second._state)
        self.assertEqual(3, len(second.test_subscribers))

    def test_disabled_power_retains_legacy_shared_lifecycle(self):
        p = self.make_plugin(power_enabled=False)
        node = p._state
        self.assertIsNone(p._power_card)
        self.assertNotIn('battery_power', [t['name'] for t in p.get_tools()])
        p.stop()
        self.assertTrue(node._bms.closed)
        self.assertTrue(node.node.destroyed)

    def test_default_yaml_capacity_matches_core_default(self):
        config, snapshot = self.capacity_config_snapshot()
        self.assertEqual(648.0, config.capacity_wh)
        self.assertEqual(648.0, PowerConfig().capacity_wh)
        self.assertEqual('available', snapshot['runtime_status'])
        self.assertAlmostEqual(648 * .5 / 77.6 * 60,
                               snapshot['remaining_runtime_minutes'])

    def test_custom_yaml_capacity_overrides_nominal_default(self):
        config, snapshot = self.capacity_config_snapshot(
            {'capacity_wh': 324.0})
        self.assertEqual(324.0, config.capacity_wh)
        self.assertEqual('available', snapshot['runtime_status'])
        self.assertAlmostEqual(324 * .5 / 77.6 * 60,
                               snapshot['remaining_runtime_minutes'])

    def test_explicit_yaml_null_disables_runtime_after_warmup(self):
        config, snapshot = self.capacity_config_snapshot({'capacity_wh': None})
        self.assertIsNone(config.capacity_wh)
        self.assertEqual('capacity_unconfigured', snapshot['runtime_status'])
        self.assertIsNone(snapshot['remaining_runtime_minutes'])
        self.assertTrue(snapshot['fresh'])
        self.assertGreater(snapshot['discharged_energy_wh'], 0)
        self.assertAlmostEqual(77.6, snapshot['average_power_w'])

    def test_lifecycle_registration_other_cards_and_shutdown(self):
        p = self.make_plugin()
        node, card = p._state, p._power_card
        info = p.dispatch('info', {'_tool_name': 'battery_power'})
        self.assertEqual('idle', info['state'])
        self.assertEqual('running', info['stream_state'])
        self.assertTrue(info['always_on'])
        for n in range(3):
            with patch.object(
                    self.device.time, 'monotonic',
                    return_value=10+n*.1):
                node._on_bms(types.SimpleNamespace(**bms()))
                node._process_battery_power()
        energy, covered = card.core.energy_wh, card.core.covered
        with patch.object(self.device.time, 'monotonic', return_value=10.2):
            for _ in range(2):
                p.dispatch('stop', {'_tool_name': 'battery_power'})
                p.stop()  # Shared stop is logical while power is enabled.
                p.dispatch('start', {'_tool_name': 'battery_power'})
        self.assertEqual(energy, card.core.energy_wh)
        self.assertEqual(covered, card.core.covered)
        self.assertIs(node, p._state)
        self.assertEqual(3, len(p.test_subscribers))
        self.assertFalse(node._bms.closed)
        self.assertTrue(node.remote_enabled())
        self.assertFalse(node.node.destroyed)
        p.shutdown()
        self.assertFalse(card.running)
        self.assertIsNone(card.node)
        self.assertTrue(node._bms.closed)
        self.assertTrue(node.node.destroyed)
        for name in ('battery_power', 'battery', 'remote_controller'):
            self.assertEqual('idle', p.dispatch(
                'start', {'_tool_name': name})['state'])

    def test_shared_node_rebuild_retains_energy_without_old_baseline(self):
        p = self.make_plugin()
        old = p._state
        c = p._power_card
        p.dispatch('start', {'_tool_name': 'battery_power'})
        with c._lock:
            c.core.update(bms(), 1, 0, 0)
            c.core.update(bms(), 2, .1, 100)
            energy = c.core.energy_wh
        old.close()  # An actual node teardown creates a new observation epoch.
        replacement = types.SimpleNamespace(
            _latest_lock=threading.Lock(), _bms_generation=0,
            node=types.SimpleNamespace(create_publisher=lambda *args: None),
            _publish=lambda *args: None, close=lambda: c.detach(replacement))
        with patch.object(self.device, '_StateNode', return_value=replacement):
            p.dispatch('start', {'_tool_name': 'battery_power'})
        self.assertIs(c.node, replacement)
        self.assertTrue(c.running)
        with patch.object(self.device.time, 'monotonic', return_value=.2):
            c.tick(old, bms(), 3, .2, 200)
            self.assertEqual(energy, c.core.energy_wh)
            c.tick(replacement, bms(), 1, .2, 200)
        self.assertEqual(energy, c.core.energy_wh)
        self.assertEqual(0, len(c.core.segments))

    def test_concurrent_start_stop_shutdown(self):
        p = self.make_plugin(real_worker=True)
        node = p._state
        errors = []

        def work():
            try:
                for _ in range(80):
                    node._on_bms(types.SimpleNamespace(**bms()))
                    p.dispatch('start', {'_tool_name': 'battery_power'})
                    p.dispatch('info', {'_tool_name': 'battery_power'})
                    p.dispatch('stop', {'_tool_name': 'battery_power'})
            except Exception as exc:
                errors.append(exc)
        threads = [threading.Thread(target=work) for _ in range(4)]
        for t in threads:
            t.start()
        p.shutdown()
        for t in threads:
            t.join(3)
            self.assertFalse(t.is_alive())
        self.assertFalse(errors)
        self.assertFalse(p._power_card.running)
        self.assertFalse(node._publisher_thread.is_alive())
        self.assertTrue(node._bms.closed)

    def test_one_hz_stale_event_logical_stop_and_physical_shutdown(self):
        p = self.make_plugin()
        node = p._state
        c = p._power_card
        p.dispatch('start', {'_tool_name': 'battery_power'})
        output = []
        with patch.object(
                node, '_publish',
                side_effect=lambda publisher, value: output.append(value)):
            for n in range(21):
                with patch.object(
                        self.device.time, 'monotonic',
                        return_value=10+n*.05):
                    c.tick(node, bms(), n+1, 10+n*.05, 10000+n*50)
            self.assertEqual(2, len(output))
            with patch.object(
                    self.device.time, 'monotonic',
                    return_value=12.01):
                c.tick(node, bms(), 21, 11, 11000)
            self.assertEqual('stale', output[-1]['status'])
            self.assertIsNone(output[-1]['remaining_runtime_minutes'])
            c.stop()
            count = len(output)
            with patch.object(
                    self.device.time, 'monotonic',
                    return_value=12.1):
                c.tick(node, bms(), 22, 12.1, 12100)
            self.assertGreater(len(output), count)
            self.assertTrue(output[-1]['fresh'])
            p.shutdown()
            count = len(output)
            c.tick(node, bms(), 23, 12.2, 12200)
            self.assertEqual(count, len(output))

    def test_real_mcp_handler_and_bundle(self):
        p = self.make_plugin()
        bundle = self.main.Bundle.__new__(self.main.Bundle)
        bundle.plugins = [p]
        handler = self.main.handler(bundle)

        def request(method, params):
            h = handler.__new__(handler)
            payload = json.dumps(
                dict(
                    jsonrpc='2.0', id=1, method=method,
                    params=params)).encode()
            h.path = '/mcp'
            h.headers = {'Content-Length': str(len(payload))}
            h.rfile = io.BytesIO(payload)
            results = []
            h._send_json = lambda status, data: results.append(data)
            h.do_POST()
            return results[0]['result']
        tools = request('tools/list', {})['tools']
        t = next(t for t in tools if t['name'] == 'battery_power')
        self.assertEqual(
            [{'topic': '/offline/state/battery_power', 'format': 'data/json'}],
            t['topic_out'])
        for action, expected in (
            ('start', 'running'),
            ('info', 'running'),
                ('stop', 'idle')):
            result = request(
                'tools/call',
                {'name': 'battery_power', 'arguments': {'action': action}})
            self.assertEqual(expected, json.loads(
                result['content'][0]['text'])['state'])
        bundle.stop_all()
        self.assertTrue(p._shutdown)

    def test_shutdown_releases_resources_if_final_publish_fails(self):
        p = self.make_plugin()
        node = p._state
        p.dispatch('start', {'_tool_name': 'battery_power'})
        with patch.object(
                node, '_publish',
                side_effect=RuntimeError('ROS already shut down')):
            p.shutdown()
        self.assertTrue(node._bms.closed)
        self.assertTrue(node.node.destroyed)
        self.assertFalse(p._power_card.running)
        self.assertIsNone(p._power_card.node)

    def make_queue_node(self):
        # Run the production callback, queue drain and worker without ROS/DDS.
        d = self.device
        node = d._StateNode.__new__(d._StateNode)
        node._latest_lock = threading.Lock()
        from collections import deque
        node._bms_samples = deque(maxlen=d._BMS_QUEUE_CAPACITY)
        node._bms_queue_overflows = 0
        node._latest_low = node._latest_bms = node._latest_sport = None
        node._bms_received_mono = node._bms_timestamp_ms = None
        node._low_generation = node._bms_generation = \
            node._sport_generation = 0
        node._published_low_generation = node._published_bms_generation = \
            node._published_sport_generation = -1
        node._stop_event = threading.Event()
        node._publisher_thread = object()
        output = []

        def publisher():
            pub = types.SimpleNamespace()
            pub.publish = lambda msg: output.append(
                (pub, json.loads(msg.data)))
            return pub
        node.node = types.SimpleNamespace(
            create_publisher=lambda *args: publisher())
        node.battery = publisher()
        node.imu = publisher()
        node.joints = publisher()
        node.joint_state = publisher()
        node._maybe_publish_odom = lambda: None
        node._maybe_publish_remote = lambda: None
        card = d._BatteryPowerCard({'capacity_wh': 600}, 'offline')
        card.bind(node)
        return node, card, output

    def test_100_to_200ms_worker_delay_preserves_185s_window(self):
        for batch_size in (2, 3, 4):
            with self.subTest(delay_ms=batch_size*50):
                node, c, output = self.make_queue_node()
                clock = [0.0]
                with patch.object(
                    self.device.time, 'monotonic',
                    side_effect=lambda: clock[0]), \
                        patch.object(
                            self.device.time, 'time',
                            side_effect=lambda: 1000+clock[0]):
                    msg = types.SimpleNamespace(**bms())
                    for n in range(3701):
                        clock[0] = n*.05
                        node._on_bms(msg)
                        node._process_battery_power()
                    self.assertEqual('available', c.core.snapshot(
                        clock[0])['runtime_status'])
                    previous_sequence = c.core.sequence
                    for n in range(1, batch_size+1):
                        clock[0] = 185+n*.05
                        node._on_bms(msg)
                    self.assertEqual(batch_size, len(node._bms_samples))
                    with patch.object(
                            c.core, 'update',
                            wraps=c.core.update) as update:
                        node._process_battery_power()
                        self.assertEqual(
                            list(
                                range(
                                    previous_sequence + 1, previous_sequence +
                                    batch_size + 1)),
                            [call.args[1] for call in update.call_args_list])
                        for n, call in enumerate(update.call_args_list, 1):
                            self.assertAlmostEqual(185+n*.05, call.args[2])
                            self.assertEqual(
                                int((1000+185+n*.05)*1000), call.args[3])
                    snapshot = c.core.snapshot(clock[0])
                    self.assertEqual('available', snapshot['runtime_status'])
                    self.assertAlmostEqual(clock[0], c.core.covered)
                    self.assertAlmostEqual(77.6, snapshot['average_power_w'])
                    self.assertAlmostEqual(
                        77.6*clock[0]/3600, c.core.energy_wh)
                    self.assertEqual(0, c.core.skipped_intervals)
                    self.assertEqual(0, node._bms_queue_overflows)

    def test_1_2s_worker_delay_preserves_window_then_expires_without_samples(
            self):
        node, c, output = self.make_queue_node()
        clock = [0.0]
        with patch.object(
            self.device.time, 'monotonic',
            side_effect=lambda: clock[0]), \
                patch.object(
                    self.device.time, 'time',
                    side_effect=lambda: 1000+clock[0]):
            msg = types.SimpleNamespace(**bms())
            for n in range(3701):
                clock[0] = n*.05
                node._on_bms(msg)
                node._process_battery_power()
            self.assertAlmostEqual(185, c.core.covered)
            self.assertEqual('available', output[-1][1]['runtime_status'])
            previous_sequence = c.core.sequence
            for n in range(1, 25):
                clock[0] = 185+n*.05
                node._on_bms(msg)
            self.assertEqual(24, len(node._bms_samples))
            self.assertEqual(0, node._bms_queue_overflows)
            node._process_battery_power()
            self.assertEqual(0, len(node._bms_samples))
            self.assertEqual(previous_sequence+24, c.core.sequence)
            self.assertEqual(0, c.core.skipped_intervals)
            self.assertAlmostEqual(186.2, c.core.covered)
            value = output[-1][1]
            self.assertTrue(value['fresh'])
            self.assertEqual('available', value['runtime_status'])
            self.assertAlmostEqual(77.6, value['average_power_w'])
            self.assertAlmostEqual(77.6*186.2/3600, c.core.energy_wh)
            energy = c.core.energy_wh
            clock[0] = 186.2+c.core.config.stale_s+.01
            # No new sample; expiry must still run.
            node._process_battery_power()
            value = output[-1][1]
            self.assertFalse(value['fresh'])
            self.assertEqual('stale', value['status'])
            self.assertEqual('stale', value['runtime_status'])
            self.assertIsNone(value['remaining_runtime_minutes'])
            self.assertEqual(0, c.core.covered)
            self.assertEqual(energy, c.core.energy_wh)

    def test_real_bms_gap_rebuilds_window_without_integrating_silence(self):
        for gap in (.3, 1.2):
            with self.subTest(gap=gap):
                node, c, output = self.make_queue_node()
                clock = [0.0]
                with patch.object(
                        self.device.time, 'monotonic',
                        side_effect=lambda: clock[0]):
                    msg = types.SimpleNamespace(**bms())
                    for n in range(3701):
                        clock[0] = n*.05
                        node._on_bms(msg)
                        node._process_battery_power()
                    self.assertEqual(
                        'available', output[-1][1]['runtime_status'])
                    energy = c.core.energy_wh
                    self.assertGreater(gap, c.core.config.max_gap_s)
                    for n in range(3):
                        clock[0] = 185+gap+n*.05
                        node._on_bms(msg)
                    node._process_battery_power()
                    self.assertEqual(0, node._bms_queue_overflows)
                    self.assertEqual(1, c.core.skipped_intervals)
                    self.assertAlmostEqual(.1, c.core.covered)
                    self.assertAlmostEqual(
                        energy+77.6*.1/3600, c.core.energy_wh)
                    self.assertEqual(
                        'warming_up', output[-1][1]['runtime_status'])

    def test_lifecycle_info_during_backlog_cannot_clear_live_window(self):
        node, card, _ = self.make_queue_node()
        clock = [0.0]
        with patch.object(self.device.time, 'monotonic',
                          side_effect=lambda: clock[0]):
            for n in range(3701):
                clock[0] = n*.05
                node._on_bms(types.SimpleNamespace(**bms()))
                node._process_battery_power()
            for n in range(1, 25):
                clock[0] = 185+n*.05
                node._on_bms(types.SimpleNamespace(**bms()))
            energy = card.core.energy_wh
            card.stop()
            self.assertEqual('stale', card.info()['diagnostics']['snapshot'][
                'status'])
            card.start()
            card.info()
            self.assertAlmostEqual(185, card.core.covered)
            self.assertEqual(energy, card.core.energy_wh)
            node._process_battery_power()
            self.assertAlmostEqual(186.2, card.core.covered)
            self.assertEqual('available', card.info()['diagnostics'][
                'snapshot']['runtime_status'])

    def test_legacy_battery_uses_latest_value_once_per_worker_pass(self):
        node, c, output = self.make_queue_node()
        clock = [10.0]
        with patch.object(
                self.device.time, 'monotonic',
                side_effect=lambda: clock[0]):
            for n in range(4):
                clock[0] = 10+n*.05
                node._on_bms(types.SimpleNamespace(
                    **bms(soc=50+n, cycle=21, temperature=[37])))
            passes = []

            def wait(_timeout):
                passes.append(True)
                if len(passes) == 1:
                    for n in range(3):
                        clock[0] = 10.2+n*.05
                        node._on_bms(types.SimpleNamespace(
                            **bms(soc=60+n, cycle=21, temperature=[37])))
                elif len(passes) == 3:
                    node._stop_event.set()
            with patch.object(node._stop_event, 'wait', side_effect=wait):
                node._publish_loop()
            legacy = [v for pub, v in output if pub is node.battery]
            self.assertEqual([53, 62], [v['soc'] for v in legacy])
            self.assertEqual(7, c.core.sequence)
            self.assertAlmostEqual(.3, c.core.covered)
            self.assertEqual(0, node._bms_queue_overflows)

    def test_failure_logging_reopens_after_ten_seconds(self):
        _, c, _ = self.make_queue_node()
        with patch.object(self.device._LOG, 'warning') as warning:
            for now in (20, 25, 29.99, 30):
                with patch.object(
                        self.device.time, 'monotonic',
                        return_value=now):
                    c._record_error(RuntimeError('injected'))
            self.assertEqual(2, warning.call_count)
            self.assertEqual(4, c.error_count)

    @unittest.skipUnless(os.getenv('AS2W_BMS_REPLAY'),
                         'real JSONL path not supplied')
    def test_real_replay_through_bounded_queue_in_batches(self):
        node, c, _ = self.make_queue_node()
        rows, reference = reference_replay(
            os.environ['AS2W_BMS_REPLAY'], c.core.config)
        direct = BatteryPower(c.core.config)
        clock = [0.0]
        wall_ms = [0]
        with patch.object(self.device.time, 'monotonic',
                          side_effect=lambda: clock[0]), \
                patch.object(self.device.time, 'time',
                             side_effect=lambda: Decimal(wall_ms[0])/1000):
            for count, row in enumerate(rows, 1):
                clock[0] = row['monotonic_ns']/1e9
                wall_ms[0] = row['timestamp_ms']
                direct.update(row['bms'], row['sample'], clock[0],
                              row['timestamp_ms'])
                node._on_bms(types.SimpleNamespace(**row['bms']))
                if count % 4 == 0:
                    node._process_battery_power()
            node._process_battery_power()
        self.assertEqual(len(rows), c.core.sequence)
        self.assertEqual(reference['skipped_intervals'],
                         c.core.skipped_intervals)
        self.assertEqual(0, node._bms_queue_overflows)
        for result in (direct, c.core):
            snapshot = result.snapshot(clock[0])
            self.assertAlmostEqual(reference['energy_wh'], result.energy_wh,
                                   places=9)
            self.assertAlmostEqual(reference['window_power_w'],
                                   snapshot['average_power_w'], places=7)
            self.assertAlmostEqual(reference['window_seconds'], result.covered,
                                   places=7)
        self.assertEqual(direct.energy_wh, c.core.energy_wh)
        self.assertEqual(direct.covered, c.core.covered)
        self.assertEqual(direct.snapshot(clock[0]), c.core.snapshot(clock[0]))
        snapshot = c.core.snapshot(clock[0])
        if reference['window_seconds'] < c.core.config.min_observation_s:
            self.assertEqual('warming_up', snapshot['runtime_status'])
            self.assertIsNone(snapshot['remaining_runtime_minutes'])
        print(
            f'QUEUED REPLAY: samples={len(rows)}, '
            f'average={snapshot["average_power_w"]:.8f}W, '
            f'energy={c.core.energy_wh:.9f}Wh; direct snapshot identical')

    def test_real_queue_overflow_breaks_missing_interval(self):
        node, c, _ = self.make_queue_node()
        clock = [0.0]
        with patch.object(
                self.device.time, 'monotonic',
                side_effect=lambda: clock[0]):
            msg = types.SimpleNamespace(**bms())
            for t in (0, .1):
                clock[0] = t
                node._on_bms(msg)
                node._process_battery_power()
            energy = c.core.energy_wh
            for n in range(1, self.device._BMS_QUEUE_CAPACITY+2):
                clock[0] = .1+n*.001
                node._on_bms(msg)
            self.assertEqual(64, len(node._bms_samples))
            self.assertEqual(1, node._bms_queue_overflows)
            node._process_battery_power()
            self.assertEqual(1, c.core.skipped_intervals)
            self.assertAlmostEqual(.063, c.core.covered)
            self.assertAlmostEqual(energy+77.6*.063/3600, c.core.energy_wh)
            self.assertEqual('warming_up', c.core.snapshot(
                clock[0])['runtime_status'])

    def test_queued_samples_survive_logical_stop_without_duplicates(self):
        node, c, _ = self.make_queue_node()
        msg = types.SimpleNamespace(**bms())
        clock = [10.0]
        with patch.object(
                self.device.time, 'monotonic',
                side_effect=lambda: clock[0]):
            node._on_bms(msg)
            pending = node._drain_bms_samples()
            c.stop()
            c.tick_samples(node, pending)
            self.assertEqual(1, c.core.sequence)
            clock[0] = 10.1
            node._on_bms(msg)
            c.start()
            # Old cache from this node is deduplicated.
            c.tick_samples(node, pending)
            node._process_battery_power()
            self.assertEqual(2, c.core.sequence)
            self.assertAlmostEqual(77.6*.1/3600, c.core.energy_wh)
            self.assertAlmostEqual(.1, c.core.covered)

    def test_old_node_queued_batch_rejected_after_rebind(self):
        old, c, _ = self.make_queue_node()
        new, _, _ = self.make_queue_node()
        old._on_bms(types.SimpleNamespace(**bms()))
        pending = old._drain_bms_samples()
        c.detach(old)
        c.bind(new)
        c.start()
        c.tick_samples(old, pending)
        self.assertIsNone(c.core.sequence)
        self.assertEqual(0, c.core.energy_wh)

    def test_faults_do_not_exit_shared_worker_and_logs_are_limited(self):
        for failure in ('calculation', 'publication'):
            with self.subTest(failure=failure):
                node, c, output = self.make_queue_node()
                low = types.SimpleNamespace(
                    imu_state=types.SimpleNamespace(quaternion=[1, 0, 0, 0]),
                    motor_state=[
                        types.SimpleNamespace(q=.1, dq=.2, tau_est=.3)]*12)
                node._latest_low = low
                node._low_generation = 1
                node._on_bms(types.SimpleNamespace(
                    **bms(cycle=21, temperature=[37])))
                iterations = []

                def next_iteration(_timeout):
                    iterations.append(True)
                    if len(iterations) >= 6:
                        node._stop_event.set()
                    else:
                        node._on_low(low)
                        node._on_bms(types.SimpleNamespace(
                            **bms(cycle=21, temperature=[37])))
                target = c.core if failure == 'calculation' else c.publisher
                method = 'update' if failure == 'calculation' else 'publish'
                with patch.object(
                    target, method,
                    side_effect=RuntimeError(failure)), \
                        patch.object(
                            node._stop_event, 'wait',
                            side_effect=next_iteration), \
                        patch.object(self.device._LOG, 'warning') as warning:
                    worker = threading.Thread(target=node._publish_loop)
                    worker.start()
                    worker.join(3)
                    self.assertFalse(worker.is_alive())
                    self.assertEqual(6, len(iterations))
                    for publisher in (node.imu, node.joints, node.joint_state):
                        self.assertEqual(
                            6, len([v for pub, v in output
                                    if pub is publisher]))
                    legacy = [v for pub, v in output if pub is node.battery]
                    self.assertEqual(6, len(legacy))
                    self.assertEqual(
                        {'soc': 50, 'current_ma': -2000.0, 'cycle': 21,
                         'temperature_0': 37.0},
                        legacy[-1])
                    self.assertGreaterEqual(c.error_count, 6)
                    self.assertEqual(1, warning.call_count)
                    self.assertEqual(0, c.core.energy_wh)
                # The same path recovers on the next good sample.
                node._on_bms(types.SimpleNamespace(**bms()))
                node._process_battery_power()
                self.assertIsNotNone(c.core.sequence)

    def test_logical_stop_keeps_fresh_publication_and_statistics(self):
        node, c, output = self.make_queue_node()
        clock = [10.0]
        with patch.object(
                self.device.time, 'monotonic',
                side_effect=lambda: clock[0]):
            for n in range(3):
                clock[0] = 10+n*.05
                node._on_bms(types.SimpleNamespace(**bms()))
                node._process_battery_power()
            count, energy, covered = len(
                output), c.core.energy_wh, c.core.covered
            c.stop()
            c.stop()
            self.assertEqual(count, len(output))  # No misleading invalidation.
            self.assertEqual(energy, c.core.energy_wh)
            self.assertEqual(covered, c.core.covered)
            self.assertEqual('idle', c.info()['state'])
            self.assertEqual('running', c.info()['stream_state'])
            for n in range(3, 24):
                clock[0] = 10+n*.05
                node._on_bms(types.SimpleNamespace(**bms()))
                node._process_battery_power()
            self.assertGreater(len(output), count)
            self.assertTrue(output[-1][1]['fresh'])
            self.assertEqual(99, output[-1][1]['soh_percent'])
            self.assertGreater(c.core.energy_wh, energy)
            self.assertAlmostEqual(1.15, c.core.covered)

    def test_expired_soh_is_null_while_raw_diagnostic_is_retained(self):
        core = BatteryPower()
        core.update(bms(), 1, 10, 10000)
        self.assertEqual(99, core.snapshot(10)['soh_percent'])
        self.assertIsNone(core.snapshot(11.01)['soh_percent'])
        self.assertEqual(99, core.diagnostics(11.01)['raw']['soh_percent'])

    def test_metadata_and_docker(self):
        self.assertIn('device.py battery_power.py', (self.root /
                      'Dockerfile').read_text(encoding='utf-8'))
        self.assertIn('name: battery_power, type: sensor',
                      (self.root/'driver.yaml').read_text(encoding='utf-8'))
        self.assertIn('capacity_wh: 648.0', (self.root /
                      'config.yaml').read_text(encoding='utf-8'))
        self.assertIn('battery_power_config=p.get("battery_power")',
                      (self.root/'main.py').read_text(encoding='utf-8'))


if __name__ == '__main__':
    unittest.main()
