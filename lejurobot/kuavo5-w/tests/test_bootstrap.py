import base64
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import bootstrap
spec = importlib.util.spec_from_file_location('kuavo_bootstrap_remote', ROOT / 'host/bootstrap_remote.py')
remote = importlib.util.module_from_spec(spec)
spec.loader.exec_module(remote)


def encoded(value):
    return base64.b64encode(value.encode()).decode()


class BootstrapTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.env = patch.dict(os.environ, {'KUAVO_STATE_DIR': str(self.directory),
                                          'KUAVO_SSH_SECRETS': str(self.directory / 'ssh-secrets.json'),
                                          'CONFIG_PATH': str(self.directory / 'config.yaml')})
        self.env.start()
        self.addCleanup(self.env.stop)

    def test_secret_file_requires_private_regular_file(self):
        path = self.directory / 'secrets'
        self.assertEqual(bootstrap.load_secrets(path), {})
        path.write_text('{"controller":{"password":"   "}}')
        path.chmod(0o644)
        with self.assertRaises(bootstrap.ProvisionError):
            bootstrap.load_secrets(path)
        path.chmod(0o600)
        self.assertEqual(bootstrap.load_secrets(path)['controller']['password'], '   ')
        link = self.directory / 'link'
        link.symlink_to(path)
        with self.assertRaises(bootstrap.ProvisionError):
            bootstrap.load_secrets(link)

    def test_password_never_in_argv_and_first_use_keys_are_pinned(self):
        target = dict(bootstrap.DEFAULTS['controller'])
        result = types.SimpleNamespace(returncode=0, stdout=b'{"ok":true,"state":"ready"}')
        with patch.object(bootstrap.subprocess, 'run', return_value=result) as run:
            bootstrap.SshTransport(self.directory).run('controller', target, {'role': 'controller'},
                'print(1)', {'controller': {'password': '   ', 'sudo_password': 'private-sudo'}})
        args, kwargs = run.call_args.args[0], run.call_args.kwargs
        self.assertEqual(args[:3], ['sshpass', '-e', 'ssh'])
        self.assertNotIn('private-sudo', ' '.join(args))
        self.assertIn('StrictHostKeyChecking=accept-new', args)
        self.assertEqual(kwargs['env']['SSHPASS'], '   ')
        self.assertEqual(json.loads(kwargs['input'])['_sudo_password'], 'private-sudo')

    def test_env_empty_defaults_do_not_mask_secret_file(self):
        result = types.SimpleNamespace(returncode=0, stdout=b'{"ok":true}')
        with patch.dict(os.environ, {'KUAVO_HOST_SSH_PASSWORD': ''}), \
             patch.object(bootstrap.subprocess, 'run', return_value=result) as run:
            bootstrap.SshTransport(self.directory).run('host', bootstrap.DEFAULTS['host'], {}, '',
                                                      {'host': {'password': 'test-secret'}})
        self.assertEqual(run.call_args.kwargs['env']['SSHPASS'], 'test-secret')

    def test_commissioned_defaults_work_without_operator_secret_file(self):
        result = types.SimpleNamespace(returncode=0, stdout=b'{"ok":true}')
        for role in bootstrap.DEFAULTS:
            with patch.object(bootstrap.subprocess, 'run', return_value=result) as run:
                bootstrap.SshTransport(self.directory).run(role, bootstrap.DEFAULTS[role], {}, '', {})
            self.assertEqual(run.call_args.kwargs['env']['SSHPASS'], bootstrap.DEFAULT_PASSWORDS[role])
            self.assertEqual(json.loads(run.call_args.kwargs['input'])['_sudo_password'],
                             bootstrap.DEFAULT_PASSWORDS[role])

    def test_explicit_key_or_disabled_default_uses_batch_public_key_authentication(self):
        result = types.SimpleNamespace(returncode=0, stdout=b'{"ok":true}')
        for target, secrets in [(bootstrap.DEFAULTS['host'], {'host': {'key_file': '/config/id_test'}}),
                                ({**bootstrap.DEFAULTS['host'], 'use_default_credentials': False}, {})]:
            with patch.object(bootstrap.subprocess, 'run', return_value=result) as run:
                bootstrap.SshTransport(self.directory).run('host', target, {}, '', secrets)
            self.assertEqual(run.call_args.args[0][0], 'ssh')
            self.assertIn('BatchMode=yes', run.call_args.args[0])

    def test_ssh_failure_never_echoes_peer_output_or_password(self):
        with patch.object(bootstrap.subprocess, 'run', return_value=types.SimpleNamespace(
                returncode=255, stdout=b'', stderr=b'private-password\nremote log injection')):
            with self.assertRaisesRegex(bootstrap.ProvisionError, '^ssh_auth_host_key_or_remote_access_failed$'):
                bootstrap.SshTransport(self.directory).run('host', bootstrap.DEFAULTS['host'], {}, '', {})

    def test_remote_error_accepts_only_bounded_code(self):
        for response in ({'ok': False, 'error': 'secret\ncontent'}, {'ok': False, 'error': 'x' * 181}):
            with patch.object(bootstrap.subprocess, 'run', return_value=types.SimpleNamespace(
                    returncode=0, stdout=json.dumps(response).encode())):
                with self.assertRaisesRegex(bootstrap.ProvisionError, '^remote_provisioning_failed$'):
                    bootstrap.SshTransport(self.directory).run('host', bootstrap.DEFAULTS['host'], {}, '', {})

    def test_host_and_user_shell_injection_rejected_before_ssh(self):
        for target in ({'address': 'host;id', 'user': 'user'}, {'address': '-bad', 'user': 'user'},
                       {'address': 'localhost', 'user': 'user$(id)'}):
            with patch.object(bootstrap.subprocess, 'run') as run:
                with self.assertRaises(bootstrap.ProvisionError):
                    bootstrap.SshTransport(self.directory).run('host', target, {}, '', {})
                run.assert_not_called()

    def test_auto_provision_delivers_controller_overlay_to_upper_host(self):
        transport = Mock()
        messages = {'kuavo_msgs/msg/__init__.py': encoded('actual controller fixture')}
        transport.run.side_effect = lambda role, *args: dict(ok=True, state='ready', changed=False,
                                                            **({'messages': messages} if role == 'controller' else {}))
        manager = bootstrap.AutoProvisioner(transport=transport)
        manager.once()
        self.assertEqual([c.args[0] for c in transport.run.call_args_list], ['controller', 'host', 'chassis'])
        host_request = transport.run.call_args_list[1].args[2]
        self.assertEqual(host_request['messages'], messages)
        self.assertEqual(set(host_request['files']), set(bootstrap.FILES))
        self.assertEqual(manager.snapshot()['state'], 'ready')
        self.assertNotIn('fixture', json.dumps(manager.snapshot()))

    def test_speaker_recovery_priority_and_disabled_role(self):
        transport = Mock()
        transport.run.return_value = {'ok': True, 'state': 'ready'}
        manager = bootstrap.AutoProvisioner({'host': {'enabled': False}}, transport=transport)
        manager.request('chassis')
        manager._quick_chassis.join(timeout=1)
        manager.once()
        self.assertEqual([c.args[0] for c in transport.run.call_args_list], ['chassis', 'controller', 'chassis'])
        self.assertEqual(manager.snapshot()['targets']['host']['state'], 'disabled')

    def test_audio_recovery_does_not_queue_behind_control_host_work(self):
        entered, release, chassis = threading.Event(), threading.Event(), threading.Event()
        def call(role, *args):
            if role == 'controller':
                entered.set()
                release.wait(2)
            if role == 'chassis':
                chassis.set()
            return {'ok': True, 'state': 'ready'}
        manager = bootstrap.AutoProvisioner({'host': {'enabled': False}}, transport=types.SimpleNamespace(run=call))
        worker = threading.Thread(target=manager.once)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            manager.request('chassis')
            self.assertTrue(chassis.wait(1))
        finally:
            release.set()
            worker.join(timeout=2)
            manager.close()

    def test_disabled_automatic_provisioning_cannot_be_reenabled_by_card_start(self):
        transport = Mock()
        manager = bootstrap.AutoProvisioner({'enabled': False}, transport=transport)
        manager.request('chassis')
        self.assertIsNone(manager._quick_chassis)
        transport.run.assert_not_called()

    def test_one_host_failure_does_not_skip_other_hosts(self):
        transport = Mock()
        def run(role, *args):
            if role == 'controller':
                raise bootstrap.ProvisionError('ssh_operation_timeout')
            return {'ok': True, 'state': 'ready'}
        transport.run.side_effect = run
        manager = bootstrap.AutoProvisioner(transport=transport)
        manager.once()
        self.assertEqual(transport.run.call_count, 3)
        self.assertEqual(manager.snapshot()['targets']['controller']['state'], 'blocked')
        self.assertEqual(manager.snapshot()['targets']['chassis']['state'], 'ready')

    def test_card_requests_do_not_hammer_failed_ssh_credentials(self):
        transport = Mock()
        transport.run.side_effect = bootstrap.ProvisionError('ssh_auth_host_key_or_remote_access_failed')
        manager = bootstrap.AutoProvisioner(transport=transport)
        manager.once()
        manager.request('chassis')
        manager.once()
        self.assertEqual(transport.run.call_count, 3)
        with patch.object(bootstrap.time, 'monotonic', return_value=bootstrap.time.monotonic() + 31):
            manager.once()
        self.assertEqual(transport.run.call_count, 6)

    def test_packaged_connection_defaults_are_not_shadowed_by_yaml(self):
        import yaml
        settings = yaml.safe_load((ROOT / 'config.yaml').read_text())['provisioning']
        manager = bootstrap.AutoProvisioner(settings)
        for role in bootstrap.DEFAULTS:
            self.assertNotIn('address', settings.get(role, {}))
            self.assertNotIn('user', settings.get(role, {}))
            self.assertNotIn('ros_ip', settings.get(role, {}))
            updated = {**bootstrap.DEFAULTS[role], 'address': '192.0.2.10', 'user': 'operator'}
            with patch.dict(bootstrap.DEFAULTS, {role: updated}):
                self.assertEqual(manager._target(role)['address'], '192.0.2.10')
                self.assertEqual(manager._target(role)['user'], 'operator')


class RemoteTests(unittest.TestCase):
    def test_fresh_chassis_enables_user_audio_without_desktop_login(self):
        spec = importlib.util.spec_from_file_location('kuavo_install_speaker', ROOT / 'host/install_speaker.py')
        installer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(installer)
        with patch.object(installer.subprocess, 'run') as run:
            installer.ensure_user_audio('operator', 1001)
        commands = [call.args[0] for call in run.call_args_list]
        self.assertIn(['loginctl', 'enable-linger', 'operator'], commands)
        self.assertIn(['systemctl', 'start', 'user@1001.service'], commands)
        self.assertEqual(commands[-1][-5:], ['systemctl', '--user', 'enable', '--now', 'pulseaudio.socket'])
        self.assertIn('DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1001/bus', commands[-1])

    def test_controller_already_running_is_never_restarted(self):
        with patch.object(remote, 'run', return_value=types.SimpleNamespace(stdout=b'12345\nactive\n')) as run:
            self.assertEqual(remote.activate_controller(), 'already_running_not_restarted')
            self.assertEqual(run.call_count, 1)

    def test_controller_failure_or_previous_attempt_cannot_auto_retry(self):
        for output in (b'3456\ninactive\n', b'0\nfailed\n'):
            with patch.object(remote, 'run', return_value=types.SimpleNamespace(stdout=output)) as run:
                with self.assertRaisesRegex(remote.Blocked, 'no_retry'):
                    remote.activate_controller()
                self.assertEqual(run.call_count, 1)

    def test_first_missing_controller_can_start_only_once_per_boot(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(remote, 'run',
                return_value=types.SimpleNamespace(stdout=b'0\ninactive\n')) as run:
            self.assertEqual(remote.activate_controller(Path(directory)), 'initialization_requested_once')
            self.assertIn(['systemctl', 'start', '--no-block', 'kuavo5w-controller.service'],
                          [call.args[0] for call in run.call_args_list])
            with self.assertRaisesRegex(remote.Blocked, 'no_retry'):
                remote.activate_controller(Path(directory))

    def test_invalid_message_paths_and_missing_generated_types_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            for messages in (None, {}, {'kuavo_msgs/../evil.py': encoded('bad')},
                             {'/tmp/evil.py': encoded('bad')}, {'kuavo_msgs/msg/a.py': encoded('incomplete')}):
                with self.assertRaises(remote.Blocked):
                    remote.write_messages(Path(directory), messages)

    def test_controller_message_export_uses_only_python_no_architecture_binaries(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / 'devel/lib/python3/dist-packages/kuavo_msgs'
            for name in ('msg', 'srv'):
                (package / name).mkdir(parents=True)
                (package / name / '__init__.py').write_text('fixture')
            (package / 'native.so').write_bytes(b'not portable')
            exported = remote.export_messages(root)
            self.assertEqual(len(exported), 2)
            self.assertFalse(any(name.endswith('.so') for name in exported))

    def test_complete_image_payload_and_invalid_role_checked(self):
        with self.assertRaisesRegex(remote.Blocked, 'invalid_role'):
            remote.install({'role': 'arbitrary', 'target': {}})

    def test_catkin_private_links_inside_workspace_export_as_plain_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / 'devel/lib/python3/dist-packages/kuavo_msgs'
            private = root / 'devel/.private/kuavo_msgs'
            for name in ('msg', 'srv'):
                (private / name).mkdir(parents=True)
                (private / name / '__init__.py').write_text('generated ' + name)
                (package / name).mkdir(parents=True)
                (package / name / '__init__.py').symlink_to(private / name / '__init__.py')
            self.assertEqual(len(remote.export_messages(root)), 2)
            outside = root.parent / (root.name + '-outside.py')
            try:
                outside.write_text('outside workspace')
                (package / 'msg/_outside.py').symlink_to(outside)
                with self.assertRaisesRegex(remote.Blocked, 'outside_workspace'):
                    remote.export_messages(root)
            finally:
                outside.unlink(missing_ok=True)

    def test_disabling_initialization_only_disables_future_boot_never_stops_live_controller(self):
        with tempfile.TemporaryDirectory() as directory:
            units = Path(directory)
            (units / 'kuavo5w-controller.service').write_text('owned')
            with patch.object(remote, 'UNIT_DIR', units), \
                 patch.object(remote.pwd, 'getpwnam', return_value=types.SimpleNamespace(pw_dir=directory)), \
                 patch.object(remote, 'discover_workspace', return_value=Path(directory)), \
                 patch.object(remote, 'export_messages', return_value={}), \
                 patch.object(remote, 'run') as run:
                result = remote.install({'role': 'controller', 'target': {'user': 'test', 'ros_ip': '192.0.2.1'},
                    'master': 'http://192.0.2.1:11311', 'auto_initialize_and_pause': False})
                self.assertEqual(result['state'], 'disabled')
                self.assertEqual(run.call_args.args[0], ['systemctl', 'disable', 'kuavo5w-controller.service'])
                self.assertEqual(run.call_count, 1)

    def test_missing_upper_helpers_are_installed_with_controller_overlay_and_on_demand_head(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            roots = {key: base / key for key in remote.ROOTS}
            units = base / 'units'
            units.mkdir()
            messages = {'kuavo_msgs/' + part + '/__init__.py': encoded('controller wire fixture') for part in ('msg', 'srv')}
            request = {'role': 'host', 'target': {'user': 'test', 'ros_ip': '192.0.2.1'},
                'master': 'http://192.0.2.2:11311', 'files': {name: encoded(name) for name in remote.FILES},
                'messages': messages}
            commands = []
            def execute(command, **kwargs):
                commands.append(command)
                if command[0] == 'python3':
                    stage = Path(command[1]).parent
                    root = roots['host']
                    root.mkdir(exist_ok=True)
                    for name in remote.OWNED['host']:
                        (root / name).write_bytes((stage / name).read_bytes())
                    remote.write_messages(root / 'ros1_messages', messages)
                    for name in ('runtime-manager', 'rosbridge', 'battery-relay', 'head-camera', 'head-jpeg-relay', 'head-depth-relay'):
                        (units / ('kuavo5w-' + name + '.service')).write_text('fixture unit')
                return types.SimpleNamespace(stdout=b'active\n', returncode=0)
            with patch.dict(remote.ROOTS, roots), patch.object(remote, 'UNIT_DIR', units), \
                 patch.object(remote.pwd, 'getpwnam', return_value=types.SimpleNamespace(pw_dir=str(base))), \
                 patch.object(remote, 'discover_workspace', return_value=base), \
                 patch.object(remote, 'head_workspace', return_value=base), \
                 patch.object(remote, 'dependencies'), patch.object(remote, 'run', side_effect=execute):
                self.assertTrue(remote.install(request)['changed'])
                self.assertFalse(remote.install(request)['changed'])
                self.assertEqual(len([c for c in commands if c[0] == 'python3']), 1)
                starts = [c[-1] for c in commands if c[:3] == ['systemctl', 'start', '--no-block']]
                self.assertFalse(any('head' in name for name in starts))
                installed = roots['host'] / 'ros1_messages/kuavo_msgs/msg/__init__.py'
                installed.write_text('wrong MD5 fixture')
                self.assertTrue(remote.install(request)['changed'])
                self.assertEqual(installed.read_text(), 'controller wire fixture')

    def test_fresh_install_repeat_and_deleted_helper_repair(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            roots = {key: base / key for key in remote.ROOTS}
            units = base / 'units'
            units.mkdir()
            request = {'role': 'chassis', 'target': {'user': 'test', 'ros_ip': '192.0.2.1'},
                'master': 'http://192.0.2.2:11311', 'files': {name: encoded(name) for name in remote.FILES}}
            def execute(command, **kwargs):
                if command[0] == 'python3':
                    roots['chassis'].mkdir(exist_ok=True)
                    source = Path(command[1]).with_name('speaker_relay.py')
                    (roots['chassis'] / 'speaker_relay.py').write_bytes(source.read_bytes())
                    (units / 'kuavo5w-speaker.service').write_text('generated-unit')
                return types.SimpleNamespace(stdout=b'loaded\n' if command[:3] == ['systemctl', 'show', 'jaten.service'] else b'active\n', returncode=0)
            with patch.dict(remote.ROOTS, roots), patch.object(remote, 'UNIT_DIR', units), \
                 patch.object(remote.pwd, 'getpwnam', return_value=types.SimpleNamespace(pw_dir=str(base))), \
                 patch.object(remote, 'dependencies'), patch.object(remote, 'run', side_effect=execute) as run:
                first = remote.install(request)
                second = remote.install(request)
                self.assertTrue(first['changed'])
                self.assertFalse(second['changed'])
                self.assertEqual(len([c for c in run.call_args_list if c.args[0][0] == 'python3']), 1)
                (roots['chassis'] / 'speaker_relay.py').unlink()
                self.assertTrue(remote.install(request)['changed'])
                (units / 'kuavo5w-speaker.service').write_text('corrupted-unit')
                self.assertTrue(remote.install(request)['changed'])
                self.assertEqual((units / 'kuavo5w-speaker.service').read_text(), 'generated-unit')
                def inactive(command, **kwargs):
                    if command[:2] == ['systemctl', 'is-active']:
                        return types.SimpleNamespace(stdout=b'inactive\n', returncode=3)
                    return execute(command, **kwargs)
                with patch.object(remote, 'run', side_effect=inactive):
                    self.assertEqual(remote.install(request)['state'], 'starting')

    def test_vendor_chassis_services_are_started_idempotently_not_restarted(self):
        with patch.object(remote, 'run', return_value=types.SimpleNamespace(stdout=b'loaded\n')) as run:
            remote.ensure_vendor_base()
            self.assertEqual(run.call_args.args[0], ['systemctl', 'start', '--no-block', 'jaten.service'])
        with patch.object(remote, 'run', return_value=types.SimpleNamespace(stdout=b'not-found\n')):
            with self.assertRaisesRegex(remote.Blocked, 'missing_vendor_jaten_service'):
                remote.ensure_vendor_base()


if __name__ == '__main__':
    unittest.main()
