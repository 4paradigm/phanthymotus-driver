"""Image-owned provisioning with replaceable commissioned connection defaults.

Private overrides come from environment or a mode-0600 file, never argv.
OpenSSH pins first-use host keys in the driver's persistent configuration directory.
"""
import base64
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import threading
import time
from connection_defaults import DEFAULTS, DEFAULT_PASSWORDS

ROOT = Path(__file__).resolve().parent
FILES = ('config.yaml', 'host/install.py', 'host/install_controller.py',
         'host/controller_boot.py', 'host/install_speaker.py', 'host/speaker_relay.py',
         'host/rosbridge_loopback.py', 'host/rosbridge.launch', 'host/runtime_manager.py',
         'host/battery_relay.py', 'host/head_camera.launch', 'host/head_jpeg.py', 'host/head_depth.py')



class ProvisionError(RuntimeError):
    pass


def load_secrets(path):
    path = Path(path)
    if not path.exists():
        return {}
    if path.is_symlink() or not path.is_file() or path.stat().st_mode & 0o077 or path.stat().st_size > 16384:
        raise ProvisionError('secrets_require_regular_mode_0600_file')
    result = json.loads(path.read_text())
    if not isinstance(result, dict):
        raise ProvisionError('invalid_secret_file')
    return result


class SshTransport:
    def __init__(self, state_dir):
        self.state_dir = Path(state_dir)

    def run(self, role, target, request, source, secrets):
        address, user = target['address'], target['user']
        if not re.fullmatch(r'[A-Za-z0-9_.:-]+', address) or address.startswith('-'):
            raise ProvisionError('invalid_ssh_address')
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]*', user):
            raise ProvisionError('invalid_ssh_user')
        self.state_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        known_hosts = self.state_dir / 'known_hosts'
        known_hosts.touch(mode=0o600, exist_ok=True)
        if known_hosts.is_symlink() or known_hosts.stat().st_mode & 0o077:
            raise ProvisionError('insecure_known_hosts_file')
        role_secret = secrets.get(role, {})
        if not isinstance(role_secret, dict):
            raise ProvisionError('invalid_role_secret')
        prefix = 'KUAVO_' + role.upper()
        key = os.environ.get(prefix + '_SSH_KEY') or role_secret.get('key_file')
        password = os.environ.get(prefix + '_SSH_PASSWORD') or role_secret.get('password')
        if password is None and not key and target.get('use_default_credentials', True):
            password = DEFAULT_PASSWORDS[role]
        sudo_password = os.environ.get(prefix + '_SUDO_PASSWORD') or role_secret.get('sudo_password', password)
        if any(value is not None and (not isinstance(value, str) or '\x00' in value or '\n' in value)
               for value in (password, sudo_password, key)):
            raise ProvisionError('invalid_ssh_secret')
        # Do not strip: some deployed controllers use a spaces-only password.
        env = dict(os.environ)
        command = ['ssh', '-o', 'ConnectTimeout=5', '-o', 'ConnectionAttempts=1',
                   '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2',
                   '-o', 'StrictHostKeyChecking=accept-new',
                   '-o', 'UserKnownHostsFile=' + str(known_hosts),
                   '-o', 'NumberOfPasswordPrompts=1', '-o', 'LogLevel=ERROR']
        if password is not None:
            env['SSHPASS'] = password
            command = ['sshpass', '-e', *command, '-o', 'PreferredAuthentications=password',
                       '-o', 'PubkeyAuthentication=no']
        else:
            command += ['-o', 'BatchMode=yes']
        if key:
            command += ['-i', key, '-o', 'IdentitiesOnly=yes']
        encoded = base64.b64encode(source.encode()).decode()
        launcher = "import base64; BOOTSTRAP_SOURCE=base64.b64decode(%r).decode(); exec(compile(BOOTSTRAP_SOURCE,'bootstrap_remote.py','exec'))" % encoded
        command += [f'{user}@{address}', 'python3 -c ' + shlex.quote(launcher)]
        payload = dict(request, _sudo_password=sudo_password)
        try:
            result = subprocess.run(command, input=json.dumps(payload).encode(), capture_output=True,
                                    timeout=240, env=env)
        except subprocess.TimeoutExpired:
            raise ProvisionError('ssh_operation_timeout') from None
        except FileNotFoundError:
            raise ProvisionError('image_missing_openssh_or_sshpass') from None
        # Never report command/stdout/stderr: they can contain secrets or a
        # remote peer's arbitrary content. Return only the bounded structured result.
        if result.returncode:
            raise ProvisionError('ssh_auth_host_key_or_remote_access_failed')
        if len(result.stdout) > 4_000_000:
            raise ProvisionError('oversized_remote_response')
        try:
            response = json.loads(result.stdout)
        except (ValueError, UnicodeError):
            raise ProvisionError('invalid_remote_response') from None
        if not isinstance(response, dict) or response.get('ok') is not True:
            error = response.get('error') if isinstance(response, dict) else None
            if not isinstance(error, str) or not re.fullmatch(r'[A-Za-z0-9_./:-]{1,180}', error):
                error = 'remote_provisioning_failed'
            raise ProvisionError(error)
        return response


class AutoProvisioner:
    def __init__(self, settings=None, transport=None):
        self.settings = settings or {}
        self.enabled = self.settings.get('enabled', True)
        self.state_dir = Path(os.environ.get('KUAVO_STATE_DIR', '/config'))
        self.transport = transport or SshTransport(self.state_dir)
        self.status = {'state': 'waiting' if self.enabled else 'disabled', 'targets': {}}
        self._closed = threading.Event()
        self._wake = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._retry_at = {}
        self._results = {}
        self._role_locks = {role: threading.Lock() for role in DEFAULTS}
        self._quick_chassis = None

    def start(self):
        if self.enabled:
            self._thread = threading.Thread(target=self._run, daemon=True, name='kuavo-provision')
            self._thread.start()

    def snapshot(self):
        with self._lock:
            return json.loads(json.dumps(self.status))

    def _target(self, role):
        config = {**DEFAULTS[role], **self.settings.get(role, {})}
        prefix = 'KUAVO_' + role.upper()
        for field in ('address', 'user', 'ros_ip', 'workspace'):
            config[field] = os.environ.get(prefix + '_' + field.upper(), config.get(field))
        return config

    def once(self):
        source = (ROOT / 'host/bootstrap_remote.py').read_text()
        files = {name: base64.b64encode((ROOT / name).read_bytes()).decode() for name in FILES}
        secrets = load_secrets(os.environ.get('KUAVO_SSH_SECRETS', str(self.state_dir / 'ssh-secrets.json')))
        targets, messages = {}, None
        # The bridge must use THIS controller's generated messages, not the
        # upper application workspace's potentially incompatible kuavo_msgs.
        for role in ('controller', 'host', 'chassis'):
            if self._closed.is_set():
                return
            target = self._target(role)
            if target.get('enabled', True) is False:
                targets[role] = {'state': 'disabled'}
                continue
            with self._lock:
                if time.monotonic() < self._retry_at.get(role, 0):
                    targets[role] = dict(self._results[role])
                    continue
            request = {'role': role, 'target': target, 'files': files,
                       'master': self.settings.get('master', 'http://192.168.26.1:11311'),
                       'install_missing_packages': self.settings.get('install_missing_packages', True),
                       'auto_initialize_and_pause': self.settings.get('auto_initialize_and_pause', True)}
            if role == 'host':
                request['messages'] = messages
            with self._lock:
                self.status = {'state': 'provisioning', 'targets': {**targets, role: {'state': 'checking'}}}
            try:
                with self._role_locks[role]:
                    response = self.transport.run(role, target, request, source, secrets)
                if role == 'controller':
                    messages = response.get('messages')
                # No passwords, blobs or unbounded vendor metadata in telemetry.
                targets[role] = {k: response[k] for k in ('state', 'changed', 'workspace',
                    'head_workspace', 'services', 'controller_activation') if k in response}
            except ProvisionError as exc:
                targets[role] = {'state': 'blocked', 'error': str(exc)}
            with self._lock:
                self._results[role] = dict(targets[role])
                self._retry_at[role] = time.monotonic() + 30 if targets[role]['state'] == 'blocked' else 0
                self.status = {'state': 'ready' if len(targets) == 3 and all(
                    t['state'] in ('ready', 'disabled') for t in targets.values()) else 'provisioning',
                    'targets': dict(targets)}
        with self._lock:
            self.status = {'state': 'ready' if all(t['state'] in ('ready', 'disabled')
                for t in targets.values()) else 'blocked', 'targets': dict(targets)}

    def _run(self):
        while not self._closed.is_set():
            try:
                self.once()
            except Exception as exc:
                with self._lock:
                    self.status = {'state': 'blocked', 'error': type(exc).__name__}
            self._wake.wait(30)
            self._wake.clear()

    def request(self, role=None):
        if not self.enabled:
            return
        if role == 'chassis':
            with self._lock:
                if (self._quick_chassis is not None and self._quick_chassis.is_alive()
                        or time.monotonic() < self._retry_at.get(role, 0)):
                    return
                self._quick_chassis = threading.Thread(target=self._recover_chassis,
                    daemon=True, name='kuavo-speaker-ensure')
                self._quick_chassis.start()
            return
        self._wake.set()

    def _recover_chassis(self):
        # Audio start cannot queue behind a large controller message export or
        # upper-host provisioning. Only this non-motor role gets an independent
        # on-demand worker; same-host installs remain serialized.
        target = self._target('chassis')
        if target.get('enabled', True) is False or self._closed.is_set():
            return
        try:
            source = (ROOT / 'host/bootstrap_remote.py').read_text()
            files = {name: base64.b64encode((ROOT / name).read_bytes()).decode() for name in FILES}
            secrets = load_secrets(os.environ.get('KUAVO_SSH_SECRETS', str(self.state_dir / 'ssh-secrets.json')))
            request = {'role': 'chassis', 'target': target, 'files': files,
                       'master': self.settings.get('master', 'http://192.168.26.1:11311'),
                       'install_missing_packages': self.settings.get('install_missing_packages', True)}
            with self._role_locks['chassis']:
                response = self.transport.run('chassis', target, request, source, secrets)
            result = {k: response[k] for k in ('state', 'changed', 'services') if k in response}
        except Exception as exc:
            result = {'state': 'blocked', 'error': str(exc) if isinstance(exc, ProvisionError) else type(exc).__name__}
        with self._lock:
            self._results['chassis'] = result
            self._retry_at['chassis'] = time.monotonic() + 30 if result['state'] == 'blocked' else 0
            targets = {**self.status.get('targets', {}), 'chassis': result}
            self.status = {'state': 'ready' if len(targets) == 3 and all(t['state'] in ('ready', 'disabled')
                           for t in targets.values()) else 'provisioning', 'targets': targets}

    def close(self):
        self._closed.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout=1)
        if self._quick_chassis:
            self._quick_chassis.join(timeout=1)
