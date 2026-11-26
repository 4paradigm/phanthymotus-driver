"""Fixed native ROS operations, executed over authenticated SSH on the chassis.

Uses the factory-generated classes on THAT robot; no copied message ABI, daemon,
network listener, workspace modification or downloaded SDK is required.
"""
import json
import hashlib
from pathlib import Path
import math
import re
import os
import sys
import time
import base64
import fcntl
import shutil
import urllib.request
from contextlib import contextmanager
import stat

STOP_EPOCH_PATH = '/tmp/motus-kuavo-native-stop-epoch'


@contextmanager
def stop_epoch_file(exclusive=False):
    fd = os.open(STOP_EPOCH_PATH, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise RuntimeError('native_navigation_stop_epoch_insecure')
        fcntl.flock(fd, fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH)
        with os.fdopen(fd, 'r+', closefd=False) as stream:
            yield stream
    finally:
        os.close(fd)


def stop_epoch(advance=False):
    with stop_epoch_file(advance) as stream:
        value = int(stream.read() or '0')
        if advance:
            value += 1
            stream.seek(0)
            stream.write(str(value))
            stream.truncate()
            stream.flush()
        return value


def guarded_navigation_release(expected_epoch, release):
    # A stop increments under an exclusive lock before dispatching its hold.
    # It either prevents this release, or follows an already-started release.
    with stop_epoch_file() as stream:
        if int(stream.read() or '0') != expected_epoch:
            raise RuntimeError('native_navigation_superseded_by_stop')
        return release()


def map_archive_read(operation, args, root=Path('/home/jt/jmap_config/jmaps')):
    name = args.get('map_name')
    if (not isinstance(name, str) or not name.strip() or len(name) > 80 or name in ('.', '..')
            or any(c in name for c in '/\\') or any(ord(c) < 32 for c in name)):
        raise ValueError('invalid_map_name')
    if map_save_status(name, root).get('complete') is not True:
        raise RuntimeError('map_save_not_complete_for_archive')
    suffixes = ('.pgm', '.yaml', '.pbstream', '.path')
    def path(suffix):
        p = root / (name+suffix)
        if p.is_symlink() or not p.is_file() or not 0 < p.stat().st_size <= 512*1024*1024:
            raise RuntimeError('invalid_map_archive_source')
        return p
    if operation == 'map_archive_manifest':
        files = {}
        for suffix in suffixes:
            p = path(suffix)
            before = p.stat()
            digest = hashlib.sha256()
            with p.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024*1024), b''):
                    digest.update(chunk)
            after = p.stat()
            if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                raise RuntimeError('map_changed_during_archive')
            files[suffix] = {'size': after.st_size, 'mtime_ns': after.st_mtime_ns, 'sha256': digest.hexdigest()}
        return {'map_name': name, 'files': files}
    suffix, offset, expected = args.get('suffix'), args.get('offset'), args.get('expected', {})
    if suffix not in suffixes or type(offset) is not int or offset < 0 or offset % (1024*1024):
        raise ValueError('invalid_map_archive_chunk')
    p = path(suffix)
    stat = p.stat()
    if (stat.st_size != expected.get('size') or stat.st_mtime_ns != expected.get('mtime_ns')
            or offset >= stat.st_size):
        raise RuntimeError('map_changed_during_archive')
    with p.open('rb') as stream:
        stream.seek(offset)
        data = stream.read(min(1024*1024, stat.st_size-offset))
    return {'data': base64.b64encode(data).decode('ascii')}


_MAP_FILES = ('.yaml', '.pbstream', '.path', '.pgm')
_MAP_CHUNK = 1024 * 1024


def _assert_inactive_map(name):
    request = 'http://127.0.0.1:8888/map/current?returnMapData=false'
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(request, timeout=5) as response:
        result = json.loads(response.read(65536))
    current = result.get('data')
    if result.get('success') is not True or not isinstance(current, dict) or not current.get('mapId'):
        raise RuntimeError('current_map_identity_unavailable')
    if current['mapId'] == name:
        raise ValueError('cannot_modify_active_map')


def map_archive_write(operation, args, root=Path('/home/jt/jmap_config/jmaps'),
                      assert_inactive=_assert_inactive_map):
    name, token = args.get('map_name'), args.get('token')
    if (not isinstance(name, str) or not name.strip() or len(name) > 80 or name in ('.', '..')
            or any(c in name for c in '/\\') or any(ord(c) < 32 for c in name)):
        raise ValueError('invalid_map_name')
    if not isinstance(token, str) or not re.fullmatch(r'[a-f0-9]{32}', token):
        raise ValueError('invalid_map_transfer_id')

    # Private staging/quarantine on the same filesystem as the factory maps.
    work = root / '.motus-map-transfers'
    work.mkdir(mode=0o700, exist_ok=True)
    if work.is_symlink() or work.stat().st_mode & 0o077:
        raise RuntimeError('invalid_map_transfer_directory')
    stage = work / (('deleted-' if operation == 'map_delete' else 'upload-') + token)
    live = {suffix: root / (name + suffix) for suffix in _MAP_FILES}
    with (work / 'lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if operation == 'map_upload_begin':
            assert_inactive(name)
            if any(os.path.lexists(path) for path in live.values()):
                raise ValueError('map_name_already_exists')
            stage.mkdir(mode=0o700)
        elif operation == 'map_upload_abort':
            if stage.is_dir():
                shutil.rmtree(stage)
        elif operation == 'map_upload_chunk':
            suffix, offset = args.get('suffix'), args.get('offset')
            if suffix not in live or type(offset) is not int or offset < 0 or offset % _MAP_CHUNK:
                raise ValueError('invalid_map_archive_chunk')
            data = base64.b64decode(args['data'], validate=True)
            if not 0 < len(data) <= _MAP_CHUNK or offset + len(data) > 512 * _MAP_CHUNK:
                raise ValueError('invalid_map_archive_chunk_size')
            path = stage / suffix
            with path.open('xb' if offset == 0 else 'r+b') as stream:
                if stream.seek(0, os.SEEK_END) != offset:
                    raise ValueError('unexpected_map_archive_offset')
                stream.write(data)
        elif operation == 'map_upload_commit':
            manifest = args['manifest']
            if manifest['map_name'] != name or set(manifest['files']) != set(live):
                raise ValueError('invalid_map_archive_manifest')
            for suffix, item in manifest['files'].items():
                path = stage / suffix
                digest = hashlib.sha256()
                with path.open('rb') as stream:
                    for chunk in iter(lambda: stream.read(_MAP_CHUNK), b''):
                        digest.update(chunk)
                    os.fsync(stream.fileno())
                if path.stat().st_size != item['size'] or digest.hexdigest() != item['sha256']:
                    raise ValueError('map_archive_digest_mismatch')
            # The backup verifier uses the road-map write as the completion fence.
            # Restore that ordering regardless of chunk transfer order.
            newest = max((stage / suffix).stat().st_mtime_ns for suffix in _MAP_FILES)
            os.utime(stage / '.path', ns=(newest, newest))
            assert_inactive(name)
            installed = []
            try:
                # The factory lists maps via PGM. Publish that file last.
                for suffix in _MAP_FILES:
                    os.link(stage / suffix, live[suffix])
                    installed.append(live[suffix])
            except Exception:
                for path in reversed(installed):
                    path.unlink()
                raise
            shutil.rmtree(stage)
        elif operation == 'map_delete':
            manifest = map_delete_manifest(name, root)
            if manifest != args['manifest']:
                raise ValueError('map_changed_since_backup')
            assert_inactive(name)
            stage.mkdir(mode=0o700)
            moved = []
            try:
                # Remove the entry from the factory list before its sidecars.
                for suffix in reversed(_MAP_FILES):
                    if suffix not in manifest['files']:
                        continue
                    live[suffix].rename(stage / live[suffix].name)
                    moved.append(suffix)
            except Exception:
                for suffix in reversed(moved):
                    (stage / live[suffix].name).rename(live[suffix])
                raise
            shutil.rmtree(stage)
        else:
            raise ValueError('unsupported_map_archive_write')
    return {'success': True}


def dispatch_service_class(service_class, server_md5):
    if service_class._md5sum == server_md5:
        return service_class
    # Some factory images ship stale Python bindings but current C++ headers.
    # Reconstruct ONLY the fixed, verified layout; never patch an MD5 blindly.
    expected = hashlib.md5(b'uint8 control_modebool success\nstring message').hexdigest()
    root = Path('/opt/jaten/include/leju_mobile_base_msgs')
    header = (root / 'SetDispatchMode.h').read_text()
    if server_md5 != expected or ('return "' + expected + '";') not in header:
        raise RuntimeError('dispatch_service_abi_mismatch')
    layouts = {'Request': ['uint8 control_mode'], 'Response': ['bool success', 'string message']}
    for suffix, layout in layouts.items():
        data = (root / ('SetDispatchMode' + suffix + '.h')).read_text()
        definition = data.split('struct Definition<', 1)[1].split(';', 1)[0]
        pieces = re.findall(r'"(?:[^"\\]|\\.)*"', definition)
        content = ''.join(json.loads(piece) for piece in pieces)
        fields = [line.split('#', 1)[0].strip() for line in content.splitlines()]
        if [line for line in fields if line] != layout:
            raise RuntimeError('dispatch_header_layout_mismatch')
    from genpy.dynamic import generate_dynamic
    request_type = 'leju_mobile_base_msgs/SetDispatchModeRequest'
    response_type = 'leju_mobile_base_msgs/SetDispatchModeResponse'
    request_class = generate_dynamic(request_type, 'uint8 control_mode\n')[request_type]
    response_class = generate_dynamic(response_type, 'bool success\nstring message\n')[response_type]
    return type('SetDispatchMode', (), {'_type': 'leju_mobile_base_msgs/SetDispatchMode',
                '_md5sum': expected, '_request_class': request_class, '_response_class': response_class})


def configure_motion_music(enabled, directory=Path.home() / 'jmap_config/music'):
    """Configure only the verified ID-1 movement track, reversibly."""
    return _configure_factory_track(enabled, directory, 'movement')


def configure_standby_voice(enabled, directory=Path.home() / 'jmap_config/music'):
    """Configure only the verified ID-7 standby voice, not TTS or alarms."""
    return _configure_factory_track(enabled, directory, 'standby')


def _configure_factory_track(enabled, directory, kind):
    # Fixed whitelist: remote requests cannot select arbitrary audio files.
    filename, backup_name, scope = {
        'movement': ('1_flower.mp3', '.motus-motion-music', 'factory_id1_movement_track_only'),
        'standby': ('7_standby.mp3', '.motus-standby-voice', 'factory_id7_standby_track_only'),
    }[kind]
    import shutil
    import subprocess
    import tempfile
    if type(enabled) is not bool:
        raise ValueError('factory_audio_enabled_must_be_boolean')
    track = directory / filename
    backup = directory / backup_name
    original, silent, record = [backup / n for n in ('original.mp3', 'silent.mp3', 'identity.json')]
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    if track.is_symlink() or not track.is_file() or not 0 < track.stat().st_size < 4_000_000:
        return {'state': 'unsupported', 'reason': kind + '_track_not_found'}
    if record.is_file():
        identity = json.loads(record.read_text())
        if (not original.is_file() or not silent.is_file() or
                digest(original) != identity['original'] or digest(silent) != identity['silent'] or
                digest(track) not in (identity['original'], identity['silent'])):
            return {'state': 'blocked', 'reason': kind + '_track_changed_by_operator'}
    elif enabled:
        return {'state': 'enabled', 'changed': False}
    else:
        ffmpeg = shutil.which('ffmpeg')
        if not ffmpeg:
            return {'state': 'unsupported', 'reason': 'existing_ffmpeg_required_no_package_installed'}
        backup.mkdir(mode=0o700, exist_ok=True)
        if original.exists() or silent.exists():
            return {'state': 'blocked', 'reason': 'incomplete_' + kind + '_backup'}
        shutil.copyfile(track, original)
        try:
            subprocess.run([ffmpeg, '-nostdin', '-v', 'error', '-f', 'lavfi', '-i',
                            'anullsrc=r=44100:cl=stereo', '-t', '1', '-codec:a', 'libmp3lame',
                            '-q:a', '9', str(silent)], check=True, timeout=8,
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        except (OSError, subprocess.SubprocessError):
            original.unlink(missing_ok=True)
            silent.unlink(missing_ok=True)
            return {'state': 'unsupported', 'reason': 'existing_ffmpeg_cannot_encode_silent_mp3'}
        identity = {'original': digest(original), 'silent': digest(silent)}
        record.write_text(json.dumps(identity))
    chosen = original if enabled else silent
    changed = digest(track) != digest(chosen)
    if changed:
        mode = track.stat().st_mode & 0o777
        with tempfile.NamedTemporaryFile(dir=directory, prefix='.motus-music-', delete=False) as temporary:
            temporary.write(chosen.read_bytes())
            temporary_path = Path(temporary.name)
        try:
            temporary_path.chmod(mode)
            os.replace(temporary_path, track)
        finally:
            temporary_path.unlink(missing_ok=True)
    return {'state': 'enabled' if enabled else 'disabled', 'changed': changed,
            'scope': scope, 'other_audio_unchanged': True,
            'active_playback_may_finish': True}


def map_save_status(name, root=Path('/home/jt/jmap_config/jmaps')):
    """Read-only completion fence for the factory SaveCartoMap implementation.

    The installed MapManager writes the JSON .path AFTER its synchronous
    Cartographer SaveMap returns. A PGM alone can appear before saving ends.
    Require that final file to be valid and newer than every map output.
    Callers additionally compare two snapshots before restarting the map stack.
    """
    if (not isinstance(name, str) or not name or len(name) > 80
            or any(c in name for c in '/\\') or any(ord(c) < 32 for c in name)):
        raise ValueError('invalid_map_name')
    root = root.resolve()
    stats = {}
    try:
        for suffix in ('.pgm', '.yaml', '.pbstream', '.path'):
            path = root / (name + suffix)
            if path.is_symlink() or not path.is_file():
                return {'complete': False}
            stat = path.stat()
            if stat.st_size <= 0:
                return {'complete': False}
            stats[suffix] = [stat.st_size, stat.st_mtime_ns]
        path = root / (name + '.path')
        if stats['.path'][0] > 32_000_000:
            return {'complete': False}
        if not isinstance(json.loads(path.read_text()), dict):
            return {'complete': False}
        complete = all(stats['.path'][1] >= stat[1] for stat in stats.values())
        return {'complete': complete, 'files': stats}
    except (OSError, ValueError):
        return {'complete': False}


def map_runtime_status():
    """Read-only boot fence; a container restart is NOT a chassis restart."""
    from uuid import UUID
    return {'boot_id': str(UUID(Path('/proc/sys/kernel/random/boot_id').read_text().strip())),
            'uptime_s': float(Path('/proc/uptime').read_text().split()[0]),
            'observed_at': time.time()}


def map_delete_manifest(name, root=Path('/home/jt/jmap_config/jmaps')):
    """Deletion needs exact existing files, including empty/partial save outputs.

    Do not weaken the complete-map fence used by backup and upload.
    """
    if (not isinstance(name, str) or not name.strip() or len(name) > 80 or name in ('.', '..')
            or any(c in name for c in '/\\') or any(ord(c) < 32 for c in name)):
        raise ValueError('invalid_map_name')
    files = {}
    for suffix in _MAP_FILES:
        path = root / (name + suffix)
        if not os.path.lexists(path):
            continue
        if path.is_symlink() or not path.is_file() or path.stat().st_size > 512 * _MAP_CHUNK:
            raise RuntimeError('invalid_map_delete_source')
        before = path.stat()
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(_MAP_CHUNK), b''):
                digest.update(chunk)
        after = path.stat()
        if (before.st_ino, before.st_size, before.st_mtime_ns) != (after.st_ino, after.st_size, after.st_mtime_ns):
            raise RuntimeError('map_changed_during_delete_inspection')
        files[suffix] = {'size': after.st_size, 'mtime_ns': after.st_mtime_ns, 'sha256': digest.hexdigest()}
    if not files:
        raise ValueError('grid_map_not_found')
    return {'map_name': name, 'files': files}


def tracker_command(rospy, operation, request=None):
    """Pause the actual LQR task, not only the Java dispatch task wrapper.

    The factory DELETE operation cannot remove the currently executing edge.
    Keep that task paused until a new point goal has been accepted instead.
    """
    import rosgraph
    import rosservice
    from jaten_msgs.srv import PathTrackRequestService
    from jaten_msgs.msg import PathTrackRequest, TrackEdge, MotionState
    from geometry_msgs.msg import Twist
    name = '/path_track_request_service'
    master = rosgraph.Master('/motus_navigation_rpc')
    owners = dict(master.getSystemState()[2]).get(name, [])
    headers = rosservice.get_service_headers(name, master.lookupService(name))
    if (owners != ['/lqr_path_track'] or headers.get('type') != PathTrackRequestService._type
            or headers.get('md5sum') != PathTrackRequestService._md5sum):
        raise RuntimeError('native_tracker_service_identity_mismatch')
    message = PathTrackRequest()
    message.header.stamp = rospy.Time.now()
    message.request_id = int.from_bytes(os.urandom(2), 'big') or 1
    message.op.operation = operation  # factory OP_PAUSE=3, OP_RESUME=4
    if operation not in (3, 4):
        raise ValueError('unsupported_tracker_control_operation')
    # This factory binary logs edge_to_add.front()/back() BEFORE checking op,
    # and segfaults on the valid empty PAUSE/RESUME payload. Supply an inert
    # preview object for that logger only. edge_num stays zero; OP_ADD is never
    # used and the pause/resume handlers ignore these geometry fields.
    message.edge_num = 0
    message.edge_to_add = [TrackEdge()]
    if request is not None:
        request['_command_unresolved'] = True
    try:
        response = rospy.ServiceProxy(name, PathTrackRequestService)(request=message, next_area=False)
    except Exception:
        if request is not None:
            request['_older_command_unresolved'] = True
        raise
    if request is not None:
        request['_command_unresolved'] = False
    if response.request_id != message.request_id:
        raise RuntimeError('native_tracker_request_unconfirmed')
    # A repeated pause/resume can be rejected as already in that state. Do
    # not infer success from the service boolean: verify fresh runtime state.
    empty = response.num_edge_remain == 0
    end, quiet_since, count = time.monotonic() + 4., None, 0
    while time.monotonic() < end:
        motion = rospy.wait_for_message('/mpc_path_track/motion_state', MotionState, timeout=1.)
        # Factory MotionState.header.stamp is the last EVENT time, not the
        # publication time (verified while it publishes continuously). A new
        # nonlatched connection provides receipt freshness, not that stamp.
        if not tracker_live_message(motion):
            continue
        if operation == 4:
            if motion.motion_type in range(1, 8):
                return
            continue
        if empty and motion.motion_type == 2:  # factory FINISH_STOP
            # An empty, finished factory tracker does not publish idle Twist
            # frames. Prove the held chassis is stationary instead of waiting
            # for a zero frame that the idle publisher never sends.
            from nav_msgs.msg import Odometry
            confirm_software_stop(rospy, True)
            confirm_stationary(rospy, Odometry)
            return
        if not empty and motion.motion_type != 8:  # factory CAR_PAUSED
            continue
        velocity = rospy.wait_for_message('/twist_mux_new/in/nav/mpc_path_track', Twist, timeout=1.)
        if not tracker_live_message(velocity):
            continue
        values = (velocity.linear.x, velocity.linear.y, velocity.linear.z,
                  velocity.angular.x, velocity.angular.y, velocity.angular.z)
        now = time.monotonic()
        quiet = all(math.isfinite(v) and abs(v) <= 1e-6 for v in values)
        quiet_since = now if quiet and quiet_since is None else quiet_since if quiet else None
        count = count + 1 if quiet else 0
        if quiet_since is not None and now - quiet_since >= .3 and count >= 3:
            return
    raise RuntimeError('native_tracker_pause_unconfirmed' if operation == 3
                       else 'native_tracker_resume_unconfirmed')


def tracker_live_message(message):
    header = getattr(message, '_connection_header', {}) or {}
    return header.get('callerid') == '/lqr_path_track' and header.get('latching') == '0'


def confirm_software_stop(rospy, expected, require_healthy=False):
    from jaten_msgs.msg import RobotChassisState
    end = time.monotonic() + 3.
    while time.monotonic() < end:
        state = rospy.wait_for_message('/jaten_behavior_manager/robot_state', RobotChassisState, timeout=1.)
        if require_healthy and (state.estop or state.bumper or state.driver_error):
            raise RuntimeError('chassis_hardware_stop_or_fault_active')
        if state.soft_estop == expected:
            return
    raise RuntimeError('chassis_software_stop_feedback_unconfirmed')


def tracker_process_identity():
    """A respawned LQR process cannot confirm the previous goal's path."""
    import rosgraph
    import xmlrpc.client
    class BoundedTransport(xmlrpc.client.Transport):
        def make_connection(self, host):
            connection = super().make_connection(host)
            connection.timeout = 1.
            return connection
    try:
        uri = rosgraph.Master('/motus_navigation_rpc').lookupNode('/lqr_path_track')
        with xmlrpc.client.ServerProxy(uri, transport=BoundedTransport()) as proxy:
            code, _, pid = proxy.getPid('/motus_navigation_rpc')
        if code != 1 or type(pid) is not int or pid <= 0:
            raise ValueError('invalid_pid')
        return uri, pid
    except Exception:
        raise RuntimeError('native_navigation_tracker_unavailable') from None


def confirm_tracker_target(rospy, target, expected_process=None):
    """Do not release a paused old path merely because a goal was queued."""
    from nav_msgs.msg import Path as RosPath
    end, since, count = time.monotonic() + 6., None, 0
    while time.monotonic() < end:
        if expected_process is not None and tracker_process_identity() != expected_process:
            raise RuntimeError('native_navigation_tracker_restarted')
        try:
            path = rospy.wait_for_message('/lqr_path_track/tracking_path', RosPath,
                                          timeout=min(1., max(.01, end - time.monotonic())))
        except rospy.ROSException:
            # Planning may not publish during the first one-second sample.
            # Keep the stop asserted for the complete bounded confirmation window.
            since, count = None, 0
            continue
        if expected_process is not None and tracker_process_identity() != expected_process:
            raise RuntimeError('native_navigation_tracker_restarted')
        if not tracker_live_message(path) or path.header.frame_id.strip('/') != 'map' or not path.poses:
            continue
        point = path.poses[-1].pose.position
        matches = math.hypot(point.x - target['x'], point.y - target['y']) <= .1
        now = time.monotonic()
        since = now if matches and since is None else since if matches else None
        count = count + 1 if matches else 0
        if since is not None and now - since >= .2 and count >= 3:
            return
    raise RuntimeError('native_tracker_target_unconfirmed')


def select_navigation_source(rospy, expected='nav_mpc_path_track'):
    """One finite switch request with a new mux acknowledgement, never a latch."""
    import threading
    from std_msgs.msg import String
    if expected not in ('nav_mpc_path_track', 'nav_mpc_move_base'):
        raise ValueError('unsupported_navigation_source')
    condition = threading.Condition()
    seen = {'initial': False, 'sent': False, 'source': None}
    def feedback(message):
        with condition:
            seen['initial'] = True
            if seen['sent']:
                seen['source'] = message.data
            condition.notify_all()
    subscriber = rospy.Subscriber('/twist_mux_new/out/nav_source_used', String, feedback, queue_size=10)
    publisher = None
    try:
        publisher = rospy.Publisher('/twist_mux_new/in/switch/nav_source', String, queue_size=1, latch=False)
        end = time.monotonic() + 3.
        with condition:
            while not seen['initial'] or not publisher.get_num_connections():
                remaining = end - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError('native_navigation_mux_unavailable')
                condition.wait(min(.05, remaining))
            # Drain the initial retained value before issuing the switch.
            seen['sent'] = True
            publisher.publish(String(data=expected))
            while seen['source'] != expected:
                remaining = end - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError('native_navigation_mux_handoff_unconfirmed')
                condition.wait(remaining)
    finally:
        subscriber.unregister()
        if publisher is not None:
            publisher.unregister()


def remote_joy_neutral(message, owner):
    header = getattr(message, '_connection_header', {}) or {}
    axes, buttons = getattr(message, 'axes', ()), getattr(message, 'buttons', ())
    return (isinstance(owner, str) and owner.startswith('/')
            and header.get('callerid') == owner and header.get('latching') == '0'
            and len(axes) == 8 and len(buttons) == 11
            and all(type(v) in (int, float) and math.isfinite(v) and abs(v) <= 1.05 for v in axes)
            and all(abs(axes[i]) <= .03 for i in (0, 1, 3, 4, 6, 7))
            and all(type(v) is int and v == 0 for v in buttons))


class UpperTopicObservation:
    """Read a verified upper publisher without registering it on the chassis.

    Uses the installed ROS TCPROS transport and message deserializer. Only
    master/publisher queries and subscriber connections are made; no topic is
    advertised or relayed into the chassis control graph.
    """
    def __init__(self, master, topic, message_type, owner, callback):
        import threading
        import xmlrpc.client
        from rospy.impl.tcpros_base import TCPROSTransport
        from rospy.impl.tcpros_pubsub import TCPROSSub
        class BoundedTransport(xmlrpc.client.Transport):
            def make_connection(self, host):
                connection = super().make_connection(host)
                connection.timeout = 1.
                return connection
        caller = '/kuavo5w_upper_idle_observer'
        self.closed, self.failed = threading.Event(), threading.Event()
        self.connection, self.thread = None, None
        try:
            with xmlrpc.client.ServerProxy(master, transport=BoundedTransport()) as proxy:
                code, _, state = proxy.getSystemState(caller)
                if code != 1 or dict(state[0]).get(topic) != [owner]:
                    raise RuntimeError('boot_remote_upper_publisher_unconfirmed')
                code, _, uri = proxy.lookupNode(caller, owner)
                if code != 1:
                    raise RuntimeError('boot_remote_upper_publisher_unavailable')
            with xmlrpc.client.ServerProxy(uri, transport=BoundedTransport()) as proxy:
                code, _, protocol = proxy.requestTopic(caller, topic, [['TCPROS']])
            if (code != 1 or not isinstance(protocol, (list, tuple)) or len(protocol) != 3
                    or protocol[0] != 'TCPROS' or not isinstance(protocol[1], str)
                    or type(protocol[2]) is not int or not 1 <= protocol[2] <= 65535):
                raise RuntimeError('boot_remote_upper_transport_unconfirmed')
            # Process all received frames: a short nonzero sample must reset
            # the quiet window even if zeros follow in the same socket read.
            self.connection = TCPROSTransport(TCPROSSub(topic, message_type,
                queue_size=None, tcp_nodelay=True), topic)
            self.connection.connect(protocol[1], protocol[2], uri, timeout=1.)
            header = self.connection.header
            if (header.get('callerid') != owner or header.get('type') != message_type._type
                    or header.get('md5sum') != message_type._md5sum):
                raise RuntimeError('boot_remote_upper_transport_identity_mismatch')
            def receive():
                try:
                    while not self.closed.is_set():
                        for message in self.connection.receive_once():
                            if not self.closed.is_set():
                                callback(message)
                except Exception:
                    if not self.closed.is_set():
                        self.failed.set()
                finally:
                    self.connection.close()
            self.thread = threading.Thread(target=receive, daemon=True, name='upper-idle-observer')
            self.thread.start()
        except Exception:
            self.unregister()
            raise

    def unregister(self):
        self.closed.set()
        if self.connection is not None:
            self.connection.close()
        if self.thread is not None:
            self.thread.join(timeout=.2)


def remote_idle_guard(rospy, owner, upper_master=None, require_joy=True, allow_disabled_output=False):
    """Live guard retained through mux selection and software-stop release.

    Ignore initial latched upper velocity alone: require >=3 callbacks spanning
    0.3s plus fresh nonlatched G12. No ROS input is fabricated by this probe.
    """
    import threading
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool
    lock, sample = threading.Lock(), {}
    def joy(message):
        with lock:
            sample.update(joy_at=time.monotonic(), neutral=remote_joy_neutral(message, owner))
    def velocity(message):
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        header = getattr(message, '_connection_header', {}) or {}
        quiet = (header.get('callerid') == '/nodelet_manager'
                 and all(math.isfinite(v) and abs(v) <= 1e-6 for v in values))
        now = time.monotonic()
        with lock:
            if not quiet:
                sample.pop('since', None)
                sample['count'] = 0
            elif sample.get('quiet') is not True or now-sample.get('velocity_at', now) > .3:
                # Measure only from the first fresh zero after motion or a
                # stream gap, never from the last nonzero callback.
                sample.update(since=now, count=1)
            else:
                sample['count'] = sample.get('count', 0)+1
            sample.update(velocity_at=now, quiet=quiet)
    def enabled(message):
        header = getattr(message, '_connection_header', {}) or {}
        with lock:
            sample['disabled'] = (header.get('callerid') == '/nodelet_manager'
                                  and getattr(message, 'data', None) is False)
            if not sample['disabled']:
                sample.pop('disabled_since', None)
            elif 'disabled_since' not in sample:
                sample['disabled_since'] = time.monotonic()
    subscriptions = []
    def check():
        now = time.monotonic()
        with lock:
            # Disabled MPC publishes its final latched zero only once. Require
            # a verified live disable-state connection and a quiet observation
            # window; any nonzero output or enable callback revokes this path.
            disabled_idle = (allow_disabled_output and not require_joy
                and sample.get('disabled') is True and sample.get('quiet') is True
                and now-max(sample.get('since', now), sample.get('disabled_since', now)) >= .3)
            if ((require_joy and (sample.get('neutral') is not True or now-sample.get('joy_at', 0) > .3))
                    or (not disabled_idle and (sample.get('quiet') is not True
                        or now-sample.get('velocity_at', 0) > .3
                        or sample.get('count', 0) < 3 or now-sample.get('since', now) < .3))
                    or any(getattr(s, 'failed', None) is not None and s.failed.is_set()
                           for s in subscriptions)):
                raise RuntimeError('boot_remote_fresh_neutral_or_zero_output_unconfirmed')
    def close():
        for subscription in subscriptions:
            subscription.unregister()
    try:
        for topic, kind, callback, publisher in (('/joy', Joy, joy, owner),
                ('/move_base/base_cmd_vel', Twist, velocity, '/nodelet_manager'),
                ('/enable_vel_control_state', Bool, enabled, '/nodelet_manager')):
            if topic == '/joy' and not require_joy:
                continue
            if topic == '/enable_vel_control_state' and not allow_disabled_output:
                continue
            subscriptions.append(UpperTopicObservation(upper_master, topic, kind, publisher, callback)
                if upper_master else rospy.Subscriber(topic, kind, callback, queue_size=1, tcp_nodelay=True))
        end = time.monotonic()+3.
        while True:
            try:
                check()
                return check, close
            except RuntimeError:
                if time.monotonic() >= end:
                    raise
                time.sleep(.02)
    except Exception:
        close()
        raise


def main(request):
    if request['operation'] == 'map_runtime_status':
        return map_runtime_status()
    if request['operation'] == 'map_delete_manifest':
        return map_delete_manifest(request.get('args', {}).get('map_name'))
    if request['operation'] in ('map_upload_begin', 'map_upload_abort', 'map_upload_chunk',
                                'map_upload_commit', 'map_delete'):
        return map_archive_write(request['operation'], request.get('args', {}))
    if request['operation'] in ('map_archive_manifest', 'map_archive_chunk'):
        return map_archive_read(request['operation'], request.get('args', {}))
    if request['operation'] == 'map_save_status':
        return map_save_status(request.get('args', {}).get('map_name'))
    if request['operation'] == 'standby_voice':
        return configure_standby_voice(request.get('args', {}).get('enabled', False))
    if request['operation'] == 'motion_music':
        return configure_motion_music(request.get('args', {}).get('enabled', False))
    if request['operation'] == 'map_identity':
        name = request.get('args', {}).get('map_name')
        if (not isinstance(name, str) or not name or len(name) > 80
                or any(c in name for c in '/\\') or any(ord(c) < 32 for c in name)):
            raise ValueError('invalid_map_name')
        digest = hashlib.sha256()
        root = Path('/home/jt/jmap_config/jmaps').resolve()
        for suffix in ('.pgm', '.yaml'):
            path = (root / (name + suffix)).resolve()
            if path.parent != root or not 0 < path.stat().st_size <= 32_000_000:
                raise RuntimeError('invalid_grid_map_file')
            with path.open('rb') as stream:
                digest.update(suffix.encode())
                while True:
                    chunk = stream.read(65536)
                    if not chunk:
                        break
                    digest.update(chunk)
        result = {'sha256': digest.hexdigest()}
        if request.get('args', {}).get('include_road_md5') is True:
            road = (root / (name + '.path')).resolve()
            if road.parent != root or not 0 < road.stat().st_size <= 32_000_000:
                raise RuntimeError('invalid_road_map_file')
            road_digest = hashlib.md5(usedforsecurity=False)
            with road.open('rb') as stream:
                for chunk in iter(lambda: stream.read(65536), b''):
                    road_digest.update(chunk)
            result['road_md5'] = road_digest.hexdigest()
        return result
    os.environ['ROS_MASTER_URI'] = request['master']
    os.environ['ROS_IP'] = request['ros_ip']
    os.environ.pop('ROS_HOSTNAME', None)
    for path in ('/opt/ros/noetic/lib/python3/dist-packages', '/opt/jaten/lib/python3/dist-packages'):
        sys.path.insert(0, path)
    import rospy
    import rosgraph
    import rosservice
    from leju_mobile_base_msgs.srv import MoveToTarget, CheckArrived, SetDispatchMode
    from leju_mobile_base_msgs.msg import MoveToTargetOptions
    from nav_msgs.msg import Odometry
    rospy.init_node('motus_navigation_rpc', anonymous=True, disable_signals=True,
                    disable_rosout=True, log_level=rospy.ERROR)
    if request['operation'] == 'held_stationary':
        # Read-only settlement of a completed preparation, never a substitute
        # for cancelling a known/uncertain navigation submission.
        from jaten_msgs.msg import MotionState
        confirm_software_stop(rospy, True)
        motion = rospy.wait_for_message('/mpc_path_track/motion_state', MotionState, timeout=2.)
        if not tracker_live_message(motion) or motion.motion_type != 2:
            raise RuntimeError('native_tracker_not_finished')
        confirm_stationary(rospy, Odometry)
        confirm_software_stop(rospy, True)
        return {'success': True, 'stationary': True, 'software_estop': True,
                'completion_basis': 'held_finished_tracker_and_fresh_stationary_odometry'}
    if request['operation'] == 'stationary':
        confirm_stationary(rospy, Odometry)
        return {'success': True, 'stationary': True, 'completion_basis': 'fresh_distinct_odometry'}
    if request['operation'] == 'manual_idle_stationary':
        from jaten_msgs.msg import RobotChassisState, MotionState
        def idle():
            state = rospy.wait_for_message('/jaten_behavior_manager/robot_state', RobotChassisState, timeout=2.)
            if (state.manual_mode is not True or state.mapping or state.estop
                    or state.bumper or state.driver_error or not state.driver_enable):
                raise RuntimeError('chassis_manual_idle_state_invalid')
            motion = rospy.wait_for_message('/mpc_path_track/motion_state', MotionState, timeout=2.)
            if not tracker_live_message(motion) or motion.motion_type != 2:
                raise RuntimeError('native_tracker_not_finished')
        idle()
        confirm_stationary(rospy, Odometry)
        idle()
        return {'success': True, 'stationary': True, 'manual_mode': True, 'native_tracker_idle': True}
    if request['operation'] == 'manual_auto_guard':
        from jaten_msgs.msg import RobotChassisState, MotionState
        state = rospy.wait_for_message('/jaten_behavior_manager/robot_state', RobotChassisState, timeout=2.)
        if (state.manual_mode is not True or state.mapping or state.estop
                or state.bumper or state.driver_error or not state.driver_enable):
            raise RuntimeError('chassis_manual_auto_guard_state_invalid')
        motion = rospy.wait_for_message('/mpc_path_track/motion_state', MotionState, timeout=2.)
        if not tracker_live_message(motion) or motion.motion_type != 2:
            raise RuntimeError('native_tracker_must_finish_before_auto')
        guard, close = remote_idle_guard(rospy, None, request.get('upper_master'), require_joy=False,
                                         allow_disabled_output=True)
        try:
            guard()
            confirm_stationary(rospy, Odometry, guard=guard)
            motion = rospy.wait_for_message('/mpc_path_track/motion_state', MotionState, timeout=2.)
            if not tracker_live_message(motion) or motion.motion_type != 2:
                raise RuntimeError('native_tracker_must_finish_before_auto')
            guard()
            return {'success': True, 'stationary': True, 'native_tracker_idle': True, 'upper_zero': True}
        finally:
            close()
    names = {'navigate': '/move_base/move_to_target', 'check': '/move_base/check_arrived',
             'cancel': '/move_base/set_dispatch_mode'}
    types = {'navigate': MoveToTarget, 'check': CheckArrived, 'cancel': SetDispatchMode}
    operation = request['operation']
    args = request.get('args', {})
    if operation in ('soft_stop', 'manual_hold', 'cancel'):
        stop_epoch(advance=True)
    # Verify the native cancellation and submission really have one owner.
    services = dict(rosgraph.Master('/motus_navigation_rpc').getSystemState()[2])
    owners = [services.get(name, []) for name in names.values()]
    if any(len(owner) != 1 for owner in owners) or len({owner[0] for owner in owners}) != 1:
        raise RuntimeError('native_navigation_services_need_one_common_owner')
    for action, name in names.items():
        headers = rosservice.get_service_headers(name, rosgraph.Master('/motus_navigation_rpc').lookupService(name))
        if headers.get('type') != types[action]._type:
            raise RuntimeError('native_navigation_service_type_mismatch')
        if action == 'cancel':
            types[action] = dispatch_service_class(types[action], headers.get('md5sum'))
        elif headers.get('md5sum') != types[action]._md5sum:
            raise RuntimeError('native_navigation_service_abi_mismatch')
    if operation == 'probe':
        return {'owner': owners[0][0], 'services': names}
    if operation in ('prepare', 'remote_ready', 'upper_handoff', 'soft_stop', 'manual_hold'):
        preparation_epoch = stop_epoch()
        from jaten_msgs.msg import RobotChassisState
        proxy = rospy.ServiceProxy(names['cancel'], types['cancel'])
        def state():
            return rospy.wait_for_message('/jaten_behavior_manager/robot_state', RobotChassisState, timeout=2.)
        def command(mode):
            request['_command_unresolved'] = True
            try:
                response = proxy(control_mode=mode)
            except Exception:
                request['_older_command_unresolved'] = True
                raise
            request['_command_unresolved'] = False
            if not response.success:
                raise RuntimeError('chassis_preparation_command_rejected')
        def healthy(value):
            if value.estop or value.bumper or value.driver_error:
                raise RuntimeError('chassis_hardware_stop_or_fault_active')
        if operation in ('soft_stop', 'manual_hold'):
            command(0)
            confirm_software_stop(rospy, True)
            if operation == 'soft_stop':
                command(2)
                tracker_command(rospy, 3, request)
            confirm_stationary(rospy, Odometry)
            if not state().soft_estop:
                raise RuntimeError('chassis_soft_stop_unconfirmed')
            return {'success': True, 'stationary': True, 'software_estop': True}
        request['_preparation_stage'] = 'read_fault_state'
        healthy(state())
        request['_preparation_stage'] = 'hold_software_stop'
        command(0)
        confirm_software_stop(rospy, True)
        request['_preparation_stage'] = 'cancel_old_task'
        command(2)  # Cancel old tasks BEFORE releasing a software stop.
        request['_preparation_stage'] = 'pause_old_tracker'
        tracker_command(rospy, 3, request)
        request['_preparation_stage'] = 'confirm_cancel_stationary'
        confirm_stationary(rospy, Odometry)
        current = state()
        healthy(current)
        released = bool(current.soft_estop)
        guard, close_guard = None, None
        try:
            if operation in ('remote_ready', 'upper_handoff'):
                if (current.manual_mode or current.mapping or not current.driver_enable):
                    raise RuntimeError('boot_remote_requires_healthy_auto_nonmapping')
                guard, close_guard = remote_idle_guard(rospy, args.get('g12_owner'), request.get('upper_master'),
                                                       require_joy=operation == 'remote_ready',
                                                       allow_disabled_output=operation == 'upper_handoff')
                select_navigation_source(rospy, 'nav_mpc_move_base')
                guard()
                current = state()
                healthy(current)
                if current.manual_mode or current.mapping or not current.driver_enable or not current.soft_estop:
                    raise RuntimeError('boot_remote_chassis_state_changed')
                guard()  # state() can block; do not release on an older neutral check.
            if released:
                request['_preparation_stage'] = 'release_software_stop'
                guarded_navigation_release(preparation_epoch, lambda: command(1))
            request['_preparation_stage'] = 'confirm_release_state'
            end = time.monotonic() + 3.
            while time.monotonic() < end:
                if guard is not None:
                    guard()
                current = state()
                if guard is not None:
                    guard()
                healthy(current)
                if not current.soft_estop:
                    request['_preparation_stage'] = 'confirm_release_stationary'
                    confirm_stationary(rospy, Odometry, guard=guard)
                    result = {'success': True, 'stationary': True, 'software_estop': False,
                              'old_task_cancelled': True, 'released': released}
                    if operation == 'upper_handoff':
                        result['velocity_source'] = 'nav_mpc_move_base'
                    return result
            raise RuntimeError('chassis_release_feedback_unconfirmed')
        except Exception:
            try:
                command(0)
            except Exception:
                pass
            raise
        finally:
            if close_guard is not None:
                close_guard()
    if operation not in names:
        raise ValueError('unsupported_navigation_operation')
    proxy = rospy.ServiceProxy(names[operation], types[operation])
    if operation == 'navigate':
        navigation_epoch = stop_epoch()
        # The upper controller's velocity switch and the chassis dispatcher
        # have independent ownership state. STATUS_IDLE gives the dispatcher
        # control without changing the chassis AUTO mode or its safety limits.
        # Keep the handoff in the same ordered RPC as goal submission.
        dispatch = rospy.ServiceProxy(names['cancel'], types['cancel'])
        options = MoveToTargetOptions()
        options.avoid_enabled = True  # Never expose an obstacle-disable switch.
        options.allow_rotation = True
        options.avoid_distance = .5
        options.linear_velocity = args['speed']
        options.angular_velocity = .3
        options.position_threshold = .1
        options.angle_threshold = .1
        # Full-body control may already be enabled. Keep the chassis stopped
        # until the old tracker is held, the mux has acknowledged ownership,
        # and the replacement goal has actually been accepted.
        def mutate(callback):
            request['_command_unresolved'] = True
            result = callback()
            request['_command_unresolved'] = False
            return result
        try:
            request['_navigation_stage'] = 'hold_and_cancel'
            for mode in (0, 2):
                if not mutate(lambda: dispatch(control_mode=mode)).success:
                    raise RuntimeError('native_navigation_stop_or_cancel_rejected')
                if mode == 0:
                    confirm_software_stop(rospy, True)
            request['_navigation_stage'] = 'pause_old_tracker'
            tracker_command(rospy, 3, request)
            confirm_stationary(rospy, Odometry)
            request['_navigation_stage'] = 'dispatch_handoff'
            if not mutate(lambda: dispatch(control_mode=4)).success:  # STATUS_IDLE is not a mux switch.
                return {'success': False, 'stage': 'dispatch_handoff'}
            request['_navigation_stage'] = 'mux_handoff'
            select_navigation_source(rospy)
            request['_navigation_stage'] = 'goal_submit'
            planner_process = tracker_process_identity()
            request['_goal_submitted'] = True
            response = mutate(lambda: proxy(x=args['x'], y=args['y'], theta=args['yaw'], options=options))
            if not response.success:
                return {name: getattr(response, name) for name in response.__slots__}
            if not getattr(response, 'task_id', ''):
                raise RuntimeError('native_navigation_task_id_missing')
            request['_navigation_stage'] = 'confirm_new_path'
            confirm_tracker_target(rospy, args, planner_process)
            request['_navigation_stage'] = 'resume_new_tracker'
            tracker_command(rospy, 4, request)
            # Goal submission can itself change ownership on factory images.
            request['_navigation_stage'] = 'confirm_mux_handoff'
            select_navigation_source(rospy)
            request['_navigation_stage'] = 'release_software_stop'
            confirm_software_stop(rospy, True, require_healthy=True)
            if not guarded_navigation_release(navigation_epoch,
                    lambda: mutate(lambda: dispatch(control_mode=1))).success:
                raise RuntimeError('native_navigation_release_rejected')
            confirm_software_stop(rospy, False, require_healthy=True)
        except Exception:
            try:
                if not dispatch(control_mode=0).success:
                    request['_older_command_unresolved'] = True
                else:
                    confirm_software_stop(rospy, True)
            except Exception:
                request['_older_command_unresolved'] = True
            raise
    elif operation == 'check':
        response = proxy(task_id=args['task_id'], blocking=False, timeout=0.)
    else:
        dispatch = rospy.ServiceProxy(names['cancel'], types['cancel'])
        if not dispatch(control_mode=0).success:
            raise RuntimeError('native_navigation_soft_stop_rejected')
        confirm_software_stop(rospy, True)
        response = proxy(control_mode=2)  # SetDispatchMode.CANCEL_CURRENT_TASK
        if not response.success:
            raise RuntimeError('native_navigation_cancel_rejected')
        tracker_command(rospy, 3, request)
        confirm_stationary(rospy, Odometry)
        return {'success': True, 'stationary': True,
                'completion_basis': 'native_cancel_tracker_pause_and_fresh_stationary_odometry'}
    return {name: getattr(response, name) for name in response.__slots__}


def confirm_stationary(rospy, Odometry, guard=None):
    # A successful cancel response alone is not evidence of stopping.
    # Require new, distinctly stamped, stationary odometry spanning .3s.
    issued = time.time()
    end, since, previous = time.monotonic() + 5., None, None
    while time.monotonic() < end:
        if guard is not None:
            guard()
        try:
            msg = rospy.wait_for_message('/odom', Odometry, timeout=.1 if guard is not None else 1.)
        except rospy.ROSException:
            if guard is None:
                raise
            guard()
            continue
        if guard is not None:
            guard()
        stamp = msg.header.stamp.to_sec()
        if stamp <= issued or not 0 <= time.time() - stamp <= .5 or (previous is not None and stamp <= previous):
            continue
        previous = stamp
        twist = msg.twist.twist
        values = (twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z)
        stationary = all(math.isfinite(v) and abs(v) <= .01 for v in values)
        since = stamp if stationary and since is None else since if stationary else None
        if since is not None and stamp - since >= .3:
            return
    raise RuntimeError('native_navigation_stop_not_confirmed')


def rpc_error_code(exc):
    value = str(exc)
    if (isinstance(exc, RuntimeError) and re.fullmatch(r'[a-z][a-z0-9_]{0,159}', value)
            and value.startswith(('native_', 'chassis_', 'boot_remote_'))):
        return value
    return 'native_navigation_rpc_failed'


if __name__ == '__main__':
    request = None
    try:
        request = json.load(sys.stdin)
        print(json.dumps({'ok': True, 'result': main(request)}), flush=True)
    except Exception as exc:
        # A completed rejection/feedback failure can be stopped and retried.
        # A ROS transport failure may leave the controller service executing.
        if isinstance(request, dict) and request.get('operation') in ('prepare', 'remote_ready', 'upper_handoff', 'navigate'):
            stage_key = '_navigation_stage' if request['operation'] == 'navigate' else '_preparation_stage'
            result = {'success': False, 'stage': request.get(stage_key, 'validate_runtime'),
                      'error_code': rpc_error_code(exc),
                      'delivery_uncertain': bool(
                request.get('_command_unresolved') or request.get('_older_command_unresolved'))}
            if request['operation'] == 'navigate':
                result['goal_submitted'] = bool(request.get('_goal_submitted'))
            print(json.dumps({'ok': True, 'result': result}), flush=True)
        else:
            print(json.dumps({'ok': False, 'error': rpc_error_code(exc)}), flush=True)
