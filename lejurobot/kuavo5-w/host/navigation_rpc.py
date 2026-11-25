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
            manifest = map_archive_read('map_archive_manifest', args, root)
            if manifest != args['manifest']:
                raise ValueError('map_changed_since_backup')
            assert_inactive(name)
            stage.mkdir(mode=0o700)
            moved = []
            try:
                # Remove the entry from the factory list before its sidecars.
                for suffix in reversed(_MAP_FILES):
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


def main(request):
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
        return {'sha256': digest.hexdigest()}
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
    if request['operation'] == 'stationary':
        confirm_stationary(rospy, Odometry)
        return {'success': True, 'stationary': True, 'completion_basis': 'fresh_distinct_odometry'}
    names = {'navigate': '/move_base/move_to_target', 'check': '/move_base/check_arrived',
             'cancel': '/move_base/set_dispatch_mode'}
    types = {'navigate': MoveToTarget, 'check': CheckArrived, 'cancel': SetDispatchMode}
    operation = request['operation']
    args = request.get('args', {})
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
    if operation in ('prepare', 'soft_stop'):
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
        if operation == 'soft_stop':
            command(0)
            command(2)
            confirm_stationary(rospy, Odometry)
            if not state().soft_estop:
                raise RuntimeError('chassis_soft_stop_unconfirmed')
            return {'success': True, 'stationary': True, 'software_estop': True}
        request['_preparation_stage'] = 'read_fault_state'
        healthy(state())
        request['_preparation_stage'] = 'cancel_old_task'
        command(2)  # Cancel old tasks BEFORE releasing a software stop.
        request['_preparation_stage'] = 'confirm_cancel_stationary'
        confirm_stationary(rospy, Odometry)
        current = state()
        healthy(current)
        released = bool(current.soft_estop)
        try:
            if released:
                request['_preparation_stage'] = 'release_software_stop'
                command(1)
            request['_preparation_stage'] = 'confirm_release_state'
            end = time.monotonic() + 3.
            while time.monotonic() < end:
                current = state()
                healthy(current)
                if not current.soft_estop:
                    request['_preparation_stage'] = 'confirm_release_stationary'
                    confirm_stationary(rospy, Odometry)
                    return {'success': True, 'stationary': True, 'software_estop': False,
                            'old_task_cancelled': True, 'released': released}
            raise RuntimeError('chassis_release_feedback_unconfirmed')
        except Exception:
            try:
                command(0)
            except Exception:
                pass
            raise
    if operation not in names:
        raise ValueError('unsupported_navigation_operation')
    proxy = rospy.ServiceProxy(names[operation], types[operation])
    if operation == 'navigate':
        # The upper controller's velocity switch and the chassis dispatcher
        # have independent ownership state. STATUS_IDLE gives the dispatcher
        # control without changing the chassis AUTO mode or its safety limits.
        # Keep the handoff in the same ordered RPC as goal submission.
        dispatch = rospy.ServiceProxy(names['cancel'], types['cancel'])
        if not dispatch(control_mode=4).success:  # SetDispatchMode.STATUS_IDLE
            return {'success': False, 'stage': 'dispatch_handoff'}
        options = MoveToTargetOptions()
        options.avoid_enabled = True  # Never expose an obstacle-disable switch.
        options.allow_rotation = True
        options.avoid_distance = .5
        options.linear_velocity = args['speed']
        options.angular_velocity = .3
        options.position_threshold = .1
        options.angle_threshold = .1
        response = proxy(x=args['x'], y=args['y'], theta=args['yaw'], options=options)
    elif operation == 'check':
        response = proxy(task_id=args['task_id'], blocking=False, timeout=0.)
    else:
        response = proxy(control_mode=2)  # SetDispatchMode.CANCEL_CURRENT_TASK
        if not response.success:
            raise RuntimeError('native_navigation_cancel_rejected')
        confirm_stationary(rospy, Odometry)
        return {'success': True, 'stationary': True,
                'completion_basis': 'native_cancel_and_fresh_stationary_odometry'}
    return {name: getattr(response, name) for name in response.__slots__}


def confirm_stationary(rospy, Odometry):
    # A successful cancel response alone is not evidence of stopping.
    # Require new, distinctly stamped, stationary odometry spanning .3s.
    issued = time.time()
    end, since, previous = time.monotonic() + 5., None, None
    while time.monotonic() < end:
        msg = rospy.wait_for_message('/odom', Odometry, timeout=1.)
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


if __name__ == '__main__':
    request = None
    try:
        request = json.load(sys.stdin)
        print(json.dumps({'ok': True, 'result': main(request)}), flush=True)
    except Exception:
        # A completed rejection/feedback failure can be stopped and retried.
        # A ROS transport failure may leave the controller service executing.
        if isinstance(request, dict) and request.get('operation') == 'prepare':
            result = {'success': False, 'stage': request.get('_preparation_stage', 'validate_runtime'),
                      'delivery_uncertain': bool(
                request.get('_command_unresolved') or request.get('_older_command_unresolved'))}
            print(json.dumps({'ok': True, 'result': result}), flush=True)
        else:
            print(json.dumps({'ok': False, 'error': 'native_navigation_rpc_failed'}), flush=True)
