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
    """Replace only the verified ID-1 movement track with valid silent MP3.

    Preserve the original and its digest; never mute a sink, stop the factory
    manager, or alter ID-2 alarm / standby / TTS files. An unknown or externally
    edited layout is reported, not overwritten.
    """
    import shutil
    import subprocess
    import tempfile
    if type(enabled) is not bool:
        raise ValueError('motion_music_enabled_must_be_boolean')
    track = directory / '1_flower.mp3'
    backup = directory / '.motus-motion-music'
    original, silent, record = [backup / n for n in ('original.mp3', 'silent.mp3', 'identity.json')]
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    if track.is_symlink() or not track.is_file() or not 0 < track.stat().st_size < 4_000_000:
        return {'state': 'unsupported', 'reason': 'factory_id1_movement_track_not_found'}
    if record.is_file():
        identity = json.loads(record.read_text())
        if (not original.is_file() or not silent.is_file() or
                digest(original) != identity['original'] or digest(silent) != identity['silent'] or
                digest(track) not in (identity['original'], identity['silent'])):
            return {'state': 'blocked', 'reason': 'movement_track_changed_by_operator'}
    elif enabled:
        return {'state': 'enabled', 'changed': False}
    else:
        ffmpeg = shutil.which('ffmpeg')
        if not ffmpeg:
            return {'state': 'unsupported', 'reason': 'existing_ffmpeg_required_no_package_installed'}
        backup.mkdir(mode=0o700, exist_ok=True)
        if original.exists() or silent.exists():
            return {'state': 'blocked', 'reason': 'incomplete_movement_music_backup'}
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
            'scope': 'factory_id1_movement_track_only', 'other_audio_unchanged': True,
            'active_playback_may_finish': True}


def main(request):
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
