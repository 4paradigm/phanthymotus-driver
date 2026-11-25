"""Wire format, topics and model-side defaults for L20 joint trajectories."""
import re

FORMAT = 'data/l20_joint_trajectory'
VERSION = 'motus.l20.trajectory/1'
SIDES = ('left', 'right')
FPS = 20
SIDE_CONFIG = {'type': 'object', 'properties': {'bound_side': {
    'type': 'string', 'enum': list(SIDES), 'default': 'right', 'scope': 'instance',
    'title': '模拟手', 'description': '一个实例绑定一只虚拟手。'}}}


def instance(args):
    value = args.get('instance_id', '')
    if not isinstance(value, str) or not re.fullmatch(r'[a-zA-Z0-9_-]{1,100}', value):
        raise ValueError('请在画布实例上使用此动作')
    return value


def channel(side, name, key):
    # Encode the original instance identity without ROS name collisions.
    return f'/l20_sim/{side}/{name}/i_{key.encode().hex()}'


def validate_packet(packet):
    if not isinstance(packet, dict) or packet.get('schema') != VERSION or packet.get('side') not in SIDES:
        raise ValueError('轨迹协议或左右手无效')
    for field in ('id', 'action'):
        if not isinstance(packet.get(field), str) or not 1 <= len(packet[field]) <= 100:
            raise ValueError('轨迹缺少动作标识')
    frames, ms = packet.get('frames'), packet.get('frame_ms')
    if type(ms) is not int or not 20 <= ms <= 1000:
        raise ValueError('frame_ms 允许 20–1000 毫秒')
    if not isinstance(frames, list) or not 1 <= len(frames) <= 600:
        raise ValueError('一次动作需要 1–600 帧')
    if any(not isinstance(f, list) or len(f) != 20 or
           any(type(v) is not int or not 0 <= v <= 255 for v in f) for f in frames):
        raise ValueError('每帧应为 L20 的 20 个 0–255 整数位置')
    return packet


def initial_poses(config):
    """Starting poses for a newly added virtual L20, independent of its first trajectory."""
    hand = config.get("plugins", {}).get("hand", {})
    default = hand.get("open_pose", [255, 255, 255, 255, 255, 70, 200, 132, 55, 0] + [255] * 10)
    return {side: list(hand.get(f"{side}_open_pose", default)) for side in SIDES}
