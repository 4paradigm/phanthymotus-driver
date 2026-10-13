"""Readable joint-angle tiles using the existing Motus data/json renderer."""
from skeleton import JOINT_LABELS, skeleton_stream


ANGLE_FIELDS = tuple(
    ('左臂' if i < 7 else '右臂' if i < 14 else '头部')
    + '_J' + str(i) + '_' + (label[2:] if label.startswith('头部') else label) + '_度'
    for i, label in enumerate(JOINT_LABELS))


def angle_sample(source):
    """Use the skeleton's validated name/index map and the same raw feedback."""
    skeleton = skeleton_stream(source)
    result = {**source, **skeleton['status'], 'data': None}
    if result['available']:
        data = {name: None for name in ANGLE_FIELDS}
        for joint in skeleton['joints']:
            data[ANGLE_FIELDS[joint['idx']]] = joint['q_deg']
        result['data'] = data
    return result


def angles_stream(sample):
    # Always publish the same sixteen keys: the old frontend retains cells for
    # absent keys, so an empty packet would leave old angles on screen.
    return dict(sample['data']) if sample['available'] else dict.fromkeys(ANGLE_FIELDS)
