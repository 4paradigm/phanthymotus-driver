"""Offline, causal 10 Hz export through the pinned official LeRobot v3 API."""
from bisect import bisect_right
import importlib.metadata
import io
import json
from pathlib import Path

from episode import CAMERAS, STATE_NAMES, UNITS, atomic_json, load_episode, read_lines

LEROBOT_VERSION = '0.4.4'
STEP_NS = 100_000_000
IMAGE_ERROR_NS = 50_000_000
MAX_STATE_GAP_NS = 100_000_000


def interpolate(states, times, t):
    i = bisect_right(times, t)-1
    if i < 0:
        raise ValueError('no_past_state')
    a = states[i]
    if times[i] == t:
        q = a['q']
    else:
        if i+1 >= len(states) or times[i+1]-times[i] > MAX_STATE_GAP_NS:
            raise ValueError('state_gap')
        b = states[i+1]
        ratio = (t-times[i])/(times[i+1]-times[i])
        q = [x+(y-x)*ratio for x,y in zip(a['q'],b['q'])]
    if a.get('gripper') is None or not 0 <= t-a['gripper_received_ns'] <= MAX_STATE_GAP_NS:
        raise ValueError('gripper_missing_or_stale')
    return q[:14]+a['gripper'], q[14:]


def matched_rows(path, manifest, states):
    """Never select an image or discrete gripper observation from the future."""
    times = [s['received_ns'] for s in states]
    images = {c:list(read_lines(path/(c+'.jsonl'))) for c in CAMERAS}
    for camera, rows in images.items():
        previous, offset = -1, 0
        for sequence, row in enumerate(rows):
            if (type(row.get('received_ns')) is not int or row['received_ns'] <= previous
                    or row.get('sequence') != sequence or row.get('offset') != offset
                    or type(row.get('bytes')) is not int or not 4 <= row['bytes'] <= 4*1024*1024):
                raise ValueError('invalid_camera_index: '+camera)
            previous = row['received_ns']; offset += row['bytes']
        if offset != manifest['files'][camera+'.mjpeg']['bytes']:
            raise ValueError('camera_index_size_mismatch')
    image_times = {c:[r['received_ns'] for r in rows] for c,rows in images.items()}
    kept, excluded = [], []
    source = manifest['action_source']
    if source not in ('measured_future_state', 'vendor_target'):
        raise ValueError('unsupported_action_source')
    for t in range(times[0], times[-1]+1, STEP_NS):
        try:
            state, head = interpolate(states, times, t)
            selected = {}
            for camera, rows in images.items():
                index = bisect_right(image_times[camera], t)-1
                if index < 0 or t-image_times[camera][index] > IMAGE_ERROR_NS:
                    raise ValueError('camera_gap: '+camera)
                selected[camera] = rows[index]
            if source == 'measured_future_state':
                action, _ = interpolate(states, times, t+STEP_NS)
                action_time = t+STEP_NS
            else:
                target = states[bisect_right(times,t)-1].get('vendor_target')
                if target is None or not 0 <= t-target['received_ns'] <= MAX_STATE_GAP_NS:
                    raise ValueError('vendor_target_missing_or_stale')
                action, action_time = target['q'], target['received_ns']
            kept.append(dict(source_ns=t, state=state, head=head, action=action,
                             action_ns=action_time, images=selected))
        except ValueError as exc:
            excluded.append({'source_ns':t,'reason':str(exc)})
    # Split gaps into separate training episodes; no compressed time across gaps.
    segments = []
    for row in kept:
        if not segments or row['source_ns']-segments[-1][-1]['source_ns'] != STEP_NS:
            segments.append([])
        segments[-1].append(row)
    return segments, excluded


def export_dataset(root, ident, destination):
    if importlib.metadata.version('lerobot') != LEROBOT_VERSION:
        raise ValueError('requires_lerobot_'+LEROBOT_VERSION)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import numpy as np
    from PIL import Image

    path, manifest, states = load_episode(root, ident)
    if manifest.get('training_complete') is not True:
        raise ValueError('complete_three_camera_gripper_episode_required')
    segments, excluded = matched_rows(path, manifest, states)
    if not segments:
        raise ValueError('no_synchronized_training_frames')
    destination = Path(destination)
    if destination.exists():
        raise ValueError('export_destination_exists')
    features = {
        'observation.state':{'dtype':'float32','shape':(16,), 'names':STATE_NAMES},
        'action':{'dtype':'float32','shape':(16,), 'names':STATE_NAMES},
        'observation.head':{'dtype':'float32','shape':(2,), 'names':['pitch','yaw']},
        'source_time_ns':{'dtype':'int64','shape':(1,), 'names':None},
        'action_time_ns':{'dtype':'int64','shape':(1,), 'names':None},
        'outcome':{'dtype':'int64','shape':(1,), 'names':None},
    }
    for camera in CAMERAS:
        w,h = manifest['cameras'][camera]['size']
        features['observation.images.'+camera] = {'dtype':'video', 'shape':(h,w,3), 'names':['height','width','channels']}
    dataset = LeRobotDataset.create(repo_id='local/tron2-'+ident, fps=10, root=destination,
        robot_type='limx_tron2_fixed_base', features=features, use_videos=True, image_writer_threads=2, vcodec='h264', encoder_threads=1)
    report = dict(schema='motus.tron2.export/1', source_episode=ident, source_manifest=manifest,
        lerobot_version=LEROBOT_VERSION, action_source=manifest['action_source'],
        derived_interval_seconds=.1 if manifest['action_source']=='measured_future_state' else None,
        units=UNITS, fps=10, image_policy='nearest_past_receive_time_max_50ms',
        state_interpolation_max_gap_ms=100, excluded=excluded, segments=[], complete=False)
    try:
        for segment in segments:
            for row in segment:
                frame = {'observation.state':np.asarray(row['state'],dtype=np.float32),
                    'action':np.asarray(row['action'],dtype=np.float32),
                    'observation.head':np.asarray(row['head'],dtype=np.float32),
                    'source_time_ns':np.asarray([row['source_ns']],dtype=np.int64),
                    'action_time_ns':np.asarray([row['action_ns']],dtype=np.int64),
                    'outcome':np.asarray([{'unlabelled':-1,'failure':0,'success':1}[manifest['outcome']]],dtype=np.int64),
                    'task':manifest['task']}
                for camera, index in row['images'].items():
                    with (path/(camera+'.mjpeg')).open('rb') as stream:
                        stream.seek(index['offset']); raw = stream.read(index['bytes'])
                    with Image.open(io.BytesIO(raw)) as image:
                        frame['observation.images.'+camera] = np.asarray(image.convert('RGB'))
                dataset.add_frame(frame)
            dataset.save_episode()
            report['segments'].append({'first_source_ns':segment[0]['source_ns'],
                'last_source_ns':segment[-1]['source_ns'], 'frames':len(segment)})
        dataset.finalize()
        # Exercise the actual official reader and decode every retained camera frame.
        loaded = LeRobotDataset(repo_id='local/tron2-'+ident, root=destination, video_backend='pyav')
        for i in range(len(loaded)):
            sample = loaded[i]
            if tuple(sample['observation.state'].shape) != (16,) or tuple(sample['action'].shape) != (16,):
                raise ValueError('official_reader_state_shape')
            for camera in CAMERAS:
                w,h = manifest['cameras'][camera]['size']
                if tuple(sample['observation.images.'+camera].shape) != (3,h,w):
                    raise ValueError('official_reader_video_shape')
        if 'action' not in loaded.meta.stats or 'observation.state' not in loaded.meta.stats:
            raise ValueError('official_reader_missing_statistics')
        report.update(complete=True, official_reader_frames=len(loaded), episodes=len(segments))
    except Exception as exc:
        report['error'] = type(exc).__name__+': '+str(exc)
        atomic_json(destination/'export-report.json', report)
        raise
    atomic_json(destination/'export-report.json', report)
    return report


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode_root'); parser.add_argument('episode_id'); parser.add_argument('destination')
    args = parser.parse_args()
    print(json.dumps(export_dataset(args.episode_root, args.episode_id, args.destination), ensure_ascii=False))
