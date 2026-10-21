"""Episode failure paths and official LeRobot writer/reader integration."""
import io
import json
from pathlib import Path
import sys
import threading
from unittest.mock import patch

import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from episode import CAMERAS, EpisodeWriter, load_episode
from vla_export import export_dataset, matched_rows

BINDING = {'serial':'test-device','model':'DACH_TRON2A','model_sha256':'f'*64,'firmware':'test'}


def jpeg():
    out = io.BytesIO()
    Image.new('RGB',(64,48),(20,80,120)).save(out,format='JPEG')
    return out.getvalue()


def state(i, t=None):
    t = 1_000_000_000+i*20_000_000 if t is None else t
    return dict(sequence=i, received_ns=t, q=[i*.0001]*14+[0,0],
        gripper=[0,20],gripper_received_ns=t-1_000_000, source_timestamp=i*20,
        vendor_target=None)


def episode(tmp_path, count=51, missing=None):
    writer = EpisodeWriter(tmp_path,BINDING,'Put the block in the tray', outcome='success')
    raw = jpeg()
    for i in range(count):
        frame = state(i)
        if missing == 'gripper':frame['gripper']=None
        assert writer.put('state',frame)
        if i%2 == 0:
            for c in CAMERAS:
                if c != missing:assert writer.put(c,dict(received_ns=frame['received_ns'],jpeg=raw))
    result=writer.finish()
    assert result['complete'],result
    return writer


def test_complete_integrity_binding_and_corruption(tmp_path):
    writer = episode(tmp_path)
    _,manifest,states = load_episode(tmp_path,writer.ident,BINDING)
    assert manifest['training_complete'] and len(states)==51
    with pytest.raises(ValueError,match='binding'):
        load_episode(tmp_path,writer.ident,dict(BINDING,serial='other'))
    with (writer.path/'head.mjpeg').open('ab') as f:f.write(b'x')
    with pytest.raises(ValueError,match='checksum'):
        load_episode(tmp_path,writer.ident)


@pytest.mark.parametrize('missing',['gripper','right_wrist'])
def test_incomplete_training_is_diagnostic(tmp_path,missing):
    writer=episode(tmp_path,missing=missing)
    assert not writer.manifest['training_complete']
    with pytest.raises(ValueError,match='complete_three'):
        export_dataset(tmp_path,writer.ident,tmp_path/'dataset')


def test_disk_exhaustion_preserves_partial(tmp_path):
    writer=EpisodeWriter(tmp_path,BINDING,'test',reserve_bytes=10**30)
    writer.put('state',state(0));writer.finish()
    assert not writer.manifest['complete']
    assert 'disk_reserve' in writer.error
    assert not (writer.path/'complete.json').exists()
    assert writer.path.exists()


def test_queue_overflow_and_source_order(tmp_path):
    with patch.object(threading.Thread,'start'):
        writer=EpisodeWriter(tmp_path,BINDING,'test',capacity=1)
        assert writer.put('state',state(0))
        assert not writer.put('state',state(1))
        assert writer.error=='writer_queue_overflow'
    writer=EpisodeWriter(tmp_path,BINDING,'test')
    writer.put('state',state(1));writer.put('state',state(0));writer.finish()
    assert not writer.manifest['complete']
    assert 'nonadvancing' in writer.error


def test_causality_and_future_tail(tmp_path):
    writer=episode(tmp_path)
    path,manifest,states=load_episode(tmp_path,writer.ident)
    segments,excluded=matched_rows(path,manifest,states)
    rows=[r for s in segments for r in s]
    assert len(rows)==10
    assert rows[0]['action'][0]==pytest.approx(.0005)
    assert all(r['action_ns']-r['source_ns']==100_000_000 for r in rows)
    assert all(i['received_ns']<=r['source_ns'] for r in rows for i in r['images'].values())
    assert excluded[-1]['source_ns']==states[-1]['received_ns']
    states=[s for s in states if not 10<=s['sequence']<=20]
    segments,excluded=matched_rows(path,manifest,states)
    assert len(segments)==2
    assert any(r['reason']=='state_gap' for r in excluded)


def test_official_lerobot_v3_roundtrip(tmp_path):
    writer=episode(tmp_path)
    report=export_dataset(tmp_path,writer.ident,tmp_path/'dataset')
    assert report['complete'] and report['official_reader_frames']==10
    assert report['action_source']=='measured_future_state'
    info=json.loads((tmp_path/'dataset/meta/info.json').read_text())
    assert info['codebase_version']=='v3.0'
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from torch.utils.data import DataLoader
    ds=LeRobotDataset('local/tron2-'+writer.ident,root=tmp_path/'dataset',video_backend='pyav')
    batch=next(iter(DataLoader(ds,batch_size=3)))
    assert batch['observation.state'].shape==(3,16)
    assert batch['action'].shape==(3,16)
    assert batch['task']==['Put the block in the tray']*3
    assert all(batch['observation.images.'+c].shape==(3,3,48,64) for c in CAMERAS)
    with pytest.raises(ValueError,match='destination_exists'):
        export_dataset(tmp_path,writer.ident,tmp_path/'dataset')
