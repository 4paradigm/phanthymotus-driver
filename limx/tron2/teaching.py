"""TRON2 adapter for the G1 teaching workbench and asset lifecycle."""
import asyncio
import hashlib
import json
import math
import os
from pathlib import Path
import threading
from uuid import uuid4

from common.teaching import TeachingPlugin
from common.teaching.package import SCHEMA as PACKAGE_SCHEMA
from episode import (SCHEMA, TRAJECTORY_SCHEMA, STATE_NAMES, UNITS, atomic_json, digest,
                     encode, episode_path, load_episode)
from teaching_capture import Capture
from teaching_replay import Replay, pack_trajectory, read_trajectory


class TronTeachingAdapter:
    title='TRON2 原厂 VR 录制与回放'
    resources=['arm_l','arm_r','head']
    completion_timeout=1810
    presentation=dict(passive_recording=True, session_start='observer',
        joint_groups=[{'label':'左臂','count':7},{'label':'右臂','count':7}], grippers=False,
        recording_label='实测双臂与夹爪', prepare_hint='只读检查反馈和存储；不接管原厂 VR。',
        next_hint='使用原厂 VR 示范。录制器只读取关节、夹爪与三路图像。',
        trial_hint='训练数据可直接导出。回放前由操作者确认原厂 VR 和其他控制程序已停止，并保持控制独占。',
        conditions_placeholder='固定底座、相同物体布置、原厂 VR 已退出、控制来源独占。')
    replay_instruction='TRON2 不自动回起点或切换模式。每次执行须由操作者确认原厂 VR 和其他控制程序已停止，并明确填写 exclusive_control_confirmed=true；软件不能检测跨进程控制独占。夹爪回放须明确 gripper_speed 和 gripper_force（厂家档位）。停止发送不等于物理停止。'

    def __init__(self, owner, loop, root):
        self.owner,self.loop,self.root=owner,loop,Path(root)
        self.capture=Capture(owner.reader, owner.session, owner.settings['camera_urls'], self.root/'episodes')
        self.replay=Replay(owner.reader,None,self.peers_idle)
        from completion import MotionPeers
        owner.motion.peers.append(self.replay)
        owner.head.peers.append(self.replay)
        for gripper in owner.grippers.values():
            gripper.peer=MotionPeers(owner.motion,owner.head,self.replay)

    def sync(self, coroutine):
        return asyncio.run_coroutine_threadsafe(coroutine,self.loop).result(20)

    def peers_idle(self):
        owners=[self.owner.motion,self.owner.head,*self.owner.grippers.values()]
        return all(not owner.active and not getattr(owner,'recovering',False) and not getattr(owner,'pose_step',None) for owner in owners)

    def compatibility(self):
        binding=self.capture.binding()
        return dict(manufacturer='limx',model='TRON2',variant='DACH_TRON2A',scope='fixed_base_dual_arm',
            end_effectors='dual_limx_2fclaw',joint_names=STATE_NAMES,units=UNITS,
            model_sha256=binding['model_sha256'],driver='limx/tron2',tool='tron2_teaching',
            package_api=PACKAGE_SCHEMA,recording_schema=TRAJECTORY_SCHEMA,requires_local_trial=True)

    def recordings(self):
        result=[]
        for path in sorted((self.root/'episodes').glob('*/manifest.json'),reverse=True)[:200]:
            try:
                if path.is_symlink() or path.stat().st_size>65536:continue
                m=json.loads(path.read_bytes())
                result.append(dict(filename=path.parent.name, complete=m.get('complete') is True and (path.parent/'complete.json').is_file(),
                    training_complete=m.get('training_complete',False),task=m.get('task'),error=m.get('error')))
            except (OSError,ValueError):continue
        return result

    def info(self):
        rec=self.capture.info()
        busy=rec['recording'] or bool(self.replay.active)
        return dict(state='recording' if rec['recording'] else 'idle',busy=busy,can_prepare=not busy,
            can_record=not busy and self.owner.reader.sample('joint_state')['available'],recording=rec,
            recordings=self.recordings(),teleop_hint='录制器不发送运动命令。停止录制不停止原厂 VR。',
            replay_available=True,control_confirmation='operator_per_execution',last_action=self.replay.outcome)

    def check(self):
        checks=[]
        for key,label,operation in (
            ('feedback','双臂及头部反馈',lambda: self.owner.reader.sample('joint_state')['available']),
            ('binding','设备与模型绑定',lambda: bool(self.capture.binding())),
            ('storage','录制目录可写',self._storage)):
            try:
                ok=operation() is True
                checks.append(dict(id=key,label=label,ok=ok,reason=None if ok else 'unavailable'))
            except (OSError,ValueError) as exc:
                checks.append(dict(id=key,label=label,ok=False,reason=str(exc)))
        ready=all(c['ok'] for c in checks)
        checks.extend(self.sync(self.capture.probe()))
        return dict(ready=ready,training_ready=all(c['ok'] for c in checks),checks=checks,read_only=True,motion_sent=False,
            camera_and_gripper_validation='每帧录制时检查；缺失不能成为完整训练 episode',
            control_confirmation='operator_per_execution')

    def _storage(self):
        self.root.mkdir(parents=True,exist_ok=True,mode=0o700)
        return os.access(self.root,os.W_OK)

    def begin(self,args):
        raise ValueError('use_original_vendor_vr_recording_does_not_take_control')

    def record(self,args):
        if self.replay.busy:raise ValueError('replay_busy')
        return self.sync(self.capture.start(args.get('task',''),args.get('outcome','unlabelled'),args.get('notes','')))

    def finish(self):
        return self.sync(self.capture.finish())

    def stop(self):
        result=self.sync(self.replay.stop())
        result['recording']=self.sync(self.capture.finish())
        return result

    def raw(self,filename):
        path=episode_path(self.root/'imports',filename).with_suffix('.jsonl')
        if path.exists():
            if path.is_symlink() or path.stat().st_size>64*1024*1024:raise ValueError('invalid_import_recording')
            raw=path.read_bytes()
            read_trajectory(raw,self.capture.binding())
            return raw,None
        _,manifest,states=load_episode(self.root/'episodes',filename,self.capture.binding())
        return pack_trajectory(manifest['binding'],states),manifest

    def inspect(self,filename):
        # Inspection of complete diagnostic episodes must remain possible without valid replay.
        imported=episode_path(self.root/'imports',filename).with_suffix('.jsonl').exists()
        if imported:
            raw,manifest=self.raw(filename);header,frames=read_trajectory(raw,self.capture.binding())
            binding=header['binding'];sha=hashlib.sha256(raw).hexdigest()
        else:
            path,manifest,states=load_episode(self.root/'episodes',filename,self.capture.binding())
            binding=manifest['binding'];sha=digest(path/'trajectory.jsonl')
            start=states[0]['received_ns']
            frames=[dict(t_ns=s['received_ns']-start,q=s['q'][:14],gripper=s.get('gripper')) for s in states]
        stride=max(1,len(frames)//200)
        return dict(filename=filename,sha256=sha,binding=binding,complete=True,
            duration_seconds=frames[-1]['t_ns']/1e9,frames=len(frames),
            training_complete=bool(manifest and manifest.get('training_complete')),
            task=manifest.get('task') if manifest else None,outcome=manifest.get('outcome') if manifest else None,
            notes=manifest.get('notes') if manifest else None,quality=manifest.get('quality') if manifest else None,
            preview=[{'t':f['t_ns']/1e9,'kind':'measured','joints':[math.degrees(q) for q in f['q']],
                      'gripper_raw':f['gripper']} for f in frames[::stride]])

    def start_difference(self,filename):
        try:
            raw,_=self.raw(filename)
            frames,_=self.sync(self.replay.preflight(raw,self.capture.binding()))
            feedback=self.owner.reader.sample('joint_state')
            if not feedback['available']:raise ValueError('replay_feedback_stale')
            delta=[a-b for a,b in zip(feedback['data']['q'][:14],frames[0]['q'])]
            return dict(ready=True,can_trial=True,
                message='起点与设备预检通过；执行前须人工确认原厂 VR 已退出、其他控制程序已停止。',
                joint_delta_deg=[math.degrees(v) for v in delta],
                target_joints_deg=[math.degrees(v) for v in frames[0]['q']],tolerance_label='2°；不自动回起点')
        except (ValueError,OSError) as exc:
            return dict(ready=False,can_trial=False,error=str(exc),message=str(exc))

    def play(self,filename,args):
        if self.capture.info()['recording']:raise ValueError('finish_recording_first')
        raw,_=self.raw(filename)
        return self.sync(self.replay.play(raw,self.capture.binding(),args))

    def wait(self,ident):
        result=asyncio.run_coroutine_threadsafe(self.replay.wait(ident),self.loop).result(1810)
        asyncio.run_coroutine_threadsafe(self.owner._report_completion('tron2_teaching',ident,result['status'],result['result']),self.loop).result(10)
        return result

    def export_recording(self,recording):
        # The outer G1 package stays exactly manifest.json + trajectory.jsonl.
        raw,_=self.raw(recording['filename'])
        header,frames=read_trajectory(raw,self.capture.binding())
        header['binding']={**header['binding'],'serial':'requires_local_binding'}
        out=encode(header)+b''.join(encode(f) for f in frames)
        return out+encode(dict(type='complete',frames=len(frames),sha256=hashlib.sha256(out).hexdigest()))

    def import_recording(self,manifest,data):
        if manifest['compatibility']!=self.compatibility() or manifest['recording']['schema']!=TRAJECTORY_SCHEMA:
            raise ValueError('incompatible_tron2_package')
        header,frames=read_trajectory(data)
        current=self.capture.binding()
        if header['binding'].get('model_sha256')!=current['model_sha256']:
            raise ValueError('trajectory_model_mismatch')
        header['binding']=current
        raw=encode(header)+b''.join(encode(f) for f in frames)
        raw+=encode(dict(type='complete',frames=len(frames),sha256=hashlib.sha256(raw).hexdigest()))
        ident=uuid4().hex;directory=self.root/'imports';directory.mkdir(parents=True,exist_ok=True,mode=0o700)
        with (directory/(ident+'.jsonl')).open('xb') as stream:
            stream.write(raw);stream.flush();os.fsync(stream.fileno())
        return self.inspect(ident)


class TronTeachingPlugin(TeachingPlugin):
    def __init__(self,adapter):
        super().__init__(adapter,adapter.root/'assets','tron2_teaching')
        self.export_job=None

    def get_tools(self):
        tools=super().get_tools();schema=tools[0]['inputSchema']
        schema['properties'].update({k:{'type':'string'} for k in ('task','outcome','notes','episode_id')})
        schema['properties'].update({k:{'type':'number','minimum':0,'maximum':100} for k in ('gripper_speed','gripper_force')})
        schema['properties']['exclusive_control_confirmed']={'type':'boolean','description':'操作者明确确认本次原厂 VR 及其他控制程序已停止，并在回放期间保持独占；不得自行填写 true'}
        extra={'episode_list':([], '列出完整及诊断录制'),
            'episode_details':(['episode_id'],'查看数据质量'),
            'annotate':(['episode_id','task','outcome','notes'],'更新任务、结果和备注'),
            'training_export':(['episode_id'],'异步导出 LeRobot v3；不要求回放'),
            'training_info':([], '读取训练导出结果')}
        for action,(_,description) in extra.items():
            schema['properties']['action']['enum'].append(action)
            schema['properties']['action']['oneOf'].append({'const':action,'title':description})
            schema['x-workbench']['actions'].append(action)
        schema['x-action-params']['run']['params'] += ['gripper_speed','gripper_force','exclusive_control_confirmed']
        schema['x-workbench']['title']='TRON2 录制与回放'
        tools[0]['description']='原厂 VR 只读数据录制、实测轨迹回放和 LeRobot v3 训练导出。回放每次须由操作者确认控制独占。'
        return tools

    def dispatch(self,action,args):
        self._validate(action,args)
        if action=='info':
            return {**super().dispatch(action,args),'training_export':self.export_job}
        if action=='episode_list':return {'episodes':self.adapter.recordings()}
        if action=='training_info':return {'export':self.export_job}
        if action in ('episode_details','annotate','training_export'):
            ident=args.get('episode_id')
            path=episode_path(self.adapter.root/'episodes',ident)
            if action=='episode_details':
                manifest=path/'manifest.json'
                if manifest.is_symlink() or manifest.stat().st_size>65536:raise ValueError('invalid_manifest')
                return {'episode':json.loads(manifest.read_bytes()),'integrity_verified':False}
            with self.lock:
                _,manifest,_=load_episode(self.adapter.root/'episodes',ident)
                if action=='annotate':
                    task,outcome,notes=args.get('task',''),args.get('outcome','unlabelled'),args.get('notes','')
                    if not task.strip() or len(task)>2000 or len(notes)>4000 or outcome not in ('success','failure','unlabelled'):
                        raise ValueError('invalid_episode_annotation')
                    manifest.update(task=task,outcome=outcome,notes=notes)
                    atomic_json(path/'manifest.json',manifest)
                    atomic_json(path/'complete.json',{'schema':SCHEMA,'manifest_sha256':digest(path/'manifest.json')})
                    return {'episode':manifest,'motion_sent':False}
                if self.export_job and self.export_job['state']=='running':raise ValueError('export_busy')
                if manifest.get('training_complete') is not True:raise ValueError('complete_three_camera_gripper_episode_required')
                destination=self.adapter.root/'datasets'/ident
                self.export_job={'state':'running','episode_id':ident,'path':str(destination)}
                threading.Thread(target=self._export,args=(ident,destination),name='tron2-vla-export',daemon=True).start()
                return {'export':self.export_job}
        result=super().dispatch(action,args)
        if action in ('preview','run') and result.get('executed') is False:
            result['execution_arguments'].update(exclusive_control_confirmed=False,
                gripper_speed=args.get('gripper_speed'),gripper_force=args.get('gripper_force'))
            result['next_step']='请操作者确认原厂 VR 和其他控制程序已停止、活动空间已清空，并明确夹爪速度和力档位。确认后补齐 execution_arguments，再执行。'
        return result

    def _export(self,ident,destination):
        try:
            from vla_export import export_dataset
            report=export_dataset(self.adapter.root/'episodes',ident,destination)
            self.export_job={'state':'completed','episode_id':ident,'path':str(destination),
                'frames':report['official_reader_frames'],'episodes':report['episodes'],'excluded':len(report['excluded']),
                'action_source':report['action_source'],'official_reader_verified':True}
        except Exception as exc:
            self.export_job={'state':'error','episode_id':ident,'path':str(destination),'error':str(exc)[:240]}
