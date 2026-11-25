"""Motus L20 trajectory player, built entirely on virtual joint positions."""
from pathlib import Path

from common.vendor_runtime import action_schema, tool
from mujoco_card.trajectories import FORMAT, SIDE_CONFIG, instance

HERE=Path(__file__).resolve().parent
CARD='l20_mujoco'
PROTOCOL='motus.workbench/1'


def input_topics(args):
    topics=args.get('input_topics')
    if topics is None:
        topics=[args['input_topic']] if args.get('input_topic') else []
    return topics


class Simulation:
    def __init__(self,runtime):
        self.runtime=runtime

    def get_tool(self):
        actions={'view':([], '打开虚拟手三维工作台'),'info':([], '虚拟手状态'),'stop':([], '停止模拟'),
                 'start':([], '绑定画布输入轨迹'),'config':([], '绑定模拟侧别'),'geometry':([], '已校准网格'),
                 'live':([], '当前姿态和最近完整动作'),'inspect':([], '最近完整动作的帧')}
        schema=action_schema(actions,{})
        schema['x-workbench']={'protocol':PROTOCOL,'kind':'simulation','title':'L20 · 轨迹仿真',
                               'entry':'view','actions':['info','stop','geometry','live','inspect']}
        schema['x-action-params']={'view':schema['x-action-params']['view']}
        result=tool(CARD,'processor','仅仿真 · 播放连入的 L20 关节轨迹，输出实时虚拟手画面',schema,
                    topic_in=[{'format':FORMAT}],topic_out=[{'format':'image/jpeg'}])
        result.update(title='L20 轨迹仿真',multiInstance=True,configSchema=SIDE_CONFIG)
        return result

    def dispatch(self,action,args):
        if action=='view':
            return {'protocol':PROTOCOL,'html':(HERE/'workbench.html').read_text()}
        if not args.get('instance_id') and action in ('info','start','stop','config'):
            return {'state':'ready','busy':False,'hardware':False,'topic_in':[], 'topic_out':[]}
        key=instance(args)
        side=self.runtime.side_for(CARD,key,args,configuring=action=='config')
        model=self.runtime.models[side]
        if action=='geometry':
            return {'side':side,'meshes':model.meshes}
        if action=='inspect':
            frames=self.runtime.replay(key,side,args.get('run_id'),args.get('offset',0),args.get('count',24))
            return {'frames':[model.transforms(frame) for frame in frames]}
        if action=='start':
            self.runtime.start(key,side,input_topics(args))
        elif action=='stop':
            self.runtime.stop(key)
        elif action not in ('config','info','live'):
            raise ValueError('未知仿真动作')
        r=self.runtime.read(key,side)
        if action=='live':r['poses']=model.transforms(r['pose'])
        # The workbench protocol asks info.busy to protect unsaved mutations.
        # Virtual playback may continue when the viewport closes.
        if action=='info':r['playing']=r['busy'];r['busy']=False
        return r
