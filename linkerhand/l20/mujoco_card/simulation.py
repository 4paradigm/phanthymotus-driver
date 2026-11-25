"""Motus L20 trajectory player, built entirely on virtual joint positions."""
import os
from http.server import ThreadingHTTPServer
from pathlib import Path
import signal
import threading
import yaml

from common.vendor_runtime import DriverBundle, action_schema, make_handler, tool
from hand_model import HandModel
from playback import Playback
from trajectories import FORMAT, SIDE_CONFIG, SIDES, SimulatedMotionCard, MotionPlanner, instance

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


def main():
    from sim_bus import SimulationBus, deny_hardware_sockets
    deny_hardware_sockets()
    models={s:HandModel(HERE/'vendor.zip',s) for s in SIDES}
    config=yaml.safe_load(Path('/work/config.yaml').read_text())
    planner=MotionPlanner(config)
    bus=SimulationBus(int(os.getenv('ROS_DOMAIN_ID','42')))
    rt=Playback(models,planner,bus,os.getenv('SIM_STATE_DIR','/var/lib/l20-sim'))
    bundle=DriverBundle([Simulation(rt),*(SimulatedMotionCard(n,rt,planner) for n in ('hand_grasp','hand_gesture'))])
    server=ThreadingHTTPServer((os.getenv('MCP_BIND','127.0.0.1'),int(os.getenv('MCP_PORT','15769'))),
                    make_handler(lambda:bundle,'L20 仅仿真','linkerhand-g20-simulation'))
    def halt(*_):
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,halt);signal.signal(signal.SIGINT,halt)
    try:server.serve_forever()
    finally:rt.shutdown();bus.shutdown();server.server_close()


if __name__=='__main__': main()
