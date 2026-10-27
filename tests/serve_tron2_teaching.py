"""Local browser fixture: real teaching HTTP/recording, fake vendor WebSocket (no hardware)."""
import argparse
import asyncio
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'limx/tron2'),str(ROOT/'limx/tron2/tests')]
import aiohttp
from aiohttp import web
from device import Adapter
from test_teaching_runtime import Reader
from test_teaching_episode import jpeg
from test_sensors import settings
from skeleton import JOINT_NAMES
import common.teaching.service as teaching_service

HOST='''<!doctype html><html><head><title>TRON2 teaching local fixture</title></head><body style="margin:0"><p style="margin:0;padding:8px;background:#fff3ce">本机假厂家验收；不连接机器人。</p><iframe title="TRON2 示教" src="/workbench" style="width:100%;height:calc(100vh - 40px);border:0"></iframe><script>
const frame=document.querySelector('iframe');
frame.onload=()=>{const channel=new MessageChannel();channel.port1.onmessage=async e=>{
const {id,method,args}=e.data;
try{let result;if(method==='context')result={conversation_ready:false,tool:'tron2_teaching'};
else{const driverArgs={...args};delete driverArgs.auto_execute;
const r=await fetch('/mcp',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({jsonrpc:'2.0',id:1,method:'tools/call',params:{name:'tron2_teaching',arguments:method==='save'?{action:'save',...driverArgs}:driverArgs}})});
const b=await r.json();if(b.error)throw Error(b.error.message);result=JSON.parse(b.result.content[0].text);if(b.result.isError)throw Error(result.error);
if(method==='save' && typeof args.auto_execute==='boolean')await fetch('/fixture/grant',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({asset_id:result.asset_id,enabled:args.auto_execute})});}
channel.port1.postMessage({id,result});}catch(err){channel.port1.postMessage({id,error:err.message});}};
frame.contentWindow.postMessage({protocol:'motus.workbench/1'},location.origin,[channel.port2]);};
</script></body></html>'''


async def run(port,root):
    reader=Reader();raw=jpeg();fail_camera=False
    grants={}
    def policy(tool,instance_id,device_binding,asset_id,asset_binding):
        enabled=grants.get(asset_id)==(device_binding,asset_binding)
        return dict(enabled=enabled,source='skill' if enabled else 'manual',binding_changed=False)
    teaching_service.read_execution_policy=policy
    async def camera(request):
        if fail_camera:return web.Response(status=503)
        response=web.StreamResponse(headers={'Content-Type':'multipart/x-mixed-replace; boundary=frame'})
        await response.prepare(request)
        try:
            while not fail_camera:
                await response.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(raw)).encode()+b'\r\n\r\n'+raw+b'\r\n')
                await asyncio.sleep(.033)
        except (ConnectionResetError,asyncio.CancelledError):pass
        return response
    async def failure(request):
        nonlocal fail_camera
        fail_camera=not fail_camera
        return web.json_response({'camera_failed':fail_camera,'motion_commands':len(reader.commands)})
    async with aiohttp.ClientSession() as session:
        cfg=settings(enabled_cards=['joint_state','tron2_teaching'])
        cfg.update(teaching_directory=root,camera_urls={c:f'http://127.0.0.1:{port}/camera' for c in ('camera','camera_left','camera_right')})
        reader.settings = {'url': f'ws://127.0.0.1:{port}'}
        adapter=Adapter(lambda *a:None,cfg,reader=reader,session=session)
        async def home(request):
            if request.headers.get('Upgrade', '').lower() != 'websocket':
                return web.Response(text=HOST,content_type='text/html')
            ws=web.WebSocketResponse();await ws.prepare(request)
            await ws.send_json(dict(title='notify_robot_info',accid=reader.serial,data={}))
            async for message in ws:
                packet=json.loads(message.data);title=packet['title'];data=packet.get('data',{})
                reply={'result':'success'}
                if title=='request_get_joint_state':reply['q']=reader.q[:]
                elif title=='request_get_limx_2fclaw_state':
                    reply.update(left_opening=reader.grip[0],right_opening=reader.grip[1])
                elif title=='request_movej':reader.q[:14]=data['joint']
                elif title=='request_moveh':reader.q[14:]=data['joint']
                elif title=='request_servoj':reader.q[:]=data['q']
                elif title=='request_set_limx_2fclaw_cmd':
                    reader.grip[:]=[data['left_opening'],data['right_opening']]
                if not title.startswith('request_get_'):reader.commands.append(packet)
                await ws.send_json(dict(title=title.replace('request_','response_'),accid=reader.serial,
                    guid=packet.get('guid'),timestamp=time.monotonic(),data=reply))
            return ws
        async def workbench(request):return web.FileResponse(ROOT/'limx/tron2/workbench.html')
        async def wait(request):
            # Server clock keeps background-browser checks independent of timer throttling.
            await asyncio.sleep(min(5000,max(0,int(request.query.get('ms','100'))))/1000)
            return web.Response(text='ready')
        async def grant(request):
            data=await request.json()
            binding=await asyncio.to_thread(adapter.teaching._execution_binding,data['asset_id'])
            grants[data['asset_id']]=(binding['device_binding'],binding['asset_binding']) if data['enabled'] else None
            return web.json_response({'fixture_only':True})
        app=web.Application(client_max_size=17*1024*1024)
        app.router.add_get('/',home);app.router.add_get('/workbench',workbench);app.router.add_get('/camera',camera)
        app.router.add_post('/mcp',adapter.rpc);app.router.add_post('/file/upload',adapter.teaching_upload)
        app.router.add_post('/fixture/camera-failure',failure)
        app.router.add_get('/fixture/wait',wait)
        app.router.add_post('/fixture/grant',grant)
        runner=web.AppRunner(app);await runner.setup();await web.TCPSite(runner,'127.0.0.1',port).start()
        print(f'http://127.0.0.1:{port} fixture_root={root}',flush=True)
        try:
            while True:
                if reader.teaching_observer:
                    reader.teaching_observer(dict(q=reader.q,names=JOINT_NAMES,dq=[0]*16,tau=[0]*16,
                        robot_timestamp=time.monotonic()*1000),time.monotonic_ns())
                await asyncio.sleep(.02)
        finally:
            await adapter.teaching.adapter.replay.stop()
            await adapter.teaching.adapter.capture.finish();await runner.cleanup()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=15892);p.add_argument('--root',required=True)
    args=p.parse_args();asyncio.run(run(args.port,args.root))
