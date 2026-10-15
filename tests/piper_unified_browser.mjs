/** Actual single control card and background streams; simulated SDK only. */
import assert from 'node:assert/strict';
import {spawnSync} from 'node:child_process';
import {createServer} from 'node:http';
import {readFile, mkdir, writeFile} from 'node:fs/promises';
import path from 'node:path';
import {createRequire} from 'node:module';
const require = createRequire(import.meta.url);
const {chromium} = require(process.env.PIPER_PLAYWRIGHT || 'playwright');
const core = path.resolve(process.env.PIPER_CORE_WEB);
const out = path.resolve(process.env.PIPER_MONITOR_EVIDENCE);
const uncalibrated = process.env.PIPER_BROWSER_UNCALIBRATED === '1';
await mkdir(out, {recursive:true});
const fixtureResult = spawnSync(process.env.PIPER_PYTHON, ['-c', `
import json,tempfile,sys
sys.path.insert(0,'tests')
from test_piper_intent import IntentDriver
from agilex.piper.device import build_plugins
from common.vendor_runtime import DriverBundle
with tempfile.TemporaryDirectory() as d:
 p=build_plugins({'data_dir':d,'name':'PiPER arm 1','async_actions':False,'intent_control':{'tcp_verified':__import__('os').environ.get('PIPER_BROWSER_UNCALIBRATED') != '1','tcp_offset_mm_deg':[0,0,0,0,0,0],'preset_joints_deg':None if __import__('os').environ.get('PIPER_BROWSER_UNCALIBRATED') == '1' else [1,10,-10,0,20,0]},'camera':{'live':{'enabled':True}}},'agilex',None)
 p[0].factory=IntentDriver;p[0].connected()
 try:
  import cv2,numpy as np,base64
  jpeg=base64.b64encode(cv2.imencode('.jpg',np.zeros((48,64,3),dtype=np.uint8))[1]).decode()
  print(json.dumps({'tools':DriverBundle(p).get_all_tools(),'frame':p[1].monitor.read(),'urdf':DriverBundle(p).dispatch('model',{})['urdf'],'jpeg':jpeg}))
 finally:p[0].stop()
`], {encoding:'utf8',env:{...process.env,PIPER_MOTION_ENABLED:'1'}});
assert.equal(fixtureResult.status,0,fixtureResult.stderr);
const fixture=JSON.parse(fixtureResult.stdout);
const requests=[];const controlResults=[];
function callFake(arguments_) {
 const run=spawnSync(process.env.PIPER_PYTHON,['-c',`
import sys,json,tempfile
from unittest.mock import patch
sys.path.insert(0,'tests')
from test_piper_intent import IntentDriver
from test_piper_unified import fast_execute
from agilex.piper.device import build_plugins
from common.vendor_runtime import DriverBundle
args=json.load(sys.stdin)
with tempfile.TemporaryDirectory() as d:
 p=build_plugins({'data_dir':d,'async_actions':False,'intent_control':{'tcp_verified':__import__('os').environ.get('PIPER_BROWSER_UNCALIBRATED') != '1','tcp_offset_mm_deg':[0,0,0,0,0,0],'preset_joints_deg':None if __import__('os').environ.get('PIPER_BROWSER_UNCALIBRATED') == '1' else [1,10,-10,0,20,0]},'camera':{'live':{'enabled':True}}},'agilex',None)
 p[0].factory=IntentDriver;driver=p[0].connected()
 try:
  with patch('agilex.piper.unified_control.execute_target',side_effect=fast_execute):
   result=DriverBundle(p).dispatch('arm_control',args)
  print(json.dumps({'result':result,'commands':driver.arm.calls,'frame':p[1].monitor.read()}))
 finally:p[0].stop()
`],{encoding:'utf8',input:JSON.stringify(arguments_),env:{...process.env,PIPER_MOTION_ENABLED:'1'}});
 assert.equal(run.status,0,run.stderr);return JSON.parse(run.stdout);
}
const topic='/agilex/piper/state/joints';
const params='/agilex/piper/state/parameters';
const camera='/agilex/piper/camera/live';
const mcp={id:'intent-arm1',name:'PiPER arm 1',online:true,tools:fixture.tools};
const layout={cards:[{id:'arm1-control',mcpId:mcp.id,toolName:'arm_control',driverName:'PiPER simulated arm',x:30,y:20,topicOut:fixture.tools.find(t=>t.name==='arm_control').topic_out}],connections:[]};
const server=createServer(async(req,res)=>{
 try{
  const url=new URL(req.url,'http://localhost');
  let data;
  if(url.pathname==='/api/mcp')data={data:[mcp]};
  else if(url.pathname==='/api/topics')data={data:[{topic,format:'sensor/skeleton',mcp_id:mcp.id},{topic:params,format:'data/json',mcp_id:mcp.id},{topic:camera,format:'image/jpeg',mcp_id:mcp.id}]};
  else if(url.pathname==='/favicon.ico'){res.statusCode=204;res.end();return;}
  else if(url.pathname==='/api/canvas/layout')data={data:layout};
  else if(url.pathname==='/api/config/project-running')data={running:false};
  else if(url.pathname==='/api/canvas/claim-edit'||url.pathname==='/api/canvas/keep-edit'||url.pathname==='/api/activity')data={code:200};
  else if(url.pathname===`/api/mcp/${mcp.id}/call`){
   let body='';for await(const chunk of req)body+=chunk;
   const call=JSON.parse(body);requests.push(call);
   if(call.tool==='model')data={code:200,data:[{type:'text',text:JSON.stringify({urdf:fixture.urdf})}]};
   else {assert.equal(call.tool,'arm_control');const result=callFake(call.arguments);controlResults.push(result);data={code:200,data:[{type:'text',text:JSON.stringify(result.result)}]};}
  }
  if(data){res.setHeader('Content-Type','application/json');res.end(JSON.stringify(data));return;}
  if(url.pathname==='/'){
   res.setHeader('Content-Type','text/html');
   res.end(`<meta charset="utf-8"><title>PiPER intent monitor — simulated feedback</title>
   <link rel="stylesheet" href="/css/style.css"><style>body{background:#eee;font:16px system-ui}#monitor-dashboard-grid{height:700px;margin-left:520px}#canvas-area{position:absolute;width:500px;height:850px}#canvas-viewport{position:absolute;inset:0}.canvas-card{width:410px}.monitor-card{background:white}.monitor-card-body{height:350px;max-height:350px;overflow:auto}</style>
   <h2>PiPER single-card control — simulated SDK, no robot connection</h2><div id="canvas-area"><div id="canvas-viewport"></div><div id="canvas-empty"></div><svg id="canvas-connectors-svg"></svg></div><div id="monitor-dashboard-grid"></div>
   <script type="importmap">{"imports":{"three":"/three.module.js","three/addons/controls/OrbitControls.js":"/OrbitControls.js"}}</script>
   <script type="module">
    window.sockets=[];window.models=[];
    window.WebSocket=class{constructor(url){this.url=url;window.sockets.push(this);}close(){}};
    const {SkeletonRenderer}=await import('/js/renderers/skeleton.js');
    const mount=SkeletonRenderer.mount;SkeletonRenderer.mount=function(...a){window.models.push(this);return mount.apply(this,a);};
    const dashboard=await import('/js/monitor-dashboard.js');dashboard.activate();window.dashboard=dashboard;
    const canvas=await import('/js/canvas.js');await canvas.initCanvas(${JSON.stringify([mcp])});window.cardReady=true;
   </script>`);return;
  }
  const file=url.pathname==='/three.module.js'||url.pathname==='/OrbitControls.js'
   ?path.join(process.env.PIPER_THREE_DIR,path.basename(url.pathname))
   :path.join(core,url.pathname);
  res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':'text/plain');
  res.end(await readFile(file));
 }catch(e){console.error('Mock endpoint failed',String(e));res.statusCode=500;res.end(String(e));}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const browser=await chromium.launch({headless:true,executablePath:process.env.PIPER_BROWSER,
 args:['--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1400,height:900}});
page.setDefaultTimeout(15000);const errors=[];page.on('pageerror',e=>{errors.push(String(e));console.error(String(e));});
try{
 await page.goto(`http://127.0.0.1:${server.address().port}`);
 await page.waitForFunction(()=>window.models?.[0]?._links?.joint1||window.models?.[0]?._jointMap?.joint1||Object.keys(window.models?.[0]?._links||{}).includes('link1'));
 assert.equal(requests.filter(r=>r.tool==='model').length,1);
 await page.waitForFunction(()=>window.cardReady);
 const card=page.locator('[data-card-id="arm1-control"]');
 const action=card.locator('select[data-key="action"]');
 assert.equal(await action.inputValue(),'select_action');
 assert.equal(await card.locator('input[type=number]:visible').count(),0);
 const defaultResponse=page.waitForResponse(r=>r.url().endsWith('/call')&&r.request().postDataJSON()?.arguments?.action==='select_action');
 await card.locator('.canvas-exec-btn').click();await defaultResponse;
 await page.waitForFunction(()=>document.querySelector('[data-card-id="arm1-control"] .canvas-exec-btn')?.disabled===false);
 assert.equal(controlResults.at(-1).result.executed,false);
 assert.equal(controlResults.at(-1).commands.length,0);
 if(uncalibrated){
  const choices=await action.locator('option').evaluateAll(es=>es.map(e=>e.value));
  assert.ok(!choices.includes('pose'));assert.ok(!choices.includes('reset'));
  assert.equal(await card.locator('[data-key="rotation_deg"]').count(),0);
 }
 for(const chosen of (uncalibrated?['move','set_joints','set_position','get_status','stop_motion','open_gripper','close_gripper']:['move','pose','set_joints','set_position','reset','get_status','stop_motion','open_gripper','close_gripper'])) {
  await action.selectOption(chosen);
  const visible=await card.locator('.canvas-field-input:visible').evaluateAll(elements=>elements.map(el=>el.dataset.key));
  const expected={move:uncalibrated?['action','translation_mm','speed_percent']:['action','translation_mm','rotation_deg','speed_percent'],pose:['action','pose_mm_deg','motion_mode','speed_percent'],
    set_joints:['action','joints_deg','joint_mode','speed_percent'],reset:['action','speed_percent'],
    set_position:['action'],get_status:['action'],stop_motion:['action'],open_gripper:['action'],close_gripper:['action']}[chosen];
  assert.deepEqual(visible.sort(),expected.sort());
 }
 await action.selectOption('set_gripper_opening');
 await card.locator('input[data-key="width_mm"]').fill('50');
 await action.selectOption('move');
 await card.locator('input[data-key="translation_mm"]').fill('[0,0,1]');
 const moveResponse=page.waitForResponse(r=>r.url().endsWith('/call')&&r.request().postDataJSON()?.arguments?.action==='move');
 await card.locator('.canvas-exec-btn').click();await moveResponse;
 await page.waitForFunction(()=>document.querySelector('[data-card-id="arm1-control"] .canvas-exec-btn')?.disabled===false);
 const move=requests.find(r=>r.arguments?.action==='move');
 assert.equal(move.arguments.translation_mm,'[0,0,1]');
 assert.equal(move.arguments.width_mm,undefined);
 assert.equal(controlResults.at(-1).result.state,'completed');
 assert.equal(controlResults.at(-1).commands.filter(c=>c[0]==='end_pose').length,1);
 await action.selectOption('set_joints');
 await card.locator('input[data-key="joints_deg"]').fill('[null,11,null,null,null,null]');
 await card.locator('select[data-key="joint_mode"]').selectOption('absolute');
 const jointResponse=page.waitForResponse(r=>r.url().endsWith('/call')&&r.request().postDataJSON()?.arguments?.action==='set_joints');
 await card.locator('.canvas-exec-btn').click();await jointResponse;
 await page.waitForFunction(()=>document.querySelector('[data-card-id="arm1-control"] .canvas-exec-btn')?.disabled===false);
 assert.equal(controlResults.at(-1).result.state,'completed');
 assert.deepEqual(controlResults.at(-1).commands.find(c=>c[0]==='joints')[1],[1000,11000,-10000,0,20000,0]);
 await action.selectOption('get_status');
 const statusResponse=page.waitForResponse(r=>r.url().endsWith('/call')&&r.request().postDataJSON()?.arguments?.action==='get_status');
 await card.locator('.canvas-exec-btn').click();await statusResponse;
 await page.waitForFunction(()=>document.querySelector('[data-card-id="arm1-control"] .canvas-exec-btn')?.disabled===false);
 assert.equal(controlResults.at(-1).commands.length,0);
 await action.selectOption('move');
 const urls=await page.evaluate(()=>window.sockets.map(s=>s.url));
 assert.ok(urls.some(u=>u.endsWith('/state/joints')));
 assert.ok(urls.some(u=>u.endsWith('/state/parameters')));
 assert.ok(urls.some(u=>u.endsWith('/camera/live')));
 assert.equal(layout.cards.length,1);
 assert.ok(!layout.cards.some(c=>c.toolName==='piper_camera_live'));
 await page.evaluate(jpeg=>{for(const ws of window.sockets){if(ws.url.endsWith('/camera/live'))ws.onmessage({data:Uint8Array.from(atob(jpeg),c=>c.charCodeAt(0)).buffer});}},fixture.jpeg);
 await page.waitForFunction(()=>document.querySelector('#monitor-dashboard-grid img')?.naturalWidth===64);
 for(const angle of [.1,.2,.3]){
  const frame=structuredClone(controlResults.find(item=>item.result.state==='completed').frame);frame.skeleton.joints[0].q=angle;frame.parameters['J1 angle (deg)']=angle*180/Math.PI;
  await page.evaluate(frame=>{for(const ws of window.sockets){if(ws.url.endsWith('/camera/live'))continue;const data=ws.url.endsWith('/joints')?frame.skeleton:frame.parameters;ws.onmessage({data:new TextEncoder().encode(JSON.stringify(data)).buffer});}},frame);
 }
 assert.equal(await page.locator('#monitor-dashboard-grid .monitor-card').count(),3);
 assert.ok(await page.locator('#monitor-dashboard-grid canvas').count());
 await page.waitForFunction(()=>document.querySelector('#monitor-dashboard-grid').textContent.includes('completed'));
 assert.deepEqual(errors,[]);
 await page.screenshot({path:path.join(out,'single-control-card-dashboard.png'),fullPage:false});
 await writeFile(path.join(out,'single-card-browser-result.json'),JSON.stringify({canvas_nodes:1,tool:'arm_control',subscriptions:urls,requests,control_results:controlResults,script_errors:errors},null,2));
 console.log('Actual single card passed: action-specific arrays, tip Move, nullable absolute joints, passive status, PiPER model and background camera.');
}catch(e){await page.screenshot({path:path.join(out,'browser-failure.png'),fullPage:false});console.error({errors,requests});throw e;}finally{await page.evaluate(()=>window.dashboard?.deactivate());await browser.close();server.close();}
