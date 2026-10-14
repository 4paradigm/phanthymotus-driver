/** Unchanged Core card + dashboard, production schemas and simulated SDK only. */
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
await mkdir(out, {recursive:true});
const fixtureResult = spawnSync(process.env.PIPER_PYTHON, ['-c', `
import json,tempfile,sys
sys.path.insert(0,'tests')
from test_piper_intent import IntentDriver
from agilex.piper.device import build_plugins
from common.vendor_runtime import DriverBundle
with tempfile.TemporaryDirectory() as d:
 p=build_plugins({'data_dir':d,'name':'PiPER arm 1','async_actions':False},'agilex',None)
 p[0].factory=IntentDriver;p[0].connected()
 try:print(json.dumps({'tools':DriverBundle(p).get_all_tools(),'frame':p[1].monitor.read(),'urdf':DriverBundle(p).dispatch('model',{})['urdf']}))
 finally:p[0].stop()
`], {encoding:'utf8'});
assert.equal(fixtureResult.status,0,fixtureResult.stderr);
const fixture=JSON.parse(fixtureResult.stdout);
const requests=[];const controlResults=[];
function callFake(arguments_) {
 const run=spawnSync(process.env.PIPER_PYTHON,['-c',`
import sys,json,tempfile
from unittest.mock import patch
sys.path.insert(0,'tests')
from test_piper_intent import IntentDriver,fast_move
from agilex.piper.device import build_plugins
from common.vendor_runtime import DriverBundle
args=json.load(sys.stdin)
with tempfile.TemporaryDirectory() as d:
 p=build_plugins({'data_dir':d,'async_actions':False},'agilex',None)
 p[0].factory=IntentDriver;driver=p[0].connected()
 try:
  with patch('agilex.piper.intent_control.move_cartesian',side_effect=fast_move):
   result=DriverBundle(p).dispatch('arm_control',args)
  print(json.dumps({'result':result,'commands':driver.arm.calls,'frame':p[1].monitor.read()}))
 finally:p[0].stop()
`],{encoding:'utf8',input:JSON.stringify(arguments_)});
 assert.equal(run.status,0,run.stderr);return JSON.parse(run.stdout);
}
const topic='/agilex/piper/state/joints';
const params='/agilex/piper/state/parameters';
const mcp={id:'intent-arm1',name:'PiPER arm 1',online:true,tools:fixture.tools};
const layout={cards:[{id:'arm1-control',mcpId:mcp.id,toolName:'arm_control',driverName:'PiPER simulated arm',x:30,y:20,topicOut:fixture.tools.find(t=>t.name==='arm_control').topic_out}],connections:[]};
const server=createServer(async(req,res)=>{
 try{
  const url=new URL(req.url,'http://localhost');
  let data;
  if(url.pathname==='/api/mcp')data={data:[mcp]};
  else if(url.pathname==='/api/topics')data={data:[{topic,format:'sensor/skeleton',mcp_id:mcp.id},{topic:params,format:'data/json',mcp_id:mcp.id}]};
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
   <h2>Direction micro-adjustment — simulated SDK, no robot connection</h2><div id="canvas-area"><div id="canvas-viewport"></div><div id="canvas-empty"></div><svg id="canvas-connectors-svg"></svg></div><div id="monitor-dashboard-grid"></div>
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
 for(const direction of ['forward','backward','left','right','up','down']){
  await action.selectOption('move_'+direction);
  assert.equal(await card.locator('input[type=number]:visible').count(),0);
 }
 await action.selectOption('set_gripper_opening');
 assert.equal(await card.locator('input[data-key="width_mm"]:visible').count(),1);
 await action.selectOption('move_up');
 const moveResponse=page.waitForResponse(r=>r.url().endsWith('/call')&&r.request().postDataJSON()?.arguments?.action==='move_up');
 await card.locator('.canvas-exec-btn').click();await moveResponse;
 await page.waitForFunction(()=>document.querySelector('[data-card-id="arm1-control"] .canvas-exec-btn')?.disabled===false);
 const move=requests.find(r=>r.arguments?.action==='move_up');
 assert.deepEqual(move.arguments,{action:'move_up'});
 assert.equal(controlResults.at(-1).result.state,'completed');
 assert.ok(controlResults.at(-1).commands.some(c=>c[0]==='end_pose'&&c[1][2]===251000));
 const urls=await page.evaluate(()=>window.sockets.map(s=>s.url));
 assert.ok(urls.some(u=>u.endsWith('/state/joints')));
 assert.ok(urls.some(u=>u.endsWith('/state/parameters')));
 for(const angle of [.1,.2,.3]){
  const frame=structuredClone(controlResults.at(-1).frame);frame.skeleton.joints[0].q=angle;frame.parameters['J1 angle (deg)']=angle*180/Math.PI;
  await page.evaluate(frame=>{for(const ws of window.sockets){const data=ws.url.endsWith('/joints')?frame.skeleton:frame.parameters;ws.onmessage({data:new TextEncoder().encode(JSON.stringify(data)).buffer});}},frame);
 }
 assert.equal(await page.locator('#monitor-dashboard-grid .monitor-card').count(),2);
 assert.ok(await page.locator('#monitor-dashboard-grid canvas').count());
 await page.waitForFunction(()=>document.querySelector('#monitor-dashboard-grid').textContent.includes('completed'));
 assert.deepEqual(errors,[]);
 await page.screenshot({path:path.join(out,'direction-card-dashboard.png'),fullPage:false});
 await writeFile(path.join(out,'browser-result.json'),JSON.stringify({canvas_nodes:1,tool:'arm_control',subscriptions:urls,requests,control_results:controlResults,script_errors:errors},null,2));
 console.log('Unchanged Core card passed: six no-parameter direction actions, default no-op, simulated production Move, PiPER model and both topics.');
}catch(e){await page.screenshot({path:path.join(out,'browser-failure.png'),fullPage:false});console.error({errors,requests});throw e;}finally{await page.evaluate(()=>window.dashboard?.deactivate());await browser.close();server.close();}
