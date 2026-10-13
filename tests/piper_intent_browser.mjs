/** Actual unchanged Core dashboard, one arm_control canvas node, fake hardware. */
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
const requests=[];
const topic='/agilex/piper/state/joints';
const params='/agilex/piper/state/parameters';
const mcp={id:'intent-arm1',name:'PiPER arm 1',online:true,tools:fixture.tools};
const layout={cards:[{id:'arm1-control',mcpId:mcp.id,toolName:'arm_control',topicOut:fixture.tools.find(t=>t.name==='arm_control').topic_out}],connections:[]};
const server=createServer(async(req,res)=>{
 try{
  const url=new URL(req.url,'http://localhost');
  let data;
  if(url.pathname==='/api/mcp')data={data:[mcp]};
  else if(url.pathname==='/api/topics')data={data:[{topic,format:'sensor/skeleton',mcp_id:mcp.id},{topic:params,format:'data/json',mcp_id:mcp.id}]};
  else if(url.pathname==='/api/canvas/layout')data={data:layout};
  else if(url.pathname===`/api/mcp/${mcp.id}/call`){
   let body='';for await(const chunk of req)body+=chunk;
   const call=JSON.parse(body);requests.push(call);
   assert.equal(call.tool,'model');
   data={code:0,data:[{type:'text',text:JSON.stringify({urdf:fixture.urdf})}]};
  }
  if(data){res.setHeader('Content-Type','application/json');res.end(JSON.stringify(data));return;}
  if(url.pathname==='/'){
   res.setHeader('Content-Type','text/html');
   res.end(`<meta charset="utf-8"><title>PiPER intent monitor — simulated feedback</title>
   <link rel="stylesheet" href="/css/style.css"><style>body{background:#eee;font:16px system-ui}#monitor-dashboard-grid{height:700px}.monitor-card{background:white}.monitor-card-body{height:350px}</style>
   <h2>Arm control only — simulated SDK, no robot connection</h2><div id="monitor-dashboard-grid"></div>
   <script type="importmap">{"imports":{"three":"/three.module.js","three/addons/controls/OrbitControls.js":"/OrbitControls.js"}}</script>
   <script type="module">
    window.sockets=[];window.models=[];
    window.WebSocket=class{constructor(url){this.url=url;window.sockets.push(this);}close(){}};
    const {SkeletonRenderer}=await import('/js/renderers/skeleton.js');
    const mount=SkeletonRenderer.mount;SkeletonRenderer.mount=function(...a){window.models.push(this);return mount.apply(this,a);};
    const dashboard=await import('/js/monitor-dashboard.js');dashboard.activate();window.dashboard=dashboard;
   </script>`);return;
  }
  const file=url.pathname==='/three.module.js'||url.pathname==='/OrbitControls.js'
   ?path.join(process.env.PIPER_THREE_DIR,path.basename(url.pathname))
   :path.join(core,url.pathname);
  res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':'text/plain');
  res.end(await readFile(file));
 }catch(e){res.statusCode=500;res.end(String(e));}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const browser=await chromium.launch({headless:true,executablePath:process.env.PIPER_BROWSER,
 args:['--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1400,height:900}});
const errors=[];page.on('pageerror',e=>errors.push(String(e)));
try{
 await page.goto(`http://127.0.0.1:${server.address().port}`);
 await page.waitForFunction(()=>window.models?.[0]?._links?.joint1||window.models?.[0]?._jointMap?.joint1||Object.keys(window.models?.[0]?._links||{}).includes('link1'));
 assert.equal(requests.length,1);assert.deepEqual(requests[0],{tool:'model',arguments:{}});
 const urls=await page.evaluate(()=>window.sockets.map(s=>s.url));
 assert.ok(urls.some(u=>u.endsWith('/state/joints')));
 assert.ok(urls.some(u=>u.endsWith('/state/parameters')));
 for(const angle of [.1,.2,.3]){
  const frame=structuredClone(fixture.frame);frame.skeleton.joints[0].q=angle;
  await page.evaluate(frame=>{for(const ws of window.sockets){const data=ws.url.endsWith('/joints')?frame.skeleton:frame.parameters;ws.onmessage({data:new TextEncoder().encode(JSON.stringify(data)).buffer});}},frame);
 }
 assert.equal(await page.locator('#monitor-dashboard-grid .monitor-card').count(),2);
 assert.ok(await page.locator('#monitor-dashboard-grid canvas').count());
 assert.deepEqual(errors,[]);
 await page.screenshot({path:path.join(out,'intent-one-card-dashboard.png'),fullPage:true});
 await writeFile(path.join(out,'browser-result.json'),JSON.stringify({canvas_nodes:1,tool:'arm_control',subscriptions:urls,model_requests:requests,script_errors:errors},null,2));
 console.log('Actual Core dashboard passed: one Arm control node, PiPER model and both topics.');
}finally{await page.evaluate(()=>window.dashboard?.deactivate());await browser.close();server.close();}
