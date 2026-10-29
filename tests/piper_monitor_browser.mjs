/** Actual Core renderer simulation; no site API, ROS, SDK or CAN access.
 *
 * Required: PIPER_CORE_WEB=/path/to/agent-core/web,
 * PIPER_THREE_DIR=/path/to/offline/three (three.module.js and OrbitControls.js).
 * Optional: PIPER_PYTHON, PIPER_PLAYWRIGHT (module path), PIPER_BROWSER
 * (Chromium executable), PIPER_MONITOR_EVIDENCE (output directory).
 * Run with Node: node tests/piper_monitor_browser.mjs
 */
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { spawnSync } from 'node:child_process';
import { readFile, mkdir, writeFile, mkdtemp } from 'node:fs/promises';
import { createServer } from 'node:http';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const { chromium } = require(process.env.PIPER_PLAYWRIGHT || 'playwright');
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const core = process.env.PIPER_CORE_WEB;
const three = process.env.PIPER_THREE_DIR;
assert.ok(core && three, 'Set PIPER_CORE_WEB and PIPER_THREE_DIR; see script header');
const output = process.env.PIPER_MONITOR_EVIDENCE || await mkdtemp(path.join(tmpdir(), 'piper-monitor-browser-'));
await mkdir(output, { recursive: true });

const python = spawnSync(process.env.PIPER_PYTHON || 'python3', ['-c', `
import json, math, sys
from pathlib import Path
sys.path.insert(0, 'tests')
from test_piper_monitor import snapshot, forward, multiply, rotation
from agilex.piper.monitor import monitor_frame, load_model, RESOURCE_DIR
source = (RESOURCE_DIR / 'upstream/piper_description.urdf').read_text()
frames = []
for degrees in ([0]*6, [15,30,-40,20,-10,60], [-90,100,-110,-50,35,-80]):
    data = snapshot()
    data.update(feedback_time=1791856800., joints_deg=degrees)
    skeleton, parameters, stamp = monitor_frame(data, now=1791856800., device_name='机械臂 1')
    transforms = forward(source, [math.radians(x) for x in degrees])
    expected = {name: multiply(rotation((1,0,0), -math.pi/2), matrix) for name, matrix in transforms.items()}
    frames.append(dict(skeleton=skeleton, parameters=parameters, expected=expected))
stale = monitor_frame(data, now=1791856800.6, device_name='机械臂 1', last_valid_time=stamp)
error = monitor_frame({}, now=1791856800.7, device_name='机械臂 1', last_valid_time=stamp, error='模拟 CAN 断线')
print(json.dumps(dict(urdf=load_model(), frames=frames,
    stale=dict(skeleton=stale[0], parameters=stale[1]),
    error=dict(skeleton=error[0], parameters=error[1])), ensure_ascii=False, allow_nan=False))
`], { cwd: root, encoding: 'utf8' });
assert.equal(python.status, 0, python.stderr);
const fixture = JSON.parse(python.stdout);
const routes = new Map();
for (const [url, filename] of [
  ['/skeleton.js', path.join(core, 'js/renderers/skeleton.js')],
  ['/kv-latest.js', path.join(core, 'js/renderers/kv-latest.js')],
  ['/json-util.js', path.join(core, 'js/renderers/json-util.js')],
  ['/three.module.js', path.join(three, 'three.module.js')],
  ['/OrbitControls.js', path.join(three, 'OrbitControls.js')],
]) routes.set(url, await readFile(filename));
const calls = [];
const html = `<!doctype html><meta charset="utf-8"><title>PiPER 监控模拟验收</title>
<style>
body { margin:24px; background:#141419; color:#eee; font:14px system-ui; }
h1 { font-size:22px } h2 { font-size:17px } p { color:#aab }
.row { display:grid; grid-template-columns:600px 620px; gap:16px; margin-bottom:24px }
.skeleton { height:410px; border:1px solid #444; border-radius:8px; overflow:hidden }
.renderer-skeleton { width:100%; height:100% }
.params { height:410px; overflow:auto; border:1px solid #444; border-radius:8px; padding:0 12px }
.kv-cell { display:grid; grid-template-columns:230px 1fr; gap:8px; border-bottom:1px solid #333; padding:7px }
.kv-key { color:#aab } .kv-value { overflow-wrap:anywhere }
</style>
<h1>PiPER 姿态与参数｜模拟反馈验收</h1><p>复用现有 Core 渲染器；数据来自假快照，未连接机械臂。</p>
<h2>机械臂 1</h2><div class="row"><div id="arm1" class="skeleton"></div><div id="params1" class="params"></div></div>
<h2>机械臂 2</h2><div class="row"><div id="arm2" class="skeleton"></div><div id="params2" class="params"></div></div>
<script type="importmap">{"imports":{"three":"/three.module.js","three/addons/controls/OrbitControls.js":"/OrbitControls.js"}}</script>
<script type="module">
import { SkeletonRenderer } from '/skeleton.js';
import { KvLatestRenderer } from '/kv-latest.js';
window.renderers = [];
for (let i=1; i<=2; i++) {
  const skeleton=Object.create(SkeletonRenderer), params=Object.create(KvLatestRenderer);
  skeleton.mount(document.getElementById('arm'+i), 'arm'+i);
  params.mount(document.getElementById('params'+i));
  window.renderers.push({skeleton,params});
}
window.sendFrame = (i, frame) => {
  const r=window.renderers[i];
  if (frame.skeleton) r.skeleton.onData(new TextEncoder().encode(JSON.stringify(frame.skeleton)).buffer);
  r.params.onData(new TextEncoder().encode(JSON.stringify(frame.parameters)).buffer);
};
window.linkMatrices = i => {
  const r=window.renderers[i].skeleton;
  r._scene.updateMatrixWorld(true);
  return Object.fromEntries(Object.entries(r._links).map(([name,obj]) => [name,
    Array.from({length:4},(_,row)=>Array.from({length:4},(_,col)=>obj.matrixWorld.elements[col*4+row]))]));
};
</script>`;

const server = createServer(async (req, res) => {
  if (/^\/api\/mcp\/arm[12]\/call$/.test(req.url) && req.method === 'POST') {
    let body = '';
    for await (const part of req) body += part;
    calls.push({url:req.url, body:JSON.parse(body)});
    res.writeHead(200, {'Content-Type':'application/json'});
    res.end(JSON.stringify({code:0, data:[{type:'text', text:JSON.stringify({urdf:fixture.urdf})}]}));
  } else if (req.url === '/') {
    res.writeHead(200, {'Content-Type':'text/html; charset=utf-8'}); res.end(html);
  } else if (routes.has(req.url)) {
    res.writeHead(200, {'Content-Type':'text/javascript'}); res.end(routes.get(req.url));
  } else { res.writeHead(404); res.end(); }
});
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
let browser;
try {
  browser = await chromium.launch({headless:true, ...(process.env.PIPER_BROWSER ? {executablePath:process.env.PIPER_BROWSER} : {}),
    args:['--use-angle=swiftshader', '--enable-unsafe-swiftshader']});
  const page = await browser.newPage({viewport:{width:1300, height:1100}, deviceScaleFactor:1});
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  await page.goto(`http://127.0.0.1:${server.address().port}`);
  await page.waitForFunction(() => window.renderers?.length===2 && window.renderers.every(r=>r.skeleton._loaded));
  await page.evaluate(frame => {
    frame.parameters['设备名称']='机械臂 2'; window.sendFrame(1,frame);
  }, fixture.frames[1]);
  const arm2 = await page.evaluate(() => window.linkMatrices(1));
  let comparisons = 0;
  for (const frame of fixture.frames) {
    await page.evaluate(frame => window.sendFrame(0, frame), frame);
    const matrices = await page.evaluate(() => window.linkMatrices(0));
    for (const [link, expected] of Object.entries(frame.expected)) {
      for (let i=0; i<4; i++) for (let j=0; j<4; j++) {
        assert.ok(Math.abs(matrices[link][i][j]-expected[i][j])<1e-10, `${link}[${i}][${j}]`);
        comparisons++;
      }
    }
    assert.deepEqual(await page.evaluate(() => window.linkMatrices(1)), arm2, 'Arm 1 must not update Arm 2');
    assert.ok((await page.locator('#params1').innerText()).includes(String(frame.parameters['J2 角度（°）'])));
  }
  const last = await page.evaluate(() => window.linkMatrices(0));
  for (const frame of [fixture.stale, fixture.error]) {
    assert.equal(frame.skeleton, null);
    await page.evaluate(frame => window.sendFrame(0, frame), frame);
    assert.deepEqual(await page.evaluate(() => window.linkMatrices(0)), last, 'Invalid feedback must not move skeleton');
    assert.ok((await page.locator('#params1').innerText()).includes(frame.parameters['反馈状态']));
  }
  await page.evaluate(frame => window.sendFrame(0, frame), fixture.frames[2]);
  assert.ok((await page.locator('#params1').innerText()).includes('新鲜'));
  assert.ok((await page.locator('#params2').innerText()).includes('机械臂 2'));
  assert.equal(calls.length, 2);
  for (const call of calls) assert.deepEqual(call.body, {tool:'model', arguments:{}});
  assert.deepEqual(errors, []);
  await page.screenshot({path:path.join(output, 'monitor-simulation.png'), fullPage:true});
  const report = {result:'passed', simulated:true, matrixComparisons:comparisons,
    scenarios:['three measured poses', 'stale feedback freezes skeleton', 'reader failure visible', 'recovery', 'dual device isolation'],
    modelCalls:calls, pageErrors:errors,
    coreSkeletonSha256:createHash('sha256').update(routes.get('/skeleton.js')).digest('hex'),
    screenshot:path.join(output, 'monitor-simulation.png')};
  await writeFile(path.join(output, 'result.json'), JSON.stringify(report, null, 2));
  console.log(JSON.stringify(report, null, 2));
  await page.evaluate(() => {for (const r of window.renderers) {r.skeleton.unmount();r.params.unmount();}});
} finally {
  if (browser) await browser.close();
  server.closeAllConnections();
  await new Promise(resolve => server.close(resolve));
}
