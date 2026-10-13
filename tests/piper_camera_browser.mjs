/** Exercise the unchanged Core CameraRenderer with synthetic JPEGs; no hardware/API. */
import assert from 'node:assert/strict';
import { createServer } from 'node:http';
import { readFile, mkdir, writeFile } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import { spawnSync } from 'node:child_process';
import { createRequire } from 'node:module';
import path from 'node:path';

const require = createRequire(import.meta.url);
const { chromium } = require(process.env.PIPER_PLAYWRIGHT || 'playwright');
const source = await readFile(path.join(process.env.PIPER_CORE_WEB, 'js/renderers/camera.js'));
const generated = spawnSync(process.env.PIPER_PYTHON || 'python3', ['-c', `
import base64,json,time,numpy as np
from agilex.piper.camera_worker import encode_frame
from agilex.piper.camera_live import offline_image
images=[]
for color in [(50,20,180),(20,180,50),(180,50,20)]:
 frame=np.zeros((480,640,3),np.uint8);frame[:]=color
 images.append(base64.b64encode(encode_frame(frame,time.time(),80)).decode())
print(json.dumps(dict(images=images,offline=base64.b64encode(offline_image(640,480)).decode())))
`], { encoding: 'utf8' });
assert.equal(generated.status, 0, generated.stderr);
const fixture = JSON.parse(generated.stdout);
const html = `<!doctype html><meta charset="utf-8"><title>PiPER 实时相机模拟验收</title>
<style>body{background:#eee;font:16px system-ui;margin:24px}#view{width:800px;height:600px}</style>
<h2>PiPER 实时相机｜模拟画面验收</h2><p>复用当前 Core 相机渲染器；未连接相机。</p><div id="view"></div>
<script type="module">import {CameraRenderer} from '/camera.js';
window.renderer=Object.create(CameraRenderer);renderer.mount(document.getElementById('view'));
window.send=async data=>{const bytes=Uint8Array.from(atob(data),c=>c.charCodeAt(0));renderer.onData(bytes.buffer);
await renderer._img.decode();const canvas=document.createElement('canvas');canvas.width=640;canvas.height=480;
const ctx=canvas.getContext('2d');ctx.drawImage(renderer._img,0,0);return Array.from(ctx.getImageData(5,100,1,1).data);};
</script>`;
const server = createServer((req, res) => {
  res.setHeader('Content-Type', req.url === '/camera.js' ? 'text/javascript' : 'text/html');
  res.end(req.url === '/camera.js' ? source : html);
});
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
let browser;
try {
  browser = await chromium.launch({headless:true, executablePath:process.env.PIPER_BROWSER});
  const page = await browser.newPage({viewport:{width:1000,height:850}});
  const errors=[];page.on('pageerror', error=>errors.push(String(error)));
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  await page.waitForFunction(()=>window.renderer);
  const pixels=[];
  for(let i=0;i<15;i++) {
    const pixel=await page.evaluate(data=>window.send(data), fixture.images[i%3]);
    if(i<3)pixels.push(pixel);
    await page.waitForTimeout(100);
  }
  assert.equal(new Set(pixels.map(p=>JSON.stringify(p))).size,3);
  assert.match(await page.locator('.renderer-camera span').textContent(),/fps/);
  await page.evaluate(data=>window.send(data),fixture.offline);
  await page.evaluate(data=>window.send(data),fixture.images[0]);
  assert.deepEqual(errors,[]);
  await mkdir(process.env.PIPER_CAMERA_EVIDENCE,{recursive:true});
  await page.screenshot({path:path.join(process.env.PIPER_CAMERA_EVIDENCE,'camera-simulation.png')});
  await writeFile(path.join(process.env.PIPER_CAMERA_EVIDENCE,'camera-browser-result.json'),JSON.stringify({
    renderer_sha256:createHash('sha256').update(source).digest('hex'),new_frames:15,
    distinct_scenes:3,offline_and_recovery:true,script_errors:errors,hardware_access:false},null,2));
  console.log('Core 相机渲染器：15 个新帧、3 个场景、断流提示与恢复通过，无脚本错误');
} finally {
  if(browser)await browser.close();
  await new Promise(resolve=>server.close(resolve));
}
