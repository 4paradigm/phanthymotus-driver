// Browser regression for the progressive pairing flow; no robot or headset.
import assert from 'node:assert/strict';
import {spawnSync} from 'node:child_process';
const {chromium} = await import(process.env.PLAYWRIGHT_MODULE);
const result = spawnSync(process.env.PYTHON || 'python3', ['-c', "import sys; sys.path.insert(0, 'pico/4ultra'); from ext_vr.admin_page import PAGE; print(PAGE)"], {encoding:'utf8'});
assert.equal(result.status, 0, result.stderr);
const browser = await chromium.launch({headless:true, executablePath:process.env.CHROME_BINARY});
const page = await browser.newPage({viewport:{width:900,height:1000}});
let paired=false, connected=false, invitation=false, pending=null, failInvite=true;
const errors=[];
page.on('pageerror', e=>errors.push(e.message));
await page.route('https://robot.test/**', async route=>{
 const path=new URL(route.request().url()).pathname;
 let value;
 if(path==='/onboarding')return route.fulfill({contentType:'text/html',body:result.stdout});
 if(path==='/onboarding/package')value={available:true,version:'0.4.3-pico-input'};
 else if(path==='/manage/status')value={pairing:{pending,invitation:invitation?{}:null},capture:{paired_devices:+paired,connected}};
 else if(path==='/manage/invite'){if(failInvite){failInvite=false;return route.fulfill({status:503,contentType:'application/json',body:JSON.stringify({error:'服务暂不可用，请重试'})});}invitation=true;value={deep_link:'motus-teleop://connect#fixture'};}
 else if(path==='/manage/approve'){paired=true;pending=null;invitation=false;value={};}
 else if(path==='/manage/revoke_headset'){paired=false;connected=false;value={};}
 else throw Error('unexpected request: '+path);
 return route.fulfill({contentType:'application/json',body:JSON.stringify(value)});
});
try{
 await page.goto('https://robot.test/onboarding');
 await page.waitForFunction(()=>document.getElementById('status').textContent.includes('尚未配对'));
 assert.equal(await page.locator('details').getAttribute('open'),null);
 assert.equal(await page.locator('#pending').isVisible(),false);
 await page.locator('#prepare').click();
 await page.waitForFunction(()=>document.getElementById('error').textContent.includes('服务暂不可用'));
 assert.equal(await page.locator('#prepare').isEnabled(),true);
 await page.locator('#prepare').click();
 await page.waitForFunction(()=>document.getElementById('connect').getAttribute('href')==='motus-teleop://connect#fixture');
 await page.waitForTimeout(2200);
 assert.equal(await page.locator('#prepare').isVisible(),false);
 assert.equal(await page.locator('#connect').isVisible(),true);
 invitation=false;
 await page.waitForFunction(()=>!document.getElementById('prepare').hidden);
 assert.equal(await page.locator('#connect').isVisible(),false);
 pending={request_id:'1',fingerprint:'ABCD EFGH'};
 await page.waitForFunction(()=>!document.getElementById('pending').hidden);
 await page.locator('[data-action=approve]').click();
 await page.waitForFunction(()=>document.getElementById('status').textContent.includes('已配对'));
 assert.equal(await page.locator('#pending').isVisible(),false);
 assert.equal(await page.locator('#prepare').isVisible(),false);
 connected=true;
 await page.waitForFunction(()=>document.getElementById('status').textContent.includes('PICO 已连接'));
 if(process.env.SCREENSHOT)await page.screenshot({path:process.env.SCREENSHOT,fullPage:true});
 assert.deepEqual(errors,[]);
 console.log('Onboarding browser flow PASS: invitation, expiry, pending approval, reconnect');
}finally{await browser.close();}
