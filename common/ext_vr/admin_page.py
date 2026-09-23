"""Small same-origin pairing page served by the PICO Driver itself."""

PAGE = r"""<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>PICO 安装与配对</title>
<style>body{font:18px system-ui;max-width:760px;margin:32px auto;padding:16px;line-height:1.6}button,input{font:inherit;padding:10px;margin:6px 0}button,a{touch-action:manipulation}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#f3f4f6;padding:16px}section{margin:20px 0}a{display:inline-block;padding:8px}</style>
<h1>PICO 安装与配对</h1><p>1. 在机器人 Canvas 部署 PICO 与天轶 Driver。2. 下载 App 并在头显确认安装。3. 返回本页，打开并连接当前设备。连接不会自动启动机器人。</p>
<a href="/onboarding/apk">下载 PICO App</a><p id="package"></p>
<section><h2>配对管理</h2><p>首次使用请在 Canvas 齿轮页设置“配对管理密码”（至少 12 个字符），并确认已经安装遥操驱动。配对操作只授权设备连接，不授予机器人运动权限。</p>
<input id="password" type="password" autocomplete="current-password" placeholder="配对管理密码"><button id="login">登录</button></section>
<section id="controls" hidden><button data-action="status">刷新状态</button><button data-action="open">允许新设备配对</button><button data-action="invite">生成一次性连接邀请</button><button data-action="revoke_invitation">撤销邀请</button><button data-action="approve">批准当前指纹</button><button data-action="reject">拒绝</button><button data-action="revoke_headset">撤销已配对设备</button><p>批准前核对下面指纹与头显显示一致。</p><a id="connect" hidden>打开并连接当前设备</a></section><pre id="status">尚未登录</pre>
<script>
let csrf='', pending=null;const status=document.querySelector('#status');
async function call(action,values={}) {const r=await fetch('/manage/'+action,{method:'POST',credentials:'same-origin',headers:{'Content-Type':'application/json','X-Pico-CSRF':csrf},body:JSON.stringify(values)});const v=await r.json();if(!r.ok)throw Error(v.error||'请求失败');return v;}
function show(v){status.textContent=JSON.stringify(v,null,2);if(v.pairing)pending=v.pairing.pending;if(v.deep_link){const a=document.querySelector('#connect');a.href=v.deep_link;a.hidden=false;}}
document.querySelector('#login').onclick=async()=>{try{const p=document.querySelector('#password');const v=await call('login',{password:p.value});p.value='';csrf=v.csrf;document.querySelector('#controls').hidden=false;show(await call('status'));}catch(e){status.textContent=e.message;}};
for(const b of document.querySelectorAll('[data-action]'))b.onclick=async()=>{try{const a=b.dataset.action;let data={};if(a==='approve'||a==='reject'){if(!pending)throw Error('没有等待配对的设备');data={request_id:pending.request_id,fingerprint:pending.fingerprint};}if(a==='revoke_headset'&&!confirm('撤销后需要重新配对，继续？'))return;show(await call(a,data));}catch(e){status.textContent=e.message;}};
fetch('/onboarding/package').then(r=>r.json()).then(v=>{document.querySelector('#package').textContent=v.available?`${v.version} · SHA256 ${v.sha256}`:'安装包尚未就绪，请联系部署人员。';});
</script></html>"""
