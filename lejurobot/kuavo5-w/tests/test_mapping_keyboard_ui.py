"""Run the actual panel script with a deterministic canvas/message DOM stub."""
from pathlib import Path
import shutil
import subprocess
import unittest


@unittest.skipUnless(shutil.which('node'), 'Node.js is required for panel event tests')
class KeyboardPanelTests(unittest.TestCase):
    def test_keyboard_focus_release_and_preparation_never_replay_directions(self):
        html = Path(__file__).resolve().parents[1]/'resource/mapping_keyboard.html'
        script = r'''
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync(process.argv[1],'utf8').split('<script>')[1].split('</script>')[0];
const listeners={},docListeners={},elements={};let tick,ready=false,focused=true,holdOpen=false,heldOpen=null;
function element(){return {disabled:false,textContent:'',dataset:{},classList:{toggle(){}},addEventListener(){},setPointerCapture(){}};}
for(const id of ['state','release','stop'])elements[id]=element();
const buttons=['KeyW','KeyS','KeyA','KeyD'].map(code=>({...element(),dataset:{code}}));
const requests=[];
let mappingStarting=true;
const port={start(){},postMessage(m){requests.push(m.args);let result={state:'ready'};
 if(m.args.action==='keyboard_open')result=mappingStarting?{state:'loading',mapping_state:'starting'}:{state:'loading',session_id:'lease'};
 if(m.args.action==='keyboard_status')result={state:'idle',mapping_state:mappingStarting?'starting':'mapping',console_authorized:true};
 if(m.args.action==='keyboard_command')result={state:ready?'ready':'loading'};
 if(m.args.action==='keyboard_release')result={state:'idle'};
 if(m.args.action==='keyboard_open'&&holdOpen){heldOpen=m;return;}
 queueMicrotask(()=>port.onmessage({data:{id:m.id,result}}));
}};
const parent={};const context={parent,setTimeout(fn,ms){if(ms===500){mappingStarting=false;queueMicrotask(fn);return 0;}return setTimeout(fn,ms);},clearTimeout,Promise,Map,Set,Number,Error,Date,
 setInterval(fn){tick=fn;},window:{focus(){focused=true;},addEventListener(k,f){listeners[k]=f;}},
 document:{hidden:false,hasFocus(){return focused;},getElementById(id){return elements[id];},querySelectorAll(){return buttons;},
 addEventListener(k,f){docListeners[k]=f;}}};
vm.createContext(context);vm.runInContext(source,context);
const key=(code,altKey=true)=>({code,altKey,preventDefault(){}});
const values=()=>JSON.parse(JSON.stringify(requests.filter(x=>x.action==='keyboard_command').at(-1).values));
(async()=>{
 listeners.message({source:parent,data:{protocol:'motus.workbench/1'},ports:[port]});
 await new Promise(resolve=>setImmediate(resolve));
 assert.equal(requests[0].action,'keyboard_open','Panel prepares automatically without an enable confirmation');
 assert(requests.some(r=>r.action==='keyboard_status'),'Early opening must wait for mapping completion');
 assert.equal(requests.filter(r=>r.action==='keyboard_open').length,2,'Prepare keyboard after mapping completes');
 listeners.keydown(key('KeyW'));await tick();assert.deepEqual(values(),[0,0]);
 ready=true;await tick();assert.deepEqual(values(),[0,0]);
 await tick();assert.deepEqual(values(),[0,0],'Preparation keys must not replay');
 listeners.keydown({...key('KeyW'),repeat:true});await tick();assert.deepEqual(values(),[0,0],'OS key repeat must not resume preparation input');
 listeners.keydown(key('KeyI',false));await tick();assert.deepEqual(values(),[0,0]);
 listeners.keydown(key('KeyI'));await tick();assert.deepEqual(values(),[.15,0]);
 listeners.keyup(key('KeyI'));await tick();assert.deepEqual(values(),[0,0]);
 listeners.keydown(key('KeyA'));await tick();assert.deepEqual(values(),[0,.3]);
 listeners.keyup(key('AltLeft'));await tick();assert.deepEqual(values(),[0,0]);
 listeners.keydown(key('KeyK'));await tick();assert.deepEqual(values(),[-.15,0]);
 focused=false;listeners.blur();await new Promise(resolve=>setImmediate(resolve));
 assert.equal(requests.at(-1).action,'keyboard_release');
 const n=requests.length;listeners.keydown(key('KeyW'));await tick();assert.equal(requests.length,n);
 focused=true;listeners.focus();await new Promise(resolve=>setImmediate(resolve));await tick();
 context.document.hidden=true;docListeners.visibilitychange();
 await new Promise(resolve=>setImmediate(resolve));assert.equal(requests.at(-1).action,'keyboard_release');
 context.document.hidden=false;holdOpen=true;listeners.focus();
 assert(heldOpen,'A new focused session should prepare automatically');
 focused=false;listeners.blur();
 port.onmessage({data:{id:heldOpen.id,result:{state:'loading',session_id:'late-lease'}}});
 await new Promise(resolve=>setImmediate(resolve));
 assert.equal(requests.at(-1).action,'keyboard_release','Late preparation after blur must be released');
 assert.equal(requests.at(-1).session_id,'late-lease');
 await elements.stop.onclick();assert.equal(requests.at(-2).action,'stop');assert.equal(requests.at(-1).action,'keyboard_status');
 const stopped=requests.length;focused=true;listeners.focus();await tick();
 assert.equal(requests.length,stopped,'Stop all motion must not automatically re-arm on focus');
})().catch(e=>{console.error(e);process.exitCode=1;});
'''
        result = subprocess.run(['node', '-e', script, str(html)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)


if __name__ == '__main__':
    unittest.main()
