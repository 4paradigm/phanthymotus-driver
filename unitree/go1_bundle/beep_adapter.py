#!/usr/bin/env python3
"""Go1 头部扬声器 beep 专属适配器（跑在 Head Nano，不在驱动容器内）。

与 speaker 音频流播放的 head_adapter.py 分离：本文件只服务 beep 卡片需要的动作
（beep / set_volume / get_volume / info），端点 /v1/beep/actions，独立端口（默认 18082）。
不含 speaker 流播放（start/stop/audio 持久 aplay）与 LED——那些在 head_adapter.py。

只接受固定的 beep 卡 API；不接受任何调用方传入的 shell 命令 / URL / 设备路径。
"""
import argparse, json, math, re, struct, subprocess, threading, time
from http.server import BaseHTTPRequestHandler
try:                                    # Nano 是 Python 3.6，无 ThreadingHTTPServer(3.7+)
    from http.server import ThreadingHTTPServer
except ImportError:
    from http.server import HTTPServer
    from socketserver import ThreadingMixIn
    class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
        daemon_threads = True

def now(): return int(time.time()*1000)
def reply(action, request_id, state, applied): return {"ok":True,"card":"beep","action":action,"request_id":request_id,"state":state,"applied":applied,"timestamp_ms":now()}
def fail(action, rid, code, message, retryable=False, details=None): return {"ok":False,"card":"beep","action":action,"request_id":rid,"code":code,"message":message,"details":details or {},"retryable":retryable,"timestamp_ms":now()}

class BeepAdapter:
 def __init__(self, cfg):
  self.cfg=cfg; self.lock=threading.RLock(); self.beep_proc=None
  self.mixer=self.cfg.get('mixer_control','Speaker')
  # Nano 是 Python 3.6：subprocess 的 text= 要到 3.7 才有，统一用 universal_newlines。
  self.device,self.mixer_card=self._discover_device(); self.volume=self._volume()
 def _playback_cards(self):
  try:
   out=subprocess.check_output(['aplay','-l'],universal_newlines=True,stderr=subprocess.DEVNULL,timeout=2)
   return [(int(c),int(d)) for c,d in re.findall(r'^card (\d+):.*?device (\d+):',out,re.M)]
  except Exception: return []
 def _card_has_mixer(self,card):
  try:
   out=subprocess.check_output(['amixer','-c',str(card),'get',self.mixer],universal_newlines=True,stderr=subprocess.DEVNULL,timeout=2)
   return 'pvolume' in out or '%]' in out
  except Exception: return False
 def _discover_device(self):
  # 动态发现真正带音量控件的声卡（不盲取第一张=常是 HDMI）。Go1 头部 3W 扬声器是 USB Audio。
  preferred=self.cfg.get('audio_device','auto'); cards=self._playback_cards()
  mixer_card=next((c for c,_ in cards if self._card_has_mixer(c)),None)
  if preferred!='auto': return preferred,mixer_card
  if mixer_card is not None:
   dev=next((d for c,d in cards if c==mixer_card),0); return 'plughw:%d,%d'%(mixer_card,dev),mixer_card
  if cards: c,d=cards[0]; return 'plughw:%d,%d'%(c,d),mixer_card
  return None,mixer_card
 def _volume(self):
  if self.mixer_card is None: return None
  try:
   out=subprocess.check_output(['amixer','-c',str(self.mixer_card),'get',self.mixer],universal_newlines=True,stderr=subprocess.DEVNULL,timeout=2)
   values=[int(v) for v in re.findall(r'\[(\d+)%\]',out)]; return values[-1] if values else None
  except Exception: return None
 def _volume_detail(self):
  if self.mixer_card is None: return (None,None,None,None)
  try:
   out=subprocess.check_output(['amixer','-c',str(self.mixer_card),'get',self.mixer],universal_newlines=True,stderr=subprocess.DEVNULL,timeout=2)
   pct=[int(v) for v in re.findall(r'\[(\d+)%\]',out)]
   lim=re.search(r'Limits:\s*Playback\s+(\d+)\s*-\s*(\d+)',out)
   raw=re.findall(r'Playback\s+(\d+)\s*\[',out)
   return (pct[-1] if pct else None, int(raw[-1]) if raw else None,
           int(lim.group(1)) if lim else None, int(lim.group(2)) if lim else None)
  except Exception: return (None,None,None,None)
 def _free_audio_device(self):
  # 放 beep 前腾出扬声器 PCM：优雅 kill 掉占用它的进程（通常是 autostart 的 wsaudio），
  # 每次调用自愈，免手动 pkill。🔴 只 SIGTERM，不用 -9。同 unitree 用户无需 sudo。
  m=re.search(r'(\d+),(\d+)',self.device or '')
  if not m:return
  pcm='/dev/snd/pcmC%sD%sp'%(m.group(1),m.group(2))
  try:
   if subprocess.call(['fuser',pcm],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)!=0:return
   subprocess.call(['fuser','-k','-TERM',pcm],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
   for _ in range(15):
    time.sleep(0.2)
    if subprocess.call(['fuser',pcm],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)!=0:return
  except Exception:pass
 def actions(self, action, p):
  rid=p.get('request_id')
  with self.lock:
   if action in ('set_volume','get_volume'):
    if self.volume is None:return fail(action,rid,'VOLUME_CONTROL_UNAVAILABLE','speaker mixer control is unavailable')
    if action=='set_volume':
     try: subprocess.check_call(['amixer','-c',str(self.mixer_card),'set',self.mixer,'%d%%'%p['volume_percent']],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=2);self.volume=self._volume()
     except Exception:return fail(action,rid,'VOLUME_CONTROL_UNAVAILABLE','unable to set speaker volume')
    pct,raw,rmin,rmax=self._volume_detail()
    return reply(action,rid,'idle',{'volume_percent':pct if pct is not None else self.volume,'mixer_raw':raw,'mixer_raw_min':rmin,'mixer_raw_max':rmax})
   if action=='info':return reply(action,rid,'idle',{'device':self.device,'mixer_available':self.volume is not None,'volume_percent':self.volume})
   if action=='beep':
    # 生成 duration_sec 秒、frequency_hz 的正弦音，异步喂给独立 aplay（不阻塞控制 RPC）。
    try:dur=float(p.get('duration_sec',0.3));freq=float(p.get('frequency_hz',1000))
    except (TypeError,ValueError):return fail(action,rid,'INVALID_ARGUMENT','duration_sec/frequency_hz must be numbers')
    if not 0<dur<=10 or not 100<=freq<=8000:return fail(action,rid,'INVALID_ARGUMENT','duration_sec in (0,10], frequency_hz in [100,8000]')
    if not self.device:return fail(action,rid,'DEVICE_NOT_FOUND','speaker device was not found')
    if self.beep_proc and self.beep_proc.poll() is None:return fail(action,rid,'RESOURCE_BUSY','a beep is already playing')
    sr=16000;n=int(sr*dur)
    fade=min(int(sr*0.01),n//2) or 1   # 10ms 淡入淡出去爆音
    def amp(i):
     g=1.0
     if i<fade:g=i/fade
     elif i>=n-fade:g=(n-i)/fade
     return int(0.6*g*32767*math.sin(2*math.pi*freq*i/sr))
    pcm=b''.join(struct.pack('<h',amp(i)) for i in range(n))
    self._free_audio_device()   # 先腾扬声器 PCM（自愈：杀掉占用的 wsaudio 等）再放
    try:proc=subprocess.Popen(['aplay','-q','-D',self.device,'-f','S16_LE','-r','16000','-c','1'],stdin=subprocess.PIPE,stderr=subprocess.DEVNULL)
    except OSError:return fail(action,rid,'PLAYBACK_FAILED','unable to start beep playback')
    time.sleep(.15)   # 让 aplay 尝试打开设备；已退出=打不开(被占/错误)
    if proc.poll() is not None:
     try:proc.stdin.close()
     except Exception:pass
     return fail(action,rid,'RESOURCE_BUSY','speaker device is busy or unavailable (may be held by another process such as wsaudio)',True,{'device_id':self.device,'possible_owner':'wsaudio'})
    self.beep_proc=proc
    threading.Thread(target=self._feed_beep,args=(proc,pcm),daemon=True).start()
    return reply(action,rid,'beeping',{'beeped':True,'duration_sec':dur,'frequency_hz':freq,'device':self.device,'volume_percent':self.volume})
   return fail(action,rid,'INVALID_ARGUMENT','unsupported beep action')
 def _feed_beep(self,proc,pcm):
  try:proc.communicate(pcm,timeout=15)
  except Exception:
   try:proc.kill()
   except Exception:pass

def handler(adapter):
 class H(BaseHTTPRequestHandler):
  def log_message(self,*a):pass
  def do_POST(self):
   try:p=json.loads(self.rfile.read(int(self.headers.get('Content-Length','0')))); path=self.path
   except Exception:p={};path=''
   if path=='/v1/beep/actions':out=adapter.actions(p.get('action'),p)
   else:out=fail('request',None,'INVALID_ARGUMENT','unsupported adapter endpoint')
   raw=json.dumps(out).encode();self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
 return H
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--config',default='/etc/go1-beep-adapter.json');args=ap.parse_args()
 with open(args.config) as f: cfg=json.load(f)
 server=ThreadingHTTPServer((cfg.get('bind_host','0.0.0.0'),int(cfg.get('port',18082))),handler(BeepAdapter(cfg)));server.serve_forever()
