#!/usr/bin/env python3
"""Go1 头部扬声器音频流播放适配器（跑在 Head Nano，不在驱动容器内）。

服务 speaker 卡片的音频流播放，两种传输通道：
  - TCP 二进制帧（:18084，低延迟 PCM 流）← v2 主通道
  - HTTP JSON（:18083，控制命令 volume/stop/info + 兼容 v1 base64 play）

与 beep 专属的 beep_adapter.py（:18082，正弦 beep）分离、互不影响——两者都「用时才起 aplay、
放完释放」，不长占设备（真同时放时后到的拿 RESOURCE_BUSY）。

播放模型（连续「音频流」）：维持一个常驻 aplay 会话（S16_LE / sample_rate / channels），
每次 play 把 PCM **同步写进 aplay 的 stdin**（管道满则自然背压，等播够再收下一段
→ 天然流控），从而多段无缝续播；空闲超过 idle_timeout 自动收掉会话。format(采样率/声道)变了会重开。

TCP 二进制帧格式：
  [4B uint32 BE] frame_size (含帧头 8 字节)
  [2B uint16 BE] sample_rate
  [1B uint8]     channels
  [1B int8]      type (0x01=pcm_data)
  [...raw PCM S16_LE...]

只接受固定的 speaker 卡 API；不接受任何调用方传入的 shell 命令 / URL / 设备路径。
Nano 是 Python 3.6：无 f-string 限制但 subprocess 用 universal_newlines、ThreadingHTTPServer 需 fallback。
"""
import argparse, base64, json, re, socket, struct, subprocess, threading, time
from http.server import BaseHTTPRequestHandler
try:                                    # Nano 是 Python 3.6，无 ThreadingHTTPServer(3.7+)
    from http.server import ThreadingHTTPServer
except ImportError:
    from http.server import HTTPServer
    from socketserver import ThreadingMixIn
    class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
        daemon_threads = True

# 单次 play 解码后 PCM 上限（4MB ~ 128s@16k/16bit/mono）；连续音频请分段多次 play。
MAX_PCM_BYTES = 4 * 1024 * 1024

# TCP 帧常量
_FRAME_HEADER_SIZE = 8
_FRAME_PCM = 0x01


def now(): return int(time.time() * 1000)


def reply(card, action, request_id, state, applied):
    return {"ok": True, "card": card, "action": action, "request_id": request_id,
            "state": state, "applied": applied, "timestamp_ms": now()}


def fail(card, action, rid, code, message, retryable=False, details=None):
    return {"ok": False, "card": card, "action": action, "request_id": rid, "code": code,
            "message": message, "details": details or {}, "retryable": retryable, "timestamp_ms": now()}


class SpeakerAdapter:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.RLock()
        self.mixer = cfg.get('mixer_control', 'Speaker')
        self.device, self.mixer_card = self._discover_device()
        self.volume = self._volume()
        self.idle_timeout = float(cfg.get('idle_timeout_sec', 15.0))
        # 低延迟关键：给 aplay 设小 ALSA 缓冲/周期（默认 aplay 会用 ~500ms 大缓冲=固定高延迟）。
        # buffer_us=75ms/period_us=10ms → 延迟大降；若在狗上听到卡顿/爆音(欠载)就把这两个调大点。
        self.aplay_buffer_us = int(cfg.get('aplay_buffer_us', 75000))
        self.aplay_period_us = int(cfg.get('aplay_period_us', 10000))
        self.proc = None                 # 常驻 aplay 进程（None=空闲）
        self.cur_sr = None
        self.cur_ch = None
        self.last_activity = time.time()
        threading.Thread(target=self._idle_reaper, daemon=True).start()

        # TCP 流监听（v2 低延迟二进制通道）
        self.stream_port = int(cfg.get('stream_port', 18084))
        self.stream_bind = cfg.get('bind_host', '0.0.0.0')
        threading.Thread(target=self._tcp_server, daemon=True, name='speaker_tcp').start()

    # ── 设备发现 / 音量（逻辑同 beep_adapter，自包含复制）──────────────────────
    def _playback_cards(self):
        try:
            out = subprocess.check_output(['aplay', '-l'], universal_newlines=True,
                                          stderr=subprocess.DEVNULL, timeout=2)
            return [(int(c), int(d)) for c, d in re.findall(r'^card (\d+):.*?device (\d+):', out, re.M)]
        except Exception:
            return []

    def _card_has_mixer(self, card):
        try:
            out = subprocess.check_output(['amixer', '-c', str(card), 'get', self.mixer],
                                          universal_newlines=True, stderr=subprocess.DEVNULL, timeout=2)
            return 'pvolume' in out or '%]' in out
        except Exception:
            return False

    def _discover_device(self):
        # 动态发现真正带音量控件的声卡（不盲取第一张=常是 HDMI）。Go1 头部 3W 扬声器是 USB Audio。
        preferred = self.cfg.get('audio_device', 'auto')
        cards = self._playback_cards()
        mixer_card = next((c for c, _ in cards if self._card_has_mixer(c)), None)
        if preferred != 'auto':
            return preferred, mixer_card
        if mixer_card is not None:
            dev = next((d for c, d in cards if c == mixer_card), 0)
            return 'plughw:%d,%d' % (mixer_card, dev), mixer_card
        if cards:
            c, d = cards[0]
            return 'plughw:%d,%d' % (c, d), mixer_card
        return None, mixer_card

    def _volume(self):
        if self.mixer_card is None:
            return None
        try:
            out = subprocess.check_output(['amixer', '-c', str(self.mixer_card), 'get', self.mixer],
                                          universal_newlines=True, stderr=subprocess.DEVNULL, timeout=2)
            values = [int(v) for v in re.findall(r'\[(\d+)%\]', out)]
            return values[-1] if values else None
        except Exception:
            return None

    def _volume_detail(self):
        if self.mixer_card is None:
            return (None, None, None, None)
        try:
            out = subprocess.check_output(['amixer', '-c', str(self.mixer_card), 'get', self.mixer],
                                          universal_newlines=True, stderr=subprocess.DEVNULL, timeout=2)
            pct = [int(v) for v in re.findall(r'\[(\d+)%\]', out)]
            lim = re.search(r'Limits:\s*Playback\s+(\d+)\s*-\s*(\d+)', out)
            raw = re.findall(r'Playback\s+(\d+)\s*\[', out)
            return (pct[-1] if pct else None, int(raw[-1]) if raw else None,
                    int(lim.group(1)) if lim else None, int(lim.group(2)) if lim else None)
        except Exception:
            return (None, None, None, None)

    def _free_audio_device(self):
        # 放音前腾出扬声器 PCM：优雅 kill 掉占用它的进程（通常是 autostart 的 wsaudio），每次自愈。
        m = re.search(r'(\d+),(\d+)', self.device or '')
        if not m:
            return
        pcm = '/dev/snd/pcmC%sD%sp' % (m.group(1), m.group(2))
        try:
            if subprocess.call(['fuser', pcm], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) != 0:
                return
            subprocess.call(['fuser', '-k', '-TERM', pcm], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            for _ in range(15):
                time.sleep(0.2)
                if subprocess.call(['fuser', pcm], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) != 0:
                    return
        except Exception:
            pass

    # ── aplay 会话生命周期（都在持锁下调用）────────────────────────────────────
    def _open_proc(self, sr, ch):
        """开一个常驻 aplay 读 stdin。成功 True；设备被占/打不开 False。"""
        if not self.device:
            return False
        self._free_audio_device()   # 先腾扬声器 PCM（自愈：杀掉占用的 wsaudio 等）再放
        try:
            cmd = ['aplay', '-q', '-D', self.device, '-f', 'S16_LE', '-r', str(sr), '-c', str(ch)]
            # 小 ALSA 缓冲/周期 → 低延迟（0 表示不指定，用 aplay 默认大缓冲）。
            if self.aplay_buffer_us > 0:
                cmd += ['--buffer-time=%d' % self.aplay_buffer_us]
            if self.aplay_period_us > 0:
                cmd += ['--period-time=%d' % self.aplay_period_us]
            p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
        except OSError:
            return False
        time.sleep(0.15)   # 让 aplay 尝试打开设备；已退出=打不开(被占/错误)
        if p.poll() is not None:
            try:
                p.stdin.close()
            except Exception:
                pass
            return False
        self.proc, self.cur_sr, self.cur_ch = p, sr, ch
        return True

    def _close_proc(self, drain):
        """drain=True：关 stdin 让 aplay 把已缓冲音频放完再退（空闲收）；False：立即 terminate（stop/换格式）。"""
        p, self.proc, self.cur_sr, self.cur_ch = self.proc, None, None, None
        if p is None:
            return
        try:
            if drain:
                if p.stdin:
                    p.stdin.close()
                p.wait(timeout=3)
            else:
                p.terminate()
                p.wait(timeout=1)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass

    def _idle_reaper(self):
        while True:
            time.sleep(1.0)
            with self.lock:
                if self.proc is not None and time.time() - self.last_activity > self.idle_timeout:
                    self._close_proc(drain=True)

    # ── 共用 PCM 播放逻辑（TCP 和 HTTP 都调这个）────────────────────────────────
    def _play_pcm(self, sr, ch, data):
        """把一段裸 PCM 写入 aplay stdin。必须在 self.lock 下调用。
        返回 (ok: bool, error_tuple_or_None)。"""
        if not self.device:
            return False, ('DEVICE_NOT_FOUND', 'speaker device was not found', False)
        if not 8000 <= sr <= 48000 or ch not in (1, 2):
            return False, ('INVALID_ARGUMENT', 'sample_rate in [8000,48000], channels in {1,2}', False)
        if len(data) > MAX_PCM_BYTES:
            return False, ('INVALID_ARGUMENT', 'PCM chunk too large (max %d bytes)' % MAX_PCM_BYTES, False)
        # 无会话 / 采样率或声道变了 → 重开 aplay
        if self.proc is None or sr != self.cur_sr or ch != self.cur_ch:
            self._close_proc(drain=False)
            if not self._open_proc(sr, ch):
                return False, ('RESOURCE_BUSY',
                               'speaker device is busy or unavailable (may be held by another process such as wsaudio)',
                               True)
        # 同步写入 aplay stdin：管道满则阻塞→天然背压。BrokenPipe=aplay 挂了，重开一次。
        try:
            self.proc.stdin.write(data)
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError):
            self._close_proc(drain=False)
            if not self._open_proc(sr, ch):
                return False, ('RESOURCE_BUSY', 'speaker device became unavailable', True)
            try:
                self.proc.stdin.write(data)
                self.proc.stdin.flush()
            except Exception:
                self._close_proc(drain=False)
                return False, ('PLAYBACK_FAILED', 'unable to write audio to speaker', False)
        self.last_activity = time.time()
        return True, None

    # ── TCP 二进制流服务（v2 低延迟通道）────────────────────────────────────────
    def _tcp_server(self):
        """在 stream_port 上监听，接受单条持久 TCP 连接，循环读二进制帧写入 aplay。"""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.stream_bind, self.stream_port))
        srv.listen(1)
        print('[speaker_adapter] TCP stream listening on %s:%d' % (self.stream_bind, self.stream_port))
        while True:
            try:
                conn, addr = srv.accept()
            except Exception:
                time.sleep(0.5)
                continue
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print('[speaker_adapter] TCP stream connected from %s:%d' % addr)
            try:
                self._handle_stream(conn)
            except Exception:
                pass
            finally:
                try:
                    conn.close()
                except Exception:
                    pass
                print('[speaker_adapter] TCP stream disconnected')

    def _handle_stream(self, conn):
        """循环读二进制帧，解帧后调 _play_pcm。连接断开或异常时返回。"""
        buf = b''
        while True:
            try:
                data = conn.recv(65536)
            except Exception:
                break
            if not data:
                break
            buf += data
            # 解帧循环：可能一次 recv 里有多帧
            while len(buf) >= _FRAME_HEADER_SIZE:
                frame_size = struct.unpack('>I', buf[:4])[0]
                if frame_size < _FRAME_HEADER_SIZE or frame_size > _FRAME_HEADER_SIZE + MAX_PCM_BYTES:
                    buf = b''   # 帧损坏，丢弃（下一轮 recv 可能恢复）
                    break
                if len(buf) < frame_size:
                    break       # 帧未收齐，继续 recv
                sr = struct.unpack('>H', buf[4:6])[0]
                ch = buf[6]
                ftype = buf[7]
                pcm = buf[_FRAME_HEADER_SIZE:frame_size]
                buf = buf[frame_size:]
                if ftype == _FRAME_PCM and pcm and len(pcm) % 2 == 0:
                    with self.lock:
                        self._play_pcm(sr, ch, pcm)

    # ── HTTP 动作（控制命令 + v1 兼容 base64 play）─────────────────────────────
    def actions(self, action, p):
        rid = p.get('request_id')
        card = p.get('card', 'speaker')
        with self.lock:
            if action in ('set_volume', 'get_volume'):
                if self.volume is None:
                    return fail(card, action, rid, 'VOLUME_CONTROL_UNAVAILABLE', 'speaker mixer control is unavailable')
                if action == 'set_volume':
                    try:
                        subprocess.check_call(['amixer', '-c', str(self.mixer_card), 'set', self.mixer,
                                               '%d%%' % p['volume_percent']],
                                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
                        self.volume = self._volume()
                    except Exception:
                        return fail(card, action, rid, 'VOLUME_CONTROL_UNAVAILABLE', 'unable to set speaker volume')
                pct, raw, rmin, rmax = self._volume_detail()
                return reply(card, action, rid, self._state(), {
                    'volume_percent': pct if pct is not None else self.volume,
                    'mixer_raw': raw, 'mixer_raw_min': rmin, 'mixer_raw_max': rmax})

            if action == 'info':
                return reply(card, action, rid, self._state(), {
                    'device': self.device, 'mixer_available': self.volume is not None,
                    'volume_percent': self.volume, 'sample_rate': self.cur_sr, 'channels': self.cur_ch})

            if action == 'stop':
                self._close_proc(drain=False)
                return reply(card, action, rid, 'idle', {'stopped': True})

            if action == 'play':
                # v1 兼容：HTTP base64 play（仍可用，但 v2 主通道走 TCP 二进制）
                b64 = p.get('pcm_base64')
                if not isinstance(b64, str) or not b64:
                    return fail(card, action, rid, 'INVALID_ARGUMENT', 'pcm_base64 must be a non-empty base64 string')
                try:
                    pcm_data = base64.b64decode(b64, validate=True)
                except Exception:
                    return fail(card, action, rid, 'INVALID_ARGUMENT', 'pcm_base64 is not valid base64')
                if not pcm_data or len(pcm_data) % 2 != 0:
                    return fail(card, action, rid, 'INVALID_ARGUMENT', 'PCM empty or not 16-bit aligned')
                try:
                    sr = int(p.get('sample_rate', 16000))
                    ch = int(p.get('channels', 1))
                except (TypeError, ValueError):
                    return fail(card, action, rid, 'INVALID_ARGUMENT', 'sample_rate/channels must be integers')
                ok, err = self._play_pcm(sr, ch, pcm_data)
                if not ok:
                    return fail(card, action, rid, err[0], err[1], err[2],
                                {'device': self.device} if 'BUSY' in err[0] else {})
                return reply(card, action, rid, 'playing', {
                    'played_bytes': len(pcm_data), 'sample_rate': sr, 'channels': ch,
                    'device': self.device, 'volume_percent': self.volume})

            return fail(card, action, rid, 'INVALID_ARGUMENT', 'unsupported speaker action')

    def _state(self):
        return 'playing' if (self.proc is not None and self.proc.poll() is None) else 'idle'


def handler(adapter):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_POST(self):
            try:
                p = json.loads(self.rfile.read(int(self.headers.get('Content-Length', '0'))))
                path = self.path
            except Exception:
                p = {}
                path = ''
            if path == '/v1/speaker/actions':
                out = adapter.actions(p.get('action'), p)
            else:
                out = fail('speaker', 'request', None, 'INVALID_ARGUMENT', 'unsupported adapter endpoint')
            raw = json.dumps(out).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
    return H


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default='/etc/go1-speaker-adapter.json')
    args = ap.parse_args()
    with open(args.config) as f:
        cfg = json.load(f)
    adapter = SpeakerAdapter(cfg)
    print('[speaker_adapter] HTTP control on %s:%d, TCP stream on %s:%d' % (
        cfg.get('bind_host', '0.0.0.0'), int(cfg.get('port', 18083)),
        cfg.get('bind_host', '0.0.0.0'), int(cfg.get('stream_port', 18084))))
    server = ThreadingHTTPServer((cfg.get('bind_host', '0.0.0.0'), int(cfg.get('port', 18083))),
                                 handler(adapter))
    server.serve_forever()
