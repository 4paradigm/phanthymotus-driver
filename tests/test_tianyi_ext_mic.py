"""Tianyi's remote mic: a DJI receiver on PC1, captured there, streamed to PC2.

Every case here is something that went wrong on a new Tianyi 2.0 Pro
(Shenzhen, 2026-11-03), where the mic was found, chosen, and silent:

  - the SSH account was not in `audio`, so `arecord -l` said "no soundcards"
    and the probe reported no devices, as if nothing were plugged in
  - the startup fallback entry had no `_ssh_host`, so a later probe could not
    replace it: two options shared one value, the card showed the IP, and a
    start that picked the stale one had no SSH config to restart the sender
  - the sender's capture thread could die and leave its port open
  - pyalsaaudio was missing on PC1, and PC1 had no route to a package index

No robot, no ROS, no SSH: the remote host is a function answering commands.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_tianyi_ext_mic.py -q
"""

from __future__ import annotations

import importlib.util
import io
import subprocess
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np  # noqa: F401  — load before sys.modules is patched

DRIVER = Path(__file__).resolve().parents[1] / "x-humanoid" / "tianyi2.0"
PASSWORD = "s3cret-pw"
HOST = "192.168.41.1"
URL = f"tcp://{HOST}:9800/pcm16k"


def _load(name: str, filename: str):
    stubs = {n: types.ModuleType(n) for n in (
        "cv2", "rclpy", "rclpy.node", "rclpy.qos", "std_msgs.msg",
        "sensor_msgs.msg", "audio_msgs.msg",
    )}
    stubs["rclpy.node"].Node = object
    qos = stubs["rclpy.qos"]
    qos.QoSProfile = lambda **kwargs: None
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    stubs["std_msgs.msg"].Header = object
    stubs["sensor_msgs.msg"].CompressedImage = object
    stubs["audio_msgs.msg"].AudioChunk = object
    spec = importlib.util.spec_from_file_location(name, DRIVER / filename)
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


ext = _load("tianyi_ext_devices_under_test", "ext_devices.py")
sender = _load("tianyi_audio_sender_under_test", "audio_sender.py")

ARECORD_L = (
    "**** List of CAPTURE Hardware Devices ****\n"
    "card 0: PCH [HDA Intel PCH], device 0: ALC897 Analog [ALC897 Analog]\n"
    "card 1: Rx [Wireless Mic Rx], device 0: USB Audio [USB Audio]\n"
)
HW_PARAMS = "FORMAT:  S24_3LE\nCHANNELS: 2\nRATE: 48000\n"


class FakeHost:
    """PC1, as seen through `sshpass ssh` and `sshpass scp`."""

    def __init__(self, in_audio=True, sudo_ok=True, sender_md5=None, scp_ok=True):
        self.in_audio = in_audio
        self.sudo_ok = sudo_ok
        self.sender_md5 = sender_md5
        self.scp_ok = scp_ok
        self.commands: list[str] = []
        self.scps = 0

    def run(self, argv, **kwargs):
        def done(out="", rc=0, err=""):
            return subprocess.CompletedProcess(argv, rc, out, err)

        if argv[3] == "scp":
            self.scps += 1
            if not self.scp_ok:
                return done(rc=1, err=f"scp: permission denied (cmd had {PASSWORD})")
            self.sender_md5 = "deployed"
            return done()
        cmd = argv[-1]
        self.commands.append(cmd)
        if cmd.startswith("arecord -l"):
            return done(ARECORD_L if self.in_audio else "arecord: no soundcards found...\n")
        if cmd.startswith("cat /proc/asound/cards"):
            groups = "ubuntu adm sudo" + (" audio" if self.in_audio else "")
            return done(" 0 [PCH ]: HDA-Intel\n 1 [Rx ]: USB-Audio - Wireless Mic Rx\n---\n" + groups)
        if "usermod -aG audio" in cmd:
            if not self.sudo_ok:
                return done(rc=1, err="Sorry, try again.")
            self.in_audio = True
            return done()
        if cmd.startswith("ls -d /proc/asound/card*/usbid"):
            return done("/proc/asound/card1/usbid\n")
        if "--dump-hw-params" in cmd:
            return done(HW_PARAMS)
        if cmd.startswith("md5sum"):
            return done(f"{self.sender_md5}  /home/ubuntu/audio_sender.py\n" if self.sender_md5 else "")
        return done()


def probe(host: FakeHost, streaming=True):
    out = io.StringIO()
    with mock.patch.object(ext.subprocess, "run", host.run), \
         mock.patch.object(ext, "_tcp_has_data", lambda *a, **k: streaming), \
         mock.patch.object(ext.time, "sleep", lambda s: None), \
         mock.patch("sys.stdout", out):
        entries = ext.probe_remote_host(HOST, "ubuntu", PASSWORD)
    return entries, out.getvalue()


def current_md5():
    import hashlib
    return hashlib.md5(ext.SENDER_SRC.read_bytes()).hexdigest()


class ProbeRemoteHostTest(unittest.TestCase):
    def test_account_outside_audio_is_added_and_the_mic_found(self):
        host = FakeHost(in_audio=False, sender_md5=current_md5())
        entries, log = probe(host)
        self.assertTrue(any("usermod -aG audio ubuntu" in c for c in host.commands))
        self.assertEqual(len(entries), 1)
        self.assertIn("Wireless Mic Rx", entries[0]["name"])
        self.assertIn("not in the 'audio' group", log)
        self.assertNotIn(PASSWORD, log)

    def test_sudo_refused_says_what_to_run_instead_of_no_devices(self):
        entries, log = probe(FakeHost(in_audio=False, sudo_ok=False))
        self.assertEqual(entries, [])
        self.assertIn("sudo usermod -aG audio ubuntu", log)
        self.assertNotIn(PASSWORD, log)

    def test_password_never_reaches_a_process_argv_on_the_remote(self):
        host = FakeHost(in_audio=False)
        probe(host)
        cmd = next(c for c in host.commands if "usermod" in c)
        # printf is a builtin; the password is in the command string, piped to sudo -S.
        self.assertTrue(cmd.startswith("printf "))
        self.assertIn("sudo -S", cmd)

    def test_offers_the_usb_card_once_with_full_management_fields(self):
        entries, _ = probe(FakeHost(sender_md5=current_md5()))
        self.assertEqual(len(entries), 1, "one sender, one card, one option")
        e = entries[0]
        self.assertEqual(e["alsa_id"], URL)
        self.assertEqual(e["_ssh_card"], "1")
        self.assertEqual((e["_ssh_host"], e["_ssh_user"], e["_ssh_pass"]), (HOST, "ubuntu", PASSWORD))
        self.assertIn("S24_3LE/48000Hz/2ch", e["name"])

    def test_missing_sender_is_deployed(self):
        host = FakeHost(sender_md5=None)
        probe(host)
        self.assertEqual(host.scps, 1)

    def test_stale_sender_is_replaced_and_restarted(self):
        host = FakeHost(sender_md5="0" * 32)
        probe(host)
        self.assertEqual(host.scps, 1)
        self.assertTrue(any(c.startswith("pkill -9 -f") for c in host.commands),
                        "the old process would keep running the old file")

    def test_current_sender_that_streams_is_left_alone(self):
        host = FakeHost(sender_md5=current_md5())
        probe(host)
        self.assertEqual(host.scps, 0)
        self.assertFalse(any(c.startswith(("pkill", "nohup")) for c in host.commands))

    def test_failed_scp_is_reported_without_the_password(self):
        _, log = probe(FakeHost(scp_ok=False))
        self.assertIn("deploy failed", log)
        self.assertNotIn(PASSWORD, log)

    def test_not_streaming_is_visible_in_the_option_and_log(self):
        entries, log = probe(FakeHost(sender_md5=current_md5()), streaming=False)
        self.assertIn("[not streaming]", entries[0]["name"])
        self.assertIn("audio_sender is not streaming", log)


def plugin(network_sources=None, remote_devices=None):
    with mock.patch.object(ext, "_enumerate_ext_mics", lambda: []):
        return ext.ExtMicPlugin({"network_sources": network_sources or []}, "ns", None,
                                remote_devices=remote_devices)


SOURCE = {"ssh_host": HOST, "ssh_user": "ubuntu", "ssh_password": PASSWORD,
          "port": 9800, "sender_path": "/home/ubuntu/audio_sender.py"}


class ExtMicPluginListTest(unittest.TestCase):
    def probed(self, name="Wireless Mic Rx (S24_3LE/48000Hz/2ch) @ " + HOST):
        return [{"index": URL, "alsa_id": URL, "name": name, "network": True,
                 "_ssh_host": HOST, "_ssh_user": "ubuntu", "_ssh_pass": PASSWORD,
                 "_ssh_card": "1", "_ssh_script": "/home/ubuntu/audio_sender.py", "_port": 9800}]

    def test_fallback_entry_carries_ssh_config(self):
        p = plugin([SOURCE])
        [e] = p._available_devices
        self.assertEqual(e["alsa_id"], URL)
        self.assertEqual((e["_ssh_host"], e["_ssh_user"], e["_ssh_pass"]), (HOST, "ubuntu", PASSWORD))
        self.assertEqual(e["_ssh_script"], "/home/ubuntu/audio_sender.py")

    def test_no_fallback_when_startup_probe_found_the_host(self):
        p = plugin([SOURCE], remote_devices=self.probed())
        self.assertEqual(len(p._available_devices), 1)

    def test_successful_probe_replaces_the_fallback(self):
        p = plugin([SOURCE])
        with mock.patch.object(ext, "probe_remote_host", lambda *a, **k: self.probed()):
            p._probe_remote(HOST, "ubuntu", PASSWORD, 9800, "/home/ubuntu/audio_sender.py")
        values = [d["alsa_id"] for d in p._available_devices]
        self.assertEqual(values, [URL])
        self.assertIn("Wireless Mic Rx", p._available_devices[0]["name"])

    def test_legacy_entry_without_ssh_host_is_replaced_by_url(self):
        p = plugin()
        p._available_devices = [{"index": URL, "alsa_id": URL, "name": HOST, "network": True}]
        with mock.patch.object(ext, "probe_remote_host", lambda *a, **k: self.probed()):
            p._probe_remote(HOST, "ubuntu", PASSWORD, 9800, "/home/ubuntu/audio_sender.py")
        self.assertEqual([d["name"] for d in p._available_devices], [self.probed()[0]["name"]])

    def test_repeated_probes_do_not_accumulate(self):
        p = plugin([SOURCE])
        with mock.patch.object(ext, "probe_remote_host", lambda *a, **k: self.probed()):
            for _ in range(3):
                p._probe_remote(HOST, "ubuntu", PASSWORD, 9800, "/home/ubuntu/audio_sender.py")
        self.assertEqual(len(p._available_devices), 1)

    def test_failed_probe_keeps_what_was_known(self):
        p = plugin(remote_devices=self.probed())
        with mock.patch.object(ext, "probe_remote_host", lambda *a, **k: []):
            p._probe_remote(HOST, "ubuntu", PASSWORD, 9800, "/home/ubuntu/audio_sender.py")
        self.assertEqual(p._available_devices, self.probed())

    def test_other_hosts_survive(self):
        other = {"index": "tcp://10.0.0.9:9800/pcm16k", "alsa_id": "tcp://10.0.0.9:9800/pcm16k",
                 "name": "other", "network": True, "_ssh_host": "10.0.0.9"}
        p = plugin(remote_devices=[other])
        with mock.patch.object(ext, "probe_remote_host", lambda *a, **k: self.probed()):
            p._probe_remote(HOST, "ubuntu", PASSWORD, 9800, "/home/ubuntu/audio_sender.py")
        self.assertIn(other, p._available_devices)
        self.assertEqual(len(p._available_devices), 2)

    def test_config_probe_uses_the_configured_port_and_path(self):
        src = dict(SOURCE, port=9900, sender_path="/opt/mic/audio_sender.py")
        p = plugin([src])
        seen = {}
        with mock.patch.object(ext, "probe_remote_host",
                               lambda *a, **k: seen.update(k) or []):
            p.dispatch("config", {"ssh_host": HOST, "ssh_user": "ubuntu", "ssh_pass": PASSWORD})
        self.assertEqual((seen["port"], seen["script"]), (9900, "/opt/mic/audio_sender.py"))


class AudioSenderTest(unittest.TestCase):
    def setUp(self):
        sender._use_alsaaudio = True

    def test_capture_thread_death_ends_the_process(self):
        def boom(*a):
            raise ImportError("No module named 'alsaaudio'")
        with mock.patch.object(sender, "_capture_loop", boom), \
             mock.patch.object(sender.os, "_exit", side_effect=SystemExit) as exit_, \
             mock.patch("sys.stderr", io.StringIO()), mock.patch("sys.stdout", io.StringIO()):
            with self.assertRaises(SystemExit):
                sender._capture_or_exit(1, [], None)
        exit_.assert_called_once_with(1)

    def test_missing_pyalsaaudio_falls_back_to_arecord(self):
        with mock.patch.dict(sys.modules, {"alsaaudio": None}), \
             mock.patch.object(sender.subprocess, "Popen") as popen, \
             mock.patch("sys.stdout", io.StringIO()):
            pcm = sender._open_pcm(1, 48000, True)
        self.assertIsInstance(pcm, sender._ArecordPCM)
        argv = popen.call_args[0][0]
        self.assertEqual(argv[:4], ["arecord", "-q", "-D", "hw:1,0"])
        self.assertIn("S24_3LE", argv)
        self.assertFalse(sender._use_alsaaudio, "a missing module is not retried on every reopen")

    def test_old_pyalsaaudio_without_s24_falls_back_to_arecord(self):
        old = types.ModuleType("alsaaudio")       # 0.8.4: no PCM_FORMAT_S24_3LE
        old.PCM_CAPTURE, old.PCM_NORMAL, old.PCM_FORMAT_S16_LE = 1, 0, 2
        old.PCM = lambda *a, **k: (_ for _ in ()).throw(TypeError("unexpected keyword"))
        with mock.patch.dict(sys.modules, {"alsaaudio": old}), \
             mock.patch.object(sender.subprocess, "Popen"), \
             mock.patch("sys.stdout", io.StringIO()):
            self.assertIsInstance(sender._open_pcm(1, 48000, True), sender._ArecordPCM)

    def test_arecord_pcm_reads_whole_frames_and_raises_when_it_exits(self):
        proc = mock.Mock()
        proc.stdout = io.BytesIO(b"\x01" * 6 * 10)
        proc.wait.return_value = 1
        with mock.patch.object(sender.subprocess, "Popen", return_value=proc):
            pcm = sender._ArecordPCM(1, 48000, 2, "S24_3LE", 1024)
        n, data = pcm.read()
        self.assertEqual((n, len(data)), (10, 60))
        with self.assertRaises(OSError):
            pcm.read()


if __name__ == "__main__":
    unittest.main()
