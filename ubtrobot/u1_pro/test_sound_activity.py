import unittest
import numpy as np
from sound_activity import SoundActivityGate

DEVICE = dict(sample_format='S16LE', sample_rate=16000, channels=8,
              device_name='Bothlent UAC Dongle (USB Audio)')
def pcm(amplitude=0, playback=0, dc=0):
    wave = np.sin(np.arange(1600) * 2 * np.pi * 440 / 16000)
    frame = np.zeros((1600,8), dtype='<i2')
    frame[:,:6] = (wave[:,None] * amplitude + dc).astype('<i2')
    frame[:,6:] = (wave[:,None] * playback).astype('<i2')
    return frame.tobytes()

class ActivityTests(unittest.TestCase):
    def setUp(self):
        self.gate=SoundActivityGate(); self.now=0
        self.feed(pcm(10), 12)
    def feed(self, data, count):
        for _ in range(count):
            self.now += .1; self.gate.update(data,DEVICE,self.now)
    def test_quiet_dc_and_playback_do_not_trigger(self):
        for data in (pcm(10),pcm(0,30000),pcm(dc=10000)):
            self.feed(data,20);self.assertFalse(self.gate.allows(self.now))
    def test_speech_stays_active_then_quiet_clears_and_rearms(self):
        background=self.gate.background
        self.feed(pcm(1500),40)
        self.assertTrue(self.gate.allows(self.now))
        self.assertAlmostEqual(self.gate.background,background)
        self.feed(pcm(10),9);self.assertFalse(self.gate.allows(self.now))
        self.feed(pcm(1500),2);self.assertFalse(self.gate.allows(self.now))
        self.feed(pcm(1500),12);self.assertTrue(self.gate.allows(self.now))
    def test_burst_and_invalid_pcm_never_open_gate(self):
        self.feed(pcm(2000),1);self.assertFalse(self.gate.allows(self.now))
        self.feed(pcm(2000),20);self.assertTrue(self.gate.allows(self.now))
        self.feed(b'bad',1);self.assertFalse(self.gate.allows(self.now))
        self.assertEqual(self.gate.reason,'unsupported_or_invalid_audio')
    def test_relative_threshold_blocks_background(self):
        gate=SoundActivityGate()
        for i in range(15):gate.update(pcm(1000),DEVICE,i*.1)
        for i in range(15,40):gate.update(pcm(1500),DEVICE,i*.1)
        self.assertFalse(gate.allows(4))
    def test_parameters_and_reset(self):
        for options in ({'minimum_dbfs':float('nan')},{'minimum_dbfs':True},{'margin_db':0}):
            with self.assertRaises(ValueError):SoundActivityGate(**options)
        self.feed(pcm(2000),20);self.gate.reset()
        self.assertFalse(self.gate.allows(self.now));self.assertIsNone(self.gate.background)

    def test_natural_speech_pauses_do_not_restart_settling_forever(self):
        self.feed(pcm(1500),3)
        self.assertFalse(self.gate.allows(self.now))
        self.feed(pcm(10),3)
        self.assertFalse(self.gate.allows(self.now))
        for _ in range(5):
            self.feed(pcm(1500),3)
            self.assertTrue(self.gate.allows(self.now))
            self.feed(pcm(10),3)
            self.assertFalse(self.gate.allows(self.now))
        self.feed(pcm(10),9)
        self.assertFalse(self.gate.active)
        self.feed(pcm(1500),2)
        self.assertFalse(self.gate.allows(self.now))
