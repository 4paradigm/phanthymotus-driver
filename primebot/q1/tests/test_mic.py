"""Check the Q1 audio conversion without ROS or a microphone."""

import math
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mic import INPUT_SAMPLES, OUTPUT_SAMPLES, SOURCE_RATE, PcmConverter


def array_pcm(samples):
    interleaved = (sample for pair in samples for sample in (28_000, -27_000, 0, 0, *pair, 0, 0))
    return b"".join(struct.pack("<h", sample) for sample in interleaved)


class PcmConverterTest(unittest.TestCase):
    def test_partial_inputs_preserve_frame_alignment_and_filter_history(self):
        source = array_pcm([(2_000, 6_000)] * (INPUT_SAMPLES * 2))
        contiguous = list(PcmConverter((5, 6)).feed(source))
        converter = PcmConverter((5, 6))
        fragmented = [
            frame
            for offset in range(0, len(source), 997)
            for frame in converter.feed(source[offset:offset + 997])
        ]
        self.assertEqual(fragmented, contiguous)
        self.assertEqual([len(frame) for frame in fragmented], [1024, 1024])
        self.assertEqual(set(struct.unpack(f"<{OUTPUT_SAMPLES}h", fragmented[1])), {4000})

    def test_downsampling_rejects_frequencies_that_would_alias_into_speech(self):
        def rms(frequency):
            tone = [
                round(12_000 * math.sin(2 * math.pi * frequency * i / SOURCE_RATE))
                for i in range(INPUT_SAMPLES * 2)
            ]
            _, steady_frame = PcmConverter((5, 6)).feed(array_pcm(zip(tone, tone)))
            output = struct.unpack(f"<{OUTPUT_SAMPLES}h", steady_frame)
            return math.sqrt(sum(sample**2 for sample in output) / OUTPUT_SAMPLES)

        speech = rms(1_000)
        self.assertGreater(speech, 8_000)
        self.assertLess(rms(11_000), speech * 0.02)

    def test_reference_channels_cannot_be_selected(self):
        with self.assertRaises(ValueError):
            PcmConverter((1, 2))


if __name__ == "__main__":
    unittest.main()
