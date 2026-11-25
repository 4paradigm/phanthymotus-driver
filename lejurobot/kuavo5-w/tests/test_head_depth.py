import importlib.util
from pathlib import Path
import struct
import types
import unittest
import zlib

spec = importlib.util.spec_from_file_location('head_depth', Path(__file__).resolve().parents[1] / 'host/head_depth.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class HeadDepthTests(unittest.TestCase):
    def frame(self, **overrides):
        return types.SimpleNamespace(**({'width': 640, 'height': 480, 'step': 1280,
            'encoding': '16UC1', 'is_bigendian': 0,
            'data': struct.pack('<HH', 0, 1234) * (640 * 480 // 2)} | overrides))

    def test_lossless_millimetres_and_invalid_zero_are_preserved(self):
        frame = self.frame()
        self.assertEqual(zlib.decompress(module.encode(frame)), frame.data)

    def test_big_endian_rows_and_padding_convert_to_exact_dashboard_contract(self):
        row = struct.pack('>H', 4321) * 640 + b'padding!'
        frame = self.frame(is_bigendian=1, step=1288, data=row * 480)
        raw = zlib.decompress(module.encode(frame))
        self.assertEqual(raw, struct.pack('<H', 4321) * (640 * 480))

    def test_wrong_dimensions_encoding_or_truncated_row_are_rejected(self):
        for overrides in ({'width': 320}, {'height': 400}, {'step': 1279}, {'step': 6000},
                          {'data': b'bad'}, {'encoding': '32FC1'}, {'is_bigendian': 2}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                module.encode(self.frame(**overrides))


if __name__ == '__main__':
    unittest.main()
