"""Offline contract tests: no ROS/OpenCV/numpy installation required."""
import importlib.util
from pathlib import Path
import types
import unittest

spec = importlib.util.spec_from_file_location('head_jpeg', Path(__file__).resolve().parents[1] / 'host/head_jpeg.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class HeadJpegTests(unittest.TestCase):
    def frame(self, **overrides):
        return types.SimpleNamespace(**({'width': 2, 'height': 2, 'step': 8,
            'encoding': 'rgb8', 'data': bytes(16)} | overrides))

    def test_unsupported_dimensions_layout_and_payload_are_rejected_before_decode(self):
        for changes in ({'width': 0}, {'width': 4097}, {'height': 2161}, {'step': 5},
                        {'step': 5000}, {'data': bytes(15)}, {'encoding': '16UC1'}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                module.encode(self.frame(**changes), None, None)

    def test_padded_rgb_rows_are_cropped_and_converted_before_jpeg(self):
        calls = []
        class Matrix:
            def reshape(self, *shape):
                calls.append(('reshape', shape))
                return self
            def __getitem__(self, slices):
                calls.append(('crop', slices[1].stop))
                return self
        class Numpy:
            uint8 = 'uint8'
            def frombuffer(self, data, dtype):
                calls.append(('input', len(data), dtype))
                return Matrix()
        class Cv2:
            COLOR_RGB2BGR, IMWRITE_JPEG_QUALITY = 'rgb_to_bgr', 'quality'
            def cvtColor(self, matrix, kind):
                calls.append(('color', kind))
                return matrix
            def imencode(self, ext, matrix, options):
                calls.append(('encode', ext, options))
                return True, types.SimpleNamespace(tobytes=lambda: b'jpeg')
        self.assertEqual(module.encode(self.frame(), Cv2(), Numpy()), b'jpeg')
        self.assertEqual(calls, [('input', 16, 'uint8'), ('reshape', (2, 8)), ('crop', 6),
            ('reshape', (2, 2, 3)), ('color', 'rgb_to_bgr'), ('encode', '.jpg', ['quality', 75])])


if __name__ == '__main__':
    unittest.main()
