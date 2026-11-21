"""Optional bounded native CPU kernels; the Python algorithms remain portable."""
import ctypes
from pathlib import Path

import numpy as np

try:
    _native = ctypes.CDLL(str(Path(__file__).with_name('_depth_kernels.so')))
except OSError:
    _native = None

if _native is not None:
    _native.s10_fill_holes.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_int, ctypes.c_double]
    _native.s10_window_sum.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p]

AVAILABLE = _native is not None


def _check(status):
    if status == 1:
        raise MemoryError('native depth kernel allocation failed')
    if status:
        raise ValueError('invalid native depth kernel input')


def fill_holes(raw, gap, jump, relative):
    if len(raw) != 640 * 480 * 2:
        raise ValueError('depth packet is not 640x480 uint16')
    output = ctypes.create_string_buffer(len(raw))
    counts = (ctypes.c_int * 2)()
    _check(_native.s10_fill_holes(raw, len(raw), output, counts, gap, jump, relative))
    return output.raw, {'filled_pixels': counts[0], 'fill_conflicts': counts[1]}


def window_sum(values):
    values = np.ascontiguousarray(values, dtype=np.float64)
    if values.shape != (480, 640):
        raise ValueError('depth window is not 640x480')
    output = np.empty_like(values)
    _check(_native.s10_window_sum(values.ctypes.data, values.size, output.ctypes.data))
    return output
