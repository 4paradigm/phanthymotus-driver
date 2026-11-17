"""LiDAR-calibrated monocular completion on the front-camera depth grid."""
import math
import zlib

import numpy as np

from deep_robotics.s10_pro import cpu_kernels


def _decode(packet):
    raw = zlib.decompress(packet)
    if len(raw) != 640 * 480 * 2:
        raise ValueError('depth packet is not 640x480 uint16')
    return np.frombuffer(raw, dtype='<u2').reshape(480, 640)


def _window_sum(values):
    # Fixed 17x17 neighbourhood, zero padding: opposite image edges never mix.
    if cpu_kernels.AVAILABLE:
        return cpu_kernels.window_sum(values)
    padded = np.pad(values, ((8, 8), (8, 8)))
    integral = np.pad(padded.cumsum(0).cumsum(1), ((1, 0), (1, 0)))
    return (integral[17:, 17:] - integral[:-17, 17:]
            - integral[17:, :-17] + integral[:-17, :-17])


def fuse_depth(raw_packet, local_packet, mono_packet, *, min_depth_m=.1, max_depth_m=50.0):
    """Return one completed packet, with calibration diagnostics.

    Fit a robust log-affine correction against ORIGINAL returns, then correct
    local log residuals near those returns. A deterministic fifth of reference
    pixels is held out of BOTH stages and checked before any measured/local
    values are restored. This validates completion near supported regions;
    it cannot validate model predictions in wholly unobserved image regions.
    """
    if (not math.isfinite(min_depth_m) or not math.isfinite(max_depth_m)
            or not 0 <= min_depth_m < max_depth_m <= 65.535):
        raise ValueError('invalid fusion depth range')
    raw, local, mono_mm = map(_decode, (raw_packet, local_packet, mono_packet))
    observed = raw != 0
    if not np.array_equal(raw[observed], local[observed]):
        raise ValueError('local depth does not preserve original observations')
    mono = mono_mm.astype(np.float64) / 1000
    measured = raw.astype(np.float64) / 1000
    # Keep the configured valid range; distance alone cannot identify a false
    # near return. Only original observations, never interpolated pixels, fit.
    reference = (measured >= max(.001, min_depth_m)) & (measured <= max_depth_m) & (mono_mm != 0)
    y, x = np.indices(raw.shape)
    train = reference & ((13 * x + 7 * y) % 5 != 0)
    holdout = reference & ~train
    cells = np.unique((y[train] // 80) * 8 + x[train] // 80)
    if train.sum() < 64 or holdout.sum() < 16 or len(cells) < 6:
        raise ValueError('insufficient distributed LiDAR reference points for depth calibration')

    log_mono = np.log(np.maximum(mono, .001))
    indices = np.flatnonzero(train)
    if len(indices) > 4096:
        indices = indices[np.linspace(0, len(indices) - 1, 4096, dtype=int)]
    predicted = log_mono.ravel()[indices]
    target = np.log(measured.ravel()[indices])
    if np.ptp(predicted) < .05:
        a, b = 1.0, float(np.median(target - predicted))
    else:
        weights = np.ones(len(predicted))
        for _ in range(5):
            # Centred two-parameter WLS avoids a LAPACK solve/thread pool per iteration.
            total = weights.sum()
            mean_x, mean_y = (weights * predicted).sum() / total, (weights * target).sum() / total
            centred = predicted - mean_x
            a = (weights * centred * (target - mean_y)).sum() / (weights * centred**2).sum()
            b = mean_y - a * mean_x
            residual = a * predicted + b - target
            scatter = max(.03, 1.4826 * float(np.median(np.abs(residual - np.median(residual)))))
            weights = np.minimum(1, 1.5 * scatter / np.maximum(np.abs(residual), 1e-6))
    if not (math.isfinite(a) and math.isfinite(b) and a > 0):
        raise ValueError('depth calibration has inconsistent metric correspondences')

    log_depth = a * log_mono + b
    residual = np.zeros(raw.shape, dtype=np.float64)
    residual[train] = np.log(measured[train]) - log_depth[train]
    count = _window_sum(train.astype(np.float64))
    correction = _window_sum(residual) / np.maximum(1, count)
    with np.errstate(over='ignore', invalid='ignore'):
        depth = np.exp(log_depth + np.where(count >= 3, correction, 0))
    error = np.abs(depth[holdout] - measured[holdout]) / measured[holdout]
    median, p90 = np.percentile(error, [50, 90])
    if not (np.isfinite(error).all() and median <= .25 and p90 <= .5):
        raise ValueError('depth calibration failed held-out LiDAR consistency check')

    valid = ((mono_mm != 0) & np.isfinite(depth)
             & (depth >= max(.001, min_depth_m)) & (depth <= max_depth_m))
    output = np.rint(np.where(valid, depth * 1000, 0)).astype('<u2')
    added = (local == 0) & (output != 0)
    output[local != 0] = local[local != 0]
    return zlib.compress(output.tobytes(), level=1), {
        'calibration_a': float(a), 'calibration_b': float(b),
        'calibration_reference_pixels': int(reference.sum()),
        'calibration_holdout_pixels': int(holdout.sum()),
        'calibration_median_relative_error': float(median),
        'calibration_p90_relative_error': float(p90),
        'monocular_pixels': int(added.sum()),
        'output_coverage': round(float(np.count_nonzero(output) / output.size), 4),
    }
