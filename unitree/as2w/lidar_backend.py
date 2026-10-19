"""Optional accelerated point-cloud conversion.

CUDA is deliberately optional: the AS2W image must remain usable on a host
without the NVIDIA container runtime.  CuPy is used only when it is already
installed and can open a CUDA device; otherwise conversion stays in the
small, dependency-free Python path.
"""
import importlib.util


class PointCloudBackend:
    def __init__(self, logger=None):
        self.kind = "cpu"
        self.reason = "CuPy is not installed"
        self._cp = None
        if importlib.util.find_spec("cupy") is None:
            return
        try:
            import cupy as cp
            if int(cp.cuda.runtime.getDeviceCount()) < 1:
                self.reason = "CUDA reports no devices"
                return
            cp.cuda.Device(0).use()
            self._cp = cp
            self.kind = "cuda"
            self.reason = ""
        except Exception as exc:
            self.reason = f"CUDA backend unavailable: {type(exc).__name__}: {str(exc)[:120]}"
        if logger:
            if self.kind == "cuda":
                logger.info("As2W lidar conversion backend=cuda device=0")
            else:
                logger.info(f"As2W lidar conversion backend=cpu fallback={self.reason}")

    def convert(self, data, point_step, point_count, offsets, endian, limit, rotation):
        if self._cp is None:
            return None
        cp = self._cp
        try:
            count = min(point_count, limit)
            stride = max(1, point_count // count)
            indices = cp.arange(0, point_count, stride, dtype=cp.int64)[:count]
            raw = cp.asarray(bytearray(data), dtype=cp.uint8)
            def field(offset):
                positions = indices * point_step + offset
                # PointCloud2 xyz fields are float32 and normally aligned.
                values = raw[positions[:, None] + cp.arange(4)].view(cp.float32).reshape(-1)
                return values.byteswap() if endian else values
            x, y, z = field(offsets["x"]), field(offsets["y"]), field(offsets["z"])
            r = cp.asarray(rotation, dtype=cp.float32)
            xyz = cp.stack((x, y, z), axis=1) @ r.T
            out = cp.stack((-xyz[:, 2], xyz[:, 0], -xyz[:, 1]), axis=1)
            return out.astype(cp.float32).get().tobytes()
        except Exception:
            # A malformed vendor layout must not take down the sensor. The
            # caller records the fallback and can still use the CPU decoder.
            return None
