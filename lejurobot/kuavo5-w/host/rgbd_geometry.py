"""Rectify both streams onto measured depth rays; host OpenCV/NumPy only."""
from copy import deepcopy
import threading


class Geometry:
    def __init__(self):
        self.lock = threading.Lock()
        self.infos, self.signature, self.value = {}, None, None

    def receive(self, kind, message):
        with self.lock:
            self.infos[kind] = message

    def get(self, cv2, np):
        with self.lock:
            if set(self.infos) != {'color', 'depth'}:
                raise ValueError('waiting for color/depth CameraInfo')
            color, depth = self.infos['color'], self.infos['depth']
            signature = tuple((i.width, i.height, i.header.frame_id, tuple(i.K), tuple(i.D),
                               i.distortion_model) for i in (color, depth))
            if signature != self.signature:
                self.value = make_geometry(color, depth, cv2, np)
                self.signature = signature
            return self.value


def make_geometry(color, depth, cv2, np):
    if (color.header.frame_id != depth.header.frame_id or not depth.header.frame_id
            or (depth.width, depth.height) != (640, 480)
            or color.distortion_model not in ('plumb_bob', 'rational_polynomial')
            or len(color.D) not in (5, 8) or any(depth.D)):
        raise ValueError('registered rectified depth and supported RGB distortion required')
    for info in (color, depth):
        k = np.asarray(info.K, dtype=float).reshape(3, 3)
        if (not np.isfinite(k).all() or not np.isfinite(info.D).all()
                or k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1])
                or k[0, 1] != 0 or k[1, 0] != 0):
            raise ValueError('invalid source calibration')
    k = np.asarray(depth.K, dtype=float).reshape(3, 3).copy()
    u, v = np.meshgrid(np.arange(640), np.arange(480))
    def maps(matrix):
        cx, cy = cv2.initUndistortRectifyMap(np.asarray(color.K).reshape(3, 3),
            np.asarray(color.D), None, matrix, (640, 480), cv2.CV_32FC1)
        dx = ((u-matrix[0, 2])/matrix[0, 0]*depth.K[0]+depth.K[2]).astype(np.float32)
        dy = ((v-matrix[1, 2])/matrix[1, 1]*depth.K[4]+depth.K[5]).astype(np.float32)
        valid = ((cx >= 0) & (cx <= color.width-1) & (cy >= 0) & (cy <= color.height-1)
                 & (dx >= 0) & (dx <= depth.width-1) & (dy >= 0) & (dy <= depth.height-1))
        return (cx, cy, dx, dy), valid
    values, valid = maps(k)
    if not valid.all():
        # A common central field, not black RGB rays paired with real depth.
        radius = np.maximum(np.abs(u-320.)/320., np.abs(v-240.)/240.)
        extent = float(radius[~valid].min())-1./240.
        if extent <= 0:
            raise ValueError('no common RGB-D field')
        zoom = 1./extent
        k[:2] *= zoom
        k[0, 2] += 320.*(1.-zoom)
        k[1, 2] += 240.*(1.-zoom)
        values, valid = maps(k)
        if not valid.all():
            raise ValueError('common RGB-D field has invalid rays')
    info = deepcopy(depth)
    info.K, info.D = k.ravel().tolist(), [0.]*5
    info.R, info.P = [1., 0., 0., 0., 1., 0., 0., 0., 1.], [k[0,0],0.,k[0,2],0.,0.,k[1,1],k[1,2],0.,0.,0.,1.,0.]
    info.distortion_model = 'plumb_bob'
    return dict(zip(('color_x', 'color_y', 'depth_x', 'depth_y'), values), info=info)


def camera_info(geometry, header):
    info = deepcopy(geometry['info'])
    info.header = header
    return info
