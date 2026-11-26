"""Geometry for the Tianyi head depth stream."""
from dataclasses import dataclass, replace
import math


@dataclass(frozen=True)
class Intrinsics:
    frame_id: str
    width: int
    height: int
    k: tuple
    d: tuple

    @classmethod
    def from_camera_info(cls, msg):
        k, d = tuple(msg.k), tuple(msg.d)
        roi = msg.roi
        if (not msg.header.frame_id or msg.width <= 0 or msg.height <= 0
                or msg.binning_x > 1 or msg.binning_y > 1
                or any((roi.x_offset, roi.y_offset, roi.width, roi.height))
                or msg.distortion_model not in ("plumb_bob", "rational_polynomial")
                or len(k) != 9 or len(d) not in (0, 4, 5, 8, 12, 14)
                or not all(math.isfinite(value) for value in (*k, *d))
                or k[0] <= 0. or k[4] <= 0.
                or not (0. <= k[2] < msg.width and 0. <= k[5] < msg.height)
                or (k[1], k[3], *k[6:]) != (0., 0., 0., 0., 1.)):
            return None
        return cls(msg.header.frame_id, msg.width, msg.height, k, d)

    def cropped(self, crop, width, height):
        left, top, crop_width, crop_height = crop
        sx, sy = width / crop_width, height / crop_height
        fx, cx, fy, cy = self.k[0], self.k[2], self.k[4], self.k[5]
        k = (fx * sx, 0., (cx - left + .5) * sx - .5,
             0., fy * sy, (cy - top + .5) * sy - .5, 0., 0., 1.)
        return replace(self, width=width, height=height, k=k)


def color_to_depth_map(depth, color, cv2, np):
    """Map each RGB pixel to the nearest measured depth ray in the same optical frame."""
    rows, cols = np.indices((color.height, color.width), dtype=np.float32)
    pixels = np.stack((cols, rows), axis=-1).reshape(-1, 1, 2)
    rays = cv2.undistortPoints(pixels, np.asarray(color.k).reshape(3, 3), np.asarray(color.d))
    if not any(depth.d):
        depth_pixels = rays * np.array([depth.k[0], depth.k[4]], dtype=np.float32)
        depth_pixels += np.array([depth.k[2], depth.k[5]], dtype=np.float32)
    else:
        xyz = np.column_stack((rays.reshape(-1, 2), np.ones(len(rays), dtype=np.float32)))
        depth_pixels, _ = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3),
                                            np.asarray(depth.k).reshape(3, 3), np.asarray(depth.d))
    return depth_pixels.reshape(color.height, color.width, 2)


def center_crop(width, height):
    crop_width = height * 4 // 3 if width * 3 > height * 4 else width
    crop_height = width * 3 // 4 if width * 3 < height * 4 else height
    return (width - crop_width) // 2, (height - crop_height) // 2, crop_width, crop_height
