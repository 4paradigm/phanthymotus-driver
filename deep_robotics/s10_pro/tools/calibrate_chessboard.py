"""Offline S10 camera calibration (requires numpy and OpenCV on the workstation).

python calibrate_chessboard.py FRAME_DIR --lane camera_rear --out OUTPUT_DIR
10x13 inner corners, 18 mm squares. Writes calibration.json in the shape of a
CALIBRATIONS row in undistortion.py; the camera cards take no calibration at
runtime, so a new measurement is installed by replacing that row.
"""
import argparse
from datetime import date
import json
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('frames', type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--lane', required=True,
                        choices=['camera_front', 'camera_rear', 'camera_left', 'camera_right'])
    parser.add_argument('--cols', type=int, default=10)
    parser.add_argument('--rows', type=int, default=13)
    parser.add_argument('--square', type=float, default=.018)
    args = parser.parse_args()
    objects = np.zeros((args.cols * args.rows, 3), np.float32)
    objects[:, :2] = np.mgrid[:args.cols, :args.rows].T.reshape(-1, 2) * args.square
    images, names, size = [], [], None
    for path in sorted(args.frames.glob('*.jpg')):
        frame = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_GRAYSCALE)
        if frame is None:
            continue
        current = frame.shape[::-1]
        if size is not None and current != size:
            raise ValueError(f'{path}: inconsistent frame size')
        size = current
        ok, corners = cv2.findChessboardCornersSB(
            frame, (args.cols, args.rows), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            images.append(corners)
            names.append(path.name)
    if len(images) < 15:
        raise ValueError(f'Only {len(images)} boards detected; need at least 15 varied poses')
    obj = [objects.copy() for _ in images]
    rms, k, d, rotations, translations = cv2.calibrateCamera(obj, images, size, None, None)
    errors = []
    coverage = np.zeros((6, 8), int)
    for points, r, t in zip(images, rotations, translations):
        projected, _ = cv2.projectPoints(objects, r, t, k, d)
        errors.append(float(np.sqrt(np.mean(np.sum((projected - points) ** 2, axis=2)))))
        for x, y in points.reshape(-1, 2):
            coverage[min(5, int(y * 6 / size[1])), min(7, int(x * 8 / size[0]))] += 1
    sx, sy = 640 / size[0], 360 / size[1]
    if abs(sx - sy) > 1e-6:
        raise ValueError('Input must have the same aspect ratio as the 640x360 driver output')
    k[0, 0] *= sx
    k[1, 1] *= sy
    k[0, 2] = (k[0, 2] + .5) * sx - .5
    k[1, 2] = (k[1, 2] + .5) * sy - .5
    cal = dict(id=f'deeprobotics/s10_pro/{args.lane}', width=640, height=360,
               distortion_model='plumb_bob', K=k.ravel().tolist(), D=d.ravel().tolist(),
               source_resolution=list(size), measured_on=date.today().isoformat(),
               frames_used=len(images), rms_px=float(rms))
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'calibration.json').write_text(json.dumps(cal, indent=2), encoding='utf-8')
    (args.out / 'report.json').write_text(json.dumps(
        dict(per_view=dict(zip(names, errors)), coverage=coverage.tolist()), indent=2), encoding='utf-8')
    print(f'{len(images)} boards; RMS {rms:.3f} px; coverage:\n{coverage}')
    print('Inspect straight lines after rectification before installing these parameters.')


if __name__ == '__main__':
    main()
