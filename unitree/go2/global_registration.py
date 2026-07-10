"""
global_registration.py — 2D FFT 互相关全局配准。

用暴力旋转搜索 + FFT 互相关找最优旋转角和平移。
完全不需要初始猜测，适用于任意角度旋转。

算法：
1. 点云投影为 2D 占据栅格（二值图）
2. 对每个旋转角 θ (0-359°, 步长 1°):
   - 旋转 source 栅格
   - FFT 互相关找峰值
3. 选峰值最高的 θ → 取负得到 bias_yaw
4. 从该角度的互相关矩阵提取平移
"""

import math
import numpy as np


def register_2d(source_pts: np.ndarray, target_pts: np.ndarray,
                resolution: float = 0.10, z_min: float = 0.25, z_max: float = 0.5,
                angle_step: int = 1) -> dict | None:
    """全局 2D 配准：FFT 找角度 + FFT 找平移。

    Args:
        source_pts: Nx3 新图点云
        target_pts: Mx3 旧图点云
        resolution: 栅格分辨率 (m/cell)
        z_min, z_max: z 过滤范围
        angle_step: 角度搜索步长 (度)

    Returns:
        {"x": bias_x, "y": bias_y, "yaw": bias_yaw_rad, "yaw_deg": deg,
         "correlation": peak_value, "fft_deg": raw_fft_angle}
    """
    # 1. 提取 2D 点 (z 过滤)
    src_2d = _extract_2d(source_pts, z_min, z_max)
    tgt_2d = _extract_2d(target_pts, z_min, z_max)

    if len(src_2d) < 50 or len(tgt_2d) < 50:
        return None

    # 2. 设置栅格参数
    grid_size = 512
    half_extent = grid_size * resolution / 2

    # 3. FFT 搜索最优旋转角（用较粗的分辨率加速）
    coarse_res = 0.15
    coarse_gs = 256
    coarse_he = coarse_gs * coarse_res / 2

    grid_tgt_coarse = _points_to_grid(tgt_2d, coarse_he, coarse_res, coarse_gs)
    fft_tgt_coarse = np.fft.fft2(grid_tgt_coarse)

    best_corr = -1
    best_deg = 0

    for deg in range(0, 360, angle_step):
        rad = math.radians(deg)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        rot_x = cos_a * src_2d[:, 0] - sin_a * src_2d[:, 1]
        rot_y = sin_a * src_2d[:, 0] + cos_a * src_2d[:, 1]
        grid_src = _points_to_grid(np.column_stack([rot_x, rot_y]), coarse_he, coarse_res, coarse_gs)
        cc = np.fft.ifft2(fft_tgt_coarse * np.conj(np.fft.fft2(grid_src))).real
        peak = cc.max()
        if peak > best_corr:
            best_corr = peak
            best_deg = deg

    # 4. 用精细栅格在 best_deg 角度提取平移
    rad = math.radians(best_deg)
    cos_a = math.cos(rad)
    sin_a = math.sin(rad)
    rot_x = cos_a * src_2d[:, 0] - sin_a * src_2d[:, 1]
    rot_y = sin_a * src_2d[:, 0] + cos_a * src_2d[:, 1]

    grid_tgt = _points_to_grid(tgt_2d, half_extent, resolution, grid_size)
    grid_src_rot = _points_to_grid(np.column_stack([rot_x, rot_y]), half_extent, resolution, grid_size)

    cc = np.fft.ifft2(np.fft.fft2(grid_tgt) * np.conj(np.fft.fft2(grid_src_rot))).real
    peak_idx = np.unravel_index(cc.argmax(), cc.shape)
    sy, sx = peak_idx
    if sy > grid_size // 2:
        sy -= grid_size
    if sx > grid_size // 2:
        sx -= grid_size
    dx = sx * resolution
    dy = sy * resolution

    # 5. bias_yaw = -best_deg (取负)
    bias_yaw = math.radians(-best_deg)

    return {
        "x": dx,
        "y": dy,
        "yaw": bias_yaw,
        "yaw_deg": -best_deg,
        "correlation": float(best_corr),
        "fft_deg": best_deg,
    }


def _extract_2d(pts: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    """提取 z 范围内的点，返回 Nx2 (x, y)。"""
    z = pts[:, 2]
    mask = (z >= z_min) & (z <= z_max)
    return pts[mask, :2]


def _points_to_grid(pts_2d: np.ndarray, half_extent: float,
                    resolution: float, grid_size: int) -> np.ndarray:
    """将 2D 点投影为二值栅格（以原点为中心）。"""
    grid = np.zeros((grid_size, grid_size), dtype=np.float32)
    gx = ((pts_2d[:, 0] + half_extent) / resolution).astype(np.int32)
    gy = ((pts_2d[:, 1] + half_extent) / resolution).astype(np.int32)
    valid = (gx >= 0) & (gx < grid_size) & (gy >= 0) & (gy < grid_size)
    grid[gy[valid], gx[valid]] = 1.0
    return grid
