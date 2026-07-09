"""
global_registration.py — 2D FFT 互相关全局配准。

用暴力旋转搜索 + FFT 互相关找最优平移，完全不需要初始猜测。
适用于大角度旋转 + 对称环境。

算法：
1. 点云投影为 2D 占据栅格（二值图）
2. 对每个旋转角 θ (0-359°, 步长 1°):
   - 旋转 source 栅格
   - FFT 互相关找最优平移
   - 记录相关峰值
3. 选峰值最高的 θ → 最优旋转 + 平移
"""

import math
import numpy as np


def register_2d(source_pts: np.ndarray, target_pts: np.ndarray,
                resolution: float = 0.15, z_min: float = 0.1, z_max: float = 1.5,
                angle_step: int = 1) -> dict | None:
    """全局 2D 配准：暴力旋转 + FFT 互相关。

    Args:
        source_pts: Nx3 源点云 (将被旋转+平移去匹配 target)
        target_pts: Mx3 目标点云 (固定)
        resolution: 栅格分辨率 (m/cell)
        z_min, z_max: z 过滤范围
        angle_step: 角度搜索步长 (度)

    Returns:
        {"x": dx, "y": dy, "yaw": angle_rad, "yaw_deg": angle_deg, "correlation": peak}
        变换含义：target = R(yaw) * source + (x, y)
    """
    # 1. 提取 2D 点 (z 过滤)
    src_2d = _extract_2d(source_pts, z_min, z_max)
    tgt_2d = _extract_2d(target_pts, z_min, z_max)

    if len(src_2d) < 50 or len(tgt_2d) < 50:
        return None

    # 2. 创建统一的大栅格（需要容纳旋转后的点云）
    # 旋转后最大范围 = 点到原点的最大距离
    src_max_r = np.sqrt((src_2d ** 2).sum(axis=1)).max()
    tgt_max_r = np.sqrt((tgt_2d ** 2).sum(axis=1)).max()
    max_extent = max(src_max_r, tgt_max_r) + 1.0  # 1m margin

    x_min = -max_extent
    x_max = max_extent
    y_min = -max_extent
    y_max = max_extent

    width = int(math.ceil((x_max - x_min) / resolution))
    height = int(math.ceil((y_max - y_min) / resolution))

    # 限制栅格大小
    max_size = 512
    if width > max_size or height > max_size:
        scale = max(width, height) / max_size
        resolution *= scale
        width = int(math.ceil((x_max - x_min) / resolution))
        height = int(math.ceil((y_max - y_min) / resolution))

    # 3. 创建 target 栅格（固定）
    grid_tgt = _points_to_grid(tgt_2d, x_min, y_min, resolution, width, height)

    # 预计算 target FFT
    fft_tgt = np.fft.fft2(grid_tgt.astype(np.float32))

    # 4. 暴力旋转搜索
    best_corr = -1
    best_angle = 0
    best_dx = 0.0
    best_dy = 0.0

    for deg in range(0, 360, angle_step):
        rad = math.radians(deg)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)

        # 旋转 source 点（绕原点旋转，因为坐标系变换是绕原点的）
        rot_x = cos_a * src_2d[:, 0] - sin_a * src_2d[:, 1]
        rot_y = sin_a * src_2d[:, 0] + cos_a * src_2d[:, 1]

        # 投影到栅格
        src_rotated = np.column_stack([rot_x, rot_y])
        grid_src = _points_to_grid(src_rotated, x_min, y_min, resolution, width, height)

        # FFT 互相关
        fft_src = np.fft.fft2(grid_src.astype(np.float32))
        cross_corr = np.fft.ifft2(fft_tgt * np.conj(fft_src)).real

        # 找峰值
        peak_val = cross_corr.max()
        if peak_val > best_corr:
            best_corr = peak_val
            best_angle = deg
            peak_idx = np.unravel_index(cross_corr.argmax(), cross_corr.shape)
            # 将峰值位置转换为实际平移
            shift_y = peak_idx[0]
            shift_x = peak_idx[1]
            # 处理环绕
            if shift_y > height // 2:
                shift_y -= height
            if shift_x > width // 2:
                shift_x -= width
            best_dx = shift_x * resolution
            best_dy = shift_y * resolution

    if best_corr <= 0:
        return None

    # 5. 最终结果：target = R(yaw) * source + (dx, dy)
    # 但我们需要的 bias 是：新图坐标 → 旧图坐标
    # source=新图, target=旧图
    # 旧图点 ≈ R(yaw) * 新图点 + (dx, dy)
    yaw_rad = math.radians(best_angle)

    return {
        "x": best_dx,
        "y": best_dy,
        "yaw": yaw_rad,
        "yaw_deg": best_angle,
        "correlation": float(best_corr),
        "grid_size": (width, height),
        "resolution": resolution,
    }


def _extract_2d(pts: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    """提取 z 范围内的点，返回 Nx2 (x, y)。"""
    z = pts[:, 2]
    mask = (z >= z_min) & (z <= z_max)
    return pts[mask, :2]


def _points_to_grid(pts_2d: np.ndarray, x_min: float, y_min: float,
                    resolution: float, width: int, height: int) -> np.ndarray:
    """将 2D 点投影为二值栅格。"""
    grid = np.zeros((height, width), dtype=np.uint8)
    gx = ((pts_2d[:, 0] - x_min) / resolution).astype(np.int32)
    gy = ((pts_2d[:, 1] - y_min) / resolution).astype(np.int32)
    valid = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
    grid[gy[valid], gx[valid]] = 1
    return grid
