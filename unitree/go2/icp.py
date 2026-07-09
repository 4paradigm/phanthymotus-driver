"""
icp.py — Iterative Closest Point 点云对齐算法。

用于重定位：将当前实时点云与已保存的 PCD 地图对齐，
在给定初始猜测（如 tag 坐标附近）的情况下求出精确位姿。

纯 numpy 实现，无外部依赖。
"""

import math

import numpy as np


def icp_2d(source: np.ndarray, target: np.ndarray,
           init_x: float = 0, init_y: float = 0, init_yaw: float = 0,
           max_iterations: int = 30, tolerance: float = 0.001,
           max_correspond_dist: float = 2.0) -> dict | None:
    """2D ICP: 在 XY 平面上对齐 source → target。

    Args:
        source: Nx2 or Nx3 当前点云 (只用 x,y)
        target: Mx2 or Mx3 目标地图点云 (只用 x,y)
        init_x, init_y, init_yaw: 初始猜测位姿
        max_iterations: 最大迭代次数
        tolerance: 收敛阈值 (变换增量)
        max_correspond_dist: 最大对应距离 (过滤异常值)

    Returns:
        {"x": float, "y": float, "yaw": float, "score": float, "iterations": int}
        or None if failed.
    """
    if source.shape[0] < 10 or target.shape[0] < 10:
        return None

    # 只用 x, y
    src = source[:, :2].astype(np.float64).copy()
    tgt = target[:, :2].astype(np.float64).copy()

    # 降采样 (加速)
    if len(src) > 2000:
        idx = np.random.choice(len(src), 2000, replace=False)
        src = src[idx]
    if len(tgt) > 5000:
        idx = np.random.choice(len(tgt), 5000, replace=False)
        tgt = tgt[idx]

    # 应用初始变换
    cos_y = math.cos(init_yaw)
    sin_y = math.sin(init_yaw)
    R_init = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
    t_init = np.array([init_x, init_y])

    # 累积变换
    R_acc = R_init.copy()
    t_acc = t_init.copy()

    # 变换 source
    src_transformed = (R_acc @ src.T).T + t_acc

    prev_error = float('inf')

    for iteration in range(max_iterations):
        # 1. 找最近对应点 (暴力搜索，分块加速)
        correspondences = _find_correspondences(src_transformed, tgt, max_correspond_dist)
        if len(correspondences) < 10:
            print(f"[ICP] iter {iteration}: too few correspondences ({len(correspondences)})")
            return None

        src_matched = src_transformed[correspondences[:, 0]]
        tgt_matched = tgt[correspondences[:, 1]]

        # 2. 计算最优 2D 刚体变换 (SVD)
        R_step, t_step = _compute_rigid_transform_2d(src_matched, tgt_matched)

        # 3. 更新累积变换
        R_acc = R_step @ R_acc
        t_acc = R_step @ t_acc + t_step

        # 4. 应用变换
        src_transformed = (R_acc @ src.T).T + t_acc

        # 5. 计算误差
        diffs = src_transformed[correspondences[:, 0]] - tgt[correspondences[:, 1]]
        mean_error = np.mean(np.linalg.norm(diffs, axis=1))

        # 检查收敛
        delta = abs(prev_error - mean_error)
        if delta < tolerance:
            break
        prev_error = mean_error

    # 提取最终位姿
    final_yaw = math.atan2(R_acc[1, 0], R_acc[0, 0])
    final_x = float(t_acc[0])
    final_y = float(t_acc[1])

    return {
        "x": final_x,
        "y": final_y,
        "yaw": final_yaw,
        "score": float(mean_error),
        "iterations": iteration + 1,
        "correspondences": len(correspondences),
    }


def _find_correspondences(src: np.ndarray, tgt: np.ndarray,
                          max_dist: float) -> np.ndarray:
    """为 src 中每个点找 tgt 中最近点。返回 Kx2 索引对 [[src_i, tgt_j], ...]。"""
    # 分块计算避免内存爆炸
    BLOCK = 500
    pairs = []
    max_dist_sq = max_dist * max_dist

    for start in range(0, len(src), BLOCK):
        end = min(start + BLOCK, len(src))
        block = src[start:end]  # Bx2

        # 计算距离矩阵 BxM
        # ||a - b||^2 = ||a||^2 + ||b||^2 - 2*a.b
        a_sq = np.sum(block ** 2, axis=1, keepdims=True)  # Bx1
        b_sq = np.sum(tgt ** 2, axis=1, keepdims=True).T  # 1xM
        dist_sq = a_sq + b_sq - 2 * (block @ tgt.T)       # BxM
        dist_sq = np.maximum(dist_sq, 0)  # 避免浮点负数

        # 每行最小值
        min_idx = np.argmin(dist_sq, axis=1)  # B
        min_dist_sq = dist_sq[np.arange(len(block)), min_idx]  # B

        # 过滤
        valid = min_dist_sq < max_dist_sq
        src_indices = np.arange(start, end)[valid]
        tgt_indices = min_idx[valid]

        if len(src_indices) > 0:
            pairs.append(np.column_stack([src_indices, tgt_indices]))

    if not pairs:
        return np.empty((0, 2), dtype=np.int64)
    return np.vstack(pairs)


def _compute_rigid_transform_2d(src: np.ndarray, tgt: np.ndarray):
    """计算最优 2D 刚体变换 (旋转+平移) 使 src → tgt。SVD 方法。"""
    centroid_src = src.mean(axis=0)
    centroid_tgt = tgt.mean(axis=0)

    src_centered = src - centroid_src
    tgt_centered = tgt - centroid_tgt

    H = src_centered.T @ tgt_centered  # 2x2
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # 确保是旋转矩阵（det=1）
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_tgt - R @ centroid_src
    return R, t
