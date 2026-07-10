"""
scan_context.py — Scan Context 指纹生成与匹配。

基于论文 "Scan Context: Egocentric Spatial Descriptor for Place Recognition
within 3D Point Cloud Map" (Kim & Kim, 2018)。

纯 numpy 实现，用于自主地图发现（discover_map）：
- 建图时每个关键帧生成指纹并存储到 SQLite
- 查询时对当前点云生成指纹，与所有地图的关键帧指纹做匹配
"""

import sqlite3
import threading

import numpy as np


class ScanContextManager:
    """Scan Context 指纹管理：生成、存储、查询匹配。"""

    RING_NUM = 20          # 径向环数
    SECTOR_NUM = 60        # 角度扇区数
    MAX_RADIUS = 20.0      # 最大感知半径 (m)
    SC_DIST_THRES = 0.15   # 匹配阈值 (余弦距离, 越小越严格)
    TOP_K_RING = 10        # Ring Key 粗筛候选数

    def __init__(self, db_path: str):
        self._db_path = db_path
        self._lock = threading.Lock()
        self._init_db()
        # 内存缓存: list of {map_name, kf_id, ring_key, sc, pose}
        self._cache: list[dict] = []
        self._load_cache()

    def _init_db(self):
        conn = sqlite3.connect(self._db_path)
        conn.execute("""
            CREATE TABLE IF NOT EXISTS scan_context_kf (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                map_name TEXT NOT NULL,
                descriptor BLOB NOT NULL,
                pose_x REAL NOT NULL,
                pose_y REAL NOT NULL,
                pose_z REAL NOT NULL,
                created_at REAL NOT NULL
            )
        """)
        conn.execute("CREATE INDEX IF NOT EXISTS idx_sc_map ON scan_context_kf(map_name)")
        conn.commit()
        conn.close()

    def _load_cache(self):
        """启动时从 SQLite 加载所有指纹到内存。"""
        conn = sqlite3.connect(self._db_path)
        rows = conn.execute(
            "SELECT id, map_name, descriptor, pose_x, pose_y, pose_z FROM scan_context_kf"
        ).fetchall()
        conn.close()

        self._cache = []
        for row in rows:
            kf_id, map_name, desc_blob, px, py, pz = row
            sc = np.frombuffer(desc_blob, dtype=np.float32).reshape(self.RING_NUM, self.SECTOR_NUM)
            ring_key = sc.mean(axis=1)
            self._cache.append({
                "kf_id": kf_id,
                "map_name": map_name,
                "sc": sc,
                "ring_key": ring_key,
                "pose": (px, py, pz),
            })

    def make_scan_context(self, points_xyz: np.ndarray) -> np.ndarray:
        """从 Nx3 点云生成 Scan Context 描述子 (RING_NUM x SECTOR_NUM)。

        每个 bin 存储该区域内点的最大高度值。
        """
        if points_xyz.shape[0] == 0:
            return np.zeros((self.RING_NUM, self.SECTOR_NUM), dtype=np.float32)

        x = points_xyz[:, 0]
        y = points_xyz[:, 1]
        z = points_xyz[:, 2]

        # 极坐标
        r = np.sqrt(x ** 2 + y ** 2)
        theta = np.arctan2(y, x) + np.pi  # [0, 2pi]

        # 过滤超出范围的点
        valid = r < self.MAX_RADIUS
        r = r[valid]
        theta = theta[valid]
        z = z[valid]

        if len(r) == 0:
            return np.zeros((self.RING_NUM, self.SECTOR_NUM), dtype=np.float32)

        # 分桶
        ring_idx = np.clip(
            (r / self.MAX_RADIUS * self.RING_NUM).astype(np.int32),
            0, self.RING_NUM - 1
        )
        sector_idx = np.clip(
            (theta / (2 * np.pi) * self.SECTOR_NUM).astype(np.int32),
            0, self.SECTOR_NUM - 1
        )

        # 构建 Scan Context: 每 bin 取最大 z
        sc = np.full((self.RING_NUM, self.SECTOR_NUM), -np.inf, dtype=np.float32)
        # vectorized scatter-max via np.maximum.at
        linear_idx = ring_idx * self.SECTOR_NUM + sector_idx
        np.maximum.at(sc.ravel(), linear_idx, z.astype(np.float32))

        # 未填充的 bin 设为 0
        sc[sc == -np.inf] = 0.0

        return sc

    def make_ring_key(self, sc: np.ndarray) -> np.ndarray:
        """Ring Key = 每环均值，用于快速粗筛 (旋转不变)。"""
        return sc.mean(axis=1).astype(np.float32)

    def add_keyframe(self, map_name: str, sc: np.ndarray, pose: tuple) -> int:
        """存储一个关键帧指纹。返回 keyframe id。"""
        import time as _time

        desc_blob = sc.astype(np.float32).tobytes()
        with self._lock:
            conn = sqlite3.connect(self._db_path)
            cur = conn.execute(
                "INSERT INTO scan_context_kf (map_name, descriptor, pose_x, pose_y, pose_z, created_at) "
                "VALUES (?, ?, ?, ?, ?, ?)",
                (map_name, desc_blob, pose[0], pose[1], pose[2], _time.time()),
            )
            kf_id = cur.lastrowid
            conn.commit()
            conn.close()

            ring_key = self.make_ring_key(sc)
            self._cache.append({
                "kf_id": kf_id,
                "map_name": map_name,
                "sc": sc.copy(),
                "ring_key": ring_key,
                "pose": pose,
            })
        return kf_id

    def query(self, current_sc: np.ndarray, exclude_map: str | None = None) -> dict | None:
        """查询最匹配的地图。

        返回 {"map_name": str, "pose": {"x","y","z"}, "score": float} 或 None。

        Args:
            exclude_map: 排除此地图的 keyframes（用于排除当前正在建的图）

        流程:
        1. Ring Key 余弦距离粗筛 top-K
        2. 逐列旋转对齐 + 余弦距离精确匹配
        3. 最佳得分 < SC_DIST_THRES → 匹配成功
        """
        with self._lock:
            if exclude_map:
                cache = [c for c in self._cache if c["map_name"] != exclude_map]
            else:
                cache = list(self._cache)

        if not cache:
            return None

        current_ring_key = self.make_ring_key(current_sc)

        # Step 1: Ring Key 粗筛
        ring_keys = np.array([c["ring_key"] for c in cache], dtype=np.float32)
        ring_dists = self._cosine_distance_batch(current_ring_key, ring_keys)
        top_k_indices = np.argsort(ring_dists)[:self.TOP_K_RING]

        # Step 2: 精确匹配（旋转对齐）
        best_score = float("inf")
        best_entry = None

        for idx in top_k_indices:
            entry = cache[idx]
            dist = self._sc_distance_with_rotation(current_sc, entry["sc"])
            if dist < best_score:
                best_score = dist
                best_entry = entry

        # Step 3: 阈值判断
        if best_score < self.SC_DIST_THRES and best_entry is not None:
            return {
                "map_name": best_entry["map_name"],
                "pose": {"x": best_entry["pose"][0], "y": best_entry["pose"][1], "z": best_entry["pose"][2]},
                "score": round(float(best_score), 4),
            }

        return None

    def clear_map(self, map_name: str):
        """删除某地图的所有指纹。"""
        with self._lock:
            conn = sqlite3.connect(self._db_path)
            conn.execute("DELETE FROM scan_context_kf WHERE map_name = ?", (map_name,))
            conn.commit()
            conn.close()
            self._cache = [c for c in self._cache if c["map_name"] != map_name]

    def get_map_names(self) -> list[str]:
        """获取所有有指纹的地图名称。"""
        with self._lock:
            return list(set(c["map_name"] for c in self._cache))

    # ── 内部方法 ──────────────────────────────────────────────────────────────

    @staticmethod
    def _cosine_distance_batch(query: np.ndarray, keys: np.ndarray) -> np.ndarray:
        """计算 query 与 keys 中每行的余弦距离。"""
        q_norm = np.linalg.norm(query)
        if q_norm < 1e-9:
            return np.ones(len(keys), dtype=np.float32)
        k_norms = np.linalg.norm(keys, axis=1)
        k_norms[k_norms < 1e-9] = 1e-9
        cos_sim = (keys @ query) / (k_norms * q_norm)
        return 1.0 - cos_sim

    def _sc_distance_with_rotation(self, sc_a: np.ndarray, sc_b: np.ndarray) -> float:
        """计算两个 Scan Context 的距离（旋转对齐后的最小余弦距离）。

        旋转不变性：对 sc_b 逐列循环移位，找最小距离。
        """
        min_dist = float("inf")
        # 优化：只搜索 SECTOR_NUM 个旋转
        for shift in range(self.SECTOR_NUM):
            sc_b_shifted = np.roll(sc_b, shift, axis=1)
            dist = self._column_cosine_distance(sc_a, sc_b_shifted)
            if dist < min_dist:
                min_dist = dist
        return min_dist

    @staticmethod
    def _column_cosine_distance(sc_a: np.ndarray, sc_b: np.ndarray) -> float:
        """逐列余弦距离的均值。"""
        num_sectors = sc_a.shape[1]
        total_dist = 0.0
        valid_cols = 0
        for j in range(num_sectors):
            col_a = sc_a[:, j]
            col_b = sc_b[:, j]
            norm_a = np.linalg.norm(col_a)
            norm_b = np.linalg.norm(col_b)
            if norm_a < 1e-9 or norm_b < 1e-9:
                continue
            cos_sim = np.dot(col_a, col_b) / (norm_a * norm_b)
            total_dist += 1.0 - cos_sim
            valid_cols += 1
        return total_dist / max(valid_cols, 1)
