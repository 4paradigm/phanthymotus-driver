"""
path_planner.py — PCD 地图路径规划器。

功能：
1. 加载 PCD 点云 → 投影为 2D 占据栅格
2. 障碍物膨胀（机器人安全半径）
3. A* 全局路径规划
4. Douglas-Peucker 路径简化
5. 输出世界坐标 waypoints

无 ROS/SLAM 依赖，纯 Python + numpy + scipy。
"""

import heapq
import math
import os
import struct

import numpy as np

try:
    from scipy.ndimage import binary_dilation
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


def _numpy_dilation(grid: np.ndarray, radius: int) -> np.ndarray:
    """Pure numpy fallback for binary dilation (when scipy unavailable)."""
    if radius <= 0:
        return grid
    kernel_size = 2 * radius + 1
    padded = np.pad(grid, radius, mode='constant', constant_values=0)
    result = np.zeros_like(grid)
    for dy in range(kernel_size):
        for dx in range(kernel_size):
            result |= padded[dy:dy + grid.shape[0], dx:dx + grid.shape[1]]
    return result


class PathPlanner:
    """PCD → 2D occupancy grid → A* path planning."""

    def __init__(self, resolution: float = 0.15, robot_radius: float = 0.10,
                 z_min: float = 0.25, z_max: float = 0.5, min_hits: int = 5):
        """
        Args:
            resolution: 栅格分辨率 (m/cell), 默认 15cm
            robot_radius: 机器人安全半径 (m), 用于障碍物膨胀
            z_min: 有效点最低高度 (过滤地面), Go2 腿高约15cm
            z_max: 有效点最高高度 (过滤高处), Go2 身高约40cm,取0.8m安全
            min_hits: 一个栅格中至少有多少个点才算障碍 (过滤噪声)
        """
        self._resolution = resolution
        self._robot_radius = robot_radius
        self._z_min = z_min
        self._z_max = z_max
        self._min_hits = min_hits

        self._grid: np.ndarray | None = None  # 2D array: 0=free, 1=occupied
        self._origin_x: float = 0.0  # 栅格 (0,0) 对应的世界 x 坐标
        self._origin_y: float = 0.0  # 栅格 (0,0) 对应的世界 y 坐标
        self._width: int = 0
        self._height: int = 0

    @property
    def is_loaded(self) -> bool:
        return self._grid is not None

    def load_pcd(self, pcd_path: str) -> bool:
        """加载 PCD 文件，生成 2D 占据栅格。

        Returns:
            True if successful, False otherwise.
        """
        points = self._parse_pcd(pcd_path)
        if points is None or len(points) < 10:
            print(f"[PathPlanner] Failed to load PCD or too few points: {pcd_path}")
            return False

        # 过滤 z 范围 (保留障碍物高度的点)
        z = points[:, 2]
        mask = (z >= self._z_min) & (z <= self._z_max)
        obstacle_points = points[mask]

        if len(obstacle_points) < 5:
            print(f"[PathPlanner] No obstacle points in z=[{self._z_min}, {self._z_max}]")
            # 没有障碍物 → 全 free grid
            all_x, all_y = points[:, 0], points[:, 1]
        else:
            all_x, all_y = points[:, 0], points[:, 1]

        # 确定栅格边界 (使用所有点的范围)
        margin = 1.0  # 额外边距
        x_min = float(all_x.min()) - margin
        x_max = float(all_x.max()) + margin
        y_min = float(all_y.min()) - margin
        y_max = float(all_y.max()) + margin

        self._origin_x = x_min
        self._origin_y = y_min
        self._width = int(math.ceil((x_max - x_min) / self._resolution))
        self._height = int(math.ceil((y_max - y_min) / self._resolution))

        # 限制最大栅格尺寸 (防止内存爆炸)
        max_cells = 2000
        if self._width > max_cells or self._height > max_cells:
            scale = max(self._width, self._height) / max_cells
            self._resolution *= scale
            self._width = int(math.ceil((x_max - x_min) / self._resolution))
            self._height = int(math.ceil((y_max - y_min) / self._resolution))
            print(f"[PathPlanner] Grid too large, rescaled resolution to {self._resolution:.3f}m")

        # 创建空栅格
        grid = np.zeros((self._height, self._width), dtype=np.uint8)

        # 标记障碍物 (需要 min_hits 个点才算障碍)
        if len(obstacle_points) >= 5:
            ox = obstacle_points[:, 0]
            oy = obstacle_points[:, 1]
            gx = ((ox - self._origin_x) / self._resolution).astype(np.int32)
            gy = ((oy - self._origin_y) / self._resolution).astype(np.int32)

            # 裁剪到栅格范围
            valid = (gx >= 0) & (gx < self._width) & (gy >= 0) & (gy < self._height)
            gx, gy = gx[valid], gy[valid]

            # 统计每个栅格的命中数
            hit_count = np.zeros((self._height, self._width), dtype=np.int32)
            np.add.at(hit_count, (gy, gx), 1)
            grid[hit_count >= self._min_hits] = 1

        # 膨胀障碍物 (机器人安全半径)
        inflate_cells = int(math.ceil(self._robot_radius / self._resolution))
        if inflate_cells > 0:
            if _HAS_SCIPY:
                struct_elem = np.ones((2 * inflate_cells + 1, 2 * inflate_cells + 1), dtype=np.uint8)
                grid = binary_dilation(grid, structure=struct_elem).astype(np.uint8)
            else:
                grid = _numpy_dilation(grid, inflate_cells)

        self._grid = grid
        print(f"[PathPlanner] Loaded PCD: {len(points)} pts, grid {self._width}x{self._height}, "
              f"resolution={self._resolution:.3f}m, obstacles={np.sum(grid > 0)} cells")
        return True

    def load_from_buffer(self, points: np.ndarray) -> bool:
        """从内存中的点云数组加载（Nx3 float32）。"""
        if points is None or len(points) < 10:
            return False
        z = points[:, 2]
        mask = (z >= self._z_min) & (z <= self._z_max)
        obstacle_points = points[mask]

        all_x, all_y = points[:, 0], points[:, 1]
        margin = 1.0
        x_min = float(all_x.min()) - margin
        x_max = float(all_x.max()) + margin
        y_min = float(all_y.min()) - margin
        y_max = float(all_y.max()) + margin

        self._origin_x = x_min
        self._origin_y = y_min
        self._width = int(math.ceil((x_max - x_min) / self._resolution))
        self._height = int(math.ceil((y_max - y_min) / self._resolution))

        max_cells = 2000
        if self._width > max_cells or self._height > max_cells:
            scale = max(self._width, self._height) / max_cells
            self._resolution *= scale
            self._width = int(math.ceil((x_max - x_min) / self._resolution))
            self._height = int(math.ceil((y_max - y_min) / self._resolution))

        grid = np.zeros((self._height, self._width), dtype=np.uint8)

        if len(obstacle_points) >= 5:
            ox = obstacle_points[:, 0]
            oy = obstacle_points[:, 1]
            gx = ((ox - self._origin_x) / self._resolution).astype(np.int32)
            gy = ((oy - self._origin_y) / self._resolution).astype(np.int32)
            valid = (gx >= 0) & (gx < self._width) & (gy >= 0) & (gy < self._height)
            gx, gy = gx[valid], gy[valid]
            hit_count = np.zeros((self._height, self._width), dtype=np.int32)
            np.add.at(hit_count, (gy, gx), 1)
            grid[hit_count >= self._min_hits] = 1

        inflate_cells = int(math.ceil(self._robot_radius / self._resolution))
        if inflate_cells > 0:
            if _HAS_SCIPY:
                struct_elem = np.ones((2 * inflate_cells + 1, 2 * inflate_cells + 1), dtype=np.uint8)
                grid = binary_dilation(grid, structure=struct_elem).astype(np.uint8)
            else:
                grid = _numpy_dilation(grid, inflate_cells)

        self._grid = grid
        return True

    def plan(self, start_xy: tuple, goal_xy: tuple) -> list[tuple] | None:
        """A* 路径规划。

        Args:
            start_xy: (x, y) 世界坐标起点
            goal_xy: (x, y) 世界坐标终点

        Returns:
            [(x, y), ...] 世界坐标 waypoint 列表, 或 None 如果无路径。
        """
        if self._grid is None:
            return None

        # 世界坐标 → 栅格坐标
        sx, sy = self._world_to_grid(start_xy[0], start_xy[1])
        gx, gy = self._world_to_grid(goal_xy[0], goal_xy[1])

        # 边界检查
        if not self._in_bounds(sx, sy) or not self._in_bounds(gx, gy):
            return None

        # 如果起点或终点在障碍物中，尝试寻找最近的 free cell
        if self._grid[sy, sx] != 0:
            free = self._find_nearest_free(sx, sy)
            if free is None:
                return None
            sx, sy = free

        if self._grid[gy, gx] != 0:
            free = self._find_nearest_free(gx, gy)
            if free is None:
                return None
            gx, gy = free

        # A* search
        path_grid = self._astar(sx, sy, gx, gy)
        if path_grid is None:
            return None

        # 栅格坐标 → 世界坐标
        path_world = [(self._grid_to_world(cx, cy)) for cx, cy in path_grid]

        # Douglas-Peucker 简化
        path_simplified = self._douglas_peucker(path_world, epsilon=0.2)

        # 确保包含起点和终点
        if len(path_simplified) < 2:
            path_simplified = [start_xy, goal_xy]

        return path_simplified

    def is_reachable(self, x: float, y: float) -> bool:
        """检查世界坐标点是否在 free space 中。"""
        if self._grid is None:
            return False
        gx, gy = self._world_to_grid(x, y)
        if not self._in_bounds(gx, gy):
            return False
        return self._grid[gy, gx] == 0

    def get_grid_as_points(self) -> np.ndarray | None:
        """返回障碍物栅格的世界坐标点云 (Nx3 float32, z=0.5)，用于可视化。"""
        if self._grid is None:
            return None
        gy_arr, gx_arr = np.where(self._grid > 0)
        if len(gx_arr) == 0:
            return None
        x = gx_arr * self._resolution + self._origin_x + self._resolution / 2
        y = gy_arr * self._resolution + self._origin_y + self._resolution / 2
        z = np.full_like(x, 0.5)
        return np.column_stack([x, y, z]).astype(np.float32)

    # ── 内部方法 ──────────────────────────────────────────────────────────────

    def _world_to_grid(self, x: float, y: float) -> tuple:
        gx = int((x - self._origin_x) / self._resolution)
        gy = int((y - self._origin_y) / self._resolution)
        return gx, gy

    def _grid_to_world(self, gx: int, gy: int) -> tuple:
        x = gx * self._resolution + self._origin_x + self._resolution / 2
        y = gy * self._resolution + self._origin_y + self._resolution / 2
        return (x, y)

    def _in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self._width and 0 <= gy < self._height

    def _find_nearest_free(self, gx: int, gy: int, max_radius: int = 50) -> tuple | None:
        """BFS 找最近的 free cell。"""
        from collections import deque
        visited = set()
        q = deque([(gx, gy)])
        visited.add((gx, gy))
        while q:
            cx, cy = q.popleft()
            if self._in_bounds(cx, cy) and self._grid[cy, cx] == 0:
                return (cx, cy)
            if abs(cx - gx) > max_radius or abs(cy - gy) > max_radius:
                continue
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) not in visited and self._in_bounds(nx, ny):
                    visited.add((nx, ny))
                    q.append((nx, ny))
        return None

    def _astar(self, sx: int, sy: int, gx: int, gy: int) -> list[tuple] | None:
        """A* on 2D grid with 8-connectivity."""
        # 8-directional movement
        DIRS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]

        def heuristic(x, y):
            return math.sqrt((x - gx) ** 2 + (y - gy) ** 2)

        open_set = [(heuristic(sx, sy), 0.0, sx, sy)]
        came_from = {}
        g_score = {(sx, sy): 0.0}
        closed = set()

        max_iterations = self._width * self._height  # prevent infinite loop

        iterations = 0
        while open_set and iterations < max_iterations:
            iterations += 1
            f, g, cx, cy = heapq.heappop(open_set)

            if (cx, cy) in closed:
                continue
            closed.add((cx, cy))

            if cx == gx and cy == gy:
                # Reconstruct path
                path = [(gx, gy)]
                while (path[-1][0], path[-1][1]) != (sx, sy):
                    path.append(came_from[(path[-1][0], path[-1][1])])
                path.reverse()
                return path

            for dx, dy, cost in DIRS:
                nx, ny = cx + dx, cy + dy
                if not self._in_bounds(nx, ny):
                    continue
                if (nx, ny) in closed:
                    continue
                if self._grid[ny, nx] != 0:
                    continue

                new_g = g + cost
                if new_g < g_score.get((nx, ny), float('inf')):
                    g_score[(nx, ny)] = new_g
                    came_from[(nx, ny)] = (cx, cy)
                    heapq.heappush(open_set, (new_g + heuristic(nx, ny), new_g, nx, ny))

        return None  # no path found

    def _douglas_peucker(self, points: list[tuple], epsilon: float) -> list[tuple]:
        """Douglas-Peucker 路径简化算法。"""
        if len(points) <= 2:
            return points

        # 找离首尾连线最远的点
        start = np.array(points[0])
        end = np.array(points[-1])
        line_vec = end - start
        line_len = np.linalg.norm(line_vec)

        if line_len < 1e-9:
            return [points[0], points[-1]]

        line_unit = line_vec / line_len

        max_dist = 0.0
        max_idx = 0
        for i in range(1, len(points) - 1):
            pt = np.array(points[i])
            proj = np.dot(pt - start, line_unit)
            proj = max(0, min(line_len, proj))
            closest = start + proj * line_unit
            dist = np.linalg.norm(pt - closest)
            if dist > max_dist:
                max_dist = dist
                max_idx = i

        if max_dist > epsilon:
            left = self._douglas_peucker(points[:max_idx + 1], epsilon)
            right = self._douglas_peucker(points[max_idx:], epsilon)
            return left[:-1] + right
        else:
            return [points[0], points[-1]]

    @staticmethod
    def _parse_pcd(path: str) -> np.ndarray | None:
        """Parse ASCII/binary PCD file, extract x,y,z. Returns Nx3 float32 array."""
        try:
            with open(path, 'rb') as f:
                header_lines = []
                while True:
                    line = f.readline()
                    if not line:
                        return None
                    line_str = line.decode('ascii', errors='ignore').strip()
                    header_lines.append(line_str)
                    if line_str.startswith('DATA'):
                        break

                fields = []
                num_points = 0
                data_type = "ascii"
                field_sizes = []
                for hl in header_lines:
                    parts = hl.split()
                    if not parts:
                        continue
                    if parts[0] == "FIELDS":
                        fields = parts[1:]
                    elif parts[0] == "SIZE":
                        field_sizes = [int(s) for s in parts[1:]]
                    elif parts[0] == "POINTS":
                        num_points = int(parts[1])
                    elif parts[0] == "DATA":
                        data_type = parts[1].lower()

                if num_points == 0:
                    return None

                try:
                    xi = fields.index("x")
                    yi = fields.index("y")
                    zi = fields.index("z")
                except ValueError:
                    return None

                if data_type == "ascii":
                    points = []
                    for _ in range(num_points):
                        line = f.readline().decode('ascii', errors='ignore').strip()
                        if not line:
                            break
                        vals = line.split()
                        if len(vals) <= max(xi, yi, zi):
                            continue
                        px = float(vals[xi])
                        py = float(vals[yi])
                        pz = float(vals[zi])
                        if px != px or py != py or pz != pz:
                            continue
                        points.append((px, py, pz))
                    return np.array(points, dtype=np.float32) if points else None

                elif data_type == "binary":
                    point_size = sum(field_sizes)
                    raw = f.read(num_points * point_size)
                    if len(raw) < num_points * point_size:
                        num_points = len(raw) // point_size

                    offsets = [0]
                    for s in field_sizes[:-1]:
                        offsets.append(offsets[-1] + s)

                    x_off = offsets[xi]
                    y_off = offsets[yi]
                    z_off = offsets[zi]

                    pts = np.zeros((num_points, 3), dtype=np.float32)
                    for i in range(num_points):
                        base = i * point_size
                        pts[i, 0] = struct.unpack_from('<f', raw, base + x_off)[0]
                        pts[i, 1] = struct.unpack_from('<f', raw, base + y_off)[0]
                        pts[i, 2] = struct.unpack_from('<f', raw, base + z_off)[0]

                    valid = ~np.isnan(pts).any(axis=1)
                    return pts[valid]

        except Exception as e:
            print(f"[PathPlanner] PCD parse error: {e}")
            return None
