"""栅格 A* 路径规划。

加它的直接原因是真地图:把北京 2F 展厅的 `bj-2f` 解出来之后,P3→P15 这 12 段里
**有 5 段直线穿墙**。展厅里本来就有展墙,真机的 Slamtec 底盘会绕过去 —— 只会直
着撞上的后端跑不完一趟真实导览,而"跑不完"的原因和被测的编排毫无关系。

三个实现上的取舍,每个都是被真数据逼出来的:

1. **按车宽膨胀,但起终点豁免。** 不膨胀,路径会贴着墙走;膨胀了,`P3` 和 `P7`
   的终点格**自己**会被吃掉 —— 展位 POI 本来就紧贴展屏停,这是正常的停法,不是
   坏数据。所以起终点及其邻域按未膨胀的图判定,中途按膨胀图判定。
2. **拉直(string pulling)。** 八连通 A* 出来的是锯齿,机器人走起来像在抖。逐点
   尝试跳过:只要两点之间按真实车宽扫得过去,中间的点就不要。用的是和碰撞检测
   **同一个** `segment_blocked`,所以规划认为能过的,积分器就真的能过。
3. **膨胀图按 `grid.revision` 缓存。** 虚拟墙是原地改栅格,identity 不变;只按
   id 缓存会一直拿加墙之前的图去规划,然后径直撞上那道墙。
"""

from __future__ import annotations

import heapq
import math

from simulator.generic.geometry import OCCUPIED, OccupancyGrid

# 八连通。对角代价 √2,否则规划会偏爱对角线走出一条更长的实际路径。
_NEIGHBOURS = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, 1.41421), (-1, 1, 1.41421), (1, -1, 1.41421), (1, 1, 1.41421))


class GridPlanner:
    """在占用栅格上规划,按车宽膨胀。"""

    def __init__(self, grid: OccupancyGrid, radius: float = 0.25, margin: float | None = None):
        self._grid = grid
        # 规划半径 **大于** 车体半径，真实导航栈也是这么做的。两者相等时规划会贴着
        # 墙找出一条「刚好通过」的路，执行时一点余量都没有，采样精度上的一点擦碰
        # 就判撞。在 bj-2f 上这表现为一个反直觉的现象：**车越小反而走不通**
        # （0.25m 走通 12/13，0.18m 只有 9/13）—— 因为膨胀越小、A* 找到的路越贴墙。
        self._radius = float(radius)
        self._margin = (grid.resolution * 2.0 + float(radius) * 0.25
                        if margin is None else float(margin))
        self._plan_radius = self._radius + self._margin
        self._inflated: bytearray | None = None
        self._inflated_for: tuple[int, int, float] | None = None

    # ---- 膨胀 ---------------------------------------------------------

    def _inflation(self) -> bytearray:
        grid = self._grid
        key = (id(grid), grid.revision, self._plan_radius)
        if self._inflated is not None and self._inflated_for == key:
            return self._inflated

        cells = grid.cells
        w, h = grid.width, grid.height
        pad = int(math.ceil(self._plan_radius / grid.resolution))
        inflated = bytearray(cells)
        if pad > 0:
            disc = [(dx, dy) for dy in range(-pad, pad + 1) for dx in range(-pad, pad + 1)
                    if dx * dx + dy * dy <= pad * pad]
            for index, value in enumerate(cells):
                if value != OCCUPIED:
                    continue
                cx, cy = index % w, index // w
                for dx, dy in disc:
                    x, y = cx + dx, cy + dy
                    if 0 <= x < w and 0 <= y < h:
                        inflated[y * w + x] = OCCUPIED
        self._inflated, self._inflated_for = inflated, key
        return inflated

    # ---- 规划 ---------------------------------------------------------

    def plan(self, start: tuple[float, float], goal: tuple[float, float],
             max_expansions: int = 200_000) -> list[tuple[float, float]] | None:
        """返回从 start 到 goal 的世界坐标折线（不含 start），无解返回 None。"""
        grid = self._grid
        sx, sy = grid.world_to_cell(*start)
        gx, gy = grid.world_to_cell(*goal)
        if not (grid.in_bounds(sx, sy) and grid.in_bounds(gx, gy)):
            return None
        if grid.at(gx, gy) == OCCUPIED:
            return None                      # 目标本身在墙里,不是规划能解决的

        inflated = self._inflation()
        w, h = grid.width, grid.height
        # 起终点豁免:机器人已经在那儿,也必须能停到那儿。半径取膨胀半径,
        # 保证「离开起点」和「进入终点」这两小段不会被自身膨胀锁死。
        pad = int(math.ceil(self._plan_radius / grid.resolution))

        def exempt(x: int, y: int) -> bool:
            return (abs(x - sx) <= pad and abs(y - sy) <= pad) or \
                   (abs(x - gx) <= pad and abs(y - gy) <= pad)

        def passable(x: int, y: int) -> bool:
            if not (0 <= x < w and 0 <= y < h):
                return False
            if inflated[y * w + x] != OCCUPIED:
                return True
            # 豁免区内退回未膨胀的图:那里只有真墙才拦。
            return exempt(x, y) and grid.cells[y * w + x] != OCCUPIED

        if not passable(sx, sy) or not passable(gx, gy):
            return None

        openq = [(0.0, 0.0, sx, sy)]
        came: dict[tuple[int, int], tuple[int, int]] = {}
        best = {(sx, sy): 0.0}
        expansions = 0
        while openq:
            _, cost, x, y = heapq.heappop(openq)
            if (x, y) == (gx, gy):
                return self._to_world(self._smooth(self._trace(came, gx, gy)), goal)
            if cost > best.get((x, y), math.inf):
                continue
            expansions += 1
            if expansions > max_expansions:
                return None
            for dx, dy, step in _NEIGHBOURS:
                nx, ny = x + dx, y + dy
                if not passable(nx, ny):
                    continue
                # 不许贴着墙角斜穿 —— 真机过不去的缝隙,规划也不该用。
                if dx and dy and not (passable(x + dx, y) and passable(x, y + dy)):
                    continue
                tentative = cost + step
                if tentative < best.get((nx, ny), math.inf):
                    best[(nx, ny)] = tentative
                    came[(nx, ny)] = (x, y)
                    heapq.heappush(openq, (tentative + math.hypot(nx - gx, ny - gy),
                                           tentative, nx, ny))
        return None

    # ---- 后处理 -------------------------------------------------------

    @staticmethod
    def _trace(came: dict, x: int, y: int) -> list[tuple[int, int]]:
        path = [(x, y)]
        while (x, y) in came:
            x, y = came[(x, y)]
            path.append((x, y))
        return path[::-1]

    def _smooth(self, cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """拉直:能一步扫过去的中间点全部丢掉。

        判定用的是 `swept_blocked` —— 和积分器碰撞检测**同一个函数、同一个前伸量**。
        先前两边各有一套定义（规划只按车宽，积分器还向前伸一个半径），于是拉直出
        一条看着直、走起来撞墙的路径：真展厅十三站里七站如此。
        """
        if len(cells) <= 2:
            return cells
        grid = self._grid
        out = [cells[0]]
        anchor = 0
        for probe in range(2, len(cells)):
            ax, ay = grid.cell_to_world(*cells[anchor])
            bx, by = grid.cell_to_world(*cells[probe])
            if grid.swept_blocked(ax, ay, bx, by, self._plan_radius):
                out.append(cells[probe - 1])
                anchor = probe - 1
        out.append(cells[-1])
        return out

    def _to_world(self, cells: list[tuple[int, int]],
                  goal: tuple[float, float]) -> list[tuple[float, float]]:
        points = [self._grid.cell_to_world(cx, cy) for cx, cy in cells[1:]]
        # 终点用真实坐标收尾,而不是它所在格的中心 —— 差半个格就够让
        # 「到达」判定和真实位姿对不上。
        if points:
            points[-1] = (float(goal[0]), float(goal[1]))
        else:
            points = [(float(goal[0]), float(goal[1]))]
        return points
