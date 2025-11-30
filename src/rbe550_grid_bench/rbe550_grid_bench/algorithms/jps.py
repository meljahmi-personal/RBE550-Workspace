"""
Jump Point Search (JPS) algorithm on a 2D grid.

Interface:
    from .jps import plan
    path, stats = plan(grid, start, goal, neighbor_fn)
"""

from typing import Callable, Tuple, List, Dict, Optional, Set
import heapq
import math

import numpy as np

from .base import BasePlanner, PlanResult

Coord = Tuple[int, int]
NeighborFn = Callable[[Coord], List[Tuple[int, int, float]]]


def manhattan(a: Coord, b: Coord) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class JumpPointSearchPlanner(BasePlanner):
    """JPS-lite: A* with simple directional pruning on a uniform-cost grid."""

    def __init__(self) -> None:
        super().__init__(name="JPS")

    def _is_valid(self, grid: np.ndarray, pos: Coord) -> bool:
        """Check if a position is within bounds and not blocked."""
        r, c = pos
        h, w = grid.shape
        return (0 <= r < h and 0 <= c < w and grid[r, c] == 0)

    def plan(
        self,
        grid: np.ndarray,
        start: Coord,
        goal: Coord,
        neighbor_fn: NeighborFn,  # bench passes in the moves=4 or 8 neighbor function
    ) -> PlanResult:
        h, w = grid.shape
        sr, sc = start
        gr, gc = goal

        # Basic validation
        if not (0 <= sr < h and 0 <= sc < w):
            return PlanResult(False, [], 0, 0, 0)
        if not (0 <= gr < h and 0 <= gc < w):
            return PlanResult(False, [], 0, 0, 0)
        if grid[sr, sc] != 0 or grid[gr, gc] != 0:
            return PlanResult(False, [], 0, 0, 0)

        if start == goal:
            return PlanResult(True, [start], 0, 1, 1)

        # A* data structures
        open_heap: List[Tuple[float, Coord]] = []
        g_cost: Dict[Coord, float] = {start: 0.0}
        parent: Dict[Coord, Optional[Coord]] = {start: None}
        closed: Set[Coord] = set()

        # f = g + h
        f_start = manhattan(start, goal)
        heapq.heappush(open_heap, (f_start, start))

        nodes_expanded = 0
        peak_open = len(open_heap)

        def direction_of(a: Coord, b: Coord) -> Optional[Coord]:
            """Return normalized direction from a -> b, or None if same cell."""
            dr = b[0] - a[0]
            dc = b[1] - a[1]
            if dr == 0 and dc == 0:
                return None
            # normalize to -1, 0, or 1
            if dr != 0:
                dr = int(dr / abs(dr))
            if dc != 0:
                dc = int(dc / abs(dc))
            return (dr, dc)

        while open_heap:
            f_current, current = heapq.heappop(open_heap)

            if current in closed:
                continue

            closed.add(current)
            nodes_expanded += 1

            if current == goal:
                break

            # Use the neighbor_fn provided by bench (respects moves=4 or 8)
            raw_neighbors = neighbor_fn(current)

            # parent-based directional pruning (JPS-lite flavor)
            parent_node = parent.get(current)
            if parent_node is not None:
                dir_prev = direction_of(parent_node, current)
            else:
                dir_prev = None

            for nr, nc, step_cost in raw_neighbors:
                neighbor = (nr, nc)

                if not self._is_valid(grid, neighbor):
                    continue
                if neighbor in closed:
                    continue

                # Simple pruning: don’t go directly back to the parent direction
                if dir_prev is not None:
                    dir_curr = direction_of(current, neighbor)
                    if dir_curr is not None:
                        # If the new direction is exactly opposite the incoming direction, skip it.
                        if dir_curr == (-dir_prev[0], -dir_prev[1]):
                            continue

                tentative_g = g_cost[current] + step_cost

                if tentative_g < g_cost.get(neighbor, float("inf")):
                    g_cost[neighbor] = tentative_g
                    parent[neighbor] = current
                    f_neighbor = tentative_g + manhattan(neighbor, goal)
                    heapq.heappush(open_heap, (f_neighbor, neighbor))

            peak_open = max(peak_open, len(open_heap))

        if goal not in parent:
            return PlanResult(False, [], nodes_expanded, peak_open, len(closed))

        # Reconstruct path
        path: List[Coord] = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = parent[cur]
        path.reverse()

        # Calculate total path cost
        total_cost = 0.0
        for i in range(len(path) - 1):
            total_cost += manhattan(path[i], path[i + 1])

        return PlanResult(
            True, path, nodes_expanded, peak_open, len(closed), total_cost
        )


def plan(
    grid: np.ndarray,
    start: Coord,
    goal: Coord,
    neighbor_fn: NeighborFn,
):
    """
    Module-level convenience wrapper used by bench_runner.

    Returns:
        path: list[(r, c)]
        stats: dict with path_len, nodes_expanded, peak_open, peak_closed, cost
    """
    planner = JumpPointSearchPlanner()
    res = planner.plan(grid, start, goal, neighbor_fn)

    path = res.path
    stats = {
        "success": res.success,
        "path_len": len(path),
        "nodes_expanded": res.nodes_expanded,
        "peak_open": res.peak_open,
        "peak_closed": res.peak_closed,
        "cost": res.cost if res.cost is not None else 0.0,
    }
    return path, stats
