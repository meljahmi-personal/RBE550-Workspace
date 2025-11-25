#!/usr/bin/env python3
"""
A* search on a 2D grid.

Interface:
    from .astar import plan
    path, stats = plan(grid, start, goal, neighbor_fn)
"""

from typing import Callable, Tuple, List, Dict, Optional
import heapq
import math

import numpy as np

from .base import BasePlanner, PlanResult

Coord = Tuple[int, int]
NeighborFn = Callable[[Coord], List[Tuple[int, int, float]]]


def manhattan(a: Coord, b: Coord) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class AStarPlanner(BasePlanner):
    """Standard A* using g(n) + h(n)."""

    def __init__(self, heuristic: str = "manhattan") -> None:
        super().__init__(name="A*")
        self.heuristic = heuristic

    def _h(self, state: Coord, goal: Coord) -> float:
        if self.heuristic == "manhattan":
            return manhattan(state, goal)
        dr = state[0] - goal[0]
        dc = state[1] - goal[1]
        return math.hypot(dr, dc)

    def plan(
        self,
        grid: np.ndarray,
        start: Coord,
        goal: Coord,
        neighbor_fn: NeighborFn,
    ) -> PlanResult:
        h, w = grid.shape
        sr, sc = start
        gr, gc = goal

        if not (0 <= sr < h and 0 <= sc < w):
            return PlanResult(False, [], 0, 0, 0)
        if not (0 <= gr < h and 0 <= gc < w):
            return PlanResult(False, [], 0, 0, 0)
        if grid[sr, sc] != 0 or grid[gr, gc] != 0:
            return PlanResult(False, [], 0, 0, 0)

        if start == goal:
            return PlanResult(True, [start], 0, 1, 1)

        open_heap: List[Tuple[float, Coord]] = []
        heapq.heappush(open_heap, (self._h(start, goal), start))

        g_cost: Dict[Coord, float] = {start: 0.0}
        came_from: Dict[Coord, Optional[Coord]] = {start: None}
        closed: set[Coord] = set()

        nodes_expanded = 0
        peak_open = len(open_heap)

        while open_heap:
            f_cur, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)
            nodes_expanded += 1

            if current == goal:
                break

            cur_g = g_cost[current]

            for nr, nc, step_cost in neighbor_fn(current):
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                if grid[nr, nc] != 0:
                    continue

                nxt = (nr, nc)
                tentative_g = cur_g + float(step_cost)

                old_g = g_cost.get(nxt, float("inf"))
                if tentative_g < old_g:
                    g_cost[nxt] = tentative_g
                    came_from[nxt] = current
                    f_nxt = tentative_g + self._h(nxt, goal)
                    heapq.heappush(open_heap, (f_nxt, nxt))

            peak_open = max(peak_open, len(open_heap))

        if goal not in came_from:
            return PlanResult(False, [], nodes_expanded, peak_open, len(closed))

        # Reconstruct path
        path: List[Coord] = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = came_from[cur]
        path.reverse()

        return PlanResult(True, path, nodes_expanded, peak_open, len(closed))


def plan(
    grid: np.ndarray,
    start: Coord,
    goal: Coord,
    neighbor_fn: NeighborFn,
):
    planner = AStarPlanner()
    res = planner.plan(grid, start, goal, neighbor_fn)

    path = res.path
    stats = {
        "success": res.success,
        "path_len": len(path),
        "nodes_expanded": res.nodes_expanded,
        "peak_open": res.peak_open,
        "peak_closed": res.peak_closed,
    }
    return path, stats

