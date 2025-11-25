#!/usr/bin/env python3
"""
Breadth-First Search (BFS) on a 2D grid.

Interface:
    from .bfs import plan
    path, stats = plan(grid, start, goal, neighbor_fn)

`neighbor_fn` should take a state (r, c) and return an iterable of
(neighbor_r, neighbor_c, step_cost) triples.
"""

from collections import deque
from typing import Callable, Tuple, List, Dict

import numpy as np

from .base import BasePlanner, PlanResult


Coord = Tuple[int, int]
NeighborFn = Callable[[Coord], List[Tuple[int, int, float]]]


class BFSPlanner(BasePlanner):
    """Unweighted BFS. Ignores the step_cost from neighbor_fn."""

    def __init__(self) -> None:
        super().__init__(name="BFS")

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

        # Start or goal out of bounds / blocked → immediate failure
        if not (0 <= sr < h and 0 <= sc < w):
            return PlanResult(False, [], 0, 0, 0)
        if not (0 <= gr < h and 0 <= gc < w):
            return PlanResult(False, [], 0, 0, 0)
        if grid[sr, sc] != 0 or grid[gr, gc] != 0:
            return PlanResult(False, [], 0, 0, 0)

        if start == goal:
            return PlanResult(True, [start], 0, 1, 1)

        q = deque([start])
        came_from: Dict[Coord, Coord | None] = {start: None}
        closed: set[Coord] = set([start])

        nodes_expanded = 0
        peak_open = len(q)

        while q:
            current = q.popleft()
            nodes_expanded += 1

            if current == goal:
                break

            for nr, nc, _step_cost in neighbor_fn(current):
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                if grid[nr, nc] != 0:
                    continue
                nxt = (nr, nc)
                if nxt in closed:
                    continue

                closed.add(nxt)
                came_from[nxt] = current
                q.append(nxt)

            peak_open = max(peak_open, len(q))

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
    """
    Module-level convenience wrapper used by bench_runner.

    Returns:
        path: list[(r, c)]
        stats: dict with path_len, nodes_expanded, peak_open, peak_closed
    """
    planner = BFSPlanner()
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

