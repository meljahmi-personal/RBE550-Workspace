# rbe550_grid_bench/algorithms/weighted_astar.py

from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Callable, Iterable, List, Tuple, Optional

import numpy as np

from .base import BasePlanner, PlanResult

Coord = Tuple[int, int]
NeighborFn = Callable[[Coord], Iterable[Tuple[int, int, float]]]


def manhattan(a: Coord, b: Coord) -> float:
    """Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


@dataclass
class WeightedAStarPlanner(BasePlanner):
    """Weighted A* planner: f = g + w * h."""

    weight: float = 1.0

    def __init__(self, weight: float = 1.0):
        super().__init__(name="WeightedAStar")
        self.weight = float(weight)

    def plan(
        self,
        grid: np.ndarray,
        start: Coord,
        goal: Coord,
        neighbor_fn: NeighborFn,
    ) -> PlanResult:
        # Standard A* / Weighted A* structures
        open_heap: List[Tuple[float, int, Coord]] = []
        g_cost: dict[Coord, float] = {start: 0.0}
        came_from: dict[Coord, Coord] = {}

        # stats
        nodes_expanded = 0
        peak_open = 0
        peak_closed = 0

        # closed set
        closed: set[Coord] = set()

        # tie-breaker counter
        counter = 0

        h0 = manhattan(start, goal)
        f0 = g_cost[start] + self.weight * h0
        heapq.heappush(open_heap, (f0, counter, start))

        while open_heap:
            peak_open = max(peak_open, len(open_heap))
            f, _, current = heapq.heappop(open_heap)

            if current in closed:
                continue

            closed.add(current)
            peak_closed = max(peak_closed, len(closed))
            nodes_expanded += 1

            if current == goal:
                path = self._reconstruct_path(came_from, current)
                total_cost = g_cost[current]
                return PlanResult(
                    path=path,
                    nodes_expanded=nodes_expanded,
                    success=True,
                    peak_open=peak_open,
                    peak_closed=peak_closed,
                    cost=total_cost,
                )

            for nr, nc, step_cost in neighbor_fn(current):
                nxt: Coord = (nr, nc)

                if nxt in closed:
                    continue

                tentative_g = g_cost[current] + float(step_cost)

                old_g = g_cost.get(nxt, float("inf"))
                if tentative_g < old_g:
                    g_cost[nxt] = tentative_g
                    came_from[nxt] = current
                    counter += 1
                    h_val = manhattan(nxt, goal)
                    f_val = tentative_g + self.weight * h_val
                    heapq.heappush(open_heap, (f_val, counter, nxt))

        # Failed: no path
        return PlanResult(
            path=[],
            nodes_expanded=nodes_expanded,
            success=False,
            peak_open=peak_open,
            peak_closed=peak_closed,
            cost=float("inf"),
        )

    @staticmethod
    def _reconstruct_path(
        came_from: dict[Coord, Coord],
        current: Coord,
    ) -> List[Coord]:
        path: List[Coord] = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


def plan(
    grid: np.ndarray,
    start: Coord,
    goal: Coord,
    neighbor_fn: NeighborFn,
    weight: float = 1.0,
) -> Tuple[List[Coord], dict]:
    """
    Convenience function mirroring other modules' `plan(...)` signature.

    Returns:
        path, stats_dict
    """
    planner = WeightedAStarPlanner(weight=weight)
    res = planner.plan(grid, start, goal, neighbor_fn)

    stats = {
        "path_len": len(res.path),
        "nodes_expanded": res.nodes_expanded,
        "peak_open": res.peak_open,
        "peak_closed": res.peak_closed,
        "cost": res.cost,
    }
    return res.path, stats

