"""
Theta* algorithm on a 2D grid - any-angle path planning.

Interface:
    from .theta_star import plan
    path, stats = plan(grid, start, goal, neighbor_fn)
"""

from typing import Callable, Tuple, List, Dict, Optional
import heapq
import math

import numpy as np

from .base import BasePlanner, PlanResult

Coord = Tuple[int, int]
NeighborFn = Callable[[Coord], List[Tuple[int, int, float]]]


def euclidean_distance(a: Coord, b: Coord) -> float:
    """Euclidean distance for any-angle planning."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


class ThetaStarPlanner(BasePlanner):
    """Theta* algorithm: A* with line-of-sight path smoothing."""

    def __init__(self) -> None:
        super().__init__(name="Theta*")

    def _has_line_of_sight(self, grid: np.ndarray, a: Coord, b: Coord) -> bool:
        """
        Check if there's a clear line of sight between two points using Bresenham's line algorithm.
        """
        r1, c1 = a
        r2, c2 = b
        
        # Simple cases
        if a == b:
            return True
            
        # Check if start or goal are blocked
        if grid[r1, c1] != 0 or grid[r2, c2] != 0:
            return False
        
        # Bresenham's line algorithm
        dr = abs(r2 - r1)
        dc = abs(c2 - c1)
        
        r, c = r1, c1
        r_inc = 1 if r2 > r1 else -1
        c_inc = 1 if c2 > c1 else -1
        
        error = dr - dc
        
        while r != r2 or c != c2:
            if grid[r, c] != 0:
                return False
                
            double_error = error * 2
            
            if double_error > -dc:
                error -= dc
                r += r_inc
                
            if double_error < dr:
                error += dr
                c += c_inc
                
        return True

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

        # Basic validation
        if not (0 <= sr < h and 0 <= sc < w):
            return PlanResult(False, [], 0, 0, 0)
        if not (0 <= gr < h and 0 <= gc < w):
            return PlanResult(False, [], 0, 0, 0)
        if grid[sr, sc] != 0 or grid[gr, gc] != 0:
            return PlanResult(False, [], 0, 0, 0)

        if start == goal:
            return PlanResult(True, [start], 0, 1, 1)

        # Theta* data structures
        open_heap: List[Tuple[float, Coord]] = []
        g_cost: Dict[Coord, float] = {start: 0.0}
        parent: Dict[Coord, Optional[Coord]] = {start: None}
        closed: set[Coord] = set()

        # Initialize
        f_start = euclidean_distance(start, goal)
        heapq.heappush(open_heap, (f_start, start))

        nodes_expanded = 0
        peak_open = len(open_heap)

        while open_heap:
            f_current, current = heapq.heappop(open_heap)
            
            if current in closed:
                continue
                
            closed.add(current)
            nodes_expanded += 1

            if current == goal:
                break

            # Check if we can improve the path to current's parent
            current_parent = parent[current]
            if current_parent is not None:
                # Try to connect current's parent directly to neighbors
                for nr, nc, step_cost in neighbor_fn(current):
                    if not (0 <= nr < h and 0 <= nc < w):
                        continue
                    if grid[nr, nc] != 0:
                        continue
                        
                    neighbor = (nr, nc)
                    if neighbor in closed:
                        continue
                    
                    # Line of sight check from parent to neighbor
                    if self._has_line_of_sight(grid, current_parent, neighbor):
                        new_g = g_cost[current_parent] + euclidean_distance(current_parent, neighbor)
                        
                        if new_g < g_cost.get(neighbor, float('inf')):
                            g_cost[neighbor] = new_g
                            parent[neighbor] = current_parent
                            f_neighbor = new_g + euclidean_distance(neighbor, goal)
                            heapq.heappush(open_heap, (f_neighbor, neighbor))

            # Standard A* expansion for current node
            for nr, nc, step_cost in neighbor_fn(current):
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                if grid[nr, nc] != 0:
                    continue
                    
                neighbor = (nr, nc)
                if neighbor in closed:
                    continue

                new_g = g_cost[current] + euclidean_distance(current, neighbor)
                
                if new_g < g_cost.get(neighbor, float('inf')):
                    g_cost[neighbor] = new_g
                    parent[neighbor] = current
                    f_neighbor = new_g + euclidean_distance(neighbor, goal)
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
            total_cost += euclidean_distance(path[i], path[i + 1])

        return PlanResult(True, path, nodes_expanded, peak_open, len(closed), total_cost)


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
    planner = ThetaStarPlanner()
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
