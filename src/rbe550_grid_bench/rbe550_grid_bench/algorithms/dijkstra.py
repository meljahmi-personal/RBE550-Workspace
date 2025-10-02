# rbe550_grid_bench/algorithms/dijkstra.py
import heapq
from .base import BasePlanner, PlanResult

class DijkstraPlanner(BasePlanner):
    name = "dijkstra"
    def plan(self, obst, start, goal, neighbor_fn):
        h, w = obst.shape
        sx, sy = start; gx, gy = goal
        if obst[sy, sx] or obst[gy, gx]:
            return PlanResult(False, [], 0, 0, 0)
        g = {(sx, sy): 0.0}
        parent = {(sx, sy): None}
        openq = [(0.0, (sx, sy))]
        closed = set(); peak_open = 1
        while openq:
            peak_open = max(peak_open, len(openq))
            cost, (x, y) = heapq.heappop(openq)
            if (x, y) == (gx, gy):
                path = []
                cur = (x, y)
                while cur is not None:
                    path.append(cur); cur = parent[cur]
                path.reverse()
                return PlanResult(True, path, len(closed), peak_open, len(closed))
            if (x, y) in closed: continue
            closed.add((x, y))
            for nx, ny, step in neighbor_fn(x, y):
                if 0 <= nx < w and 0 <= ny < h and not obst[ny, nx]:
                    ng = g[(x, y)] + step
                    if ng < g.get((nx, ny), float("inf")):
                        g[(nx, ny)] = ng
                        parent[(nx, ny)] = (x, y)
                        heapq.heappush(openq, (ng, (nx, ny)))
        return PlanResult(False, [], len(closed), peak_open, len(closed))

