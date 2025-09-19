import math, heapq
from .base import BasePlanner, PlanResult

def manhattan(a,b): return abs(a[0]-b[0]) + abs(a[1]-b[1])
def euclidean(a,b): return math.hypot(a[0]-b[0], a[1]-b[1])

class AStarPlanner(BasePlanner):
    name = "astar"

    def __init__(self, heuristic="euclidean"):
        self.hfun = euclidean if heuristic == "euclidean" else manhattan

    def plan(self, obst, start, goal, neighbor_fn):
        h, w = obst.shape
        sx, sy = start
        gx, gy = goal
        if obst[sy, sx] or obst[gy, gx]:
            return PlanResult(False, [], 0, 0, 0)

        g = { (sx,sy): 0.0 }
        parent = { (sx,sy): None }
        openq = []
        heapq.heappush(openq, (self.hfun(start, goal), (sx,sy)))
        in_open = { (sx,sy) }
        closed = set()
        peak_open = 1

        while openq:
            peak_open = max(peak_open, len(openq))
            _, (x,y) = heapq.heappop(openq)
            if (x,y) == (gx,gy):
                # reconstruct
                path = []
                cur = (x,y)
                while cur is not None:
                    path.append(cur)
                    cur = parent[cur]
                path.reverse()
                return PlanResult(True, path, len(closed), peak_open, len(closed))

            if (x,y) in closed:
                continue
            closed.add((x,y))

            for nx, ny, cost in neighbor_fn(x,y):
                if 0 <= nx < w and 0 <= ny < h and not obst[ny, nx]:
                    ng = g[(x,y)] + cost
                    if ng < g.get((nx,ny), float('inf')):
                        g[(nx,ny)] = ng
                        parent[(nx,ny)] = (x,y)
                        f = ng + self.hfun((nx,ny), (gx,gy))
                        heapq.heappush(openq, (f, (nx,ny)))
                        in_open.add((nx,ny))

        return PlanResult(False, [], len(closed), peak_open, len(closed))

