from collections import deque
from .base import BasePlanner, PlanResult

class BFSPlanner(BasePlanner):
    name = "bfs"

    def plan(self, obst, start, goal, neighbor_fn):
        h, w = obst.shape
        sx, sy = start
        gx, gy = goal
        if obst[sy, sx] or obst[gy, gx]:
            return PlanResult(False, [], 0, 0, 0)

        parent = { (sx,sy): None }
        q = deque([(sx,sy)])
        closed = set()
        peak_open = 1

        while q:
            peak_open = max(peak_open, len(q))
            x, y = q.popleft()
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

            for nx, ny, _ in neighbor_fn(x,y):
                if 0 <= nx < w and 0 <= ny < h and not obst[ny, nx]:
                    if (nx,ny) not in parent:
                        parent[(nx,ny)] = (x,y)
                        q.append((nx,ny))

        return PlanResult(False, [], len(closed), peak_open, len(closed))

