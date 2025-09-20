# rbe550_grid_bench/algorithms/bfs.py

from collections import deque
from .base import BasePlanner, PlanResult
from ..neighbors import neighbors4, neighbors8  # shared neighbor generators


class BFSPlanner(BasePlanner):
    name = "bfs"

    def plan(self, obst, start, goal, neighbor_fn):
        """
        obst: 2D numpy array (1=obstacle, 0=free) indexed as obst[row, col] == obst[y, x]
        start, goal: tuples (x, y)
        neighbor_fn: callable (x, y) -> yields (nx, ny, step_cost)  # cost ignored by BFS
        """
        h, w = obst.shape
        sx, sy = start
        gx, gy = goal

        # If start/goal are blocked, early out
        if obst[sy, sx] or obst[gy, gx]:
            return PlanResult(False, [], 0, 0, 0)

        parent = {(sx, sy): None}
        q = deque([(sx, sy)])
        closed = set()
        peak_open = 1

        while q:
            peak_open = max(peak_open, len(q))
            x, y = q.popleft()

            if (x, y) == (gx, gy):
                # reconstruct path
                path = []
                cur = (x, y)
                while cur is not None:
                    path.append(cur)
                    cur = parent[cur]
                path.reverse()
                return PlanResult(True, path, len(closed), peak_open, len(closed))

            if (x, y) in closed:
                continue
            closed.add((x, y))

            for nx, ny, _ in neighbor_fn(x, y):  # BFS ignores step cost
                if 0 <= nx < w and 0 <= ny < h and not obst[ny, nx]:
                    if (nx, ny) not in parent:
                        parent[(nx, ny)] = (x, y)
                        q.append((nx, ny))

        return PlanResult(False, [], len(closed), peak_open, len(closed))


def plan(grid_rc, start_rc, goal_rc, neighbors_fn=neighbors4):
    """
    Facade for the bench runner:
      - Accepts grid indexed by (row, col)
      - Accepts start/goal as (row, col)
      - Converts to (x, y) for internal planner (x=col, y=row)
      - Uses neighbors_fn (default: 4-connected)
      - Returns (path_rc, stats)
    """
    bfs = BFSPlanner()

    # (r,c) -> (x,y)
    sx, sy = start_rc[1], start_rc[0]
    gx, gy = goal_rc[1], goal_rc[0]

    res = bfs.plan(grid_rc, (sx, sy), (gx, gy), neighbors_fn)

    # path back to (r,c)
    path_xy = getattr(res, "path", []) or []
    path_rc = [(y, x) for (x, y) in path_xy]

    stats = {
        "nodes_expanded": getattr(res, "nodes_expanded", getattr(res, "expanded", 0)),
        "peak_open": getattr(res, "peak_open", None),
        "path_len": len(path_rc),
        "success": getattr(res, "success", bool(path_rc)),
    }
    return path_rc, stats

