from dataclasses import dataclass

@dataclass
class PlanResult:
    success: bool
    path: list        # list of (gx, gy)
    nodes_expanded: int
    peak_open: int
    peak_closed: int

class BasePlanner:
    name = "base"
    def plan(self, obst, start, goal, neighbor_fn):
        raise NotImplementedError

