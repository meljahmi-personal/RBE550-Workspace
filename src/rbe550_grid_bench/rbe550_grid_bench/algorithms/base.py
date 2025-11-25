# rbe550_grid_bench/algorithms/base.py

from dataclasses import dataclass
from typing import List, Tuple, Optional

Coord = Tuple[int, int]


@dataclass
class PlanResult:
    """
    Common container for planner results.

    Fields:
      - success: True if a path to the goal was found
      - path: list of (r, c) cells from start to goal (inclusive)
      - nodes_expanded: how many nodes were popped from the open set / queue
      - peak_open: maximum size of the open frontier during search
      - peak_closed: maximum size of the closed / visited set
      - cost: (optional) final path cost, if the algorithm maintains one
    """
    success: bool
    path: List[Coord]
    nodes_expanded: int
    peak_open: int = 0
    peak_closed: int = 0
    cost: Optional[float] = None


class BasePlanner:
    """
    Minimal base class so subclasses can call super().__init__(name=...).
    """

    def __init__(self, name: str = "planner") -> None:
        self.name = name

    def plan(self, grid, start: Coord, goal: Coord, neighbor_fn):
        """
        Subclasses must implement this. Should return a PlanResult.
        """
        raise NotImplementedError("Subclasses must implement plan()")

