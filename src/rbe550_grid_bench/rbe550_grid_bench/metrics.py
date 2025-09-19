import time, csv, os
from dataclasses import dataclass, asdict

@dataclass
class Metrics:
    planner: str
    success: bool
    runtime_ms: float
    nodes_expanded: int
    path_len_grid: float
    turns: int
    peak_open: int
    peak_closed: int
    grid_w: int
    grid_h: int
    density_pct: float
    seed: int

class MetricLogger:
    def __init__(self, csv_path=None):
        self.csv_path = csv_path or os.path.join(os.path.expanduser("~"), "rbe550_metrics.csv")
        self._header_written = os.path.exists(self.csv_path)

    def write(self, m: Metrics):
        with open(self.csv_path, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=asdict(m).keys())
            if not self._header_written:
                w.writeheader()
                self._header_written = True
            w.writerow(asdict(m))

