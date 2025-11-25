#!/usr/bin/env python3
"""
Quick-and-dirty visualization: parse console logs and plot
runtime_ms vs nodes_expanded for different planners.

Usage:
    # run a bunch of experiments and save console logs:
    ./scripts/run.sh --grid 64 --fill 0.20 --seed 42 --algo bfs    --moves 8 --no-show | tee outputs/bfs_seed42.log
    ./scripts/run.sh --grid 64 --fill 0.20 --seed 42 --algo dijkstra --moves 8 --no-show | tee outputs/dijkstra_seed42.log
    ./scripts/run.sh --grid 64 --fill 0.20 --seed 42 --algo greedy --moves 8 --no-show | tee outputs/greedy_seed42.log
    ./scripts/run.sh --grid 64 --fill 0.20 --seed 42 --algo astar  --moves 8 --no-show | tee outputs/astar_seed42.log

    # Then:
    python3 scripts/plot_from_logs.py outputs/*.log
"""

import re
import sys
from pathlib import Path
from typing import List, Dict

import matplotlib.pyplot as plt

# pattern matches lines like:
# [rbe550] success=True, path_len=45, runtime_ms=1.926, nodes_expanded=455
LINE_RE = re.compile(
    r"success=(True|False).*?path_len=(\d+),\s*runtime_ms=([\d\.]+),\s*nodes_expanded=(\d+)"
)


def parse_log(path: Path) -> Dict[str, float]:
    text = path.read_text()
    m = LINE_RE.search(text)
    if not m:
        raise RuntimeError(f"Could not find metrics line in {path}")
    success_str, path_len, runtime_ms, nodes_expanded = m.groups()
    return {
        "success": success_str == "True",
        "path_len": float(path_len),
        "runtime_ms": float(runtime_ms),
        "nodes_expanded": float(nodes_expanded),
    }


def main(log_paths: List[str]) -> None:
    if not log_paths:
        print("Usage: plot_from_logs.py outputs/*.log")
        sys.exit(1)

    planners = []
    runtimes = []
    expanded = []

    for p in log_paths:
        path = Path(p)
        metrics = parse_log(path)

        # Infer planner name from filename (e.g., bfs_seed42.log -> bfs)
        name = path.stem.split("_")[0]
        planners.append(name)
        runtimes.append(metrics["runtime_ms"])
        expanded.append(metrics["nodes_expanded"])

    # Plot runtime
    plt.figure()
    plt.bar(planners, runtimes)
    plt.ylabel("Runtime (ms)")
    plt.title("Planner runtime comparison")
    plt.tight_layout()
    plt.savefig("outputs/runtime_comparison.png")

    # Plot nodes expanded
    plt.figure()
    plt.bar(planners, expanded)
    plt.ylabel("Nodes expanded")
    plt.title("Planner search effort comparison")
    plt.tight_layout()
    plt.savefig("outputs/nodes_expanded_comparison.png")

    print("Saved plots to outputs/runtime_comparison.png and outputs/nodes_expanded_comparison.png")


if __name__ == "__main__":
    main(sys.argv[1:])

