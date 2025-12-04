# rbe550_grid_bench/bench_runner.py
#!/usr/bin/env python3

import time
import numpy as np
import os
import csv
from .grid_utils import (
    make_random_grid,
    load_grid_from_ascii,
    random_free_cell,
    parse_rc_pair,
)
from .neighbors import neighbors4, neighbors8
from .algorithms import bfs, dijkstra, greedy, astar, weighted_astar, theta, jps


def compute_turns(path_rc):
    """
    Count direction changes along a path.
    path_rc: list of (r, c)
    """
    if path_rc is None or len(path_rc) < 3:
        return 0

    turns = 0
    for i in range(2, len(path_rc)):
        r1, c1 = path_rc[i - 2]
        r2, c2 = path_rc[i - 1]
        r3, c3 = path_rc[i]

        dir1 = (r2 - r1, c2 - c1)
        dir2 = (r3 - r2, c3 - c2)

        if dir1 != dir2:
            turns += 1
    return turns
 

def make_neighbor_fn(grid, moves: int):
    """Return a function neighbor_fn(rc) -> iterable[(nr, nc, step_cost)]."""
    if moves == 4:
        def neighbor_fn(rc):
            r, c = rc
            return neighbors4(r, c)
    elif moves == 8:
        def neighbor_fn(rc):
            r, c = rc
            return neighbors8(r, c)
    else:
        raise ValueError(f"Unsupported moves={moves}; expected 4 or 8.")
    return neighbor_fn


def run_planner(grid, start, goal, args):
    """
    Dispatch to the correct planner based on args.algo.

    Returns:
        path: list of (r, c)
        stats: dict with keys path_len, nodes_expanded, peak_open, peak_closed, cost, runtime_ms
    """
    neighbor_fn = make_neighbor_fn(grid, args.moves)

    # Pick planner
    if args.algo == "bfs":
        planner = bfs.BFSPlanner()
    elif args.algo == "dijkstra":
        planner = dijkstra.DijkstraPlanner()
    elif args.algo == "greedy":
        planner = greedy.GreedyBestFirstPlanner()
    elif args.algo == "astar":
        planner = astar.AStarPlanner()
    elif args.algo in ("wastar", "weighted_astar"):
        # Weighted A*
        planner = weighted_astar.WeightedAStarPlanner(weight=args.weight)
    elif args.algo == "theta_star":
        # Theta* - any-angle paths
        planner = theta.ThetaStarPlanner()
    elif args.algo == "jps":
        # Jump Point Search - optimization for uniform-cost grids
        planner = jps.JumpPointSearchPlanner()
    else:
        raise ValueError(f"Unknown planner algo={args.algo}")

    t0 = time.perf_counter()
    res = planner.plan(grid, start, goal, neighbor_fn=neighbor_fn)
    dt_ms = (time.perf_counter() - t0) * 1000.0

    path = res.path
    turns = compute_turns(path)
    stats = {
        "path_len": len(path),
        "nodes_expanded": res.nodes_expanded,
        "peak_open": getattr(res, "peak_open", None),
        "peak_closed": getattr(res, "peak_closed", None),
        "cost": getattr(res, "cost", None),
        "runtime_ms": round(dt_ms, 3),
        "turns": turns,
    }

    return path, stats


def run_bench(args):
    extra = ""
    if getattr(args, "algo", "") in ("wastar", "weighted_astar"):
        extra = f", weight={getattr(args, 'weight', 1.0)}"

    print(f"[rbe550] Running steps={args.steps}, render_every={args.render_every}, "
          f"grid={args.grid}, fill={args.fill}, enemies={args.enemies}, "
          f"algo={args.algo}, seed={args.seed}, gif={args.gif}, show={args.show}, "
          f"moves={args.moves}{extra}")

    # Build grid
    if getattr(args, "map_path", None):
        grid = load_grid_from_ascii(args.map_path)
        src = f"map='{args.map_path}'"
    else:
        grid = make_random_grid(size=args.grid, fill=args.fill, seed=args.seed)
        src = f"grid={args.grid}, fill={args.fill}, seed={args.seed}"

    # Choose start/goal
    rng = np.random.default_rng(args.seed)
    if getattr(args, "start_goal", None):
        start, goal = parse_rc_pair(args.start_goal)
    else:
        start = random_free_cell(grid, rng)
        goal = random_free_cell(grid, rng)
        tries = 0
        while goal == start and tries < 10:
            goal = random_free_cell(grid, rng)
            tries += 1

    # Ensure free cells
    grid[start] = 0
    grid[goal] = 0

    h, w = grid.shape
    blocked = int(grid.sum())
    obstacle_frac = blocked / (h * w)

    print(f"[rbe550] {src}, enemies={args.enemies}, algo={args.algo}, gif={args.gif}, show={args.show}")
    print(f"[rbe550] Grid: {h}x{w}, obstacles={blocked} ({obstacle_frac:.1%}), start={start}, goal={goal}")

    # Run planner
    path, stats = run_planner(grid, start, goal, args)
    ok = bool(path)
    


    print(f"[rbe550] success={ok}, "
          f"path_len={stats.get('path_len')}, "
          f"runtime_ms={stats.get('runtime_ms')}, "
          f"nodes_expanded={stats.get('nodes_expanded')}")

    # 🔹 NEW: optional CSV logging
    csv_path = getattr(args, "csv_path", None)
    if csv_path:
        write_header = not os.path.exists(csv_path)
        with open(csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow([
                    "algo",
                    "moves",
                    "grid",
                    "fill",
                    "obstacles",
                    "obstacle_frac",
                    "start_r",
                    "start_c",
                    "goal_r",
                    "goal_c",
                    "success",
                    "path_len",
                    "runtime_ms",
                    "nodes_expanded",
                    "cost",
                    "peak_open",
                    "peak_closed",
                    "turns",
                    "seed",
                ])

            writer.writerow([
                args.algo,
                args.moves,
                args.grid,
                args.fill,
                blocked,
                round(obstacle_frac, 4),
                start[0], start[1],
                goal[0], goal[1],
                int(ok),
                stats.get("path_len"),
                stats.get("runtime_ms"),
                stats.get("nodes_expanded"),
                stats.get("cost"),
                stats.get("peak_open"),
                stats.get("peak_closed"),
                stats.get("turns"),
                args.seed,
            ])



def main():
    # Allow `ros2 run rbe550_grid_bench bench_runner` for quick smoke test
    class A:
        pass

    a = A()
    a.steps = 200
    a.render_every = 4
    a.grid = 64
    a.fill = 0.20
    a.enemies = 10
    a.algo = "bfs"
    a.seed = None
    a.gif = None
    a.show = False
    a.map_path = None
    a.start_goal = None
    a.moves = 4
    run_bench(a)

