# rbe550_grid_bench/bench_runner.py

import time
import numpy as np

from .grid_utils import (
    make_random_grid,
    load_grid_from_ascii,
    random_free_cell,
    parse_rc_pair,
)
from .neighbors import neighbors4, neighbors8
from .algorithms import bfs, dijkstra, greedy, astar, weighted_astar



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
        # NEW: Weighted A*
        planner = weighted_astar.WeightedAStarPlanner(weight=args.weight)
    else:
        raise ValueError(f"Unknown planner algo={args.algo}")

    t0 = time.perf_counter()
    res = planner.plan(grid, start, goal, neighbor_fn=neighbor_fn)
    dt_ms = (time.perf_counter() - t0) * 1000.0

    path = res.path
    stats = {
        "path_len": len(path),
        "nodes_expanded": res.nodes_expanded,
        "peak_open": getattr(res, "peak_open", None),
        "peak_closed": getattr(res, "peak_closed", None),
        "cost": getattr(res, "cost", None),
        "runtime_ms": round(dt_ms, 3),
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
    print(f"[rbe550] {src}, enemies={args.enemies}, algo={args.algo}, gif={args.gif}, show={args.show}")
    print(f"[rbe550] Grid: {h}x{w}, obstacles={blocked} ({blocked/(h*w):.1%}), start={start}, goal={goal}")

    # Run planner
    path, stats = run_planner(grid, start, goal, args)
    ok = bool(path)

    print(f"[rbe550] success={ok}, "
          f"path_len={stats.get('path_len')}, "
          f"runtime_ms={stats.get('runtime_ms')}, "
          f"nodes_expanded={stats.get('nodes_expanded')}")


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

