# rbe550_grid_bench/bench_runner.py
from .grid_utils import make_random_grid, load_grid_from_ascii, random_free_cell, parse_rc_pair
import numpy as np
from .algorithms import bfs as bfs_mod
from .algorithms import astar as astar_mod
from .neighbors import neighbors4, neighbors8   # <-- FIX: was from grid_utils
import time

def run_planner(algo, grid, start, goal, moves='4'):
    neighbors_fn = neighbors4 if moves == '4' else neighbors8

    # Optional safety: BFS + weighted diagonals isn't correct
    if algo == 'bfs' and moves == '8':
        # If your neighbors8 uses cost=1.0 (Chebyshev), you can delete this guard.
        raise ValueError("BFS with 8-connected weighted diagonals is invalid. Use A*.")

    t0 = time.perf_counter()
    if algo == 'bfs':
        path, stats = bfs_mod.plan(grid, start, goal, neighbors_fn)
    elif algo == 'astar':
        # astar.plan accepts neighbors_fn (and optional heuristic kwarg if you ever add it)
        path, stats = astar_mod.plan(grid, start, goal, neighbors_fn)
    else:
        raise ValueError(f"Unknown algo: {algo}")

    dt_ms = round((time.perf_counter() - t0) * 1000.0, 3)
    stats = stats or {}
    stats.setdefault('runtime_ms', dt_ms)
    stats.setdefault('path_len', len(path) if path else 0)
    return path, stats


def run_bench(args):
    print(f"[rbe550] Running steps={args.steps}, render_every={args.render_every}, "
          f"grid={args.grid}, fill={args.fill}, enemies={args.enemies}, "
          f"algo={args.algo}, seed={args.seed}, gif={args.gif}, show={args.show}, "
          f"moves={getattr(args,'moves','4')}")

    # Build grid
    if getattr(args, 'map_path', None):
        grid = load_grid_from_ascii(args.map_path)
        src = f"map='{args.map_path}'"
    else:
        grid = make_random_grid(size=args.grid, fill=args.fill, seed=args.seed)
        src = f"grid={args.grid}, fill={args.fill}, seed={args.seed}"

    # Choose start/goal
    rng = np.random.default_rng(args.seed)
    if getattr(args, 'start_goal', None):
        start, goal = parse_rc_pair(args.start_goal)
    else:
        start = random_free_cell(grid, rng)
        goal = random_free_cell(grid, rng)
        tries = 0
        while goal == start and tries < 10:
            goal = random_free_cell(grid, rng); tries += 1

    # Ensure free start/goal
    grid[start] = 0
    grid[goal]  = 0

    h, w = grid.shape
    blocked = int(grid.sum())
    print(f"[rbe550] {src}, enemies={args.enemies}, algo={args.algo}, gif={args.gif}, show={args.show}")
    print(f"[rbe550] Grid: {h}x{w}, obstacles={blocked} ({blocked/(h*w):.1%}), start={start}, goal={goal}")

    # Pass moves through
    path, stats = run_planner(args.algo, grid, start, goal, moves=getattr(args, 'moves', '4'))

    ok = bool(path)
    print(f"[rbe550] success={ok}, path_len={stats.get('path_len')}, "
          f"runtime_ms={stats.get('runtime_ms')}, nodes_expanded={stats.get('nodes_expanded')}")


def main():
    # Allow `ros2 run rbe550_grid_bench bench_runner`
    class A: pass
    a = A()
    a.steps=200; a.render_every=4; a.grid=64; a.fill=0.20
    a.enemies=10; a.algo='bfs'; a.seed=None; a.gif=None; a.show=False
    a.map_path=None; a.start_goal=None
    a.moves='4'  # switch to '8' to test diagonals
    run_bench(a)

