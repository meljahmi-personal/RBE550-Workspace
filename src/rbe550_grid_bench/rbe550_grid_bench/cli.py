#!/usr/bin/env python3
import argparse

from .bench_runner import run_bench


def build_parser() -> argparse.ArgumentParser:
    """
    Build the CLI parser for the rbe550_grid_bench benchmark.

    This must create and return an argparse.ArgumentParser instance.
    """
    parser = argparse.ArgumentParser(
        description="Benchmark classical grid-based planners (BFS, Dijkstra, Greedy, A*)"
    )

    # Core simulation controls
    parser.add_argument(
        "--steps",
        type=int,
        default=200,
        help="Number of simulation steps to run (default: 200)",
    )
    parser.add_argument(
        "--render-every",
        dest="render_every",
        type=int,
        default=4,
        help="Render every N steps (default: 4)",
    )
    parser.add_argument(
        "--grid",
        type=int,
        default=64,
        help="Grid size N for an N×N random grid (default: 64)",
    )
    parser.add_argument(
        "--fill",
        type=float,
        default=0.20,
        help="Obstacle fill ratio in random grid (default: 0.20)",
    )
    parser.add_argument(
        "--enemies",
        type=int,
        default=10,
        help="Number of enemies (not yet used in all experiments)",
    )

    # Planner selection
    parser.add_argument(
        "--algo",
        type=str,
        default="bfs",
        choices=["bfs", "dijkstra", "greedy", "astar"],
        help="Planner to use: bfs, dijkstra, greedy, or astar (default: bfs)",
    )

    # Randomness / seeds
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducibility (default: None)",
    )

    # Visualization / output
    parser.add_argument(
        "--gif",
        type=str,
        default=None,
        help="Optional path to save an animated GIF (default: None)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="If set, show live visualization (if supported)",
    )
    
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Disable visualization (alias for default behavior)",
    )

    # 4- or 8-connected grid
    parser.add_argument(
        "--moves",
        type=int,
        choices=[4, 8],
        default=4,
        help="Allowed moves: 4-connected (N,E,S,W) or 8-connected (adds diagonals). Default: 4",
    )

    # Map / start-goal overrides
    parser.add_argument(
        "--map",
        dest="map_path",
        type=str,
        default=None,
        help="Path to ASCII map file (overrides random grid)",
    )
    parser.add_argument(
        "--start-goal",
        dest="start_goal",
        type=str,
        default=None,
        help="Explicit start/goal as 'r1,c1:r2,c2' (optional)",
    )

    return parser


def main() -> None:
    """
    Entry point for console_scripts: 'bench'.
    Parses arguments and calls run_bench(args).
    """
    args = build_parser().parse_args()
    run_bench(args)

