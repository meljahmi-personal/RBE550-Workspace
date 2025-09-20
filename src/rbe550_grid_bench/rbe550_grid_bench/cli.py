# rbe550_grid_bench/cli.py
import argparse
from .bench_runner import run_bench

def build_parser():
    p = argparse.ArgumentParser()
    p.add_argument('--steps', type=int, default=200)
    p.add_argument('--render-every', type=int, default=4)
    p.add_argument('--grid', type=int, default=64)
    p.add_argument('--fill', type=float, default=0.20)
    p.add_argument('--enemies', type=int, default=10)
    p.add_argument('--algo', choices=['bfs','astar'], default='bfs')
    p.add_argument('--seed', type=int, default=None)
    p.add_argument('--gif', type=str, default=None)
    p.add_argument('--show', dest='show', action='store_true')
    p.add_argument('--no-show', dest='show', action='store_false')
    p.set_defaults(show=False)

    p.add_argument('--map', dest='map_path', type=str, default=None)
    p.add_argument('--start-goal', dest='start_goal', type=str, default=None)

    p.add_argument('--moves', choices=['4','8'], default='4',
                   help='Neighborhood connectivity: 4- or 8-connected')
    return p

def main():
    args = build_parser().parse_args()
    run_bench(args)

