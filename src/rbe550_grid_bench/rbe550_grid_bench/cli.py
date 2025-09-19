import argparse
from .bench_runner import run_bench

def build_args():
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
    return p

def main():
    args = build_args().parse_args()
    run_bench(args)
