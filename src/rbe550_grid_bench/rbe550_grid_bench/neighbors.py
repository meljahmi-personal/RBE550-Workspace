# rbe550_grid_bench/neighbors.py
import math

NEI4 = [(1,0),(-1,0),(0,1),(0,-1)]
NEI8 = NEI4 + [(1,1),(1,-1),(-1,1),(-1,-1)]

def neighbors4(x, y):
    """4-connected moves, unit cost"""
    for dx, dy in NEI4:
        yield x + dx, y + dy, 1.0

def neighbors8(x, y):
    """8-connected moves, diagonals cost sqrt(2)"""
    for dx, dy in NEI8:
        cost = 1.0 if (dx == 0 or dy == 0) else math.sqrt(2.0)
        yield x + dx, y + dy, cost

