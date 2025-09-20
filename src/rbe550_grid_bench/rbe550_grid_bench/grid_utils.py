import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from pathlib import Path
from typing import Tuple, Optional

NEI4 = [(1,0),(-1,0),(0,1),(0,-1)]
NEI8 = NEI4 + [(1,1),(1,-1),(-1,1),(-1,-1)]

FREE_THRESH = 50  # <=50 considered free

def grid_from_occupancy(msg: OccupancyGrid):
    w, h = msg.info.width, msg.info.height
    data = np.array(msg.data, dtype=np.int16).reshape(h, w)
    # True = obstacle
    obst = data > FREE_THRESH
    return obst, msg.info

def world_to_grid(x, y, info):
    gx = int((x - info.origin.position.x) / info.resolution)
    gy = int((y - info.origin.position.y) / info.resolution)
    return gx, gy

def grid_to_world(gx, gy, info):
    x = gx * info.resolution + info.origin.position.x + 0.5*info.resolution
    y = gy * info.resolution + info.origin.position.y + 0.5*info.resolution
    return x, y

def in_bounds(gx, gy, w, h):
    return 0 <= gx < w and 0 <= gy < h

def line_of_sight(obst, x0, y0, x1, y1):
    # Bresenham-like LoS: returns True if segment center-to-center is obstacle-free
    dx, dy = abs(x1-x0), abs(y1-y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        if obst[y, x]:
            return False
        if x == x1 and y == y1:
            break
        e2 = 2*err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return True

def path_length(path):
    if len(path) < 2: return 0.0
    total = 0.0
    for (x0,y0),(x1,y1) in zip(path[:-1], path[1:]):
        total += math.hypot(x1-x0, y1-y0)
    return total

def count_turns(path):
    if len(path) < 3: return 0
    turns = 0
    for (x0,y0),(x1,y1),(x2,y2) in zip(path[:-2], path[1:-1], path[2:]):
        v1 = (x1-x0, y1-y0)
        v2 = (x2-x1, y2-y1)
        if v1 != v2:
            turns += 1
    return turns
    


def load_grid_from_ascii(path: str):
    """Load grid from ASCII file: '#'=obstacle, '.'=free (spaces ignored).
    Returns a numpy int8 array with 1=obstacle, 0=free.
    """
    import numpy as np
    p = Path(path)
    lines = [ln.rstrip("\n") for ln in p.read_text().splitlines() if ln.strip()]
    if not lines:
        raise ValueError(f"Empty map file: {path}")
    width = max(len(ln) for ln in lines)
    # normalize and build grid
    grid = np.zeros((len(lines), width), dtype=np.int8)
    for r, ln in enumerate(lines):
        for c, ch in enumerate(ln):
            if ch == '#':
                grid[r, c] = 1
            # '.' or ' ' remain 0
    return grid
    
    
def make_random_grid(size: int, fill: float, seed: Optional[int] = None) -> np.ndarray:
    """
    Create a size x size grid with 'fill' fraction of obstacles (1=obstacle, 0=free).
    """
    rng = np.random.default_rng(seed)
    grid = (rng.random((size, size)) < fill).astype(np.int8)
    return grid

def load_grid_from_ascii(path: str) -> np.ndarray:
    """
    Load a grid from an ASCII file (#=obstacle, '.' or space = free).
    Lines may have different lengths; the grid is padded with free cells.
    Returns int8 array with 1=obstacle, 0=free.
    """
    p = Path(path)
    lines = [ln.rstrip("\n") for ln in p.read_text().splitlines() if ln.strip() != ""]
    if not lines:
        raise ValueError(f"Empty map file: {path}")

    h = len(lines)
    w = max(len(ln) for ln in lines)
    grid = np.zeros((h, w), dtype=np.int8)
    for r, ln in enumerate(lines):
        for c, ch in enumerate(ln):
            if ch == '#':
                grid[r, c] = 1
            # '.' or ' ' stays 0
    return grid

def random_free_cell(grid: np.ndarray, rng: np.random.Generator) -> Tuple[int, int]:
    free = np.argwhere(grid == 0)
    if free.size == 0:
        raise ValueError("No free cells to place start/goal.")
    r = rng.integers(0, len(free))
    return tuple(int(x) for x in free[r])

def parse_rc_pair(spec: str) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    # "r0,c0:r1,c1"
    s, g = spec.split(":")
    r0, c0 = (int(x) for x in s.split(","))
    r1, c1 = (int(x) for x in g.split(","))
    return (r0, c0), (r1, c1)
    
    

def in_bounds(grid, x, y):
    return 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]

def is_free(grid, x, y):
    # adjust FREE test to your encoding; 0 if you use 0=free
    return grid[x, y] == 0

def neighbors4(grid, x, y):
    for dx, dy in NEI4:
        nx, ny = x + dx, y + dy
        if in_bounds(grid, nx, ny) and is_free(grid, nx, ny):
            yield nx, ny, 1.0

def neighbors8(grid, x, y):
    for dx, dy in NEI8:
        nx, ny = x + dx, y + dy
        if in_bounds(grid, nx, ny) and is_free(grid, nx, ny):
            # Diagonals should cost sqrt(2) for metric correctness
            cost = 1.0 if (dx == 0 or dy == 0) else math.sqrt(2.0)
            yield nx, ny, cost




