import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

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

def neighbors_4(gx, gy):
    for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
        yield gx+dx, gy+dy, 1.0

def neighbors_8(gx, gy):
    for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),
                   (-1,-1),(-1,1),(1,-1),(1,1)]:
        cost = 1.0 if dx == 0 or dy == 0 else math.sqrt(2.0)
        yield gx+dx, gy+dy, cost

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

