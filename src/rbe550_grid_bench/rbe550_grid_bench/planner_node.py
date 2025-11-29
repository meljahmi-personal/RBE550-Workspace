# rbe550_grid_bench/planner_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA

import numpy as np
from .grid_utils import make_random_grid, load_grid_from_ascii, random_free_cell, parse_rc_pair
from .neighbors import neighbors4, neighbors8
from .algorithms import bfs, dijkstra, greedy, astar, weighted_astar

def make_neighbor_fn(grid, moves: int):
    if moves == 4:
        return lambda rc: neighbors4(grid, rc[0], rc[1])
    elif moves == 8:
        return lambda rc: neighbors8(grid, rc[0], rc[1])
    else:
        raise ValueError(f"moves must be 4 or 8, got {moves}")

def to_occupancy_grid(grid: np.ndarray, frame_id="map", resolution=1.0):
    h, w = grid.shape
    msg = OccupancyGrid()
    msg.header = Header(frame_id=frame_id)
    msg.info = MapMetaData()
    msg.info.resolution = float(resolution)
    msg.info.width = int(w)
    msg.info.height = int(h)
    # origin at (0,0), z=0; adjust if you want map centered
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    msg.info.origin.orientation.w = 1.0
    # ROS OccupancyGrid is row-major from map origin (bottom-left).
    # Our grid is [row, col] with (0,0) at top-left. Flip vertically so “up” in RViz matches your arrays.
    flipped = np.flipud(grid.astype(np.int8))
    # 0 free -> 0, 1 obstacle -> 100
    data = (flipped * 100).astype(np.int8)
    msg.data = data.flatten().tolist()
    return msg

def path_to_msg(path_rc, frame_id="map", resolution=1.0):
    p = Path()
    p.header = Header(frame_id=frame_id)
    for (r, c) in path_rc:
        ps = PoseStamped()
        ps.header = Header(frame_id=frame_id)
        ps.pose.position.x = float(c) * resolution + 0.5 * resolution
        ps.pose.position.y = float((r)) * resolution + 0.5 * resolution
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p

def point_marker(x, y, mid, rgba, scale=0.35, frame_id="map"):
    m = Marker()
    m.header.frame_id = frame_id
    m.id = mid
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color = ColorRGBA(*rgba)
    m.lifetime.sec = 0  # forever
    return m

class PlannerVizNode(Node):
    def __init__(self):
        super().__init__("planner_viz")

        # ---- params (make these launch-configurable) ----
        self.declare_parameter("algo", "astar")                # bfs|dijkstra|greedy|astar|weighted_astar
        self.declare_parameter("moves", 8)                     # 4 or 8
        self.declare_parameter("weight", 1.0)                  # only for weighted_astar
        self.declare_parameter("grid", 64)
        self.declare_parameter("fill", 0.20)
        self.declare_parameter("seed", 42)
        self.declare_parameter("map_path", "")
        self.declare_parameter("start_goal", "")               # like "r1,c1:r2,c2"
        self.declare_parameter("resolution", 1.0)
        self.declare_parameter("frame_id", "map")

        algo   = self.get_parameter("algo").get_parameter_value().string_value
        moves  = int(self.get_parameter("moves").value)
        weight = float(self.get_parameter("weight").value)
        size   = int(self.get_parameter("grid").value)
        fill   = float(self.get_parameter("fill").value)
        seed   = self.get_parameter("seed").value
        map_p  = self.get_parameter("map_path").get_parameter_value().string_value or None
        sg     = self.get_parameter("start_goal").get_parameter_value().string_value or None
        res    = float(self.get_parameter("resolution").value)
        frame  = self.get_parameter("frame_id").get_parameter_value().string_value

        # ---- build grid / start / goal ----
        if map_p:
            grid = load_grid_from_ascii(map_p)
            src = f"map='{map_p}'"
        else:
            grid = make_random_grid(size=size, fill=fill, seed=seed)
            src = f"grid={size}, fill={fill}, seed={seed}"

        rng = np.random.default_rng(seed)
        if sg:
            start, goal = parse_rc_pair(sg)
        else:
            start = random_free_cell(grid, rng)
            goal  = random_free_cell(grid, rng)
            tries = 0
            while goal == start and tries < 10:
                goal = random_free_cell(grid, rng)
                tries += 1
        grid[start] = 0; grid[goal] = 0

        # ---- choose planner ----
        neighbor_fn = make_neighbor_fn(grid, moves)
        if algo == "bfs":
            planner = bfs.BFSPlanner()
        elif algo == "dijkstra":
            planner = dijkstra.DijkstraPlanner()
        elif algo == "greedy":
            planner = greedy.GreedyBestFirstPlanner()
        elif algo == "astar":
            planner = astar.AStarPlanner()
        elif algo == "weighted_astar":
            planner = weighted_astar.WeightedAStarPlanner(weight=weight)
        else:
            raise ValueError(f"Unknown algo '{algo}'")

        self.get_logger().info(f"[viz] {src}, algo={algo}, moves={moves}, weight={weight}")

        # ---- run and prepare messages ----
        res_plan = planner.plan(grid, start, goal, neighbor_fn=neighbor_fn)
        path = res_plan.path or []
        og   = to_occupancy_grid(grid, frame_id=frame, resolution=res)
        path_msg = path_to_msg(path, frame_id=frame, resolution=res)

        # start/goal markers
        sx = start[1] * res + 0.5 * res
        sy = start[0] * res + 0.5 * res
        gx = goal[1]  * res + 0.5 * res
        gy = goal[0]  * res + 0.5 * res
        markers = MarkerArray()
        markers.markers.append(point_marker(sx, sy, 1, (0.0, 1.0, 0.0, 1.0), frame_id=frame))  # start: green
        markers.markers.append(point_marker(gx, gy, 2, (1.0, 0.0, 0.0, 1.0), frame_id=frame))  # goal: red

        # ---- publishers (latching-like QoS) ----
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.grid_pub  = self.create_publisher(OccupancyGrid, "grid", latched)
        self.path_pub  = self.create_publisher(Path,          "path", latched)
        self.mk_pub    = self.create_publisher(MarkerArray,   "markers", latched)

        # publish once (RViz will latched-subscribe and render)
        self.grid_pub.publish(og)
        self.path_pub.publish(path_msg)
        self.mk_pub.publish(markers)

        self.get_logger().info(f"[viz] published grid/path/markers  path_len={len(path)}  nodes={res_plan.nodes_expanded}")

        # exit after a short delay so RViz has time to subscribe
        self.create_timer(1.0, self._shutdown_once)

    def _shutdown_once(self):
        self.get_logger().info("[viz] done; shutting down")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = PlannerVizNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

