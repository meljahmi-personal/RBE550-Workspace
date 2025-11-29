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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def make_neighbor_fn(grid, moves: int):
    """Create a neighbor function that returns valid (r, c, cost) tuples"""
    rows, cols = grid.shape
    
    def is_valid(r, c):
        """Check if (r, c) is within grid bounds and not an obstacle"""
        return (0 <= r < rows and 0 <= c < cols and grid[r, c] == 0)
    
    if moves == 4:
        def neighbor_fn_4(rc):
            r, c = rc
            for nr, nc, cost in neighbors4(r, c):
                if is_valid(nr, nc):
                    yield nr, nc, cost
        return neighbor_fn_4
        
    elif moves == 8:
        def neighbor_fn_8(rc):
            r, c = rc
            for nr, nc, cost in neighbors8(r, c):
                if is_valid(nr, nc):
                    yield nr, nc, cost
        return neighbor_fn_8
    else:
        raise ValueError(f"moves must be 4 or 8, got {moves}")


def to_occupancy_grid(grid: np.ndarray, frame_id="map", resolution=1.0):
    h, w = grid.shape
    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    
    msg.info = MapMetaData()
    msg.info.resolution = float(resolution)
    msg.info.width = int(w)
    msg.info.height = int(h)
    
    # Center the grid in RViz view - THIS IS THE KEY FIX
    msg.info.origin.position.x = -w * resolution / 2.0
    msg.info.origin.position.y = -h * resolution / 2.0
    msg.info.origin.position.z = 0.0
    msg.info.origin.orientation.w = 1.0
    
    # ROS OccupancyGrid is row-major from map origin (bottom-left)
    # Our grid is [row, col] with (0,0) at top-left. Flip vertically
    flipped = np.flipud(grid.astype(np.int8))
    data = (flipped * 100).astype(np.int8)
    msg.data = data.flatten().tolist()
    
    return msg


def path_to_msg(path_rc, frame_id="map", resolution=1.0):
    p = Path()
    p.header.frame_id = frame_id
    p.header.stamp = rclpy.clock.Clock().now().to_msg()
    for (r, c) in path_rc:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = p.header.stamp
        # Center coordinates around origin
        ps.pose.position.x = float(c) * resolution + 0.5 * resolution - 32.0
        ps.pose.position.y = float(r) * resolution + 0.5 * resolution - 32.0
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p

def point_marker(x, y, mid, rgba, scale=0.35, frame_id="map"):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rclpy.clock.Clock().now().to_msg()
    m.id = mid
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    # Coordinates should already be centered when passed in
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.orientation.w = 1.0
    m.scale.x = scale
    m.scale.y = scale
    m.scale.z = scale
    
    m.color.r = rgba[0]
    m.color.g = rgba[1]  
    m.color.b = rgba[2]
    m.color.a = rgba[3]
    
    m.lifetime.sec = 0
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

        # === ADD DEBUG CODE RIGHT HERE ===
        self.get_logger().info(f"=== GRID DEBUG INFO ===")
        self.get_logger().info(f"Grid type: {type(grid)}")
        self.get_logger().info(f"Grid dtype: {grid.dtype}")
        self.get_logger().info(f"Grid shape: {grid.shape}")
        self.get_logger().info(f"Min value: {np.min(grid)}")
        self.get_logger().info(f"Max value: {np.max(grid)}")
        self.get_logger().info(f"Sum of obstacles: {np.sum(grid)}")
        self.get_logger().info(f"Unique values: {np.unique(grid)}")

        # Check if the grid actually has obstacles
        if np.sum(grid) == 0:
            self.get_logger().warning("WARNING: Grid has NO obstacles!")
        else:
            self.get_logger().info(f"Grid has {np.sum(grid)} obstacles")

        # Print a small sample
        self.get_logger().info(f"Top-left 5x5 sample:\n{grid[:5, :5]}")
        self.get_logger().info(f"=== END GRID DEBUG ===")
        # === END DEBUG CODE ===

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
        
        # Store messages as instance variables
        self.og = to_occupancy_grid(grid, frame_id=frame, resolution=res)
        self.path_msg = path_to_msg(path, frame_id=frame, resolution=res)

        # start/goal markers
        sx = start[1] * res + 0.5 * res - 32.0  # Centered
        sy = start[0] * res + 0.5 * res - 32.0  # Centered
        gx = goal[1]  * res + 0.5 * res - 32.0  # Centered
        gy = goal[0]  * res + 0.5 * res - 32.0  # Centered
        self.markers = MarkerArray()
        self.markers.markers.append(point_marker(sx, sy, 1, (0.0, 1.0, 0.0, 1.0), frame_id=frame))  # start: green
        self.markers.markers.append(point_marker(gx, gy, 2, (1.0, 0.0, 0.0, 1.0), frame_id=frame))  # goal: red

        # ---- publishers (use default QoS) ----
        self.grid_pub  = self.create_publisher(OccupancyGrid, "grid", 10)  # Default QoS
        self.path_pub  = self.create_publisher(Path, "path", 10)           # Default QoS
        self.mk_pub    = self.create_publisher(MarkerArray, "markers", 10) # Default QoS

        # Publish immediately
        self.publish_data()
        
        # Also publish periodically to ensure RViz gets it
        self.timer = self.create_timer(1.0, self.publish_data)  # Publish every second

        self.get_logger().info(f"[viz] published grid/path/markers  path_len={len(path)}  nodes={res_plan.nodes_expanded}")

    def publish_data(self):
        """Publish the visualization data with current timestamp"""
        current_time = self.get_clock().now().to_msg()
        
        # Update timestamps
        self.og.header.stamp = current_time
        self.path_msg.header.stamp = current_time
        for marker in self.markers.markers:
            marker.header.stamp = current_time
        
        # Publish
        self.grid_pub.publish(self.og)
        self.path_pub.publish(self.path_msg)
        self.mk_pub.publish(self.markers)
        
        self.get_logger().info("Published visualization data")


def main():
    rclpy.init()
    node = PlannerVizNode()
    
    try:
        # Keep the node alive for a while to ensure RViz receives messages
        rclpy.spin_once(node, timeout_sec=2.0)  # Wait a bit for publications
        node.get_logger().info("Published visualization data, keeping node alive...")
        
        # Keep spinning until manually stopped
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

