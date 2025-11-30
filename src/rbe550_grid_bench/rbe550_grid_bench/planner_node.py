# rbe550_grid_bench/planner_node.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_srvs.srv import Trigger
import random
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from .grid_utils import make_random_grid, load_grid_from_ascii, random_free_cell, parse_rc_pair
from .neighbors import neighbors4, neighbors8
from .algorithms import bfs, dijkstra, greedy, astar, weighted_astar, theta, jps
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


#above grid map
#def text_marker(
#    text, mid,
#    rgba=(1.0, 0.2, 0.8, 1.0),  # Bright pink!
#    frame_id="map",
#    x=0.0, y=0.0, z=35.0,       # Position ABOVE the grid
#    scale=3.5                    # Bigger text!
#):

#below grid map
def text_marker(
    text, mid,
     rgba=(1.0, 0.2, 0.8, 1.0),
    frame_id="map",
    x=0.0, y=0.0, z=-55.0,    # Position BELOW the grid
    scale=5.0
):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rclpy.clock.Clock().now().to_msg()
    m.id = mid
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD

    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z

    m.scale.z = scale  # Bigger scale for larger text

    m.color.r = rgba[0]  # 1.0 = bright red component
    m.color.g = rgba[1]  # 0.2 = low green  
    m.color.b = rgba[2]  # 0.8 = high blue = pink!
    m.color.a = rgba[3]  # 1.0 = fully opaque

    m.text = text
    m.lifetime.sec = 0
    return m



class PlannerVizNode(Node):
    def __init__(self):
        super().__init__("planner_viz")


        # ---- params (make these launch-configurable) ----
        self.declare_parameter("algo", "astar")                # bfs|dijkstra|greedy|astar|weighted_astar|theta_star|jps
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
        
        self.algo = algo
        self.moves = moves
        self.weight = weight
        self.size = size
        self.fill = fill
        self.map_p = map_p
        self.sg = sg
        self.res = res
        self.frame = frame


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
        elif algo == "theta_star":
            # Theta* - any-angle paths
            planner = theta.ThetaStarPlanner()
        elif algo == "jps":
            # Jump Point Search - optimization for uniform-cost grids
            planner = jps.JumpPointSearchPlanner()
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
        # Use the actual grid shape instead of hard-coded 32.0
        grid_h, grid_w = grid.shape
        offset_x = 0.5 * grid_w * res
        offset_y = 0.5 * grid_h * res

        sx = start[1] * res + 0.5 * res - offset_x
        sy = start[0] * res + 0.5 * res - offset_y
        gx = goal[1]  * res + 0.5 * res - offset_x
        gy = goal[0]  * res + 0.5 * res - offset_y

 
             
        # In both __init__ and handle_randomize_grid, replace the info_text with:
        info_text = (
            f"{algo.upper()} | Moves: {moves} | Grid: {size}x{size} | "
            f"Path: {len(path)} | Nodes: {res_plan.nodes_expanded}"
        )
        
        
        # Hot pink - very bright
        #rgba=(1.0, 0.0, 0.8, 1.0)

        # Softer pink  
        #rgba=(1.0, 0.4, 0.7, 1.0)

        # Neon pink
        #rgba=(1.0, 0.1, 0.9, 1.0)
        
        self.markers = MarkerArray()
        # Only add text marker (no background)
        self.markers.markers.append(text_marker(info_text, 11, frame_id=frame))

        # Start and goal markers
        self.markers.markers.append(point_marker(sx, sy, 1, (0.0, 1.0, 0.0, 1.0), frame_id=frame))
        self.markers.markers.append(point_marker(gx, gy, 2, (1.0, 0.0, 0.0, 1.0), frame_id=frame))

        # ---- publishers (use default QoS) ----
        self.grid_pub  = self.create_publisher(OccupancyGrid, "grid", 10)  # Default QoS
        self.path_pub  = self.create_publisher(Path, "path", 10)           # Default QoS
        self.mk_pub    = self.create_publisher(MarkerArray, "markers", 10) # Default QoS

        # Publish immediately
        self.publish_data()
        
        # Also publish periodically to ensure RViz gets it
        self.timer = self.create_timer(1.0, self.publish_data)  # Publish every second
        
        # Service to randomize grid and replan
        self.randomize_srv = self.create_service(
            Trigger,
            'randomize_grid',
            self.handle_randomize_grid
        )


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
        
        
    def handle_randomize_grid(self, request, response):
        # pick a new seed
        new_seed = random.randint(0, 10_000_000)
        self.get_logger().info(f"[viz] randomize_grid seed={new_seed}")

        # rebuild grid
        if self.map_p:
            grid = load_grid_from_ascii(self.map_p)
            src = f"map='{self.map_p}'"
        else:
            grid = make_random_grid(size=self.size, fill=self.fill, seed=new_seed)
            src = f"grid={self.size}, fill={self.fill}, seed={new_seed}"

        rng = np.random.default_rng(new_seed)
        if self.sg:
            start, goal = parse_rc_pair(self.sg)
        else:
            start = random_free_cell(grid, rng)
            goal  = random_free_cell(grid, rng)
            tries = 0
            while goal == start and tries < 10:
                goal = random_free_cell(grid, rng)
                tries += 1
        grid[start] = 0
        grid[goal] = 0

        # *** read latest algo / moves from parameters ***
        algo  = self.get_parameter("algo").get_parameter_value().string_value
        moves = int(self.get_parameter("moves").value)

        # planner (same as in __init__, but use algo/moves we just read)
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
            planner = weighted_astar.WeightedAStarPlanner(weight=self.weight)
        elif algo == "theta_star":
            planner = theta.ThetaStarPlanner()
        elif algo == "jps":
            planner = jps.JumpPointSearchPlanner()
        else:
            raise ValueError(f"Unknown algo '{algo}'")

        res_plan = planner.plan(grid, start, goal, neighbor_fn=neighbor_fn)
        path = res_plan.path or []

        # rebuild messages
        self.og = to_occupancy_grid(grid, frame_id=self.frame, resolution=self.res)
        self.path_msg = path_to_msg(path, frame_id=self.frame, resolution=self.res)

        # --- center start/goal using **actual** grid size ---
        grid_h, grid_w = grid.shape
        offset_x = 0.5 * grid_w * self.res
        offset_y = 0.5 * grid_h * self.res

        sx = start[1] * self.res + 0.5 * self.res - offset_x
        sy = start[0] * self.res + 0.5 * self.res - offset_y
        gx = goal[1]  * self.res + 0.5 * self.res - offset_x
        gy = goal[0]  * self.res + 0.5 * self.res - offset_y

        # updated info text (uses algo/moves we just read)
        info_text = (
            f"{algo.upper()} | Moves: {moves} | Grid: {grid_w}x{grid_h} | "
            f"Path: {len(path)} | Nodes: {res_plan.nodes_expanded}"
        )

        self.markers = MarkerArray()
        self.markers.markers.append(text_marker(info_text, 11, frame_id=self.frame))
        self.markers.markers.append(
            point_marker(sx, sy, 1, (0.0, 1.0, 0.0, 1.0), frame_id=self.frame)
        )
        self.markers.markers.append(
            point_marker(gx, gy, 2, (1.0, 0.0, 0.0, 1.0), frame_id=self.frame)
        )

        # publish immediately so RViz refreshes
        self.publish_data()

        self.get_logger().info(
            f"[viz randomize] {src}, algo={algo}, moves={moves}, path_len={len(path)}, nodes={res_plan.nodes_expanded}"
        )

        response.success = True
        response.message = f"Randomized grid with seed={new_seed}"
        return response



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

