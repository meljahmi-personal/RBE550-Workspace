import rclpy, time, random
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from .grid_utils import grid_from_occupancy, world_to_grid, grid_to_world, neighbors_4, neighbors_8, path_length, count_turns
from .rviz_helpers import path_marker, expansions_marker
from .metrics import Metrics, MetricLogger
from .algorithms.bfs import BFSPlanner
from .algorithms.astar import AStarPlanner
# from .algorithms.dijkstra import DijkstraPlanner
# from .algorithms.theta_star import ThetaStarPlanner
# from .algorithms.jps import JPSPlanner

class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")
        self.declare_parameter("planner", "astar")
        self.declare_parameter("connectivity", 8)     # 4 or 8
        self.declare_parameter("heuristic", "euclidean")
        self.declare_parameter("log_csv", True)
        self.declare_parameter("seed", 123)

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        self.start_sub = self.create_subscription(PoseStamped, "/start", self.on_start, 10)
        self.goal_sub  = self.create_subscription(PoseStamped, "/goal", self.on_goal, 10)
        self.path_pub  = self.create_publisher(Path, "/planned_path", 10)
        self.mark_pub  = self.create_publisher(MarkerArray, "/bench_markers", 10)
        self.metric_logger = MetricLogger() if self.get_parameter("log_csv").value else None

        self.map_msg = None
        self.obst = None
        self.info = None
        self.start_g = None
        self.goal_g = None

        random.seed(self.get_parameter("seed").value)

    def on_map(self, msg):
        self.obst, self.info = grid_from_occupancy(msg)
        self.map_msg = msg
        self.get_logger().info(f"Map received {self.info.width}x{self.info.height}")

    def on_start(self, msg: PoseStamped):
        if not self.info: return
        self.start_g = world_to_grid(msg.pose.position.x, msg.pose.position.y, self.info)
        self.try_plan()

    def on_goal(self, msg: PoseStamped):
        if not self.info: return
        self.goal_g = world_to_grid(msg.pose.position.x, msg.pose.position.y, self.info)
        self.try_plan()

    def select_planner(self):
        name = self.get_parameter("planner").value
        if name == "bfs":
            return BFSPlanner()
        elif name == "astar":
            return AStarPlanner(heuristic=self.get_parameter("heuristic").value)
        # elif name == "dijkstra":
        #     return DijkstraPlanner()
        # elif name == "theta_star":
        #     return ThetaStarPlanner(heuristic=self.get_parameter("heuristic").value)
        # elif name == "jps":
        #     return JPSPlanner()
        else:
            self.get_logger().warn("Unknown planner; defaulting to A*")
            return AStarPlanner()

    def try_plan(self):
        if self.obst is None or self.start_g is None or self.goal_g is None: return
        planner = self.select_planner()
        neigh = neighbors_8 if int(self.get_parameter("connectivity").value) == 8 else neighbors_4

        t0 = time.perf_counter()
        result = planner.plan(self.obst, self.start_g, self.goal_g, neigh)
        dt_ms = 1000.0 * (time.perf_counter() - t0)

        # Path -> world coords
        path_w = [grid_to_world(x, y, self.info) for (x,y) in result.path]

        # Publish nav_msgs/Path
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for x,y in path_w:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = x
            ps.pose.position.y = y
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

        # Publish markers (path + expansions as dots along path parent set for demo)
        ma = MarkerArray()
        ma.markers.append(path_marker(path_w, "map", 0))
        # For simplicity we plot only path points; if you log expansions during planning, add them here.
        self.mark_pub.publish(ma)

        # Metrics
        if self.metric_logger:
            plen = len(result.path)
            length_grid = 0.0
            if plen >= 2:
                # compute in grid coords
                import math
                length_grid = sum(math.hypot(x2-x1, y2-y1) for (x1,y1),(x2,y2) in zip(result.path[:-1], result.path[1:]))
            m = Metrics(
                planner=planner.name,
                success=result.success,
                runtime_ms=dt_ms,
                nodes_expanded=result.nodes_expanded,
                path_len_grid=length_grid,
                turns=0,  # you can compute turns via grid_utils.count_turns(result.path)
                peak_open=result.peak_open,
                peak_closed=result.peak_closed,
                grid_w=self.info.width,
                grid_h=self.info.height,
                density_pct=100.0 * self.obst.sum() / (self.info.width * self.info.height),
                seed=0
            )
            self.metric_logger.write(m)

        self.get_logger().info(f"[{planner.name}] success={result.success} time={dt_ms:.2f}ms nodes={result.nodes_expanded} open*={result.peak_open}")

def main():
    rclpy.init()
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

