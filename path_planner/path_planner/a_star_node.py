#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from queue import PriorityQueue
import numpy as np
from scipy.ndimage import distance_transform_edt
from std_srvs.srv import Empty
from std_msgs.msg import Bool


class GraphNode:
    """Simple helper class representing a grid cell."""
    def __init__(self, x, y, cost=0, heuristic=0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.prev = prev

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __add__(self, other):
        return GraphNode(self.x + other[0], self.y + other[1])


class AStarPlanner(Node):
    """A* global planner — plans once per goal, resets when controller confirms goal reached."""

    def __init__(self):
        super().__init__("a_star_node")

        # Parameters
        self.declare_parameter("safe_distance_m", 0.5)
        self.safe_distance_m = self.get_parameter("safe_distance_m").value

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS for map and path
        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = ReliabilityPolicy.RELIABLE

        # Publishers and subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, "/a_star/path", path_qos)
        self.visited_pub = self.create_publisher(OccupancyGrid, "/a_star/visited_map", 10)

        # Listen to controller's goal reached topic
        self.goal_reached_sub = self.create_subscription(Bool, "/goal_reached", self.goal_reached_callback, 10)

        # Service to manually clear goal
        self.create_service(Empty, "/a_star/clear_goal", self.clear_goal_callback)

        # Timer to republish last path for controller stability
        self.timer = self.create_timer(1.0, self.republish_path)

        # Internal state
        self.map_ = None
        self.dist_field = None
        self.visited_map_ = OccupancyGrid()
        self.has_active_goal = False
        self.current_goal = None
        self.last_path = None

        self.get_logger().info("✅ A* planner started — waiting for map and goal...")

    # --------------------------
    # MAP CALLBACK
    # --------------------------
    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.visited_map_.header.frame_id = map_msg.header.frame_id
        self.visited_map_.info = map_msg.info
        self.visited_map_.data = [-1] * (map_msg.info.height * map_msg.info.width)

        grid = np.array(map_msg.data, dtype=np.int8).reshape(map_msg.info.height, map_msg.info.width)
        obstacle_mask = grid > 50
        self.dist_field = distance_transform_edt(~obstacle_mask)

        self.get_logger().info("✅ Map received and distance field computed.")

    # --------------------------
    # GOAL CALLBACK
    # --------------------------
    def goal_callback(self, pose: PoseStamped):
        if self.has_active_goal:
            self.get_logger().warn("⚠️ Already executing a goal — ignoring new one.")
            return

        if self.map_ is None or self.dist_field is None:
            self.get_logger().warn("⚠️ Map not yet ready, cannot plan.")
            return

        self.current_goal = pose
        self.has_active_goal = True
        self.get_logger().info("🧭 Received new goal — computing global path...")
        self.plan_and_publish(pose)

    # --------------------------
    # CLEAR GOAL SERVICE
    # --------------------------
    def clear_goal_callback(self, request, response):
        self.reset_planner_state("🧹 Goal cleared via service — ready for next goal.")
        return response

    # --------------------------
    # GOAL REACHED CALLBACK (from controller)
    # --------------------------
    def goal_reached_callback(self, msg: Bool):
        if msg.data and self.has_active_goal:
            self.get_logger().info("🏁 Controller confirmed goal reached — resetting A* planner.")
            self.reset_planner_state()

    # --------------------------
    # RESET STATE
    # --------------------------
    def reset_planner_state(self, log_msg=None):
        self.has_active_goal = False
        self.current_goal = None
        self.last_path = None
        if log_msg:
            self.get_logger().info(log_msg)
        else:
            self.get_logger().info("🏁 Goal reached — resetting A* planner.")

    # --------------------------
    # PLAN AND PUBLISH PATH
    # --------------------------
    def plan_and_publish(self, goal_pose_stamped: PoseStamped):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, "base_footprint",
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)
            )
        except (LookupException, ExtrapolationException):
            self.get_logger().error("❌ Could not get transform map→base_footprint.")
            self.reset_planner_state()
            return

        start_pose = Pose()
        start_pose.position.x = tf.transform.translation.x
        start_pose.position.y = tf.transform.translation.y
        start_pose.orientation = tf.transform.rotation

        goal_pose = goal_pose_stamped.pose
        path = self.plan(start_pose, goal_pose)

        if path.poses:
            self.last_path = path
            self.get_logger().info(f"✅ Global path planned ({len(path.poses)} poses).")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("⚠️ No valid path found.")
            self.reset_planner_state()

    # --------------------------
    # PERIODIC RE-PUBLISH
    # --------------------------
    def republish_path(self):
        if self.last_path and self.has_active_goal:
            self.path_pub.publish(self.last_path)

    # --------------------------
    # A* PATH PLANNING
    # --------------------------
    def plan(self, start: Pose, goal: Pose):
        explore_dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        open_set = PriorityQueue()
        visited = set()

        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)
        start_node.heuristic = self.manhattan_distance(start_node, goal_node)
        open_set.put(start_node)
        visited.add(start_node)

        res = self.map_.info.resolution
        safe_distance_cells = int(self.safe_distance_m / res)
        active_node = None

        while not open_set.empty() and rclpy.ok():
            active_node = open_set.get()
            if active_node == goal_node:
                break

            for dx, dy in explore_dirs:
                neighbor = active_node + (dx, dy)
                if not self.pose_on_map(neighbor):
                    continue
                idx = self.pose_to_cell(neighbor)
                if self.map_.data[idx] != 0:
                    continue

                if neighbor not in visited:
                    base_cost = 1.0
                    d = self.dist_field[neighbor.y, neighbor.x]
                    penalty = 0.0
                    if d < safe_distance_cells:
                        penalty = (safe_distance_cells - d) * 2.0
                    neighbor.cost = active_node.cost + base_cost + penalty
                    neighbor.heuristic = self.manhattan_distance(neighbor, goal_node)
                    neighbor.prev = active_node
                    open_set.put(neighbor)
                    visited.add(neighbor)

        # Construct path
        path = Path()
        path.header.frame_id = self.map_.header.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        while active_node and active_node.prev and rclpy.ok():
            p = self.grid_to_world(active_node)
            ps = PoseStamped()
            ps.header.frame_id = path.header.frame_id
            ps.pose = p
            path.poses.append(ps)
            active_node = active_node.prev

        path.poses.reverse()
        return path

    # --------------------------
    # HELPERS
    # --------------------------
    def manhattan_distance(self, n1: GraphNode, n2: GraphNode):
        return abs(n1.x - n2.x) + abs(n1.y - n2.y)

    def pose_on_map(self, node: GraphNode):
        return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

    def world_to_grid(self, pose: Pose) -> GraphNode:
        x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return GraphNode(x, y)

    def grid_to_world(self, node: GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = node.x * self.map_.info.resolution + self.map_.info.origin.position.x
        pose.position.y = node.y * self.map_.info.resolution + self.map_.info.origin.position.y
        return pose

    def pose_to_cell(self, node: GraphNode):
        return node.y * self.map_.info.width + node.x


# --------------------------
# MAIN
# --------------------------
def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Planner stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
