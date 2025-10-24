#!/usr/bin/env python3
"""
Time Parameterization Node
==========================

Subscribes to:  /smoothed_path   (nav_msgs/Path)
Publishes:      /time_trajectory (nav_msgs/Path)

It assigns timestamps to each pose along the smoothed path using
a constant-velocity model (or any speed parameter you choose).
This completes Step 2 of your assignment: trajectory generation.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class TimeParameterizer(Node):
    def __init__(self):
        super().__init__("time_parameterizer_node")

        # --- Parameters ---
        self.declare_parameter("target_speed", 0.25)  # m/s
        self.declare_parameter("frame_id", "map")

        self.v = self.get_parameter("target_speed").value
        self.frame_id = self.get_parameter("frame_id").value

        # --- ROS Interfaces ---
        self.path_sub = self.create_subscription(Path, "/smoothed_path", self.path_callback, 10)
        self.traj_pub = self.create_publisher(Path, "/time_trajectory", 10)

        self.get_logger().info("✅ Time Parameterizer Node initialized.")

    # ----------------------------------------------------------------------
    def path_callback(self, msg: Path):
        poses = msg.poses
        if len(poses) < 2:
            self.get_logger().warn("Received too few points to parameterize.")
            return

        # Compute distances and assign times
        trajectory = Path()
        trajectory.header.frame_id = self.frame_id
        trajectory.header.stamp = self.get_clock().now().to_msg()

        t = 0.0
        for i in range(len(poses)):
            pose = PoseStamped()
            pose.header = trajectory.header
            pose.pose = poses[i].pose
            trajectory.poses.append(pose)

            if i > 0:
                x0, y0 = poses[i - 1].pose.position.x, poses[i - 1].pose.position.y
                x1, y1 = poses[i].pose.position.x, poses[i].pose.position.y
                dist = math.hypot(x1 - x0, y1 - y0)
                dt = dist / self.v if self.v > 1e-6 else 0.0
                t += dt

            # store t in the header stamp (for visualization / debugging)
            pose.header.stamp.sec = int(t)
            pose.header.stamp.nanosec = int((t % 1.0) * 1e9)

        # Publish result
        self.traj_pub.publish(trajectory)
        self.get_logger().info(
            f"Published time-parameterized trajectory with {len(trajectory.poses)} points "
            f"(duration ≈ {t:.2f}s)."
        )


def main(args=None):
    rclpy.init(args=args)
    node = TimeParameterizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
