#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.signal import savgol_filter
import math


class SavitzkyGolaySmootherNode(Node):
    def __init__(self):
        super().__init__('savitzky_golay_smoother')

        # Parameters
        self.declare_parameter('window_size', 7)
        self.declare_parameter('poly_order', 3)
        self.declare_parameter('do_refinement', True)
        self.declare_parameter('refinement_num', 2)

        # Load parameters
        self.window_size = self.get_parameter('window_size').value
        self.poly_order = self.get_parameter('poly_order').value
        self.do_refinement = self.get_parameter('do_refinement').value
        self.refinement_num = self.get_parameter('refinement_num').value

        # Subscribers and publishers
        self.path_sub = self.create_subscription(Path, '/input_path', self.path_callback, 10)
        self.path_pub = self.create_publisher(Path, '/smoothed_path', 10)

        self.get_logger().info("✅ Savitzky-Golay Smoother Node initialized.")

    def path_callback(self, msg: Path):
        n = len(msg.poses)
        if n < 2:
            self.get_logger().warn("Received path too short to smooth (less than 2 poses).")
            return

        # --- Adaptive window size ---
        win_size = min(self.window_size, n if n % 2 == 1 else n - 1)
        if win_size < 3:
            win_size = 3  # minimum valid window
        poly_order = min(self.poly_order, win_size - 1)

        # Extract coordinates
        x = np.array([p.pose.position.x for p in msg.poses])
        y = np.array([p.pose.position.y for p in msg.poses])

        # If path is very short, do simple linear interpolation
        if n < 5:
            smoothed_x = np.linspace(x[0], x[-1], n)
            smoothed_y = np.linspace(y[0], y[-1], n)
        else:
            smoothed_x = savgol_filter(x, win_size, poly_order)
            smoothed_y = savgol_filter(y, win_size, poly_order)
            if self.do_refinement:
                for _ in range(self.refinement_num):
                    smoothed_x = savgol_filter(smoothed_x, win_size, poly_order)
                    smoothed_y = savgol_filter(smoothed_y, win_size, poly_order)

        # --- Build smoothed path ---
        smoothed_path = Path()
        smoothed_path.header = msg.header

        for i in range(n):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(smoothed_x[i])
            pose.pose.position.y = float(smoothed_y[i])

            # Orientation from local slope
            if i < n - 1:
                dx = smoothed_x[i + 1] - smoothed_x[i]
                dy = smoothed_y[i + 1] - smoothed_y[i]
            else:
                dx = smoothed_x[i] - smoothed_x[i - 1]
                dy = smoothed_y[i] - smoothed_y[i - 1]

            yaw = math.atan2(dy, dx)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            smoothed_path.poses.append(pose)

        self.path_pub.publish(smoothed_path)
        self.get_logger().info(f"✅ Published smoothed path with {n} points (window={win_size}).")


def main(args=None):
    rclpy.init(args=args)
    node = SavitzkyGolaySmootherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
