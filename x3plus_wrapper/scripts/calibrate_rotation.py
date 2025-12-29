#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
from rcl_interfaces.msg import SetParametersResult

def quat_to_yaw(q: Quaternion):
    """Convert quaternion to yaw (radians)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class YawCalibrator(Node):
    def __init__(self):
        super().__init__('yaw_calibrator')

        # Parameters
        self.declare_parameter('num_turns', 1.0)  # how many turns you commanded
        self.declare_parameter('compute_now', False)

        # Internal state
        self.prev_yaw = None
        self.unwrapped_yaw = 0.0
        self.initial_unwrapped = None

        # Subscriber
        self.sub = self.create_subscription(
            Odometry,
            '/wheel/odometry',
            self.odom_callback,
            10
        )

        # Parameter callback
        self.add_on_set_parameters_callback(self.param_callback)

        self.get_logger().info("YawCalibrator node started.")

    def odom_callback(self, msg: Odometry):
        yaw = quat_to_yaw(msg.pose.pose.orientation)

        if self.prev_yaw is None:
            self.prev_yaw = yaw
            self.initial_unwrapped = 0.0
            return

        dy = yaw - self.prev_yaw

        # Fix wraparound
        if dy > math.pi:
            dy -= 2 * math.pi
        elif dy < -math.pi:
            dy += 2 * math.pi

        self.unwrapped_yaw += dy
        self.prev_yaw = yaw

    def param_callback(self, params):
        """React to parameter updates."""
        for p in params:
            if p.name == 'compute_now' and p.value is True:
                self.compute_k_s()
                # Reset flag so it doesn't compute repeatedly
                self.set_parameters([rclpy.parameter.Parameter(
                    'compute_now',
                    rclpy.Parameter.Type.BOOL,
                    False
                )])
        return SetParametersResult(successful=True)

    def compute_k_s(self):
        num_turns = self.get_parameter('num_turns').value
        true_angle = num_turns * 2 * math.pi
        measured_angle = self.unwrapped_yaw - self.initial_unwrapped

        if abs(measured_angle) < 1e-6:
            self.get_logger().warn("Measured angle too small — cannot compute k_s.")
            return

        k_s = true_angle / measured_angle

        self.get_logger().info("----- Rotation Calibration Result -----")
        self.get_logger().info(f"Commanded turns: {num_turns}")
        self.get_logger().info(f"True angle:      {true_angle:.4f} rad")
        self.get_logger().info(f"Measured angle:  {measured_angle:.4f} rad")
        self.get_logger().info(f"k_s:             {k_s:.6f}")
        self.get_logger().info("---------------------------------------")

        # You can optionally publish or save this value here


def main(args=None):
    rclpy.init(args=args)
    node = YawCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
