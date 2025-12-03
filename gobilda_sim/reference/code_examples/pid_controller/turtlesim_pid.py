#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        # Target position and orientation
        self.target_x = 5.0
        self.target_y = 5.0
        self.target_theta = 0.0

        # PID gains
        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.1

        self.kp_angular = 1.0
        self.ki_angular = 0.0
        self.kd_angular = 0.1

        # Errors and integrals
        self.prev_error_linear = 0.0
        self.integral_linear = 0.0

        self.prev_error_angular = 0.0
        self.integral_angular = 0.0

        # Subscriber and Publisher
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def pose_callback(self, pose):
        # Calculate linear error (distance to target)
        error_linear = math.sqrt((self.target_x - pose.x)**2 + (self.target_y - pose.y)**2)

        # Calculate angular error (angle to target)
        target_angle = math.atan2(self.target_y - pose.y, self.target_x - pose.x)
        error_angular = target_angle - pose.theta
        error_angular = math.atan2(math.sin(error_angular), math.cos(error_angular))  # Normalize angle

        # PID for linear velocity
        self.integral_linear += error_linear
        derivative_linear = error_linear - self.prev_error_linear
        linear_velocity = (self.kp_linear * error_linear) + (self.ki_linear * self.integral_linear) + (self.kd_linear * derivative_linear)
        self.prev_error_linear = error_linear

        # PID for angular velocity
        self.integral_angular += error_angular
        derivative_angular = error_angular - self.prev_error_angular
        angular_velocity = (self.kp_angular * error_angular) + (self.ki_angular * self.integral_angular) + (self.kd_angular * derivative_angular)
        self.prev_error_angular = error_angular

        # Publish velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        self.velocity_publisher.publish(cmd_vel)

        # Logging
        self.get_logger().info(f'Error Linear: {error_linear}, Error Angular: {error_angular}')
        self.get_logger().info(f'Linear Vel: {linear_velocity}, Angular Vel: {angular_velocity}')

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()