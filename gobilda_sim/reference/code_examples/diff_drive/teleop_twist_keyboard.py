#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys
import select

import tty
import termios

''' A node that reads in keyboard inputs to control the differential drive robot
that we simulate using gazebo.
'''

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, 'diff_drive/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.twist = Twist()
        self.linear_velocity = 0.5
        self.angular_velocity = 0.35

        self.get_logger().info('Instructions for running node')
        self.get_logger().info('Press Keys: w/s ---> forward/backward')
        self.get_logger().info('Press Keys: a/d ---> left/right')
        self.get_logger().info('Press any key to make robot stop')
        self.get_logger().info('Press Key q to EXIT node')

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            input = select.select([sys.stdin], [], [], 0.1)[0]
            if input:
                return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return None

    def timer_callback(self):
        key = self.get_key()
        if key is None:
            return

        if key == 'w':
            self.twist.linear.x = self.linear_velocity
        elif key == 's':
            self.twist.linear.x = -self.linear_velocity
        elif key == 'a':
            self.twist.angular.z = self.angular_velocity
        elif key == 'd':
            self.twist.angular.z = -self.angular_velocity
        elif key == 'q':
            self.destroy_node()  # Exit the node
            return
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    
    teleop_keyboard = TeleopKeyboard()
    
    rclpy.spin(teleop_keyboard)
    
    teleop_keyboard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
