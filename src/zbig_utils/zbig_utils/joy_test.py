#!/usr/bin/env python3
# 12/4/2024
# for setting continuous constant speed of motors so can work on PID constants
#
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('joy_publisher')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        # axis 1 is motor speed (numbering starts at 0)
        #            0    1    2    3    4
        msg.axes = [0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        # button 5 is enable (numbering starts at 0)
        #              0  1  2  3  4  5  6  7
        msg.buttons = [0, 0, 0, 0, 0, 1, 0, 1, 0, 0]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()