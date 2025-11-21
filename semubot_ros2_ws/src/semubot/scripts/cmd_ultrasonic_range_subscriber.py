#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

class CmdRangeSubscriber(Node):
    
    def __init__(self):
        super().__init__("ultrasonic_range_subscriber")

        self.subscription = self.create_subscription(Int32, "cmd_range", self.print_to_terminal, 10)

    def print_to_terminal(self, msg):
        self.get_logger().info(f"Distance: {msg.data} cm")
        

def main(args=None):
    rclpy.init(args=args)
    node = CmdRangeSubscriber()

    rclpy.spin(node)

    rclpy.shutdown()
    


if __name__ == "__main__":
    main()