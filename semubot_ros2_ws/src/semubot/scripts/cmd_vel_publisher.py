#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from std_msgs.msg import Int32

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        #self.timer = self.create_timer(1.0, self.publish_cmd_vel)

        self.subscription = self.create_subscription(Int32, "cmd_range", self.publish_cmd_vel, 10)


    def publish_cmd_vel(self, distance):
        msg = Twist()
        if distance.data > 0 and distance.data < 10:
            msg.linear.x = 0.0  # Move forward (backward in our case)
            msg.angular.z = 1.0  # No rotation
        else:
            msg.linear.x = -0.5  # Move forward (backward in our case)
            msg.angular.z = 0.0  # No rotation

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}, distance={distance.data}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

