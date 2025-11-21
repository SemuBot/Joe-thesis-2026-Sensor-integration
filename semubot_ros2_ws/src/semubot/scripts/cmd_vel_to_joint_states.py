#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class CmdVelToJointStates(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_joint_states')

        self.joint_pub = self.create_publisher(JointState, 'cmd_vel_joint_states', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.last_time = self.get_clock().now()
        self.wheel_positions = {
            'omni_ball_1_joint': 0.0,
            'omni_ball_2_joint': 0.0,
            'omni_ball_3_joint': 0.0,
        }

    def cmd_vel_callback(self, msg):
        # Get current time and calculate time difference
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Extract linear and angular velocities from the cmd_vel message
        v = msg.linear.x  # Linear velocity in x direction
        omega = msg.angular.z  # Angular velocity around z axis

        # Update wheel positions based on cmd_vel (you can fine-tune this conversion)
        self.wheel_positions['omni_ball_1_joint'] += (v + omega) * dt
        self.wheel_positions['omni_ball_2_joint'] += (v - omega) * dt
        self.wheel_positions['omni_ball_3_joint'] += (v * 0.5) * dt

        # Create a JointState message to publish the new joint positions
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = list(self.wheel_positions.keys())
        js.position = list(self.wheel_positions.values())

        # Publish joint states
        self.joint_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJointStates()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

