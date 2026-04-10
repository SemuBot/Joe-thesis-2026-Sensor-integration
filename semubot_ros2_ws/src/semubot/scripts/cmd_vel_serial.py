#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports

class CmdVelSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_serial')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.send_to_serial, 10)
        
        # Dynamically find available serial port
        port = self.find_serial_port()
        if port is None:
            self.get_logger().error('No valid serial port found. Exiting.')
            rclpy.shutdown()
            return

        try:
            self.serial_port = serial.Serial(port, 115200, timeout=1)
            self.get_logger().info(f'Using serial port: {self.serial_port.name}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            rclpy.shutdown()

    def find_serial_port(self):
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if port.device in ['/dev/ttyACM0', '/dev/ttyACM1']:
                return port.device
        return None

    def send_to_serial(self, msg):
        data = f"{{linear_x:{msg.linear.x:.2f},angular_z:{msg.angular.z:.2f}}}\n"
        self.serial_port.write(data.encode())
        self.get_logger().info(f'Sent to Nucleo: {data.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerial()
    if hasattr(node, 'serial_port'):
        rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

