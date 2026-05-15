import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Range
from geometry_msgs.msg import Twist
import time

class PIController:
    def __init__(self, kp: float, ki: float, output_limit: float):
        self.kp = kp
        self.ki = ki
        self.output_limit = output_limit
        self.integral = 0.0
        self.last_time = time.monotonic()

    def reset(self):
        self.integral = 0.0
        self.last_time = time.monotonic()

    def control(self, error: float) -> float:
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        dt = min(dt, 0.1)
        self.integral += error * dt

        k_i = self.ki * self.integral
        k_i = max(-self.output_limit, min(self.output_limit, k_i))

        output = self.kp * error + k_i
        output = max(-self.output_limit, min(self.output_limit, output))
        return output

class JoyVel(Node):
    def __init__(self):
        super().__init__('joy_vel')

        # Parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('deadman_button', 6)

        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('axis_angular_z', 2)

        self.declare_parameter('stop_dist',       0.35)
        self.declare_parameter('sensor_timeout',  0.5)

        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('max_range', 3.0)

        self.declare_parameter('safety_kp',    1.2)
        self.declare_parameter('safety_ki',    0.3)


        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.deadman_button = self.get_parameter('deadman_button').value

        self.axis_linear_x  = self.get_parameter('axis_linear_x').value
        self.axis_linear_y  = self.get_parameter('axis_linear_y').value
        self.axis_angular_z = self.get_parameter('axis_angular_z').value

        self.stop_dist       = self.get_parameter('stop_dist').value
        self.sensor_timeout  = self.get_parameter('sensor_timeout').value

        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value

        kp = self.get_parameter('safety_kp').value
        ki = self.get_parameter('safety_ki').value

        self.readings = {
            'middle': (float('inf'), 0.0),
            'left':   (float('inf'), 0.0),
            'right':  (float('inf'), 0.0),
        }

        self.pi_middle = PIController(kp, ki, self.linear_speed)
        self.pi_left   = PIController(kp, ki, self.linear_speed)
        self.pi_right  = PIController(kp, ki, self.linear_speed)

        self.create_subscription(Joy,   'joy',               self.joy_callback,    10)
        self.create_subscription(Range, 'ultrasonic/middle', lambda m: self.range_cb('middle', m), 10)
        self.create_subscription(Range, 'ultrasonic/left',   lambda m: self.range_cb('left',   m), 10)
        self.create_subscription(Range, 'ultrasonic/right',  lambda m: self.range_cb('right',  m), 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def range_cb(self, side: str, msg: Range):
        r = msg.range
        if r < self.min_range or r > self.max_range:
            r = float('inf')
        self.readings[side] = (r, time.monotonic())

    def fresh(self, side: str):
        r, t = self.readings[side]
        if (time.monotonic() - t) > self.sensor_timeout:
            return float('inf')
        return r

    def safety(self, vx: float, vy: float, wz: float):
        m = self.fresh('middle')
        l = self.fresh('left')
        r = self.fresh('right')

        if m < self.stop_dist:
            error = self.stop_dist - m
            correction = self.pi_middle.control(error)
            vx = vx - correction

        else:
            self.pi_middle.reset()


        if l < self.stop_dist:
            error = self.stop_dist - l
            correction = self.pi_left.control(error)
            vy = vy - correction
        else:
            self.pi_left.reset()

        if r < self.stop_dist:
            error = self.stop_dist - r
            correction = self.pi_right.control(error)
            vy = vy + correction
        else:
            self.pi_right.reset()

        vx = max(-self.linear_speed, min(self.linear_speed, vx))
        vy = max(-self.linear_speed, min(self.linear_speed, vy))

        return vx, vy, wz



    def joy_callback(self, msg):
        twist = Twist()

        if msg.buttons[self.deadman_button] == 1:
            vx = msg.axes[self.axis_linear_x]  * self.linear_speed
            vy = msg.axes[self.axis_linear_y]  * self.linear_speed
            wz = msg.axes[self.axis_angular_z] * self.angular_speed

            vx, vy, wz = self.safety(vx, vy, wz)
            twist.linear.x  = vx
            twist.linear.y  = vy
            twist.angular.z = wz
        else:
            twist.linear.x  = 0.0
            twist.linear.y  = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()