#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from enum import Enum, auto
import math
import time

STOP_DIST_M      = 0.40
STOP_DIST_LR     = 0.20
CLEAR_DIST       = 0.55
REVERSE_DIST     = 0.20
SENSOR_TIMEOUT   = 0.5
 
FWD_SPEED        = 0.25
REV_SPEED        = -0.18
TURN_SPEED       = 0.8
TURN_MIN_SPEED   = 0.15
TURN_TOL_DEG     = 3.0
REVERSE_DURATION = 0.8
ROBOT_DIAMETER   = 0.80

KP            = 0.8
KI            = 0.15
KI_MAX        = 0.30

WALL_LOST_DIST     = 0.80
WALL_CORRIDOR_DIST = 0.25



class State(Enum):
    FORWARD     = auto()
    ASSESS      = auto()
    TURN        = auto()
    WALL_FOLLOW = auto()
    REVERSE     = auto()
    STOP        = auto()


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.declare_parameter('stop_dist_m',        STOP_DIST_M)
        self.declare_parameter('stop_dist_lr',       STOP_DIST_LR)
        self.declare_parameter('clear_dist',         CLEAR_DIST)
        self.declare_parameter('reverse_dist',       REVERSE_DIST)
        self.declare_parameter('fwd_speed',          FWD_SPEED)
        self.declare_parameter('turn_speed',         TURN_SPEED)
        self.declare_parameter('reverse_duration',   REVERSE_DURATION)
        self.declare_parameter('robot_diameter',     ROBOT_DIAMETER)
        self.declare_parameter('kp',                 KP)
        self.declare_parameter('ki',                 KI)
        self.declare_parameter('ki_max',             KI_MAX)
        self.declare_parameter('wall_lost_dist',     WALL_LOST_DIST)
        self.declare_parameter('wall_corridor_dist', WALL_CORRIDOR_DIST)
        self.declare_parameter('sensor_timeout', SENSOR_TIMEOUT)
 
        self.stop_dist_m        = self.get_parameter('stop_dist_m').value
        self.stop_dist_lr       = self.get_parameter('stop_dist_lr').value
        self.clear_dist         = self.get_parameter('clear_dist').value
        self.reverse_dist       = self.get_parameter('reverse_dist').value
        self.fwd_speed          = self.get_parameter('fwd_speed').value
        self.turn_speed         = self.get_parameter('turn_speed').value
        self.reverse_duration   = self.get_parameter('reverse_duration').value
        self.robot_diameter     = self.get_parameter('robot_diameter').value
        self.kp                 = self.get_parameter('kp').value
        self.ki                 = self.get_parameter('ki').value
        self.ki_max             = self.get_parameter('ki_max').value
        self.wall_lost_dist     = self.get_parameter('wall_lost_dist').value
        self.wall_corridor_dist = self.get_parameter('wall_corridor_dist').value
        self.sensor_timeout     = self.get_parameter('sensor_timeout').value

        now = time.monotonic()
        self.readings: dict[str, tuple[float, float]] = {
            'left':   (float('inf'), now),
            'right':  (float('inf'), now),
            'middle': (float('inf'), now),
        }

        self.yaw = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0

        self.target_yaw = 0.0

        self.wall_sensor: str   = 'left'
        self.wall_side_sign: float = 1.0
        self.wall_target_dist: float = 0.0
        self.start_x: float = 0.0
        self.start_y: float = 0.0

        self.pi_integral: float = 0.0
        self.pi_last_time: float = 0.0

        self.state      = State.FORWARD
        self.state_ts   = time.monotonic()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Range, 'ultrasonic/left', lambda m: self.range_callback('left',   m), qos)
        self.create_subscription(Range, 'ultrasonic/right', lambda m: self.range_callback('right',  m), qos)
        self.create_subscription(Range, 'ultrasonic/middle', lambda m: self.range_callback('middle', m), qos)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'ObstacleAvoidanceNode started | stop={self.stop_dist_m} m '
            f'clear_dist={self.clear_dist} m'
        )

    def odom_callback(self, msg: Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def range_callback(self, side: str, msg: Range):
        r = msg.range
        self.readings[side] = (r, time.monotonic())

    def fresh(self, side: str) -> float:
        r, ts = self.readings[side]
        if (time.monotonic() - ts) > self.sensor_timeout:
            return float('inf')
        return r


    def scale(self, dist: float) -> float:
        if dist >= self.clear_dist:
            return 1.0
        if dist <= self.stop_dist_m:
            return 0.0
        return 0.1 + ((dist - self.stop_dist_m) / (self.clear_dist - self.stop_dist_m) * 0.9)

    def limit_vx(self, vx: float) -> float:
        return vx*self.scale(self.fresh('middle'))

    def limit_vy(self, vy: float) -> float:
        if vy > 0.0:
            return vy * self.scale(self.fresh('left'))
        if vy < 0.0:
            return vy * self.scale(self.fresh('right'))
        return 0.0



    def publish(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x  = float(vx)
        t.linear.y  = float(vy)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def state_age(self) -> float:
        return time.monotonic() - self.state_ts

    def enter(self, newstate: State):
        if newstate != self.state:
            self.get_logger().info(
                f'{self.state.name} -> {newstate.name}'
            )
        self.state    = newstate
        self.state_ts = time.monotonic()

    def angle_diff(self, a, b) -> float:
        d = a - b
        while d >  math.pi: d -= 2.0 * math.pi
        while d < -math.pi: d += 2.0 * math.pi
        return d


    def travelled(self, x0: float, y0: float) -> float:
        return math.hypot(self.pos_x - x0, self.pos_y - y0)


    def assess(self):
        l = self.fresh('left')
        r = self.fresh('right')
        m = self.fresh('middle')

        if l < self.reverse_dist and r < self.reverse_dist and m < self.reverse_dist:
            self.enter(State.REVERSE)
            return

        if l >= r:
            turn_dir = +1.0
            wall_sensor = 'right'
            wall_side_sign = -1.0

        else:
            turn_dir = -1.0
            wall_sensor = 'left'
            wall_side_sign = +1.0

        raw = self.yaw + turn_dir * math.pi / 2.0
        self.target_yaw = (raw + math.pi) % (2.0 * math.pi) - math.pi

        side_now = self.fresh(wall_sensor)
        self.wall_sensor      = wall_sensor
        self.wall_side_sign   = wall_side_sign
        self.wall_target_dist = (side_now if not math.isinf(side_now) else self.stop_dist_lr)

        self.enter(State.TURN)

    def pi_reset(self):
        self.pi_integral  = 0.0
        self.pi_last_time = time.monotonic()

    def pi_update(self, wall_dist: float, stop=0.4):
        now = time.monotonic()
        dt  = now - self.pi_last_time
        self.pi_last_time = now
 
        if dt <= 0.0 or dt > 0.5:
            return 0.0
 
        dist_err = wall_dist - stop
 
        self.pi_integral += dist_err * dt
        self.pi_integral = max(-self.ki_max, min(self.ki_max, self.pi_integral))
 
        vy = self.wall_side_sign * (
            self.kp * dist_err +
            self.ki * self.pi_integral
        )
 
        return max(-self.fwd_speed, min(self.fwd_speed, vy))



    def control_loop(self):
        l = self.fresh('left')
        r = self.fresh('right')
        m = self.fresh('middle')

        any_stale = all(
            (time.monotonic() - ts) > self.sensor_timeout
            for _, ts in self.readings.values()
        )

        if any_stale:
            if self.state != State.STOP:
                self.get_logger().warn('All sensors stale — STOP')
                self.enter(State.STOP)

        if self.state == State.FORWARD:
            if m < self.stop_dist_m:
                self.get_logger().info(
                    f'ASSESS triggered | m={m:.2f} l={l:.2f} r={r:.2f}'
                )

                self.publish()
                self.enter(State.ASSESS)
            else:
                self.publish(vx=self.limit_vx(self.fwd_speed))

        elif self.state == State.ASSESS:
            self.publish()
            self.assess()

        elif self.state == State.TURN:
            err = self.angle_diff(self.target_yaw, self.yaw)
 
            if abs(err) < math.radians(TURN_TOL_DEG):
                self.start_x = self.pos_x
                self.start_y = self.pos_y
                self.pi_reset()
                self.enter(State.WALL_FOLLOW)
            else:
                speed = max(TURN_MIN_SPEED, min(self.turn_speed, abs(err) * 1.5))
                self.publish(wz=math.copysign(speed, err))

        elif self.state == State.WALL_FOLLOW:
            wall_dist = self.fresh(self.wall_sensor)
            travelled = self.travelled(self.start_x, self.start_y)

            wall_gone = math.isinf(wall_dist) or wall_dist > self.wall_lost_dist
            if wall_gone and travelled >= self.robot_diameter:
                self.enter(State.FORWARD)
                return
 
            if m < self.stop_dist_m:
                self.publish()
                self.enter(State.ASSESS)
                return

            if math.isinf(wall_dist) or wall_gone:
                self.publish(vx=self.fwd_speed)
            else:
                vy = self.pi_update(wall_dist, self.stop_dist_lr)
                self.publish(vx=self.fwd_speed, vy=vy)
                return
 
        elif self.state == State.REVERSE:
            self.publish(vx=REV_SPEED)
            if self.state_age() >= self.reverse_duration:
                self.enter(State.ASSESS)

        elif self.state == State.STOP:
            self.publish()
            if not math.isinf(l) or not math.isinf(r) or not math.isinf(m):
                self.get_logger().info('Sensors recovered — resuming FORWARD')
                self.enter(State.FORWARD)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
