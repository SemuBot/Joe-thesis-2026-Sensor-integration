#!/usr/bin/env python3
"""
Ultrasonic Obstacle Avoidance Node — ROS 2 Jazzy
Robot:   Omnidirectional drive
Topics:  ultrasonic/left  | ultrasonic/right | ultrasonic/middle  (sensor_msgs/Range)
         cmd_vel  (geometry_msgs/Twist)

State machine:
    FORWARD  -> move forward until an obstacle is closer than STOP_DIST
    ASSESS   -> decide best escape direction from sensor snapshot
    TURN     -> rotate in-place until clear
    REVERSE  -> back up if front + sides are all blocked
    STOP     -> stop (no readings for too long)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from enum import Enum, auto
import math
import time


STOP_DIST      = 0.40
CLEAR_DIST     = 0.55
REVERSE_DIST   = 0.20
SENSOR_TIMEOUT = 0.5

FWD_SPEED      = 0.25
REV_SPEED      = -0.18
TURN_SPEED     = 0.8
SIDE_SPEED   = 0.20

TURN_DURATION  = 1.2
REVERSE_DURATION = 0.8


class State(Enum):
    FORWARD = auto()
    ASSESS  = auto()
    TURN    = auto()
    REVERSE = auto()
    STOP    = auto()


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.declare_parameter('stop_dist',         STOP_DIST)
        self.declare_parameter('clear_dist',        CLEAR_DIST)
        self.declare_parameter('reverse_dist',      REVERSE_DIST)
        self.declare_parameter('fwd_speed',         FWD_SPEED)
        self.declare_parameter('turn_speed',        TURN_SPEED)
        self.declare_parameter('side_speed',        SIDE_SPEED)
        self.declare_parameter('turn_duration',     TURN_DURATION)
        self.declare_parameter('reverse_duration',  REVERSE_DURATION)

        self.stop_dist       = self.get_parameter('stop_dist').value
        self.clear_dist      = self.get_parameter('clear_dist').value
        self.reverse_dist    = self.get_parameter('reverse_dist').value
        self.fwd_speed       = self.get_parameter('fwd_speed').value
        self.turn_speed      = self.get_parameter('turn_speed').value
        self.side_speed    = self.get_parameter('side_speed').value
        self.turn_duration   = self.get_parameter('turn_duration').value
        self.reverse_duration= self.get_parameter('reverse_duration').value

        self._readings: dict[str, tuple[float, float]] = {
            'left':   (float('inf'), 0.0),
            'right':  (float('inf'), 0.0),
            'middle': (float('inf'), 0.0),
        }

        self._state      = State.FORWARD
        self._state_ts   = time.monotonic()
        self._turn_dir   = 1.0 
        self._side_dir = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Range, 'ultrasonic/left', lambda m: self._range_callback('left',   m), qos)
        self.create_subscription(Range, 'ultrasonic/right', lambda m: self._range_callback('right',  m), qos)
        self.create_subscription(Range, 'ultrasonic/middle', lambda m: self._range_callback('middle', m), qos)

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'ObstacleAvoidanceNode started | stop={self.stop_dist} m '
            f'clear={self.clear_dist} m'
        )

    def _range_callback(self, side: str, msg: Range):
        r = msg.range
        self._readings[side] = (r, time.monotonic())

    def _fresh(self, side: str) -> float:
        r, ts = self._readings[side]
        if (time.monotonic() - ts) > SENSOR_TIMEOUT:
            return float('inf')
        return r

    def _publish(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x  = float(vx)
        t.linear.y  = float(vy)
        t.angular.z = float(wz)
        self._cmd_pub.publish(t)

    def _state_age(self) -> float:
        return time.monotonic() - self._state_ts

    def _enter(self, new_state: State):
        if new_state != self._state:
            self.get_logger().info(
                f'{self._state.name} -> {new_state.name}'
            )
        self._state    = new_state
        self._state_ts = time.monotonic()

    def _assess(self):
        """
        Snapshot all sensors and decide:
          -> if ALL blocked -> REVERSE
          -> if left is clearer -> +z
          -> if right is clearer -> turn -z
          -> if sides roughly equal -> pick random CCW
          For mecanum: also compute a side component.
        """
        l = self._fresh('left')
        r = self._fresh('right')
        m = self._fresh('middle')

        if l < self.reverse_dist and r < self.reverse_dist and m < self.reverse_dist:
            self._enter(State.REVERSE)
            return

        if l > r:
            self._turn_dir   =  1.0
            self._side_dir =  1.0
        else:
            self._turn_dir   = -1.0
            self._side_dir = -1.0

        self._enter(State.TURN)

    def _control_loop(self):
        l = self._fresh('left')
        r = self._fresh('right')
        m = self._fresh('middle')

        any_stale = all(
            (time.monotonic() - ts) > SENSOR_TIMEOUT
            for _, ts in self._readings.values()
        )

        if any_stale:
            if self._state != State.STOP:
                self.get_logger().warn('All sensors stale — STOP')
                self._enter(State.STOP)

        if self._state == State.FORWARD:
            if m < self.stop_dist or l < self.stop_dist or r < self.stop_dist:
                self._publish()
                self._enter(State.ASSESS)
            else:
                self._publish(vx=self.fwd_speed)

        elif self._state == State.ASSESS:
            self._publish()
            self._assess()

        elif self._state == State.TURN:
            self._publish(
                vy = self._side_dir * self.side_speed * 0.4,
                wz = self._turn_dir   * self.turn_speed,
            )

            if self._state_age() >= self.turn_duration:
                if m > self.clear_dist and l > self.clear_dist and r > self.clear_dist:
                    self._enter(State.FORWARD)
                else:
                    self._enter(State.ASSESS)

        elif self._state == State.REVERSE:
            self._publish(vx=REV_SPEED)
            if self._state_age() >= self.reverse_duration:
                self._enter(State.ASSESS)

        elif self._state == State.STOP:
            self._publish()

            l2 = self._fresh('left')
            r2 = self._fresh('right')
            m2 = self._fresh('middle')
            if not math.isinf(l2) or not math.isinf(r2) or not math.isinf(m2):
                self.get_logger().info('Sensors recovered — resuming FORWARD')
                self._enter(State.FORWARD)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
