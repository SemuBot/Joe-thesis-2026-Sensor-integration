#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from enum import Enum, auto
import math
import time

STOP_DIST         = 0.35   # m — middle/side sensor triggers avoidance
CLEAR_DIST        = 0.80   # m — side sensor > this = back edge passed
GOAL_RADIUS       = 0.15   # m
SENSOR_TIMEOUT    = 0.5    # s
FWD_SPEED         = 0.20   # m/s
TURN_SPEED        = 0.4    # rad/s
WALL_FWD_SPEED    = 0.15   # m/s
WALL_KP           = 0.8    # P gain for distance holding
HEADING_KP        = 1.0
SCAN_SETTLE_TIME  = 0.3    # s
MIN_DRIVE_DIST    = 0.5    # m — minimum travel before checking back edge
STRAFE_SPEED      = 0.15   # m/s — strafe in FORWARD if obstacle still close
PERP_TOL          = 0.05   # m — tolerance for side sensor matching hit distance


class State(Enum):
    IDLE            = auto()
    FORWARD         = auto()
    ASSESS_LEFT     = auto()
    ASSESS_RIGHT    = auto()
    ASSESS_RETURN   = auto()
    BOUNDARY_TURN   = auto()
    BOUNDARY_DRIVE  = auto()
    BOUNDARY_RETURN  = auto()
    CLEAR_OBSTACLE   = auto()   # move away if obstacle still close after returning
    GOAL_REACHED     = auto()


class Bug2Node(Node):

    def __init__(self):
        super().__init__('bug2_node')

        self.declare_parameter('stop_dist',        STOP_DIST)
        self.declare_parameter('clear_dist',       CLEAR_DIST)
        self.declare_parameter('goal_radius',      GOAL_RADIUS)
        self.declare_parameter('fwd_speed',        FWD_SPEED)
        self.declare_parameter('turn_speed',       TURN_SPEED)
        self.declare_parameter('wall_fwd_speed',   WALL_FWD_SPEED)
        self.declare_parameter('wall_kp',          WALL_KP)
        self.declare_parameter('heading_kp',       HEADING_KP)
        self.declare_parameter('scan_settle_time', SCAN_SETTLE_TIME)
        self.declare_parameter('min_drive_dist',   MIN_DRIVE_DIST)
        self.declare_parameter('strafe_speed',     STRAFE_SPEED)
        self.declare_parameter('perp_tol',         PERP_TOL)

        self.stop_dist        = self.get_parameter('stop_dist').value
        self.clear_dist       = self.get_parameter('clear_dist').value
        self.goal_radius      = self.get_parameter('goal_radius').value
        self.fwd_speed        = self.get_parameter('fwd_speed').value
        self.turn_speed       = self.get_parameter('turn_speed').value
        self.wall_fwd_speed   = self.get_parameter('wall_fwd_speed').value
        self.wall_kp          = self.get_parameter('wall_kp').value
        self.heading_kp       = self.get_parameter('heading_kp').value
        self.scan_settle_time = self.get_parameter('scan_settle_time').value
        self.min_drive_dist   = self.get_parameter('min_drive_dist').value
        self.strafe_speed     = self.get_parameter('strafe_speed').value
        self.perp_tol         = self.get_parameter('perp_tol').value

        # ── robot pose ─────────────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        # ── goal ───────────────────────────────────────────────────────────────
        self.goal_x  = None
        self.goal_y  = None
        self.start_x = 0.0
        self.start_y = 0.0

        # ── boundary ───────────────────────────────────────────────────────────
        self.wall_side              = 1.0    # +1 right wall, -1 left wall
        self.hit_dist               = 0.30   # middle sensor reading at hit point
        self.boundary_return_target = 0.0
        self.boundary_drive_dist    = 0.0
        self.boundary_drive_last_x  = 0.0
        self.boundary_drive_last_y  = 0.0

        # ── assess scan ────────────────────────────────────────────────────────
        self.scan_origin_yaw  = 0.0
        self.scan_target_yaw  = 0.0
        self.scan_settled     = False
        self.scan_settle_ts   = 0.0
        self.scan_left_dist   = float('inf')
        self.scan_right_dist  = float('inf')

        # ── sensor readings ────────────────────────────────────────────────────
        self.readings: dict[str, tuple[float, float]] = {
            'left':   (float('inf'), 0.0),
            'right':  (float('inf'), 0.0),
            'middle': (float('inf'), 0.0),
        }

        self.state    = State.IDLE
        self.state_ts = time.monotonic()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Range, 'ultrasonic/middle', self.middle_callback, qos)
        self.create_subscription(Range, 'ultrasonic/left',   self.left_callback,   qos)
        self.create_subscription(Range, 'ultrasonic/right',  self.right_callback,  qos)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(Point,    'goal', self.goal_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            'Bug2Node ready.\n'
            'ros2 topic pub --once /goal geometry_msgs/msg/Point '
            '"{x: 2.0, y: 0.0, z: 0.0}"'
        )

    # ── callbacks ──────────────────────────────────────────────────────────────

    def left_callback(self, msg: Range):
        r = msg.range
        if r < msg.min_range or r > msg.max_range:
            r = float('inf')
        self.readings['left'] = (r, time.monotonic())

    def right_callback(self, msg: Range):
        r = msg.range
        if r < msg.min_range or r > msg.max_range:
            r = float('inf')
        self.readings['right'] = (r, time.monotonic())

    def middle_callback(self, msg: Range):
        r = msg.range
        if r < msg.min_range or r > msg.max_range:
            r = float('inf')
        self.readings['middle'] = (r, time.monotonic())

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def goal_callback(self, msg: Point):
        self.goal_x  = self.x + msg.x
        self.goal_y  = self.y + msg.y
        self.start_x = self.x
        self.start_y = self.y
        self.get_logger().info(
            f'Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}) | '
            f'Start: ({self.start_x:.2f}, {self.start_y:.2f})'
        )
        self.enter(State.FORWARD)

    # ── helpers ────────────────────────────────────────────────────────────────

    def fresh(self, side: str) -> float:
        r, ts = self.readings[side]
        if (time.monotonic() - ts) > SENSOR_TIMEOUT:
            return float('inf')
        return r

    def publish(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x  = float(vx)
        t.linear.y  = float(vy)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def enter(self, new_state: State):
        if new_state != self.state:
            self.get_logger().info(f'{self.state.name} → {new_state.name}')
        self.state    = new_state
        self.state_ts = time.monotonic()

    def state_age(self) -> float:
        return time.monotonic() - self.state_ts

    def wrap_yaw(self, yaw: float) -> float:
        while yaw >  math.pi: yaw -= 2.0 * math.pi
        while yaw < -math.pi: yaw += 2.0 * math.pi
        return yaw

    def distance_to_goal(self) -> float:
        if self.goal_x is None:
            return float('inf')
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def angle_to_goal(self) -> float:
        if self.goal_x is None:
            return 0.0
        return math.atan2(self.goal_y - self.y, self.goal_x - self.x)

    def heading_error(self) -> float:
        err = self.angle_to_goal() - self.yaw
        while err >  math.pi: err -= 2.0 * math.pi
        while err < -math.pi: err += 2.0 * math.pi
        return err

    def turn_to_yaw(self, target_yaw: float) -> bool:
        err = target_yaw - self.yaw
        while err >  math.pi: err -= 2.0 * math.pi
        while err < -math.pi: err += 2.0 * math.pi
        if abs(err) < 0.05:
            self.publish()
            return True
        self.publish(wz=math.copysign(self.turn_speed, err))
        return False

    def obstacle_ahead(self) -> bool:
        return (self.fresh('middle') < self.stop_dist or
                self.fresh('left')   < self.stop_dist or
                self.fresh('right')  < self.stop_dist)

    def side_sensor(self) -> float:
        """Raw reading of the sensor on the chosen wall side."""
        return self.fresh('right') if self.wall_side > 0 else self.fresh('left')

    # ── ASSESS scan ────────────────────────────────────────────────────────────

    def start_scan(self):
        self.scan_origin_yaw = self.yaw
        self.scan_target_yaw = self.wrap_yaw(self.yaw + math.pi / 2.0)
        self.scan_settled    = False
        self.scan_settle_ts  = 0.0
        self.scan_left_dist  = float('inf')
        self.scan_right_dist = float('inf')
        self.get_logger().info(
            f'Scan start | origin={math.degrees(self.scan_origin_yaw):.1f}°'
        )
        self.enter(State.ASSESS_LEFT)

    def assess_left(self):
        if not self.scan_settled:
            if self.turn_to_yaw(self.scan_target_yaw):
                self.scan_settle_ts = time.monotonic()
                self.scan_settled   = True
            return
        if (time.monotonic() - self.scan_settle_ts) < self.scan_settle_time:
            self.publish()
            return
        self.scan_left_dist  = self.fresh('middle')
        self.get_logger().info(f'Left={self.scan_left_dist:.2f}m')
        self.scan_target_yaw = self.wrap_yaw(self.scan_origin_yaw - math.pi / 2.0)
        self.scan_settled    = False
        self.enter(State.ASSESS_RIGHT)

    def assess_right(self):
        if not self.scan_settled:
            if self.turn_to_yaw(self.scan_target_yaw):
                self.scan_settle_ts = time.monotonic()
                self.scan_settled   = True
            return
        if (time.monotonic() - self.scan_settle_ts) < self.scan_settle_time:
            self.publish()
            return
        self.scan_right_dist = self.fresh('middle')
        self.get_logger().info(f'Right={self.scan_right_dist:.2f}m')
        self.scan_target_yaw = self.scan_origin_yaw
        self.scan_settled    = False
        self.enter(State.ASSESS_RETURN)

    def assess_return(self):
        if not self.scan_settled:
            if self.turn_to_yaw(self.scan_target_yaw):
                self.scan_settle_ts = time.monotonic()
                self.scan_settled   = True
            return
        if (time.monotonic() - self.scan_settle_ts) < self.scan_settle_time:
            self.publish()
            return

        l, r = self.scan_left_dist, self.scan_right_dist
        if l >= r:
            self.wall_side = -1.0
            side_name = 'LEFT'
        else:
            self.wall_side = 1.0
            side_name = 'RIGHT'

        self.get_logger().info(
            f'ASSESS done | left={l:.2f}m right={r:.2f}m → {side_name} wall'
        )

        # ── store hit distance from middle sensor at obstacle detection ────────
        # This is the target distance the side sensor should hold during drive
        self.get_logger().info(
            f'Hit distance (target): {self.hit_dist:.2f}m'
        )

        self.boundary_return_target = self.angle_to_goal()
        self.enter(State.BOUNDARY_TURN)

    # ── BOUNDARY_TURN ──────────────────────────────────────────────────────────

    def do_boundary_turn(self):
        side_reading = self.side_sensor()
        m            = self.fresh('middle')

        if int(self.state_age()) != int(self.state_age() - 0.05):
            self.get_logger().info(
                f'BOUNDARY_TURN | side={side_reading:.2f}m '
                f'target={self.hit_dist:.2f}m '
                f'middle={m:.2f}m'
            )

        # ── too close ahead → back away first ─────────────────────────────────
        if m < self.stop_dist:
            self.get_logger().warn(f'Too close ({m:.2f}m) → reversing')
            self.publish(vx=-0.10)   # slow reverse
            return

        # ── side sensor has matched hit distance → start driving ───────────────
        if not math.isinf(side_reading):
            error = side_reading - self.hit_dist
            if abs(error) < self.perp_tol:
                self.get_logger().info(
                    f'Side matched hit_dist | side={side_reading:.2f}m'
                )
                self.boundary_drive_dist   = 0.0
                self.boundary_drive_last_x = self.x
                self.boundary_drive_last_y = self.y
                self.enter(State.BOUNDARY_DRIVE)
                return

        # ── keep rotating toward wall side ─────────────────────────────────────
        self.publish(wz=-self.wall_side * self.turn_speed)

    # ── BOUNDARY_DRIVE ─────────────────────────────────────────────────────────

    def do_boundary_drive(self):
        """
        Drive forward alongside the obstacle.
        P controller on side sensor holds distance = hit_dist.
        Exit when side sensor clears beyond clear_dist (back edge passed).
        """
        m   = self.fresh('middle')
        age = self.state_age()

        # accumulate travel
        dx   = self.x - self.boundary_drive_last_x
        dy   = self.y - self.boundary_drive_last_y
        step = math.hypot(dx, dy)
        if step < 0.2:
            self.boundary_drive_dist += step
        self.boundary_drive_last_x = self.x
        self.boundary_drive_last_y = self.y

        side_reading = self.side_sensor()

        if int(age) != int(age - 0.05):
            self.get_logger().info(
                f'BOUNDARY_DRIVE | traveled={self.boundary_drive_dist:.2f}m '
                f'side={side_reading:.2f}m target={self.hit_dist:.2f}m '
                f'middle={m:.2f}m'
            )

        # ── new obstacle ahead → stop and re-assess ────────────────────────────
        if m < self.stop_dist:
            self.publish()
            self.get_logger().info('Obstacle ahead during drive → re-assess')
            self.start_scan()
            return

        # ── must travel minimum before checking back edge ──────────────────────
        if self.boundary_drive_dist < self.min_drive_dist:
            self._hold_distance(side_reading)
            return

        # ── back edge detected ─────────────────────────────────────────────────
        if math.isinf(side_reading) or side_reading > self.clear_dist:
            self.get_logger().info(
                f'Back edge cleared | side={side_reading:.2f}m '
                f'traveled={self.boundary_drive_dist:.2f}m → BOUNDARY_RETURN'
            )
            self.boundary_return_target = self.angle_to_goal()
            self.enter(State.BOUNDARY_RETURN)
            return

        # ── still alongside obstacle → P control ──────────────────────────────
        self._hold_distance(side_reading)

    def _hold_distance(self, side_reading: float):
        """
        P controller: drive forward while holding side sensor at hit_dist.

        error > 0 → too far from wall → rotate toward wall
        error < 0 → too close to wall → rotate away from wall

        wall_side +1 (right): toward wall = CW = negative wz
        wall_side -1 (left):  toward wall = CCW = positive wz
        """
        if math.isinf(side_reading):
            # lost wall — drive straight and rotate toward wall to re-acquire
            self.publish(
                vx=self.wall_fwd_speed * 0.5,
                wz=-self.wall_side * self.turn_speed * 0.3
            )
            return

        error = side_reading - self.hit_dist
        wz    = -self.wall_side * self.wall_kp * error
        wz    = max(-self.turn_speed * 0.5, min(self.turn_speed * 0.5, wz))
        self.publish(vx=self.wall_fwd_speed, wz=wz)

    # ── BOUNDARY_RETURN ────────────────────────────────────────────────────────

    def do_boundary_return(self):
        """Turn back to face the goal then check if obstacle is still close."""
        if self.turn_to_yaw(self.boundary_return_target):
            self.get_logger().info('Heading restored → CLEAR_OBSTACLE check')
            self.enter(State.CLEAR_OBSTACLE)

    # ── CLEAR_OBSTACLE ─────────────────────────────────────────────────────────

    def do_clear_obstacle(self):
        """
        After returning to goal heading, check if obstacle is still
        too close on the wall side. If so, strafe away until clear.
        Once clear, resume FORWARD.
        """
        side_reading = self.side_sensor()
        m            = self.fresh('middle')
        age          = self.state_age()

        if int(age) != int(age - 0.05):
            self.get_logger().info(
                f'CLEAR_OBSTACLE | side={side_reading:.2f}m '
                f'target={self.hit_dist:.2f}m middle={m:.2f}m'
            )

        # ── new obstacle ahead → re-assess ────────────────────────────────────
        if m < self.stop_dist:
            self.hit_dist = self.fresh('middle')
            self.publish()
            self.get_logger().info('Obstacle ahead in clear → re-assess')
            self.start_scan()
            return

        # ── obstacle still close on wall side → strafe away ───────────────────
        if not math.isinf(side_reading) and side_reading < self.hit_dist:
            # strafe away from wall side
            vy = self.wall_side * self.strafe_speed   # +1 right wall → vy positive = strafe left
            self.get_logger().info(
                f'Still too close ({side_reading:.2f}m < {self.hit_dist:.2f}m) → strafing away'
            )
            self.publish(vy=vy)
            return

        # ── clear → resume FORWARD ─────────────────────────────────────────────
        self.get_logger().info('Obstacle clear → FORWARD')
        self.enter(State.FORWARD)

    # ── FORWARD ────────────────────────────────────────────────────────────────

    def go_forward(self):
        l = self.fresh('left')
        r = self.fresh('right')
        m = self.fresh('middle')

        # ── obstacle directly ahead → full re-assess ───────────────────────────
        if m < self.stop_dist or l < self.stop_dist or r < self.stop_dist:
            # cap hit_dist — if middle is far, use the closest side sensor instead
            raw_hit = self.fresh('middle')
            self.hit_dist = min(raw_hit, self.stop_dist * 2.0)  # max 0.70m
            self.get_logger().info(f'Obstacle → re-assess | hit_dist={self.hit_dist:.2f}m')
            self.publish()
            self.start_scan()
            return

        # ── clear → drive toward goal ──────────────────────────────────────────
        herr  = self.heading_error()
        wz    = max(-self.turn_speed, min(self.turn_speed, self.heading_kp * herr))
        speed = self.fwd_speed * (1.0 - 0.5 * abs(herr) / math.pi)
        self.publish(vx=speed, wz=wz)

    # ── control loop ───────────────────────────────────────────────────────────

    def control_loop(self):
        if self.state == State.IDLE:
            self.publish()
            return
        if self.goal_x is None:
            self.publish()
            return
        if self.distance_to_goal() < self.goal_radius:
            self.publish()
            if self.state != State.GOAL_REACHED:
                self.get_logger().info(
                    f'Goal reached! ({self.x:.2f}, {self.y:.2f})'
                )
                self.enter(State.GOAL_REACHED)
            return

        if   self.state == State.FORWARD:         self.go_forward()
        elif self.state == State.ASSESS_LEFT:     self.assess_left()
        elif self.state == State.ASSESS_RIGHT:    self.assess_right()
        elif self.state == State.ASSESS_RETURN:   self.assess_return()
        elif self.state == State.BOUNDARY_TURN:   self.do_boundary_turn()
        elif self.state == State.BOUNDARY_DRIVE:  self.do_boundary_drive()
        elif self.state == State.BOUNDARY_RETURN:  self.do_boundary_return()
        elif self.state == State.CLEAR_OBSTACLE:    self.do_clear_obstacle()
        elif self.state == State.GOAL_REACHED:      self.publish()


def main(args=None):
    rclpy.init(args=args)
    node = Bug2Node()
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