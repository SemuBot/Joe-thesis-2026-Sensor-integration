import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from enum import Enum, auto
import math
import time

STOP_DIST         = 0.35
WALL_DIST         = 0.30
WALL_TOL          = 0.05
CLEAR_DIST        = 0.50
GOAL_RADIUS       = 0.15
MLINE_TOL         = 0.08
SENSOR_TIMEOUT    = 0.5
FWD_SPEED         = 0.20
TURN_SPEED        = 0.6
WALL_FWD_SPEED    = 0.15
WALL_KP           = 1.2
HEADING_KP        = 1.0

class State(Enum):
    IDLE            = auto()
    FORWARD         = auto()
    BOUNDARY        = auto()
    GOAL_REACHED    = auto()

class Bug2Node(Node):

    def __init__(self):
        super().__init__('bug2_node')

        self.declare_parameter('stop_dist',      STOP_DIST)
        self.declare_parameter('wall_dist',      WALL_DIST)
        self.declare_parameter('wall_tol',       WALL_TOL)
        self.declare_parameter('clear_dist',     CLEAR_DIST)
        self.declare_parameter('goal_radius',    GOAL_RADIUS)
        self.declare_parameter('mline_tol',      MLINE_TOL)
        self.declare_parameter('fwd_speed',      FWD_SPEED)
        self.declare_parameter('turn_speed',     TURN_SPEED)
        self.declare_parameter('wall_fwd_speed', WALL_FWD_SPEED)
        self.declare_parameter('wall_kp',        WALL_KP)
        self.declare_parameter('heading_kp',     HEADING_KP)

        self.stop_dist      = self.get_parameter('stop_dist').value
        self.wall_dist      = self.get_parameter('wall_dist').value
        self.wall_tol       = self.get_parameter('wall_tol').value
        self.clear_dist     = self.get_parameter('clear_dist').value
        self.goal_radius    = self.get_parameter('goal_radius').value
        self.mline_tol      = self.get_parameter('mline_tol').value
        self.fwd_speed      = self.get_parameter('fwd_speed').value
        self.turn_speed     = self.get_parameter('turn_speed').value
        self.wall_fwd_speed = self.get_parameter('wall_fwd_speed').value
        self.wall_kp        = self.get_parameter('wall_kp').value
        self.heading_kp     = self.get_parameter('heading_kp').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.goal_x = None
        self.goal_y = None

        self.start_x = 0.0
        self.start_y = 0.0

        self.obstacle_x = 0.0
        self.obstacle_y = 0.0
        self.obstacle_dist = 0.0

        self.readings: dict[str, tuple[float, float]] = {
            'left':   (float('inf'), 0.0),
            'right':  (float('inf'), 0.0),
            'middle': (float('inf'), 0.0),
        }

        self.state = State.IDLE
        self.state_ts = time.monotonic()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(Range, 'ultrasonic/middle', self.middle_callback, qos)
        self.create_subscription(Range, 'ultrasonic/left', self.left_callback, qos)
        self.create_subscription(Range, 'ultrasonic/right', self.right_callback, qos)
        self.create_subscription(Odometry, 'odom', self.odom_callback, qos)
        self.create_subscription(Point, 'goal', self.goal_callback, qos)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            'Bug2Node initialized, waiting for goal on topic /goal... \n'
            'Send a goal: ros2 topic pub --once /goal geometry_msgs/msg/Point '
            '"{x: 2.0, y: 0.0, z: 0.0}"'
        )

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
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg: Point):
        self.goal_x = msg.x
        self.goal_y = msg.y

        self.start_x = self.x
        self.start_y = self.y

        self.get_logger().info(
            f'New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})\n'
            f'Start: ({self.start_x:.2f}, {self.start_y:.2f})'
        )

        self.enter(State.FORWARD)

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
            self.get_logger().info(
                f'{self.state.name} -> {new_state.name}'
            )
        self.state    = new_state
        self.state_ts = time.monotonic()

    def distance_to_goal(self) -> float:
        if self.goal_x is None or self.goal_y is None:
            return float('inf')
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)
    
    def angle_to_goal(self) -> float:
        if self.goal_x is None or self.goal_y is None:
            return 0.0
        return math.atan2(self.goal_y - self.y, self.goal_x - self.x)
    
    def heading_error(self) -> float:
        error = self.angle_to_goal() - self.yaw
        while error > math.pi:
            error -= 2.0 * math.pi
        while error < -math.pi:
            error += 2.0 * math.pi
        return error
    
    def track_error(self) -> float:
        if self.start_x is None or self.start_y is None:
            return 0.0
        dx = self.x - self.start_x
        dy = self.y - self.start_y
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return 0.0
        
        return ((self.x - self.start_x) * dy - (self.y - self.start_y) * dx) / length
    
    def on_mline(self) -> bool:
        return abs(self.track_error()) < self.mline_tol
    
    def obstacle_ahead(self) -> bool:
        m = self.fresh('middle')
        l = self.fresh('left')
        r = self.fresh('right')
        return m < self.stop_dist or l < self.stop_dist or r < self.stop_dist
    
    def go_forward(self):
        if self.obstacle_ahead():
            self.obstacle_x = self.x
            self.obstacle_y = self.y
            self.obstacle_dist = self.distance_to_goal()
            self.publish()
            self.enter(State.BOUNDARY)
            self.get_logger().info(
                f'Obstacle detected at ({self.obstacle_x:.2f}, {self.obstacle_y:.2f}), '
                f'distance to goal: {self.obstacle_dist:.2f} m'
            )
            return
        
        heading_error = self.heading_error()
        wz = self.heading_kp * heading_error
        wz = max(-self.turn_speed, min(self.turn_speed, wz))
        speed = self.fwd_speed * (1.0 - 0.5 * abs(heading_error) / math.pi)

        self.publish(vx=speed, wz=wz)

    def follow_boundary(self):
        l = self.fresh('left')
        r = self.fresh('right')
        m = self.fresh('middle')

        dist_now = self.distance_to_goal()

        if (self.on_mline() and dist_now < self.obstacle_dist and m > self.clear_dist):
            self.get_logger().info(
                f'Leaving point reached. Distance to goal: {dist_now:.2f} m\n'
            )
        
            self.enter(State.FORWARD)
            return
        
        if (m < self.stop_dist):
            self.publish(wz=self.turn_speed)
            return
        
        if not math.isinf(r):
            wall_error = r - self.wall_dist
            vy = -self.wall_kp * wall_error
            vy = max(-0.3, min(0.3, vy))

        else:
            self.publish(wz = -self.turn_speed * 0.5)
            return
        
        self.publish(vx = self.wall_fwd_speed, vy=vy)

    def control_loop(self):
        if self.state == State.IDLE:
            self.publish()
            return
        
        if self.goal_x is None or self.goal_y is None:
            self.get_logger().warn('No goal set, cannot navigate')
            self.publish()
            return
        
        if self.distance_to_goal() < self.goal_radius:
            self.publish()
            if self.state != State.GOAL_REACHED:
                self.get_logger().info(
                    f'Goal reached!\n'
                    f'Final position: ({self.x:.2f}, {self.y:.2f}) '
                    )
                self.enter(State.GOAL_REACHED)
            return
        
        if self.state == State.FORWARD:
            self.go_forward()

        elif self.state == State.BOUNDARY:
            self.follow_boundary()

        elif self.state == State.GOAL_REACHED:
            self.publish()


def main(args=None):
    rclpy.init(args=args)
    node = Bug2Node()
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