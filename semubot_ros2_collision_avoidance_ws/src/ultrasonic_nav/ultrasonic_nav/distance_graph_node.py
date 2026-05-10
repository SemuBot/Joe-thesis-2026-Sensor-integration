import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
 
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
from collections import deque

SENSOR_CONFIG = {
    'front': {'label': 'Front',  'color': '#00c9a7', 'col': 0, 'row': 0},
    'left':  {'label': 'Left',   'color': '#ffa552', 'col': 0, 'row': 1},
    'right': {'label': 'Right',  'color': '#ff6b6b', 'col': 0, 'row': 2},
}

MAX_POINTS = 100

class DistanceGraphNode(Node):
 

    def __init__(self):
        super().__init__('distance_graph_node')

        self.declare_parameter('max_distance', 1.0)
        self.declare_parameter('warn_distance', 0.5)

        self.max_distance = self.get_parameter('max_distance').value
        self.warn_distance = self.get_parameter('warn_distance').value

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.data = {
            sensor: deque(maxlen=MAX_POINTS)
            for sensor in SENSOR_CONFIG.keys()
        }

        self.samples = {
            sensor : 0
            for sensor in SENSOR_CONFIG
        }

        self.create_subscription(Range, 'ultrasonic/middle', self.front_callback, qos_profile)
        self.create_subscription(Range, 'ultrasonic/left', self.left_callback, qos_profile)
        self.create_subscription(Range, 'ultrasonic/right', self.right_callback, qos_profile)

        self.fig = plt.figure(figsize=(13, 9), facecolor="#2B2B2B")
        self.fig.suptitle("Ultrasonic Sensor - Live Distance", 
                          color = 'white', fontsize=14, fontweight='bold')
        
        gs = gridspec.GridSpec(3, 1, figure=self.fig, hspace=0.55)
        self.axes = {}
        self.lines = {}
        self.warn_lines = {}

        for key, cfg in SENSOR_CONFIG.items():
            ax = self.fig.add_subplot(gs[cfg['row'], 0])
            ax.set_facecolor("#323646")
            for sp in ax.spines.values():
                sp.set_edgecolor('#333')
 
            ax.set_title(cfg['label'], color='white', fontsize=11)
            ax.set_xlabel('Sample #', color='#aaa', fontsize=9)
            ax.set_ylabel('Distance (m)', color='#aaa', fontsize=9)
            ax.tick_params(colors='#aaa')
            ax.set_ylim(0, self.max_distance * 1.1)
            ax.grid(color='#2a2d3a', linewidth=0.5)                

            wl = ax.axhline(self.warn_distance, color='#ffcc00',
                            linewidth=0.9, linestyle='--', alpha=0.7,
                            label=f'warn {self.warn_distance:.2f} m')
            ax.legend(fontsize=8, facecolor='#1a1d27',
                      labelcolor='white', edgecolor='#333')

            line, = ax.plot([], [], color=cfg['color'], linewidth=1.5)
            scatter = ax.scatter([], [], color=cfg['color'], s=14, zorder=3)

            self.axes[key] = ax
            self.lines[key] = (line, scatter)
            self.warn_lines[key] = wl

        self.ani = animation.FuncAnimation(
            self.fig, self._update_plot,
            interval=100, cache_frame_data=False
        )



    def front_callback(self, msg):
        distance = msg.range
        if distance < 0:
            return
        self.data['front'].append(distance)
        self.samples['front'] += 1

    def left_callback(self, msg):
        distance = msg.range
        if distance < 0:
            return
        self.data['left'].append(distance)
        self.samples['left'] += 1

    def right_callback(self, msg):
        distance = msg.range
        if distance < 0:
            return
        self.data['right'].append(distance)
        self.samples['right'] += 1


    def _update_plot(self, frame):
        rclpy.spin_once(self, timeout_sec=0.0)

        for key, cfg in SENSOR_CONFIG.items():
            ax = self.axes[key]
            line, scatter = self.lines[key]
            d = list(self.data[key])

            if not d:
                continue

            x = list(range(len(d)))
            line.set_data(x, d)
            scatter.set_offsets(list(zip(x, d)))

            ax.set_xlim(max(0, len(d) - MAX_POINTS), len(d) + 1)

            colors = [
                '#ff3333' if v < self.warn_distance else cfg['color']
                for v in d
            ]
            scatter.set_color(colors)

            mean_d = sum(d) / len(d)
            ax.set_title(
                f"{cfg['label']}  —  latest: {d[-1]:.3f} m  |  "
                f"mean: {mean_d:.3f} m  |  min: {min(d):.3f} m",
                color='white', fontsize=10
            )

        self.fig.canvas.draw_idle()            


def main(args=None):
    rclpy.init(args=args)
    node = DistanceGraphNode()
 
    try:
        plt.tight_layout(rect=[0, 0, 1, 0.97])
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
