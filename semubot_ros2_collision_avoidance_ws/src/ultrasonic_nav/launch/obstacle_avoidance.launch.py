from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessIO
from ament_index_python.packages import get_package_share_directory
import os

_node_started = False

def generate_launch_description():
    pkg_share = get_package_share_directory('ultrasonic_nav')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', '/dev/ttyACM0', '-b', '115200', '--reconnect'
        ],
        output='screen',
    )

    obstacle_node = Node(
        package='ultrasonic_nav',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[params_file],
    )

    graph_node = Node(
        package='ultrasonic_nav',
        executable='distance_graph_node',
        name='distance_graph_node',
        output='screen',
        parameters=[params_file],
    )

    def on_agent_output(event):
        global _node_started
        text = event.text.decode('utf-8')
        if 'datawriter created' in text and not _node_started:
            _node_started = True
            return [
                LogInfo(msg='Micro-ROS Agent Ready. Starting Obstacle Avoidance...'),
                obstacle_node,
                graph_node,
            ]
        return []

    micro_ros_agent_ready_handler = RegisterEventHandler(
        OnProcessIO(
            target_action=micro_ros_agent,
            on_stdout=on_agent_output,
        )
    )

    return LaunchDescription([
        micro_ros_agent_ready_handler,
        micro_ros_agent,
        ])
