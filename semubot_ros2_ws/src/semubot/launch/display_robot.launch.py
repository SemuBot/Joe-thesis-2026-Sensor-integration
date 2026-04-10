from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        'urdf',
        'semubot.urdf'
    )

    with open(urdf_path, 'r') as inf:
        robot_description_content = inf.read()

    return LaunchDescription([
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}, {'source_list': ['/cmd_vel_joint_states']}]
        ),

        # Robot State Publisher
       Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/semubot-laptop/.rviz2/semubot.rviz'],
            output='screen'
        ),

        Node(
	    package='semubot',
            executable='odom_broadcaster.py',
            name='odom_broadcaster',
            output='screen'
        ),


        # CmdVel Publisher - sends fake movement commands
        Node(
            package='semubot',
            executable='cmd_vel_publisher.py',
            name='cmd_vel_publisher',
            output='screen'
        ),

        # CmdVel Serial - sends movement commands to STM32
        Node(
            package='semubot',
            executable='cmd_vel_serial.py',
            name='cmd_vel_serial',
            output='screen'
        ),

        Node(
            package='semubot',
            executable='cmd_vel_to_joint_states.py',
            name='cmd_vel_to_joint_states',
            output='screen'
        )
    ])

