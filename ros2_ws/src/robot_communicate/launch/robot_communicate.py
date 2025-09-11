from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        Node(
            package='robot_communicate',
            executable='robot_communicate_node',
            name='robot_communicate_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'port': LaunchConfiguration('port')}
            ]
        )
    ])
