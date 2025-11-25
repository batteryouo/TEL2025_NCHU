from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM1'),
        Node(
            package='at9s',
            executable='at9s_node',
            name='at9s_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'port': LaunchConfiguration('port')}
            ]
        )
    ])
