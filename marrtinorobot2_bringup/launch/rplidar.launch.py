
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='topic_name', 
            default_value='scan',
            description='Laser Topic Name'
        ),
        DeclareLaunchArgument(
            name='frame_id', 
            default_value='laser',
            description='Laser Frame ID'
        ),

        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            remappings=[('scan', LaunchConfiguration('topic_name'))],
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # 
                'frame_id': LaunchConfiguration('frame_id'),
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])
