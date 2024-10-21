from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='marrtinorobot2_ps4_joy_control',
            executable='ps4_joy_control',
            name='ps4_joy_control'
        ),
    ])
