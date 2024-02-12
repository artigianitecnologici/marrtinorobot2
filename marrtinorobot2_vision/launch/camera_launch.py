# camera_launch.py

import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node',
            name='usb_cam_node',
            output='screen',
            parameters=[{'video_device': '/dev/video0'}],  # Adjust the video device path accordingly
        ),
    ])
