from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='face_tracker_pkg',
            executable='face_tracker_controller',
            name='face_tracker_controller',
            parameters=[
                'path/to/your/face_tracker_params.yaml'
            ]
        ),
    ])
