# tts_node_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 
    ui_node = Node(
        package='marrtinorobot2_ui',
        executable='ui_node',
        name='ui_node',
        output='screen'
    )

    # 
    ld = LaunchDescription()
    ld.add_action(ui_node)

    return ld
