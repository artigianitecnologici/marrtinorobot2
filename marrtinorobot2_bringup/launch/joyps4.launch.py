from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Percorso del file di configurazione per il joypad PS4
    joy_config_path = PathJoinSubstitution(
        [FindPackageShare("marrtinorobot2_bringup"), "config", "ps4_joy_config.yaml"]
    )
    
    # Percorso del file di configurazione per teleop_twist_joy
    teleop_config_path = PathJoinSubstitution(
        [FindPackageShare("marrtinorobot2_bringup"), "config", "teleop_ps4.yaml"]
    )

    return LaunchDescription([
        # Nodo per il joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_config_path]
        ),
        
        # Nodo per la conversione dei comandi del joystick in cmd_vel
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[teleop_config_path],
            remappings=[('/cmd_vel', '/cmd_vel')]  # Cambia il topic se necessario
        )
    ])
