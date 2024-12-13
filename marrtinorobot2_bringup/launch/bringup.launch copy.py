from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Path to description.launch.py
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_description'), 'launch', 'description.launch.py']
    )

    # Path to EKF configuration
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("marrtinorobot2_base"), "config", "ekf.yaml"]
    )

    # Path to TTS node launch file
    tts_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_voice'), 'launch', 'tts_node.launch.py']
    )

    # Path to Camera node launch file
    camera_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_vision'), 'launch', 'camera.launch.py']
    )

    # Path to LIDAR node launch file (if needed)
    lidar_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_lidar'), 'launch', 'lidar.launch.py']
    )

    return LaunchDescription([
        # Declare Launch Arguments
        DeclareLaunchArgument(
            name='base_serial_port', 
            default_value='/dev/ttyACM0',
            description='MARRtino robot Base Serial Port'
        ),

        DeclareLaunchArgument(
            name='odom_topic', 
            default_value='/odom',
            description='EKF out odometry topic'
        ),

        DeclareLaunchArgument(
            name='use_camera',
            default_value='false',
            description='Condition to launch camera node'
        ),

        DeclareLaunchArgument(
            name='use_tts',
            default_value='false',
            description='Condition to launch TTS node'
        ),

        DeclareLaunchArgument(
            name='use_lidar',
            default_value='false',
            description='Condition to launch LIDAR node'
        ),

        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))]
        ),

        # Micro-ROS Agent Node
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration("base_serial_port")]
        ),

        # Include Description Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),

        # Include Camera Launch (conditional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_robot_launch_path),
            condition=IfCondition(LaunchConfiguration('use_camera'))
        ),

        # Include TTS Node Launch (conditional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tts_robot_launch_path),
            condition=IfCondition(LaunchConfiguration('use_tts'))
        ),

        # Include LIDAR Node Launch (conditional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_robot_launch_path),
            condition=IfCondition(LaunchConfiguration('use_lidar'))
        )
    ])
