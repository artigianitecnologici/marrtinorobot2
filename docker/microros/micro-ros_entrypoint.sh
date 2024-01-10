. "/opt/ros/$ROS_DISTRO/setup.sh"
. "/marrtinorobot2_ws/install/local_setup.sh"

if [ "$MICROROS_DISABLE_SHM" = "1" ] ; then
    if [ "$ROS_LOCALHOST_ONLY" = "1" ] ; then
        export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm_localhost_only.xml
    else
        export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm.xml
    fi
fi

#exec ros2 run micro_ros_agent micro_ros_agent "$@"
exec ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
