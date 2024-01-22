 . install/setup.bash
cd marrtinorobot2/marrtinorobot2_navigation/maps
    ros2 run nav2_map_server map_saver_cli -f studio --ros-args -p save_map_timeout:=10000.
