 . install/setup.bash
cd src/marrtinorobot2_navigation/maps
ros2 run nav2_map_server map_saver_cli -f $1  --ros-args -p save_map_timeout:=10000.
