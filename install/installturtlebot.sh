sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
 sudo apt install ros-humble-hls-lfcd-lds-driver
 sudo apt install ros-humble-turtlebot3-msgs
 sudo apt install ros-humble-dynamixel-sdk
 sudo apt install libudev-dev
 mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
 git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
 git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
 cd ~/turtlebot3_ws/src/turtlebot3
 rm -r turtlebot3_cartographer turtlebot3_navigation2
 cd ~/turtlebot3_ws/
 echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
 source ~/.bashrc
 colcon build --symlink-install --parallel-workers 1
 echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
 source ~/.bashrc
  sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
 sudo udevadm control --reload-rules
 sudo udevadm trigger
