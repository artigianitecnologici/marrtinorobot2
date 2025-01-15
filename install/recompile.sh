
### 1.0 Create WorkSpaces
cd $HOME/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
# assume you have sourced ROS environment, same blow
sudo apt install -y libgflags-dev nlohmann-json3-dev libgoogle-glog-dev \
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager

cd $HOME
mkdir -p marrtinorobot2_ws/src

### 1.1 Install Microros 
cd $HOME/marrtinorobot2_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt install -y python3-vcstool build-essential
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y
colcon build
source  install/setup.bash

#### 1.2 Setup micro-ROS agent:
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source  install/setup.bash
### 2.3 Clone  marrtinorobot2


cd $HOME/marrtinorobot2_ws/src
    ln -s /home/ubuntu/src/DynamixelSDK . && \
    ln -s /home/ubuntu/src/dynamixel-workbench . && \
    ln -s /home/ubuntu/src/dynamixel-workbench-msgs .  && \
    ln -s /home/ubuntu/src/marrtinorobot2/marrtinorobot2_base . && \
    ln -s /home/ubuntu/src/marrtinorobot2/marrtinorobot2_bringup . && \
    ln -s /home/ubuntu/src/marrtinorobot2/marrtinorobot2_description . && \
    ln -s /home/ubuntu/src/marrtinorobot2/marrtinorobot2_webinterface . && \
    ln -s /home/ubuntu/src/marrtinorobot2/marrtinorobot2_navigation . && \
    ln -s /home/ubuntu/src/marrtinorobot2/marrtinorobot2_vision . && \
    ln -s /home/ubuntu/src/marrtinorobot2/marrtinorobot2_voice . && \
    ln -s /home/ubuntu/src/marrtinorobot2/marrtinorobot2_cartographer . && \
    ln -s /home/ubuntu/src/marrtinorobot2/marrtinorobot2_dynamixel . && \
    ln -s /home/ubuntu/src/m-explore-ros2/explore . && \
    ln -s /home/ubuntu/src/ldlidar_stl_ros2 . && \
    ln -s /home/ubuntu/src/sllidar_ros2 . && \
    ln -s /home/ubuntu/src/depthai-ros . && \
    ln -s /home/ubuntu/src/depthai-ros-examples . && \
    ln -s /home/ubuntu/src/vizanti . 

### 2.3 Install mexplorer


### 2.3 Install marrtinorobot2 package:
cd ..
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
colcon build 
source  install/setup.bash

cp $HOME/src/marrtinorobot2/install/script/*.sh .
sudo apt-get update -q && \
    sudo apt-get install -y \
        ros-humble-depthai-ros \ 
        ros-humble-rtabmap-ros \
        ros-humble-cartographer \
        ros-humble-cartographer-ros \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-v4l2-camera \
        ros-humble-camera-calibration-parsers \
        ros-humble-camera-info-manager \
        ros-humble-launch-testing-ament-cmake \
        ros-humble-robot-localization \
        ros-humble-rosbridge-server \
        ros-humble-joy \
        ros-humble-rplidar-ros \
        ros-humble-joint-state-publisher \  
        ros-humble-pointcloud-to-laserscan \
        ros-humble-rosbridge-suite  && \
    sudo apt-get autoclean && \
    sudo apt-get autoremove 

# sudo apt install -y ros-$ROS_DISTRO-rplidar-ros
# sudo apt-get install -y ros-${ROS_DISTRO}-v4l2-camera
# echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
# sudo udevadm control --reload-rules && sudo udevadm trigger
# # oak d lite
# sudo apt install -y ros-$ROS_DISTRO-depthai-ros
# # navigation
# sudo apt install -y ros-$ROS_DISTRO-rtabmap-ros
# ## cartographer
# sudo apt install -y ros-$ROS_DISTRO-cartographer 
# sudo apt install -y ros-$ROS_DISTRO-cartographer-ros
# #  Navigation Stack for ROS 2
# sudo apt install -y ros-$ROS_DISTRO-navigation2 
# sudo apt install -y ros-$ROS_DISTRO-nav2-bringup

# sudo apt-get install -y ros-${ROS_DISTRO}-v4l2-camera
# sudo apt install v4l-utils
# sudo apt install -y ros-$ROS_DISTRO-camera-calibration-parsers
# sudo apt install -y ros-$ROS_DISTRO-camera-info-manager
# sudo apt install -y ros-$ROS_DISTRO-launch-testing-ament-cmake
# echo "Dynamixel "
# sudo apt install -y ros-$ROS_DISTRO-dynamixel-sdk
# sudo apt install -y ros-$ROS_DISTRO-dynamixel-workbench
# sudo apt install -y ros-$ROS_DISTRO-dynamixel-workbench-msgs
 # astra camera
# sudo apt install ros-$ROS_DISTRO-image-pipeline libuvc-dev
# prerequisite marrtinorobot2_voice
pip3 install gtts
sudo apt install -y mpg321

# 


