

set -e

ROSDISTRO="$(printenv ROS_DISTRO)"
BASE=$1
LASER_SENSOR=$2
DEPTH_SENSOR=$3
ARCH="$(uname -m)"
WORKSPACE="$HOME/marrtinorobot2_ws"

#### 1.4 Download and install micro-ROS:
cd $WORKSPACE
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt install -y python3-vcstool build-essential
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y
colcon build
source $WORKSPACE/install/setup.bash

#### 1.5 Setup micro-ROS agent:
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source $WORKSPACE/install/setup.bash

mkdir -p $HOME/src
cd $HOME/src
#### 1.6 Download marrtinorobot2:
cd $WORKSPACE
git clone -b $ROS_DISTRO https://github.com/lartigianitecnologici/marrtinorobot2 


cd $WORKSPACE/src
ln -s $HOME/src/marrtinorobot2/marrtinorobot2 .
ln -s $HOME/src/marrtinorobot2/marrtinorobot2_base .
ln -s $HOME/src/marrtinorobot2/marrtinorobot2_bringup .
ln -s $HOME/src/marrtinorobot2/marrtinorobot2_description .
ln -s $HOME/src/marrtinorobot2/marrtinorobot2_gazebo .
ln -s $HOME/src/marrtinorobot2/marrtinorobot2_navigation .


### 2.3 Install marrtinorobot2 package:
cd $WORKSPACE
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
colcon build
source $WORKSPACE/install/setup.bash


# prerequisite teleop and video server
sudo apt install ros-humble-usb-cam
sudo apt install ros-humble-async-web-server-cpp 
sudo apt install ros-humble-rosbridge-server

##### no --> sudo apt-get install ros-humble-web-video-server
# launch rosbridge server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

##### no -->  git clone -b ros2 https://github.com/RobotWebTools/web_video_server.git