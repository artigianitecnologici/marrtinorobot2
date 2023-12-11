

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

#create a package
ros2 pkg create --build-type ament_cmake marrtino2_controller