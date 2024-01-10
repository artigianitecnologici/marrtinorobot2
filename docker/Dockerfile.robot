# Docker file for MARRtinorobot2 software
# ROS humble
# System image - humble
# docker build -t marrtinorobot2:robot -f Dockerfile.robot .
FROM marrtinorobot2:base AS micro-ros-agent-builder

WORKDIR /marrtinorobot2_ws

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    libtinyxml2-9 \
    tmux \
    netcat \
    wget \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Micro-ROS dependencies and build Micro-ROS agent
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build --symlink-install

# Install Gazebo
#RUN apt-get update && apt-get install -y \
#    gazebo9 \
#    ros-$ROS_DISTRO-gazebo-ros-pkgs \
#    ros-$ROS_DISTRO-gazebo-ros-control \
#    && rm -rf /var/lib/apt/lists/*
# Set environment variables for Micro-ROS
#ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#ENV MICROROS_DISABLE_SHM=1

# Set the working directory back to /marrtinorobot2_ws
#RUN cd  $HOME

RUN mkdir -p $HOME/src && \
    cd $HOME/src && \
    git clone  https://github.com/artigianitecnologici/marrtinorobot2.git


#RUN source /opt/ros/$ROS_DISTRO/setup.bash
#RUN cd $HOME/marrtinorobot2_ws/src && \
#    ln -s $HOME/src/marrtinorobot2/src/marrtino2_controller . && \
#    ln -s $HOME/src/marrtinorobot2/src/marrtino2_bringup . && \
#    ln -s $HOME/src/marrtinorobot2/src/marrtino2_description . && \
#    ln -s $HOME/src/marrtinorobot2/src/marrtino2_firmware . && \
#    ln -s $HOME/src/marrtinorobot2/src/marrtino2_localization . && \
#    ln -s $HOME/src/marrtinorobot2/src/marrtino2_msgs .



# Set entrypoint or default command if needed
# ENTRYPOINT ["/your/entrypoint/script
#CMD [ "/bin/bash", "-ci", "cd ~/src/marrtinorobot2/bringup && python3 robot_bringup.py" ]
CMD [ "/usr/bin/tmux" ]
