# Docker file for MARRtinorobot2 software
# ROS humble
# System image - humble
# docker build -t marrtinorobot2:robot -f Dockerfile.robot .
FROM marrtinorobot2:base 

ARG UID=1000
ARG GID=1000
ARG TYPE_ROBOT=yellow
ARG LASER_SENSOR=rplidar

WORKDIR /marrtinorobot2_ws

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    libtinyxml2-9 \
    tmux \
    netcat \
    wget \
    net-tools \
    && rm -rf /var/lib/apt/lists/*


# User: robot (password: robot) with sudo power
RUN useradd -ms /bin/bash robot && echo "robot:robot" | chpasswd && adduser robot sudo
RUN usermod -u $UID robot && groupmod -g $GID robot

RUN adduser robot audio
RUN adduser robot video
RUN adduser robot dialout

###### User robot ######
# Set the working directory back to /marrtinorobot2_ws
USER robot

WORKDIR /home/robot

RUN mkdir -p $HOME/src && \
    cd $HOME/src && \
    git clone  https://github.com/artigianitecnologici/marrtinorobot2.git

RUN cd $HOME && \
    mkdir -p marrtinorobot2_ws/src && \
    cd marrtinorobot2_ws/src && \
    ln -s $HOME/src/marrtinorobot2/marrtinorobot2_base . && \
    ln -s $HOME/src/marrtinorobot2/marrtinorobot2_bringup . && \
    ln -s $HOME/src/marrtinorobot2/marrtinorobot2_description . && \
    ln -s $HOME/src/marrtinorobot2/marrtinorobot2_gazebo . && \
    ln -s $HOME/src/marrtinorobot2/marrtinorobot2_navigation . && \
    ln -s $HOME/src/marrtinorobot2/marrtinorobot2_teleop .

RUN echo "export MARRTINOROBOT2_BASE=$TYPE_ROBOT" >> ~/.bashrc
RUN echo "export MARRTINOROBOT2_LASER_SENSOR=$LASER_SENSOR" >> ~/.bashrc

### 2.3 Install marrtinorobot2 package:
#RUN  cd marrtinorobot2_ws/src && \
#    rosdep update && \
#    rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent && \
#    colcon build && \
#    source  install/setup.bash

# Set entrypoint or default command if needed
# ENTRYPOINT ["/your/entrypoint/script
#CMD [ "/bin/bash", "-ci", "cd ~/src/marrtinorobot2/bringup && python3 robot_bringup.py" ]
#CMD [ "/usr/bin/tmux" ]
# Set entrypoint or default command if needed
CMD [ "tmux", "new-session", "-s", "marrtinorobot2", "-d", "/bin/bash" ]
CMD [ "tmux", "new-window", "-t", "marrtinorobot2", "/bin/bash" ]
CMD [ "tmux", "split-window", "-t", "marrtinorobot2:1", "-v", "/bin/bash" ]
CMD [ "tmux", "send-keys", "-t", "marrtinorobot2:1", "python3 robot_bringup.py", "C-m" ]
