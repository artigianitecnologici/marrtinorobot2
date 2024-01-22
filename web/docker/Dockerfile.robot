# Docker file for MARRtinorobot2 software
# ROS humble
# System image - humble
# docker build -t marrtinorobot2:robot -f Dockerfile.robot .
FROM marrtinorobot2:base 

ARG UID=1000
ARG GID=1000
ARG TYPE_ROBOT=2wd
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

RUN cd $HOME
RUN mkdir -p marrtinorobot2_ws/src

RUN cd $HOME/marrtinorobot2_ws/src
RUN ln -s $HOME/src/marrtinorobot2/marrtinorobot2_base .
RUN ln -s $HOME/src/marrtinorobot2/marrtinorobot2_bringup .
RUN ln -s $HOME/src/marrtinorobot2/marrtinorobot2_description .
RUN ln -s $HOME/src/marrtinorobot2/marrtinorobot2_gazebo .
RUN ln -s $HOME/src/marrtinorobot2/marrtinorobot2_navigation .

RUN echo "export marrtinorobot2_BASE=$TYPE_ROBOT" >> ~/.bashrc
RUN echo "export marrtinorobot2_LASER_SENSOR=$LASER_SENSOR" >> ~/.bashrc


### 2.3 Install marrtinorobot2 package:
RUN cd ..
RUN rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
RUN colcon build 
RUN source  install/setup.bash

# Set entrypoint or default command if needed
# ENTRYPOINT ["/your/entrypoint/script
#CMD [ "/bin/bash", "-ci", "cd ~/src/marrtinorobot2/bringup && python3 robot_bringup.py" ]
CMD [ "/usr/bin/tmux" ]
