#!/bin/bash

IMAGENAME=marrtinorobot2:system

# change setings here if needed
if [ "$ROBOT_IP" == "" ]; then
  echo "Set ROBOT_IP env var to IP of robot running roscore"
  export ROBOT_IP=10.3.1.1 
  #exit 1
fi

echo "Running image $IMAGENAME ..."

docker run -it \
    --name marrtinoros2 --rm \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    -e DISPLAY=$DISPLAY \
    --privileged \
    --net=host \
    $IMAGENAME \
    tmux

