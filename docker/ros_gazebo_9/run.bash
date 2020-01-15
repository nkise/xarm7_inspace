#!/usr/bin/env bash

xhost +local:root

docker run -it --rm \
    --runtime=nvidia \
    --privileged \
    --net=host \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    --device=/dev/dri:/dev/dri \
    --env="DISPLAY=$DISPLAY" \
    --env QT_X11_NO_MITSHM=1 \
   ros_gazebo_9

xhost -local:root
