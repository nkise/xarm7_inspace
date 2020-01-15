#!/usr/bin/env bash

xhost +local:root

docker run -it --rm \
    --runtime=nvidia \
    --privileged \
    --net=host \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    --device=/dev/dri:/dev/dri \
    --env="DISPLAY=$DISPLAY" \
   vrep_3_5_0 

xhost -local:root
