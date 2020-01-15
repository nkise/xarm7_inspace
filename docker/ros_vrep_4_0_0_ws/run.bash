#!/usr/bin/env bash


SCRIPT_PATH=$(readlink -f "$0")
SCRIPT_DIR_PATH=$(dirname "$SCRIPT_PATH")
WS_DIR_PATH=$(realpath "$SCRIPT_DIR_PATH/../../../..")

xhost +local:root

docker run -it --rm \
    --runtime=nvidia \
    --privileged \
    --net=host \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    --volume="$WS_DIR_PATH:/root/ws" \
    --device=/dev/dri:/dev/dri \
    --env="DISPLAY=$DISPLAY" \
    --env QT_X11_NO_MITSHM=1 \
   ros_vrep_4_0_0_ws 

xhost -local:root
