FROM ros_vrep_4_0_0

WORKDIR /root

RUN apt-get update && apt-get install -y \
    git \
    vim \
    tmux \
    build-essential \
    xsltproc \
    ros-kinetic-tf2-sensor-msgs \
    ros-kinetic-control-toolbox \
    ros-kinetic-moveit \
    ros-kinetic-moveit-visual-tools \
    ros-kinetic-ros-control \
    ros-kinetic-ros-controllers \
    && rm -rf /var/lib/apt/lists/*

ENV COPPELIASIM_ROOT_DIR=/root/V-REP

RUN mkdir -p vrep_ws/src && \
    cd vrep_ws && \
    catkin config \
        --profile release -x _release \
        --extend /opt/ros/kinetic \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DLIBPLUGIN_DIR=$COPPELIASIM_ROOT_DIR/programming/libPlugin && \
    catkin profile set release

RUN cd vrep_ws/src && \
    git clone --recursive https://github.com/CoppeliaRobotics/simExtROSInterface.git sim_ros_interface

RUN cd vrep_ws && \
    catkin build && \
    cp devel_release/lib/libsimExtROSInterface.so ~/V-REP