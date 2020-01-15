FROM nvidia/opengl:1.0-glvnd-devel-ubuntu16.04

WORKDIR /root

RUN apt-get update -y && apt-get install -y \
    vim \
    tmux \
    git \
    wget \
    lsb-release \
    lsb-core \
    tmux \
    vim \
    build-essential \
    g++

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full \
    python-catkin-tools

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get install -y gazebo9 libgazebo9-dev

RUN rosdep init && rosdep fix-permissions && rosdep update
