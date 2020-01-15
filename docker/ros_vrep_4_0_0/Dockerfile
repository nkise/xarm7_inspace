FROM vrep_4_0_0

WORKDIR /root

RUN apt-get update -y && apt-get install -y \
    lsb-release

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full \
    python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update