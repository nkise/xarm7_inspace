FROM ros_gazebo_9

WORKDIR /root

RUN apt-get update && apt-get install -y \
    ros-kinetic-tf2-sensor-msgs \
    ros-kinetic-control-toolbox \
    ros-kinetic-moveit \
    ros-kinetic-moveit-visual-tools \
    ros-kinetic-ros-control \
    ros-kinetic-ros-controllers \
    ros-kinetic-gazebo9-ros \
    ros-kinetic-gazebo9-ros-control \
    ros-kinetic-gazebo9-ros-pkgs \
    openssh-server \
    gdb \
    gdbserver \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /var/run/sshd
RUN echo 'root:root' | chpasswd
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

RUN git clone https://github.com/OpenKinect/libfreenect2.git
RUN apt update && apt install -y \
    libturbojpeg \
    libjpeg-turbo8-dev \
    libglfw3

RUN cd libfreenect2  && \
    mkdir build && \
    cd build && \
    cmake .. -DENABLE_CXX11=ON && \
    make -j8 && \
    make install

RUN mkdir -p deps_ws/src && \
    cd deps_ws && \
    catkin config \
        --profile release -x _release \
        --extend /opt/ros/kinetic \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin profile set release

RUN cd deps_ws/src && \
    git clone https://github.com/xArm-Developer/xarm_ros.git && \
    git clone https://github.com/UbiquityRobotics/fiducials.git && \
    git clone https://github.com/code-iai/iai_kinect2.git

RUN cd deps_ws && catkin build