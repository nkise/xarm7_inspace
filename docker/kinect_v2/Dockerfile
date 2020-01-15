FROM ros_gazebo_9

WORKDIR /root

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
    git clone https://github.com/code-iai/iai_kinect2.git

RUN cd deps_ws && \
    catkin build