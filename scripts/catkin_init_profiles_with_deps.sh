#!/usr/bin/env bash

catkin config \
    --profile release -x _release \
    --extend /root/deps_ws/devel_release \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

catkin config \
    --profile debug -x _debug \
    --extend /root/deps_ws/devel_release \
    --cmake-args -DCMAKE_BUILD_TYPE=Debug

catkin profile set debug