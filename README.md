# xarm_vrep

## Install

Init workspace

```bash
mkdir -p ws/src
cd ws/src
git clone https://github.com/xArm-Developer/xarm_ros
git clone <current_repo>
```

Build docker images

```bash
cd ws/src/docker/
cd vrep_4_0_0
./build.sh
cd ../ros_vrep_4_0_0
./build.sh
cd ../ros_vrep_4_0_0_ws
./build.sh
```

Run docker container

```bash
cd ws/src/docker/ros_vrep_4_0_0_ws
./run.sh
```

Inside docker container

```bash
cd /root/ws
rosdep install --from-paths src --ignore-src -r -y
./src/xarm_vrep/scripts/catkin_init_profiles.sh
catkin build
```