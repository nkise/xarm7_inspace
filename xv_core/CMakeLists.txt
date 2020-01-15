cmake_minimum_required(VERSION 2.8.3)
project(xv_core)

add_compile_options(-std=c++11)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	geometry_msgs
	std_msgs
	eigen_stl_containers
	eigen_conversions
	pcl_conversions
	pcl_ros
	rviz_visual_tools
	moveit_ros_planning
	moveit_ros_planning_interface
	xarm_msgs
	fiducial_msgs)

catkin_package(
    INCLUDE_DIRS
    	include
	LIBRARIES
		xv_core
	CATKIN_DEPENDS
		roscpp
		roslib
		geometry_msgs
		std_msgs
		eigen_stl_containers
		eigen_conversions
		pcl_conversions
		pcl_ros
		moveit_ros_planning
		moveit_ros_planning_interface
		xarm_msgs
		fiducial_msgs
	DEPENDS
		Eigen3)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
	${Eigen3_INCLUDE_DIRS})

add_library(xv_core
	src/core/gripper_bridge.cpp
	src/core/arm_bridge.cpp)
target_link_libraries(xv_core
	${catkin_LIBRARIES})

add_executable(pickup_object_demo
	src/demos/pickup_object_demo.cpp)
target_link_libraries(pickup_object_demo
	${catkin_LIBRARIES}
	xv_core)

add_executable(gripper_demo
	src/demos/gripper_demo.cpp)
target_link_libraries(gripper_demo
	${catkin_LIBRARIES}
	xv_core)

add_executable(gazebo_demo
	src/demos/gazebo_demo.cpp)
target_link_libraries(gazebo_demo
	${catkin_LIBRARIES}
	xv_core)

add_executable(camera_pose_estimation_node
	src/nodes/camera_pose_estimation_node.cpp)
target_link_libraries(camera_pose_estimation_node
	${catkin_LIBRARIES})

add_executable(object_detection_node
	src/nodes/object_detection_node.cpp)
target_link_libraries(object_detection_node
	${catkin_LIBRARIES})
