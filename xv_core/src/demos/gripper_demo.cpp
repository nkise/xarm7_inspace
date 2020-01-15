//
// Created by Denys Kotelovych on 1/4/20.
//

#include <ros/ros.h>

#include "xv_core/core/gripper_bridge.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "gripper_demo");

    if (!ros::ok()) {
        ROS_ERROR_STREAM_NAMED("gripper_demo", "ROS is not initialized");
        exit(1);
    }

    ros::NodeHandle node, node_private("~");
    GripperBridge gripper_bridge(node, node_private);

    gripper_bridge.Take();
    sleep(2);

    gripper_bridge.Release();
    sleep(2);

    return 0;
}

