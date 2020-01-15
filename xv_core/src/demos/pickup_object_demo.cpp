//
// Created by Denys Kotelovych on 1/3/20.
//

#include <ros/ros.h>

#include "xv_core/core/arm_bridge.h"
#include "xv_core/core/gripper_bridge.h"

static const std::map<std::string, double> kObject1JointValues = {
        {"xv_arm_joint1", -0.0372031},
        {"xv_arm_joint2", 0.39364},
        {"xv_arm_joint3", -0.0151552},
        {"xv_arm_joint4", 1.0421},
        {"xv_arm_joint5", 0.00862567},
        {"xv_arm_joint6", 0.648496},
        {"xv_arm_joint7", -0.0589075}
};

static const std::map<std::string, double> kObject2JointValues = {
        {"xv_arm_joint1", 0.22292494773864746},
        {"xv_arm_joint2", 0.026723496615886688},
        {"xv_arm_joint3", 0.158050075173378},
        {"xv_arm_joint4", 0.7329807281494141},
        {"xv_arm_joint5", -0.006531469989567995},
        {"xv_arm_joint6", 0.7064443230628967},
        {"xv_arm_joint7", 0.3856709897518158}
};

static const std::map<std::string, double> kObject3JointValues = {
        {"xv_arm_joint1", -0.11606179922819138},
        {"xv_arm_joint2", 0.318509578704834},
        {"xv_arm_joint3", -0.36924245953559875},
        {"xv_arm_joint4", 1.0500069856643677},
        {"xv_arm_joint5", 0.16431912779808044},
        {"xv_arm_joint6", 0.7581155896186829},
        {"xv_arm_joint7", -0.5882434844970703}
};

static void PickupObject(ArmBridge &arm_bridge,
                         GripperBridge &gripper_bridge,
                         const std::map<std::string, double> &joint_values) {
    gripper_bridge.Release();

    arm_bridge.MoveToPoseNamed("up");
    arm_bridge.MoveToPoseNamed("prepare");
    arm_bridge.MoveToJoints(joint_values);

    gripper_bridge.Take();
    sleep(2);

    arm_bridge.MoveToPoseNamed("prepare");
    arm_bridge.MoveToPoseNamed("up");
    arm_bridge.MoveToPoseNamed("prepare_reverse");
    arm_bridge.MoveToPoseNamed("drop_box");

    gripper_bridge.Release();
    sleep(2);

    arm_bridge.MoveToPoseNamed("prepare_reverse");
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pickup_object_demo");

    if (!ros::ok()) {
        ROS_ERROR_STREAM_NAMED("pickup_object_demo", "ROS is not initialized");
        exit(1);
    }

    ros::NodeHandle node, node_private("~");
    ArmBridge arm_bridge(node, node_private);
    GripperBridge gripper_bridge(node, node_private);

    PickupObject(arm_bridge, gripper_bridge, kObject1JointValues);
    PickupObject(arm_bridge, gripper_bridge, kObject2JointValues);
    PickupObject(arm_bridge, gripper_bridge, kObject3JointValues);

    return 0;
}
