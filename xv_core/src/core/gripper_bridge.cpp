//
// Created by Denys Kotelovych on 1/3/20.
//

#include <xarm_msgs/GripperMove.h>
#include <xarm_msgs/GripperConfig.h>

#include "xv_core/core/gripper_bridge.h"

static const double kGripperMaxValue = 800.0;
static const double kGripperSpeed = 1500.0;

GripperBridge::GripperBridge(const ros::NodeHandle &node, const ros::NodeHandle &node_private)
        : node_(node), node_private_(node_private) {
    gripper_move_service_ = node_.serviceClient<xarm_msgs::GripperMove>("/gripper_move");
    gripper_config_service_ = node_.serviceClient<xarm_msgs::GripperConfig>("/gripper_config");

    GripperConfig(kGripperSpeed);
}

void GripperBridge::Take() {
    PositionPercentage(0.2);
}

void GripperBridge::Release() {
    PositionPercentage(1.0);
}

void GripperBridge::PositionAbsolute(double value) {
    GripperMove(value);
}

void GripperBridge::PositionPercentage(double value) {
    assert(0.0 <= value && value <= 1.0);
    GripperMove(value * kGripperMaxValue);
}

bool GripperBridge::GripperMove(double position) {
    xarm_msgs::GripperMove msg;
    msg.request.pulse_pos = position;

    if (gripper_move_service_.call(msg)) {
        ROS_DEBUG_STREAM_NAMED("gripper", "Moved by: " << position);
        return true;
    }

    ROS_ERROR_STREAM_NAMED("gripper", "Failed to move by value: " << position);
    return false;
}

bool GripperBridge::GripperConfig(double speed) {
    xarm_msgs::GripperConfig msg;
    msg.request.pulse_vel = speed;

    if (gripper_config_service_.call(msg)) {
        ROS_DEBUG_STREAM_NAMED("gripper", "Configured gripper speed to: " << speed);
        return true;
    }

    ROS_ERROR_STREAM_NAMED("gripper", "Failed to configure gripper speed");
    return false;
}
