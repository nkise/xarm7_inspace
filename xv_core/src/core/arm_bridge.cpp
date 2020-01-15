//
// Created by Denys Kotelovych on 1/3/20.
//

#include "xv_core/core/arm_bridge.h"

ArmBridge::ArmBridge(const ros::NodeHandle &node, const ros::NodeHandle &node_private)
        : node_(node),
          node_private_(node_private),
          spinner_(2) {
    move_group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("manipulator");
    move_group_ptr_->setPoseReferenceFrame("world");
    move_group_ptr_->setEndEffectorLink("xv_gripper_gripping_link");
    move_group_ptr_->setMaxVelocityScalingFactor(0.75);
    move_group_ptr_->setMaxAccelerationScalingFactor(0.75);

    spinner_.start();
    sleep(1);
}

ArmBridge::~ArmBridge() {
    spinner_.stop();
}

bool ArmBridge::MoveToPose(const Eigen::Isometry3d &pose) {
    ROS_DEBUG_STREAM_NAMED("arm_bridge", "ArmBridge::MoveToPose");

    move_group_ptr_->setStartStateToCurrentState();
    move_group_ptr_->setApproximateJointValueTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto &result = move_group_ptr_->plan(plan);

    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR_STREAM_NAMED("arm_bridge", "Failed to build trajectory");
        return false;
    }

    if (!move_group_ptr_->execute(plan)) {
        ROS_ERROR_STREAM_NAMED("arm_bridge", "Failed to execute trajectory");
        return false;
    }

    return true;
}

bool ArmBridge::MoveToPoseNamed(const std::string &pose_name) {
    ROS_DEBUG_STREAM_NAMED("arm_bridge", "ArmBridge::MoveToPoseNamed");

    const auto &joints = move_group_ptr_->getNamedTargetValues(pose_name);
    return MoveToJoints(joints);
}

bool ArmBridge::MoveToJoints(const std::map<std::string, double> &joints) {
    ROS_DEBUG_STREAM_NAMED("arm_bridge", "ArmBridge::MoveToJoints");

    move_group_ptr_->setStartStateToCurrentState();
    move_group_ptr_->setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto &result = move_group_ptr_->plan(plan);

    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR_STREAM_NAMED("arm_bridge", "Failed to move to joints");
        return false;
    }

    if (!move_group_ptr_->execute(plan)) {
        ROS_ERROR_STREAM_NAMED("arm_bridge", "Failed to execute trajectory");
        return false;
    }

    return true;
}
