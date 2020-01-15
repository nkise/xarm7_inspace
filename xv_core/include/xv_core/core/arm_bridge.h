//
// Created by Denys Kotelovych on 1/3/20.
//

#ifndef XV_CORE_ARM_H
#define XV_CORE_ARM_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

class ArmBridge {
public:
    ArmBridge(const ros::NodeHandle &node, const ros::NodeHandle &node_private);

    ~ArmBridge();

public:
    bool MoveToPose(const Eigen::Isometry3d &pose);

    bool MoveToPoseNamed(const std::string &pose_name);

    bool MoveToJoints(const std::map<std::string, double> &joints);

private:
    ros::NodeHandle node_, node_private_;
    ros::AsyncSpinner spinner_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr_;
};

#endif //XV_CORE_ARM_H
