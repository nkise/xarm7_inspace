//
// Created by Denys Kotelovych on 1/3/20.
//

#ifndef XV_CORE_GRIPPER_BRIDGE_H
#define XV_CORE_GRIPPER_BRIDGE_H

#include <ros/ros.h>

class GripperBridge {
public:
    GripperBridge(const ros::NodeHandle &node, const ros::NodeHandle &node_private);

public:
    void Take();

    void Release();

    void PositionAbsolute(double value);

    void PositionPercentage(double value);

private:
    bool GripperMove(double position);

    bool GripperConfig(double speed);

private:
    ros::NodeHandle node_, node_private_;
    ros::ServiceClient gripper_move_service_;
    ros::ServiceClient gripper_config_service_;
};

#endif //XV_CORE_GRIPPER_BRIDGE_H
