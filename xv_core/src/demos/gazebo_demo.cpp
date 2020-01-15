//
// Created by Denys Kotelovych on 1/6/20.
//

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <gazebo_msgs/ModelStates.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>
#include <string>

#include "xv_core/core/gripper_bridge.h"
#include "xv_core/core/arm_bridge.h"

struct PickupObject {
    Eigen::Isometry3d Pose;
    Eigen::Isometry3d PoseAbove;
    std::string Name;

    static PickupObject CreateFromMsg(const geometry_msgs::Pose &msg, const std::string &name) {
        Eigen::Isometry3d pose;
        tf::poseMsgToEigen(msg, pose);

        const auto z_dir = pose.rotation() * Eigen::Vector3d::UnitZ();
        auto pose_above = pose;
        pose_above.translate(z_dir * 0.1);
        Eigen::AngleAxisd axis(M_PI, Eigen::Vector3d(1, 0, 0));
        Eigen::Quaterniond rotation(axis);
        Eigen::Translation3d translation(Eigen::Vector3d(
                pose_above.translation().x(),
                pose_above.translation().y(),
                pose_above.translation().z()));

        return {pose, translation * rotation, name};
    }
};

std::ostream &operator<<(std::ostream &os, const PickupObject &object) {
    os << "[ " << object.Name << " ]";
    return os;
}

class PickupObjectProvider {
public:
    PickupObjectProvider(const ros::NodeHandle &node, const ros::NodeHandle &node_private)
            : node_(node),
              node_private_(node_private) {
        subs_.ModelStates = node_.subscribe(
                "/gazebo/model_states", 1, &PickupObjectProvider::GazeboModelStatesMessageReceived, this);
    }

    ~PickupObjectProvider() {
        subs_.ModelStates.shutdown();
    }

public:
    const std::vector<PickupObject> &Objects() const { return objects_; }

private:
    void GazeboModelStatesMessageReceived(const gazebo_msgs::ModelStatesConstPtr &msg) {
        ROS_DEBUG_STREAM_NAMED("gazebo_demo", "PickupObjectProvider::GazeboModelStates");

        const size_t old_size = objects_.size();
        for (std::size_t i = 0; i < msg->name.size(); ++i) {
            const auto &it = msg->name[i].find("box");
            if (it != std::string::npos)
                objects_.push_back(PickupObject::CreateFromMsg(msg->pose[i], msg->name[i]));
        }

        if (old_size != objects_.size())
            subs_.ModelStates.shutdown();
    }

private:
    ros::NodeHandle node_, node_private_;
    struct {
        ros::Subscriber ModelStates;
    } subs_;
    std::vector<PickupObject> objects_;
};

class Picker {
public:
    Picker(const ros::NodeHandle &node, const ros::NodeHandle &node_private)
            : node_(node),
              node_private_(node_private),
              rviz_("world", "rviz_visual_tools"),
              pickup_object_provider_(node, node_private),
              arm_(node, node_private),
              gripper_(node, node_private) {
        timer_ = node_.createTimer(ros::Duration(1, 0), &Picker::TimerCallback, this);

        arm_.MoveToPoseNamed("zero");
    }

    ~Picker() {
        timer_.stop();
    }

private:
    void TimerCallback(const ros::TimerEvent &e) {
        ROS_DEBUG_STREAM_NAMED("gazebo_demo", "Picker::TimerCallback");

        const auto &objects = pickup_object_provider_.Objects();

        if (objects.empty()) {
            ROS_DEBUG_STREAM_NAMED("gazebo_demo", "No objects are found");
            return;
        }

        if (picked_object_names_.size() == objects.size()) {
            arm_.MoveToPoseNamed("zero");
            picked_object_names_.clear();
        }

        std::stringstream ss;
        std::copy(objects.begin(), objects.end(), std::ostream_iterator<PickupObject>(ss, " "));
        ROS_DEBUG_STREAM_NAMED("gazebo_demo", "Objects to pickup: " << ss.str());

        for (const auto &object : objects) {
            if (picked_object_names_.count(object.Name))
                continue;

            arm_.MoveToPoseNamed("prepare");

            rviz_.publishAxis(object.Pose);
            rviz_.publishAxis(object.PoseAbove);
            rviz_.trigger();

            if (arm_.MoveToPose(object.PoseAbove))
                picked_object_names_.insert(object.Name);
        }
    }

private:
    ros::NodeHandle node_, node_private_;
    ros::Timer timer_;
    rviz_visual_tools::RvizVisualTools rviz_;
    PickupObjectProvider pickup_object_provider_;
    ArmBridge arm_;
    GripperBridge gripper_;
    std::set<std::string> picked_object_names_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "gazebo_demo");

    if (!ros::ok()) {
        ROS_ERROR_STREAM_NAMED("gazebo_demo", "ROS is not initialized");
        exit(1);
    }

    ros::NodeHandle node, node_private("~");
    Picker picker(node, node_private);
    ros::waitForShutdown();
    return 0;
}