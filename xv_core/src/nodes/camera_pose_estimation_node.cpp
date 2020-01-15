//
// Created by Denys Kotelovych on 1/8/20.
//

#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_stl_containers/eigen_stl_containers.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class CameraPoseEstimator {
public:
    struct Config {
        explicit Config(const ros::NodeHandle &node) {
            node.getParam("marker_ids", MarkerIds);
            node.getParam("marker_frame_ids", MarkerFrameIds);
            node.getParam("robot_frame_id", RobotFrameId);
            node.getParam("fixed_frame_id", FixedFrameId);
            node.getParam("camera_base_frame_id", CameraBaseFrameId);
            node.getParam("camera_optical_frame_id", CameraOpticalFrameId);
            node.getParam("fiducials_used_for_transform_count", FiducialsUsedForTransformCount);

            for (std::size_t i = 0; i < MarkerIds.size(); ++i)
                MarkerIdToFrameMap[MarkerIds[i]] = MarkerFrameIds[i];
        }

        std::vector<std::int32_t> MarkerIds;
        std::vector<std::string> MarkerFrameIds;
        std::map<std::int32_t, std::string> MarkerIdToFrameMap;
        std::string RobotFrameId;
        std::string FixedFrameId;
        std::string CameraBaseFrameId;
        std::string CameraOpticalFrameId;
        std::int32_t FiducialsUsedForTransformCount;

        friend std::ostream &operator<<(std::ostream &os, const Config &config) {
            os << "Robot Frame Id: [ " << config.RobotFrameId << " ], ";
            os << "Fixed Frame Id: [ " << config.FixedFrameId << " ], ";
            os << "Camera Base Frame Id: [ " << config.CameraBaseFrameId << " ], ";
            os << "Camera Optical Frame Id: [ " << config.CameraOpticalFrameId << " ], ";
            os << "Marker ids: [ ";
            std::copy(
                    config.MarkerIds.begin(),
                    config.MarkerIds.end(),
                    std::ostream_iterator<std::int32_t>(os, ", "));
            os << "], ";
            os << "Marker frame ids: [ ";
            std::copy(
                    config.MarkerFrameIds.begin(),
                    config.MarkerFrameIds.end(),
                    std::ostream_iterator<std::string>(os, ", "));
            os << "]";
            return os;
        }
    };

public:
    CameraPoseEstimator(const ros::NodeHandle &node, const ros::NodeHandle &node_private)
            : config_(node_private),
              node_(node),
              node_private_(node_private),
              rviz_("kinect2_rgb_optical_frame", "camera_pose_estimator_rviz_tools"),
              camera_to_fixed_frame_transform_found_(false) {
        subs_.FiducialTransforms = node_.subscribe(
                "/fiducial_transforms", 1, &CameraPoseEstimator::FiducialTransformsMessageReceived, this);
        ROS_DEBUG_STREAM_NAMED("camera_pose_estimation_node", "Config: " << config_);

        timer_ = node_.createTimer(ros::Rate(20.0), &CameraPoseEstimator::TimerCallback, this);
    }

    ~CameraPoseEstimator() {
        timer_.stop();
        subs_.FiducialTransforms.shutdown();
    }

private:
    void FiducialTransformsMessageReceived(const fiducial_msgs::FiducialTransformArrayConstPtr &msg) {
        ROS_DEBUG_STREAM_NAMED("camera_pose_estimation_node", "CameraPoseEstimator::FiducialTransformsMessageReceived");

        if (msg->transforms.empty()) {
            ROS_WARN_STREAM_NAMED("camera_pose_estimation_node", "No visible aruco markers");
            return;
        }

        if (fiducial_transform_msgs_.size() >= config_.FiducialsUsedForTransformCount) {
            subs_.FiducialTransforms.shutdown();
            camera_to_fixed_frame_transform_ = CameraTransformFromFiducialMsgs(fiducial_transform_msgs_);
            camera_to_fixed_frame_transform_found_ = true;
            return;
        }

        fiducial_transform_msgs_.insert(
                fiducial_transform_msgs_.begin(), msg->transforms.begin(), msg->transforms.end());

        VisualizeTransformMsgs(fiducial_transform_msgs_);
    }

    void TimerCallback(const ros::TimerEvent &e) {
        ROS_DEBUG_STREAM_NAMED("camera_pose_estimation_node", "CameraPoseEstimator::TimerCallback");
        ROS_DEBUG_STREAM_COND_NAMED(
                !camera_to_fixed_frame_transform_found_, "camera_pose_estimation_node", "Camera tranform not found");

        if (camera_to_fixed_frame_transform_found_) {
            tf::StampedTransform transform_stamped(
                    camera_to_fixed_frame_transform_,
                    ros::Time::now(),
                    config_.FixedFrameId,
                    config_.CameraBaseFrameId);

            tf_broadcaster_.sendTransform(transform_stamped);
        }
    }

private:
    tf::Transform CameraTransformFromFiducialMsgs(
            const std::vector<fiducial_msgs::FiducialTransform> &fiducial_transform_msgs) {
        try {
            tf::StampedTransform camera_optical_to_base_transform;
            tf_listener_.lookupTransform(
                    config_.CameraOpticalFrameId,
                    config_.CameraBaseFrameId,
                    ros::Time(0),
                    camera_optical_to_base_transform);
            const auto &transforms = FiducialMsgsToTransformsInRobotBaseLink(fiducial_transform_msgs);
            const auto &camera_optical_frame_to_robot_base_link = AverageTransform(transforms);
            return camera_optical_frame_to_robot_base_link * camera_optical_to_base_transform;
        } catch (const tf::TransformException &e) {
            ROS_ERROR_STREAM_NAMED("camera_pose_estimation_node", "TF Failed: " << e.what());
            exit(1);
        }
    }

    std::vector<tf::Transform> FiducialMsgsToTransformsInRobotBaseLink(
            const std::vector<fiducial_msgs::FiducialTransform> &transform_msgs) const {
        std::vector<tf::Transform> result;
        for (const auto &transform_msg : transform_msgs) {
            if (!config_.MarkerIdToFrameMap.count(transform_msg.fiducial_id))
                continue;
            try {
                tf::StampedTransform transform;
                tf_listener_.lookupTransform(
                        config_.RobotFrameId,
                        config_.MarkerIdToFrameMap.at(transform_msg.fiducial_id),
                        ros::Time(0),
                        transform);
                tf::Transform marker_to_camera_transform;
                tf::transformMsgToTF(transform_msg.transform, marker_to_camera_transform);
                result.push_back(transform * marker_to_camera_transform.inverse());
            } catch (const tf::TransformException &e) {
                ROS_ERROR_STREAM_NAMED("camera_pose_estimation_node", "TF Failed: " << e.what());
            }
        }
        return result;
    }

    static tf::Transform AverageTransform(const std::vector<tf::Transform> &transforms) {
        tf::Vector3 avg_translation(0, 0, 0);
        for (const auto &transform : transforms)
            avg_translation += transform.getOrigin();
        avg_translation /= transforms.size();
        // TODO: add average rotation
        return tf::Transform(transforms.front().getRotation(), avg_translation);
    }

    void VisualizeTransformMsgs(const std::vector<fiducial_msgs::FiducialTransform> &transform_msgs) {
        rviz_.deleteAllMarkers();
        for (const auto &transform_msg : transform_msgs) {
            Eigen::Isometry3d pose;
            tf::transformMsgToEigen(transform_msg.transform, pose);
            rviz_.publishAxis(pose);
        }
        rviz_.trigger();
    }

private:
    Config config_;
    ros::NodeHandle node_, node_private_;
    rviz_visual_tools::RvizVisualTools rviz_;
    ros::Timer timer_;
    struct {
        ros::Subscriber FiducialTransforms;
    } subs_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::Transform camera_to_fixed_frame_transform_;
    bool camera_to_fixed_frame_transform_found_;
    std::vector<fiducial_msgs::FiducialTransform> fiducial_transform_msgs_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "camera_pose_estimation_node");

    if (!ros::ok()) {
        ROS_ERROR_STREAM_NAMED("camera_pose_estimation_node", "ROS is not initialized");
        exit(1);
    }

    ros::NodeHandle node, node_private("~");
    CameraPoseEstimator cpe(node, node_private);
    ros::spin();

    return 0;
}

