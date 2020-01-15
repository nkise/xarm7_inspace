//
// Created by Denys Kotelovych on 1/14/20.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

class ObjectDetector {
public:
    struct Config {
        explicit Config(const ros::NodeHandle &node) {
            node.getParam("input_point_cloud_topic", InputPointCloudTopic);
        }

        std::string InputPointCloudTopic;
    };

public:
    ObjectDetector(const ros::NodeHandle &node, const ros::NodeHandle &node_private)
            : config_(node_private),
              node_(node),
              node_private_(node_private),
              rviz_("world", "object_detection_rviz_tools") {
        subs_.PointCloud = node_.subscribe(
                config_.InputPointCloudTopic, 1, &ObjectDetector::PointCloudMessageReceived, this);

        ROS_DEBUG_STREAM_NAMED("object_detection_node", "Init");
    }

private:
    void PointCloudMessageReceived(const sensor_msgs::PointCloud2ConstPtr &msg) {
        ROS_DEBUG_STREAM_NAMED("object_detection_node", "ObjectDetector::PointCloudMessageReceived");

        pcl::PCLPointCloud2::Ptr pcl_point_cloud_ptr(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*msg, *pcl_point_cloud_ptr);

        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*pcl_point_cloud_ptr, *xyz_cloud_ptr);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(xyz_cloud_ptr);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.03);
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(xyz_cloud_ptr);
        ec.extract(cluster_indices);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                cloud_cluster->points.push_back(xyz_cloud_ptr->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);
        }

        ROS_DEBUG_STREAM_NAMED("object_detection_node", "Found clusters: " << clusters.size());

        std::int32_t i = 0;
        for (const auto &cluster : clusters) {
            std::vector<geometry_msgs::Point> cluster_points;
            cluster_points.reserve(cluster->points.size());

            for (auto point : cluster->points) {
                geometry_msgs::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                cluster_points.push_back(p);
            }

            rviz_.publishSpheres(
                    cluster_points,
                    static_cast<rviz_visual_tools::colors>(i % 20),
                    0.01,
                    "cluster_" + std::to_string(i));
            i++;
        }
        rviz_.trigger();
    }

private:
    Config config_;
    ros::NodeHandle node_, node_private_;
    rviz_visual_tools::RvizVisualTools rviz_;
    struct {
        ros::Subscriber PointCloud;
    } subs_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "object_detection_node");

    if (!ros::ok()) {
        ROS_ERROR_STREAM_NAMED("object_detection_node", "ROS is not initialized");
        exit(1);
    }

    ros::NodeHandle node, node_private("~");
    ObjectDetector od(node, node_private);
    ros::spin();

    return 0;
}

