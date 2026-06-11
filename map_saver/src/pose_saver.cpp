#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <fstream>

class MapSaverNode : public rclcpp::Node {
public:
    MapSaverNode() : Node("map_saver_node") {
        accumulated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        // Subscriptions
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&MapSaverNode::odom_callback, this, std::placeholders::_1));

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_map", 10, std::bind(&MapSaverNode::cloud_callback, this, std::placeholders::_1));

        pose_file_.open("poses.txt");
        RCLCPP_INFO(this->get_logger(), "ROS 2 Map Saver Active. Waiting for data...");
    }

    ~MapSaverNode() {
        RCLCPP_INFO(this->get_logger(), "Shutting down... Saving Map!");
        
        if (!accumulated_cloud_->empty()) {
            pcl::io::savePCDFileBinary("registered_cloud.pcd", *accumulated_cloud_);
            RCLCPP_INFO(this->get_logger(), "SUCCESS: Saved %lu points to registered_cloud.pcd", accumulated_cloud_->size());
        } else {
            RCLCPP_WARN(this->get_logger(), "No map data received.");
        }

        if (pose_file_.is_open()) {
            pose_file_.close();
            RCLCPP_INFO(this->get_logger(), "SUCCESS: Saved flight path to poses.txt");
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg;
        if (pose_file_.is_open()) {
            pose_file_ << msg->pose.pose.position.x << " "
                       << msg->pose.pose.position.y << " "
                       << msg->pose.pose.position.z << " "
                       << msg->pose.pose.orientation.x << " "
                       << msg->pose.pose.orientation.y << " "
                       << msg->pose.pose.orientation.z << " "
                       << msg->pose.pose.orientation.w << "\n";
        }
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!current_pose_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Odometry...");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud_in);

        // Convert ROS Odometry to Eigen Matrix
        Eigen::Quaternionf q(
            current_pose_->pose.pose.orientation.w,
            current_pose_->pose.pose.orientation.x,
            current_pose_->pose.pose.orientation.y,
            current_pose_->pose.pose.orientation.z
        );
        Eigen::Vector3f t(
            current_pose_->pose.pose.position.x,
            current_pose_->pose.pose.position.y,
            current_pose_->pose.pose.position.z
        );

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = q.toRotationMatrix();
        transform.block<3, 1>(0, 3) = t;

        // Transform and Accumulate
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud_in, *cloud_transformed, transform);
        *accumulated_cloud_ += *cloud_transformed;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Building Map... Current Size: %lu points", accumulated_cloud_->size());
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    nav_msgs::msg::Odometry::SharedPtr current_pose_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    std::ofstream pose_file_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapSaverNode>());
    rclcpp::shutdown();
    return 0;
}
