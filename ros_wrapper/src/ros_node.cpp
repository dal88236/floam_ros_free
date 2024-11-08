#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "floam_ros_free/system.h"

#include <iostream>
#include <memory>

namespace floam_ros {

class FloamNode : public rclcpp::Node {
 public:
  FloamNode(std::string node_name, const std::string& config_file_path) : rclcpp::Node(node_name) {
    RCLCPP_INFO(get_logger(), "Initializing Floam Node...");
    
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", 10, std::bind(&FloamNode::pointCloudReceiveCallback, this, std::placeholders::_1));

    system_ = new floam::System(config_file_path, 
                                std::bind(&FloamNode::publishMap, this, std::placeholders::_1), 
                                std::bind(&FloamNode::publishOdom, this, std::placeholders::_1, std::placeholders::_2));
  }

  ~FloamNode() { 
    RCLCPP_INFO(get_logger(), "Closing Floam Node...");
    delete system_;
  }

 private:
  void publishMap(pcl::PointCloud<pcl::PointXYZI>& pc_in) {
    sensor_msgs::msg::PointCloud2 pc_msg;
    pcl::toROSMsg(pc_in, pc_msg);
    pc_msg.header.stamp = get_clock()->now();
    pc_msg.header.frame_id = "map";

    map_pub_->publish(pc_msg);
  }
  void publishOdom(const Eigen::Quaterniond& q_wb, const Eigen::Vector3d& t_wb) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.orientation.x = q_wb.x();
    odom_msg.pose.pose.orientation.y = q_wb.y();
    odom_msg.pose.pose.orientation.z = q_wb.z();
    odom_msg.pose.pose.orientation.w = q_wb.w();
    odom_msg.pose.pose.position.x = t_wb.x();
    odom_msg.pose.pose.position.y = t_wb.y();
    odom_msg.pose.pose.position.z = t_wb.z();

    odom_pub_->publish(odom_msg);
  }
  void pointCloudReceiveCallback(sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::fromROSMsg(*pointcloud_msg, *cloud);
    system_->track(cloud);
  }
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  floam::System* system_;
}; // class FloamNode

} // namespace floam_ros

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage shall be: floam_ros {config_file_path}" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  std::string config_path_path(argv[1]);
  auto node = std::make_shared<floam_ros::FloamNode>("floam_node", config_path_path);
  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}