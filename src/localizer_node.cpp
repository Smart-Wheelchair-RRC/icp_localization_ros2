#include "icp_localization_ros2/ICPlocalization.hpp"
#include "icp_localization_ros2/RangeDataAccumulator.hpp"
#include "icp_localization_ros2/common/typedefs.hpp"
#include "icp_localization_ros2/helpers.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace icp_loco;

Pointcloud::Ptr loadPointcloudFromPcd(const std::string &filename) {
  Pointcloud::Ptr cloud(new Pointcloud);
  pcl::PCLPointCloud2 cloudBlob;
  if (pcl::io::loadPCDFile(filename, cloudBlob) == -1) {
    std::cerr << "❌ [PCL Loader] Failed to read PCD file: " << filename << std::endl;
    return nullptr;
  }
  pcl::fromPCLPointCloud2(cloudBlob, *cloud);
  return cloud;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ICPlocalization>(rclcpp::NodeOptions{});
  node->initializeInternal();

  // Create Standard Reliable Publisher
  auto cloudPub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "icp_map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  const std::string pclFilename = node->declare_parameter("pcd_file_path", "");

  std::cout << "🔍 [PCL Loader] Target PCD Path: '" << pclFilename << "'" << std::endl;

  Pointcloud::Ptr mapCloud = nullptr;
  if (pclFilename.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'pcd_file_path' is EMPTY! No map will be published.");
  } else {
    mapCloud = loadPointcloudFromPcd(pclFilename);
    if (mapCloud) {
      RCLCPP_INFO(node->get_logger(), "✅ Successfully loaded PCD map with %zu points", mapCloud->points.size());
    }
  }

  const Eigen::Quaterniond qInit = icp_loco::getOrientationFromParameterServer(node, "initial_pose.", true);
  const Eigen::Vector3d pInit = icp_loco::getPositionFromParameterServer(node, "initial_pose.");

  if (mapCloud) {
    node->setMapCloud(mapCloud);
  }
  node->setInitialPose(pInit, qInit);
  node->initialize();

  // Convert PCL to ROS Message once to avoid redundant serialization every second
  auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  if (mapCloud) {
    pcl::toROSMsg(*mapCloud, *ros_msg);
    ros_msg->header.frame_id = node->getFixedFrame();
  }

  // Periodic 1Hz timer created directly on the node instance
  auto mapTimer = node->create_wall_timer(
      std::chrono::seconds(1),
      [node, cloudPub, ros_msg, mapCloud]() {
        if (mapCloud && cloudPub) {
          ros_msg->header.stamp = node->now();
          cloudPub->publish(*ros_msg);
        }
      });

  RCLCPP_INFO(node->get_logger(), "🚀 ICP Localization Node & /icp_map publisher online.");
  rclcpp::spin(node);
  return 0;
}