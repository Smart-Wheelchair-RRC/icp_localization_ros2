/*
 * ICPlocalization.hpp
 *
 *  Created on: Apr 23, 2021
 *      Author: jelavice
 */

#pragma once
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include "pointmatcher/PointMatcher.h"
#include <atomic>
#include <mutex>        // ADD 1 of 4 — needed for map_mutex_ below
#include <thread>

#include "icp_localization_ros2/RangeDataAccumulator.hpp"
#include "icp_localization_ros2/common/typedefs.hpp"
#include "icp_localization_ros2/helpers.hpp"
#include "pointmatcher/IO.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/voxel_grid.h>

#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ADD 2 of 4 — service includes (all from std_srvs, no new dependencies)
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace icp_loco {

class TfPublisher;
class FrameTracker;
class ImuTracker;

class ICPlocalization : public rclcpp::Node {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ICPlocalization(const rclcpp::NodeOptions &options);
  ~ICPlocalization();
  void setMapCloud(const Pointcloud::Ptr map);
  void setInitialPose(const Eigen::Vector3d &p, const Eigen::Quaterniond &q);
  void initialize();
  DP fromPCL(const Pointcloud &pcl);
  void matchScans();

  void icpWorker();
  void publishPose() const;
  void publishRegisteredCloud() const;
  const std::string &getFixedFrame() const;
  void set2DPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr &msg);

  void initializeInternal();

private:
  // ── ADD 3 of 4 — service callback declarations ───────────────────────────
  //
  // /pause_localization  (std_srvs/SetBool)
  //   data=true  → icpWorker stops calling matchScans, keeps TF alive
  //   data=false → icpWorker resumes normally
  void callbackPauseLocalization(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  // /reset_localization  (std_srvs/Empty)
  //   Resets isFirstScanMatch_, zeroes pose, drains stale scans from the
  //   accumulator. Call this AFTER pause, BEFORE unpause on a new floor.
  void callbackResetLocalization(
      const std::shared_ptr<std_srvs::srv::Empty::Request> req,
      std::shared_ptr<std_srvs::srv::Empty::Response> res);

  // /load_map  (std_srvs/Trigger)
  //   Hot-swaps the ICP reference map. The map file path is read from the
  //   ROS parameter "map_path" which the supervisor sets before calling.
  //   Using Trigger avoids any dependency on example_interfaces.
  void callbackLoadMap(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // ── Existing private members — COMPLETELY UNCHANGED ───────────────────────
  Eigen::Vector3d userSetPosition_;
  Eigen::Quaterniond userSetQuaternion_;
  Eigen::Vector3d lastPosition_;
  Eigen::Quaterniond lastOrientation_;
  Eigen::Vector3d currentPosition_;
  Eigen::Quaterniond currentOrientation_;
  Pointcloud mapCloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      registeredCloudPublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initialPose_;
  pcl::VoxelGrid<pcl::PointXYZ> mapCloudFilter_;
  PM::ICPSequence icp_;
  PM::DataPointsFilters inputFilters_;
  DP refCloud_;
  DP regCloud_;
  bool isMapSet_ = false;
  mutable int seq_ = 0;
  std::shared_ptr<RangeDataAccumulatorRos> rangeDataAccumulator_;
  std::thread icpWorker_;
  PM::TransformationParameters optimizedPose_;
  Time optimizedPoseTimestamp_;
  Time lastOptimizedPoseTimestamp_;
  Time regCloudTimestamp_;
  std::shared_ptr<TfPublisher> tfPublisher_;
  std::shared_ptr<FrameTracker> frameTracker_;
  std::shared_ptr<ImuTracker> imuTracker_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::atomic<bool> isFirstScanMatch_{true};
  bool isUseOdometry_ = true;
  bool isSetPoseFromUser_ = false;
  std::string fixedFrame_ = "map";
  double rate_ = 10.0;
  rclcpp::TimerBase::SharedPtr tfRepublishTimer_;

  // ── ADD 4 of 4 — new members for the three services ──────────────────────
  std::atomic<bool>  is_paused_{false};   // icpWorker reads this every cycle
  mutable std::mutex map_mutex_;          // held during icp_() and setMapCloud()

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr   reset_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_map_srv_;
};

} // namespace icp_loco