/*
 * ICPlocalization.cpp
 *
 *  Created on: Apr 23, 2021
 *      Author: jelavice
 */
#include "icp_localization_ros2/ICPlocalization.hpp"
#include "icp_localization_ros2/RangeDataAccumulator.hpp"
#include "icp_localization_ros2/common/typedefs.hpp"
#include "icp_localization_ros2/helpers.hpp"
#include "icp_localization_ros2/transform/FrameTracker.hpp"
#include "icp_localization_ros2/transform/ImuTracker.hpp"
#include "icp_localization_ros2/transform/TfPublisher.hpp"
#include "pointmatcher/IO.h"
#include "pointmatcher/PointMatcher.h"
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <memory>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace icp_loco {

namespace {
const double kRadToDeg = 180.0 / M_PI;

inline bool hasMatchingPointCounts(const DP& cloud) {
  const auto nFeat = cloud.features.cols();
  const auto nDesc = cloud.descriptors.cols();
  return nFeat == nDesc && nFeat > 0;
}
}

// =============================================================================
// Constructor — UNCHANGED
// =============================================================================
ICPlocalization::ICPlocalization(const rclcpp::NodeOptions &options)
    : Node("icp_localization", options) {
  std::vector<double> leafSize =
      this->declare_parameter("leaf_size", std::vector<double>{0.1, 0.1, 0.1});
  if (leafSize.size() != 3) {
    RCLCPP_ERROR(this->get_logger(), "Leaf size must have 3 elements");
    leafSize = {0.1, 0.1, 0.1};
  }
  mapCloudFilter_.setLeafSize(leafSize[0], leafSize[1], leafSize[2]);
}

// =============================================================================
// Destructor — CHANGE: added joinable() guard (prevents crash on early exit)
// =============================================================================
ICPlocalization::~ICPlocalization() {
  if (tfRepublishTimer_) {
    tfRepublishTimer_->cancel();
  }
  // CHANGE: wake the worker if it is sleeping in the paused branch, then join
  is_paused_.store(false);
  if (icpWorker_.joinable()) {   // CHANGE: was unconditional .join()
    icpWorker_.join();
  }
  Rigid3d lastPose(lastPosition_, lastOrientation_);
  std::cout << "ICP_LOCO: Last transform map to range sensor: \n";
  std::cout << lastPose.asString() << "\n";
}

// =============================================================================
// setMapCloud — CHANGE: wrapped in map_mutex_ lock
//
// Why: callbackLoadMap() calls this from a service-callback thread while
// icpWorker() may be mid-way through icp_(). Without the lock, both threads
// write to icp_ and refCloud_ at the same time → undefined behaviour.
// The lock is always taken before modifying or reading these two objects.
// =============================================================================
void ICPlocalization::setMapCloud(const Pointcloud::Ptr map) {
  std::lock_guard<std::mutex> lock(map_mutex_);   // CHANGE: added lock

  mapCloudFilter_.setInputCloud(map);
  mapCloudFilter_.filter(mapCloud_);
  refCloud_ = fromPCL(mapCloud_);
  isMapSet_ = true;
  icp_.setMap(refCloud_);

  std::cout << "Map size is: " << map->points.size() << " (before filter), "
            << mapCloud_.points.size() << " (after filter)" << std::endl;
}

// =============================================================================
// setInitialPose — UNCHANGED
// =============================================================================
void ICPlocalization::setInitialPose(const Eigen::Vector3d &p,
                                     const Eigen::Quaterniond &q) {
  std::cout << "Init pose set to, xyz: " << p.transpose();
  std::cout << ", q: " << q.coeffs().transpose()
            << ", rpy: " << kRadToDeg * icp_loco::toRPY(q).transpose()
            << " deg \n";
  lastPosition_ = p;
  lastOrientation_ = q;
  imuTracker_->setInitialPose(p, q);
  tfPublisher_->setInitialPose(p, q);
}

// =============================================================================
// set2DPoseCallback — UNCHANGED
// =============================================================================
void ICPlocalization::set2DPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr &msg) {
  try {
    geometry_msgs::msg::Pose pose_received = msg->pose.pose;
    userSetPosition_ =
        Eigen::Vector3d(pose_received.position.x, pose_received.position.y,
                        pose_received.position.z);
    userSetQuaternion_ = Eigen::Quaterniond(
        pose_received.orientation.w, pose_received.orientation.x,
        pose_received.orientation.y, pose_received.orientation.z);
    isSetPoseFromUser_ = true;
  } catch (const std::exception &e) {
    std::cerr << "Caught exception while setting 2D pose: " << e.what() << '\n';
  }
  std::cout << "User set pose to :"
            << Rigid3d(userSetPosition_, userSetQuaternion_).asString()
            << std::endl;
}

// =============================================================================
// initializeInternal — CHANGE: registers the three services at the bottom
// =============================================================================
void ICPlocalization::initializeInternal() {

  rangeDataAccumulator_ =
      std::make_shared<RangeDataAccumulatorRos>(this->shared_from_this());
  imuTracker_ = std::make_shared<ImuTracker>();
  frameTracker_ = std::make_shared<FrameTracker>(imuTracker_);
  tfPublisher_ = std::make_shared<TfPublisher>(this->shared_from_this(),
                                               frameTracker_, imuTracker_);

  lastPosition_.setZero();
  lastOrientation_.setIdentity();

  registeredCloudPublisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "registered_cloud", rclcpp::QoS(rclcpp::KeepLast(1)));
  posePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "range_sensor_pose", rclcpp::QoS(rclcpp::KeepLast(1)));

  icp_.setDefault();

  tfBuffer_.reset(new tf2_ros::Buffer(this->get_clock()));
  tfListener_.reset(new tf2_ros::TransformListener(*tfBuffer_));

  // ── ADD: register services ───────────────────────────────────────────────
  // Declare map_path parameter here so it exists before load_map is ever called
  this->declare_parameter("map_path", std::string(""));

  pause_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "pause_localization",
      std::bind(&ICPlocalization::callbackPauseLocalization, this,
                std::placeholders::_1, std::placeholders::_2));

  reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset_localization",
      std::bind(&ICPlocalization::callbackResetLocalization, this,
                std::placeholders::_1, std::placeholders::_2));

  load_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "load_map",
      std::bind(&ICPlocalization::callbackLoadMap, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(),
              "Services registered: /pause_localization, "
              "/reset_localization, /load_map");
  // ── END ADD ───────────────────────────────────────────────────────────────
}

// =============================================================================
// initialize — UNCHANGED
// =============================================================================
void ICPlocalization::initialize() {

  const std::string configFileIcp =
      this->declare_parameter("icp_config_path", "");
  try {
    std::ifstream in(configFileIcp);
    if (!in.is_open()) {
      throw std::runtime_error("config file icp opening failed");
    }
    icp_.loadFromYaml(in);
    in.close();
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
  }

  const std::string configFileFilters =
      this->declare_parameter("input_filters_config_path", "");
  try {
    namespace fs = std::filesystem;
    fs::path pkgPath = ament_index_cpp::get_package_share_directory("icp_localization_ros2");
    fs::path fullPath = pkgPath / configFileFilters;
    std::ifstream in(fullPath.string());
    if (!in.is_open()) {
      throw std::runtime_error("config file filters opening failed");
    }
    inputFilters_ = PM::DataPointsFilters(in);
    in.close();
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
  }

  std::string rangeDataTopic =
      this->declare_parameter("icp_localization_ros2.range_data_topic", "");
  if (rangeDataTopic.empty()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "failed to load range data topic");
  }
  std::string imuDataTopic =
      this->declare_parameter("icp_localization_ros2.imu_data_topic", "");
  if (imuDataTopic.empty()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "failed to load imu data topic");
  }
  std::string odometryDataTopic =
      this->declare_parameter("icp_localization_ros2.odometry_data_topic", "");
  if (odometryDataTopic.empty()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "failed to load odometry data topic");
  }

  std::cout << "odometry data topic: " << odometryDataTopic << std::endl;
  std::cout << "imu data topic: " << imuDataTopic << std::endl;

  isUseOdometry_ =
      this->declare_parameter("icp_localization_ros2.is_use_odometry", true);
  std::cout << "Is use odometry: " << std::boolalpha << isUseOdometry_ << "\n";

  const bool isProvideOdomFrame = this->declare_parameter(
      "icp_localization_ros2.is_provide_odom_frame", true);
  std::cout << "Is provide odom frame: " << std::boolalpha
            << isProvideOdomFrame << "\n";

  tfPublisher_->setOdometryTopic(odometryDataTopic);
  tfPublisher_->setImuTopic(imuDataTopic);
  tfPublisher_->setIsProvideOdomFrame(isProvideOdomFrame);

  const double gravityVectorFilterTimeConstant = this->declare_parameter(
      "icp_localization_ros2.gravity_vector_filter_time_constant", 0.01);
  std::cout << "Gravity vector filter time constant: "
            << gravityVectorFilterTimeConstant << "\n";
  imuTracker_->setGravityVectorFilterTimeConstant(
      gravityVectorFilterTimeConstant);

  const std::string imuLidarPrefix = "calibration.imu_to_range_sensor.";
  const Rigid3d imuToRangeSensor = Rigid3d(
      getPositionFromParameterServer(this->shared_from_this(), imuLidarPrefix),
      getOrientationFromParameterServer(this->shared_from_this(), imuLidarPrefix));

  const std::string odometrySourceLidarPrefix =
      "calibration.odometry_source_to_range_sensor.";
  const Rigid3d odometrySourceToRangeSensor =
      Rigid3d(getPositionFromParameterServer(this->shared_from_this(),
                                             odometrySourceLidarPrefix),
              getOrientationFromParameterServer(this->shared_from_this(),
                                                odometrySourceLidarPrefix));
  frameTracker_->setTransformOdometrySourceToRangeSensor(
      odometrySourceToRangeSensor);
  frameTracker_->setTransformImuToRangeSensor(imuToRangeSensor);
  frameTracker_->setIsUseOdometryForRangeSensorPosePrediction(isUseOdometry_);

  const int minNumOdomMeasurements = this->declare_parameter(
      "icp_localization_ros2.min_num_odom_msgs_before_ready", 300);
  frameTracker_->setMinNumOdomMeasurementsBeforeReady(minNumOdomMeasurements);
  std::cout << "Min num odom measurements before ready: "
            << minNumOdomMeasurements << std::endl;

  std::cout << "Calibration: \n";
  std::cout << "imu to range sensor: " << imuToRangeSensor.asString() << "\n\n";
  std::cout << "odometry source to range sensor: "
            << odometrySourceToRangeSensor.asString() << "\n\n";

  RangeDataAccumulatorParamRos rangeDataAccParam;
  rangeDataAccParam.numAccumulatedRangeData_ = this->declare_parameter(
      "icp_localization_ros2.num_accumulated_range_data", 1);
  rangeDataAccParam.inputRangeDataTopic_ =
      this->get_parameter("icp_localization_ros2.range_data_topic").as_string();

  std::cout << "range data parameters: \n";
  std::cout << "topic: " << rangeDataAccParam.inputRangeDataTopic_ << "\n";
  std::cout << "num range data accumulated: "
            << rangeDataAccParam.numAccumulatedRangeData_ << "\n \n";

  fixedFrame_ =
      this->declare_parameter("icp_localization_ros2.fixed_frame", "map");
  std::cout << "Setting fixed frame to: " << fixedFrame_ << std::endl;

  rate_ = this->declare_parameter("icp_localization_ros2.rate", 1.0);
  std::cout << "Setting rate to: " << rate_ << std::endl;

  rangeDataAccumulator_->setParam(rangeDataAccParam);
  rangeDataAccumulator_->initialize();
  tfPublisher_->initialize();

  tfRepublishTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() {
        if (!isFirstScanMatch_.load() && frameTracker_->isReady()) {
          if (isUseOdometry_) {
            tfPublisher_->publishMapToOdom(optimizedPoseTimestamp_);
          } else {
            tfPublisher_->publishMapToRangeSensor(optimizedPoseTimestamp_);
          }
        }
      });

  std::cout << "ICPlocalization: Initialized \n";

  initialPose_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", rclcpp::QoS(rclcpp::KeepLast(1)),
          std::bind(&ICPlocalization::set2DPoseCallback, this,
                    std::placeholders::_1));

  auto callable = [this]() { icpWorker(); };
  icpWorker_ = std::thread(callable);
}

// =============================================================================
// fromPCL — UNCHANGED
// =============================================================================
DP ICPlocalization::fromPCL(const Pointcloud &pcl) {
  sensor_msgs::msg::PointCloud2 ros;
  pcl::toROSMsg(pcl, ros);
  return rosMsgToPointMatcherCloud<float>(ros, ros.is_dense);
}

// =============================================================================
// getFixedFrame — UNCHANGED
// =============================================================================
const std::string &ICPlocalization::getFixedFrame() const {
  return fixedFrame_;
}

// =============================================================================
// matchScans — CHANGE: icp_() call wrapped in map_mutex_ lock
//
// Why: setMapCloud() (called by callbackLoadMap from a service thread) also
// locks map_mutex_. This ensures the map is never swapped while a scan match
// is in progress. The lock is narrow — only around the icp_() call itself,
// not the whole function, so pose prediction still runs without contention.
// =============================================================================
void ICPlocalization::matchScans() {
  if (!icp_.hasMap()) {
    return;
  }

  if (!hasMatchingPointCounts(regCloud_)) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Skipping scan in matchScans: features cols="
                           << regCloud_.features.cols()
                           << " descriptors cols=" << regCloud_.descriptors.cols());
    return;
  }

  Eigen::Vector3d initPosition = lastPosition_;
  Eigen::Quaterniond initOrientation = lastOrientation_;
  Rigid3d before(lastPosition_, lastOrientation_);
  if (!isFirstScanMatch_) {
    const Rigid3d motionPoseChange =
        frameTracker_->getPoseChangeOfRangeSensorInMapFrame(
            lastOptimizedPoseTimestamp_, regCloudTimestamp_);
    const Rigid3d motionCorrectedPose =
        Rigid3d(initPosition, initOrientation) * motionPoseChange;
    initPosition = motionCorrectedPose.translation();
    initOrientation = motionCorrectedPose.rotation();
  }

  PM::TransformationParameters initPose;
  inputFilters_.apply(regCloud_);

  if (!isSetPoseFromUser_) {
    try {
      initPose = getTransformationMatrix<float>(toFloat(initPosition),
                                                toFloat(initOrientation));
      // CHANGE: narrow lock around icp_() so map swap cannot race with matching
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        optimizedPose_ = icp_(regCloud_, initPose);
      }
    } catch (const std::exception &e) {
      std::cerr << "Caught exception while scan matching: " << e.what()
                << std::endl;
      optimizedPose_ = initPose;
    }
  } else {
    optimizedPose_ = getTransformationMatrix<float>(
        toFloat(userSetPosition_), toFloat(userSetQuaternion_));
    isSetPoseFromUser_ = false;
  }

  optimizedPoseTimestamp_ = regCloudTimestamp_;
  getPositionAndOrientation<double>(toDouble(optimizedPose_), &lastPosition_,
                                    &lastOrientation_);

  frameTracker_->setTransformMapToRangeSensor(TimestampedTransform{
      optimizedPoseTimestamp_, Rigid3d(lastPosition_, lastOrientation_)});
  lastOptimizedPoseTimestamp_ = optimizedPoseTimestamp_;
  isFirstScanMatch_ = false;
}

// =============================================================================
// publishPose — UNCHANGED
// =============================================================================
void ICPlocalization::publishPose() const {
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.pose.position.x = optimizedPose_(0, 3);
  pose_msg.pose.position.y = optimizedPose_(1, 3);
  pose_msg.pose.position.z = optimizedPose_(2, 3);
  Eigen::Isometry3f iso;
  iso.matrix() = optimizedPose_;
  Eigen::Quaternionf q;
  q = iso.rotation();
  q.normalize();
  pose_msg.pose.orientation.w = q.w();
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.header.frame_id = fixedFrame_;
  pose_msg.header.stamp = toRos(optimizedPoseTimestamp_);
  posePub_->publish(pose_msg);

  if (isUseOdometry_) {
    tfPublisher_->publishMapToOdom(optimizedPoseTimestamp_);
  } else {
    tfPublisher_->publishMapToRangeSensor(optimizedPoseTimestamp_);
  }
}

// =============================================================================
// publishRegisteredCloud — UNCHANGED
// =============================================================================
void ICPlocalization::publishRegisteredCloud() const {
  DP data_out(icp_.getReadingFiltered());
  icp_.transformations.apply(data_out, optimizedPose_);
  sensor_msgs::msg::PointCloud2 ros_msg =
      pointMatcherCloudToRosMsg<float>(data_out, fixedFrame_, this->now());
  registeredCloudPublisher_->publish(ros_msg);
}

// =============================================================================
// icpWorker — CHANGE: pause guard added at the top of the loop
//
// Original loop condition: if (is_time_for_icp && scanReady && frameReady)
// New behaviour: if is_paused_ is true, the entire scan-matching block is
// skipped. The TF republish branch still runs so Nav2 does not lose the
// map→odom transform during elevator transit.
// =============================================================================
void ICPlocalization::icpWorker()
{
  rclcpp::Rate r(100);
  rclcpp::Time last_icp_time = this->now();
  const double icp_wait_seconds = rate_;

  while (rclcpp::ok()) {

    // ── ADD: pause guard ─────────────────────────────────────────────────────
    // While paused: keep the TF tree alive but do not call matchScans().
    // This is the entire change to icpWorker — one if-block at the top.
    if (is_paused_.load()) {
      if (frameTracker_->isReady() && !isFirstScanMatch_.load()) {
        if (isUseOdometry_) {
          tfPublisher_->publishMapToOdom(optimizedPoseTimestamp_);
        } else {
          tfPublisher_->publishMapToRangeSensor(optimizedPoseTimestamp_);
        }
      }
      r.sleep();
      continue;   // skip everything below — no scan matching while paused
    }
    // ── END ADD ───────────────────────────────────────────────────────────────

    bool is_time_for_icp = (this->now() - last_icp_time).seconds() >= icp_wait_seconds;
    const bool frameReady = frameTracker_->isReady();
    const bool scanReady = rangeDataAccumulator_->isAccumulatedRangeDataReady();

    if (is_time_for_icp && scanReady && frameReady) {

      last_icp_time = this->now();

      regCloudTimestamp_ =
          rangeDataAccumulator_->getAccumulatedRangeDataTimestamp();
      regCloud_ = rangeDataAccumulator_->popAccumulatedRangeData().data_;

      if (!hasMatchingPointCounts(regCloud_)) {
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Skipping non-dense scan: features cols="
                               << regCloud_.features.cols()
                               << " descriptors cols="
                               << regCloud_.descriptors.cols());
        continue;
      }

      namespace ch = std::chrono;
      const auto startTime = ch::steady_clock::now();
      matchScans();
      const auto endTime = ch::steady_clock::now();
      const double timeMs =
          ch::duration_cast<ch::microseconds>(endTime - startTime).count() / 1000.0;

      RCLCPP_INFO_STREAM(this->get_logger(), "Scan matching took: " << timeMs << " ms");

      publishPose();
      publishRegisteredCloud();

    } else {
      if (frameReady && !isFirstScanMatch_) {
        if (isUseOdometry_) {
          tfPublisher_->publishMapToOdom(optimizedPoseTimestamp_);
        } else {
          tfPublisher_->publishMapToRangeSensor(optimizedPoseTimestamp_);
        }
      }
    }

    r.sleep();
  }
}

// =============================================================================
// ADD: callbackPauseLocalization
//
// Called by the elevator supervisor when the wheelchair enters or exits the
// elevator. data=true freezes ICP; data=false resumes it.
// is_paused_ is atomic so this is safe to call from any thread.
// =============================================================================
void ICPlocalization::callbackPauseLocalization(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  is_paused_.store(req->data);
  res->success = true;
  res->message = is_paused_.load() ? "ICP PAUSED" : "ICP RESUMED";
  RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
}

// =============================================================================
// ADD: callbackResetLocalization
//
// Resets all localisation state so ICP re-initialises from a fresh anchor
// on the next scan match. Always call this AFTER pause and BEFORE unpause.
//
// What it resets:
//   isFirstScanMatch_ = true   → next scan uses the /initialpose seed, not
//                                 dead-reckoning from the old floor's pose
//   lastPosition_ / lastOrientation_ → zeroed (supervisor will inject the
//                                        correct anchor via /initialpose)
//   scan accumulator drained   → stale scans from the old floor are discarded
// =============================================================================
void ICPlocalization::callbackResetLocalization(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
{
  RCLCPP_INFO(this->get_logger(), "Resetting ICP localization state...");

  isFirstScanMatch_.store(true);
  isSetPoseFromUser_ = false;
  lastPosition_.setZero();
  lastOrientation_.setIdentity();

  if (imuTracker_) {
    imuTracker_->setInitialPose(lastPosition_, lastOrientation_);
  }
  if (tfPublisher_) {
    tfPublisher_->setInitialPose(lastPosition_, lastOrientation_);
  }

  // Drain any scans that were accumulated while the elevator was in motion.
  // These would have been captured in a feature-poor environment (elevator
  // interior) and would give a bad first match on the new floor.
  if (rangeDataAccumulator_) {
    while (rangeDataAccumulator_->isAccumulatedRangeDataReady()) {
      rangeDataAccumulator_->popAccumulatedRangeData();
    }
  }

  RCLCPP_INFO(this->get_logger(), "ICP reset complete.");
}

// =============================================================================
// ADD: callbackLoadMap
//
// Hot-swaps the ICP reference map to a new PCD file. The file path is read
// from the ROS parameter "map_path", which the supervisor sets with:
//
//   ros2 param set /icp_localization map_path "/path/to/floor4.pcd"
//   ros2 service call /load_map std_srvs/srv/Trigger
//
// For a stacked-Z single-map setup (recommended) this service is never
// called at runtime — the same map file covers all floors. It is provided
// as a fallback for per-floor PCD files.
//
// IMPORTANT: Always call /pause_localization true before this, and
//            /reset_localization + /pause_localization false after.
// =============================================================================
void ICPlocalization::callbackLoadMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::string pcd_path;
  this->get_parameter("map_path", pcd_path);

  if (pcd_path.empty()) {
    res->success = false;
    res->message = "map_path parameter is empty. Set it before calling /load_map.";
    RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", pcd_path.c_str());

  Pointcloud::Ptr new_map(new Pointcloud());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *new_map) == -1) {
    res->success = false;
    res->message = "pcl::io::loadPCDFile failed for: " + pcd_path;
    RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
    return;
  }

  // setMapCloud acquires map_mutex_ internally
  setMapCloud(new_map);

  // Force re-alignment on the next scan
  isFirstScanMatch_.store(true);

  res->success = true;
  res->message = "Map loaded: " + pcd_path;
  RCLCPP_INFO(this->get_logger(), "Map swap complete.");
}

} // namespace icp_loco