// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2017 Jonathan Huber <jonathan.huber at uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).

#pragma once

#include "asr_sdm_vio/ceres_backend/map.hpp"

#include <asr_sdm_vio/vio_common/backend_types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <memory>
#include <mutex>

namespace svo
{
class CeresBackendPublisher
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<CeresBackendPublisher> Ptr;
  const std::string kWorldFrame = "world";

  CeresBackendPublisher(rclcpp::Node * node, const std::shared_ptr<ceres_backend::Map> & map_ptr);
  ~CeresBackendPublisher() {}

  Transformation getLastT_W_B() const { return state_.get_T_W_B(); }

  void addFrame(const BundleId & bundle_id)
  {
    std::lock_guard<std::mutex> lock(mutex_frame_id_);
    last_added_frame_ = bundle_id;
  }

  void publish(const ViNodeState & state, const int64_t timestamp, const int32_t seq);

private:
  rclcpp::Node* node_;

  mutable std::mutex mutex_frame_id_;

  std::shared_ptr<ceres_backend::Map> map_ptr_;  ///< The underlying svo::Map.

  // Transform used for tracing
  ViNodeState state_;
  BundleId state_frame_id_ = -1;
  BundleId last_added_frame_ = -1;

  // publisher helpers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_imu_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_imu_pose_viz_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_;

  // publisher functions
  void publishImuPose(const ViNodeState & state, const int64_t timestamp, const int32_t seq);
  void publishBackendLandmarks(const int64_t timestamp) const;
};

}  // namespace svo
