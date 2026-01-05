/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef SVO_ROS_MSCKF_BACKEND_MSCKF_VIO_H
#define SVO_ROS_MSCKF_BACKEND_MSCKF_VIO_H

#include <map>
#include <set>
#include <string>
#include <vector>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <svo_msgs/msg/camera_measurement.hpp>

#include <svo_ros/msckf_backend/imu_state.h>
#include <svo_ros/msckf_backend/cam_state.h>
#include <svo_ros/msckf_backend/feature.hpp>

namespace msckf_vio {

class MsckfVio {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit MsckfVio(const rclcpp::Node::SharedPtr& node);
  MsckfVio(const MsckfVio&) = delete;
  MsckfVio& operator=(const MsckfVio&) = delete;
  ~MsckfVio() = default;

  bool initialize();
  void reset();

  // --- Tight-coupling friendly APIs ----------------------------------
  void processImuMsg(const sensor_msgs::msg::Imu& msg);
  void processFeatureMsg(const svo_msgs::msg::CameraMeasurement& msg);
  //-------------------------------------------------------------------

private:
  struct StateServer {
    IMUState imu_state;
    CamStateServer cam_states;

    Eigen::MatrixXd state_cov;
    Eigen::Matrix<double, 12, 12> continuous_noise_cov;
  };

  bool loadParameters();
  bool createRosIO();

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  void featureCallback(const svo_msgs::msg::CameraMeasurement::ConstSharedPtr& msg);

  void publish(const rclcpp::Time& time);
  void initializeGravityAndBias();

  bool resetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Filter related functions
  void batchImuProcessing(const double& time_bound);
  void processModel(const double& time, const Eigen::Vector3d& m_gyro, const Eigen::Vector3d& m_acc);
  void predictNewState(const double& dt, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc);

  // Measurement update
  void stateAugmentation(const double& time);
  void addFeatureObservations(const svo_msgs::msg::CameraMeasurement& msg);
  void measurementJacobian(
    const StateIDType& cam_state_id,
    const FeatureIDType& feature_id,
    Eigen::Matrix<double, 2, 6>& H_x, // For stereo, this would be 4x6
    Eigen::Matrix<double, 2, 3>& H_f,
    Eigen::Vector2d& r);

  void featureJacobian(
    const FeatureIDType& feature_id,
    const std::vector<StateIDType>& cam_state_ids,
    Eigen::MatrixXd& H_x,
    Eigen::VectorXd& r);

  void measurementUpdate(const Eigen::MatrixXd& H, const Eigen::VectorXd& r);
  bool gatingTest(const Eigen::MatrixXd& H, const Eigen::VectorXd& r, const int& dof);

  void removeLostFeatures(const std::set<FeatureIDType>& current_feature_ids);
  void findRedundantCamStates(std::vector<StateIDType>& rm_cam_state_ids);
  void pruneCamStateBuffer();
  void onlineReset();

  static std::map<int, double> chi_squared_test_table;

  StateServer state_server_;
  int max_cam_state_size_ = 30;

  MapServer map_server_;

  std::deque<sensor_msgs::msg::Imu> imu_msg_buffer_;

  bool is_gravity_set_ = false;
  bool is_first_img_ = true;

  // Noise parameters are held in IMUState and Feature static members

  // Thresholds
  double position_std_threshold_ = 8.0;
  double rotation_threshold_ = 0.2618; // rad
  double translation_threshold_ = 0.4; // m
  double tracking_rate_threshold_ = 0.5;

  // Tracking performance stats
  double tracking_rate_ = 0.0;

  rclcpp::Node::SharedPtr nh_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<svo_msgs::msg::CameraMeasurement>::SharedPtr feature_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

  std::string fixed_frame_id_ = "world";
  std::string child_frame_id_ = "robot";
  bool publish_tf_ = true;
  double frame_rate_ = 40.0;
};

}  // namespace msckf_vio

#endif  // SVO_ROS_MSCKF_BACKEND_MSCKF_VIO_H

