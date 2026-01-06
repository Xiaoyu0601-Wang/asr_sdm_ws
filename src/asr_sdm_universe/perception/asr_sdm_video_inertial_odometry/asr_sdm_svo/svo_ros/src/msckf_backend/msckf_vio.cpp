/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#include <svo_ros/msckf_backend/msckf_vio.h>
#include <svo_ros/msckf_backend/math_utils.hpp>
#include <svo_ros/msckf_frontend/utils.h>  // For getTransformEigen

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <limits>
#include <set>

#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/SparseCore>
// #include <Eigen/SPQRSupport>  // Enable later when SuiteSparseQR dev headers are available
#include <boost/math/distributions/chi_squared.hpp>

namespace msckf_vio {

// Static member variables
StateIDType IMUState::next_id = 0;
double IMUState::gyro_noise = 0.001;
double IMUState::acc_noise = 0.01;
double IMUState::gyro_bias_noise = 0.001;
double IMUState::acc_bias_noise = 0.01;
Eigen::Vector3d IMUState::gravity = Eigen::Vector3d(0, 0, -GRAVITY_ACCELERATION);
Eigen::Isometry3d IMUState::T_imu_body = Eigen::Isometry3d::Identity();

Eigen::Isometry3d CAMState::T_cam0_cam1 = Eigen::Isometry3d::Identity();

FeatureIDType Feature::next_id = 0;
double Feature::observation_noise = 0.01;
Feature::OptimizationConfig Feature::optimization_config;

std::map<int, double> MsckfVio::chi_squared_test_table;

MsckfVio::MsckfVio(const rclcpp::Node::SharedPtr& node) : nh_(node) {
  // Constructor: do nothing yet
}

bool MsckfVio::initialize() {
  if (!loadParameters()) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to load parameters for MSCKF VIO");
    return false;
  }

  // Initialize state server
  state_server_.continuous_noise_cov = Eigen::Matrix<double, 12, 12>::Zero();
  state_server_.continuous_noise_cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * IMUState::gyro_noise;
  state_server_.continuous_noise_cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * IMUState::gyro_bias_noise;
  state_server_.continuous_noise_cov.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * IMUState::acc_noise;
  state_server_.continuous_noise_cov.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * IMUState::acc_bias_noise;

  // Initialize the chi squared test table with confidence level 0.95.
  for (int i = 1; i < 100; ++i) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_test_table[i] = boost::math::quantile(chi_squared_dist, 0.05);
  }

  if (!createRosIO()) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to create ROS IO for MSCKF VIO");
    return false;
  }
  RCLCPP_INFO(nh_->get_logger(), "MSCKF VIO Initialized");
  return true;
}

bool MsckfVio::loadParameters() {
  // Frame IDs
  fixed_frame_id_ = nh_->declare_parameter<std::string>("fixed_frame_id", "world");
  child_frame_id_ = nh_->declare_parameter<std::string>("child_frame_id", "robot");
  publish_tf_ = nh_->declare_parameter<bool>("publish_tf", true);
  frame_rate_ = nh_->declare_parameter<double>("frame_rate", 40.0);
  position_std_threshold_ = nh_->declare_parameter<double>("position_std_threshold", 8.0);

  // IMU initialization (static detection)
  gravity_init_imu_buffer_size_ = static_cast<size_t>(nh_->declare_parameter<int>("gravity_init.imu_buffer_size", 200));
  gravity_init_acc_std_threshold_ = nh_->declare_parameter<double>("gravity_init.acc_std_threshold", 0.15);
  gravity_init_gyro_std_threshold_ = nh_->declare_parameter<double>("gravity_init.gyro_std_threshold", 0.02);
  gravity_init_gyro_mean_threshold_ = nh_->declare_parameter<double>("gravity_init.gyro_mean_threshold", 0.03);
  gravity_init_gravity_norm_min_ = nh_->declare_parameter<double>("gravity_init.gravity_norm_min", 9.6);
  gravity_init_gravity_norm_max_ = nh_->declare_parameter<double>("gravity_init.gravity_norm_max", 10.0);

  // Keyframe thresholds
  rotation_threshold_ = nh_->declare_parameter<double>("rotation_threshold", 0.2618);
  translation_threshold_ = nh_->declare_parameter<double>("translation_threshold", 0.4);
  tracking_rate_threshold_ = nh_->declare_parameter<double>("tracking_rate_threshold", 0.5);

  // Feature optimization config
  Feature::optimization_config.translation_threshold = nh_->declare_parameter<double>(
      "feature.config.translation_threshold", 0.2);
  Feature::optimization_config.min_depth = nh_->declare_parameter<double>(
      "feature.config.min_depth", 0.1);
  Feature::optimization_config.max_depth = nh_->declare_parameter<double>(
      "feature.config.max_depth", 150.0);
  Feature::optimization_config.max_rms_reproj_error = nh_->declare_parameter<double>(
      "feature.config.max_rms_reproj_error", 0.2);

  // Update-time safety: skip update if the stacked residual norm is too large (outlier protection).
  max_update_residual_norm_ = nh_->declare_parameter<double>(
      "filter.max_update_residual_norm", 2.0);

  // Gating scale factor (chi-square test multiplier). 2.0 is a robust default for real data.
  gating_chi2_multiplier_ = nh_->declare_parameter<double>(
      "filter.gating_chi2_multiplier", 2.0);

  // Noise parameters
  IMUState::gyro_noise = nh_->declare_parameter<double>("noise.gyro", 0.001);
  IMUState::acc_noise = nh_->declare_parameter<double>("noise.acc", 0.01);
  IMUState::gyro_bias_noise = nh_->declare_parameter<double>("noise.gyro_bias", 0.001);
  IMUState::acc_bias_noise = nh_->declare_parameter<double>("noise.acc_bias", 0.01);
  Feature::observation_noise = nh_->declare_parameter<double>("noise.feature", 0.01);

  // Use variance instead of standard deviation.
  IMUState::gyro_noise *= IMUState::gyro_noise;
  IMUState::acc_noise *= IMUState::acc_noise;
  IMUState::gyro_bias_noise *= IMUState::gyro_bias_noise;
  IMUState::acc_bias_noise *= IMUState::acc_bias_noise;
  Feature::observation_noise *= Feature::observation_noise;

  // Initial state covariance
  double velocity_cov = nh_->declare_parameter<double>("initial_covariance.velocity", 0.25);
  double gyro_bias_cov = nh_->declare_parameter<double>("initial_covariance.gyro_bias", 1e-4);
  double acc_bias_cov = nh_->declare_parameter<double>("initial_covariance.acc_bias", 1e-2);
  double extrinsic_rotation_cov = nh_->declare_parameter<double>("initial_covariance.extrinsic_rotation", 1e-6);
  double extrinsic_translation_cov = nh_->declare_parameter<double>("initial_covariance.extrinsic_translation", 1e-6);


  // Scheme B: IMU error-state dimension = 21
  // [0:3]   attitude error
  // [3:6]   gyro bias error
  // [6:9]   velocity error
  // [9:12]  acc bias error
  // [12:15] position error
  // [15:18] extrinsic rotation (imu<->cam0) error
  // [18:21] extrinsic translation error
  state_server_.state_cov = Eigen::MatrixXd::Zero(21, 21);
  for (int i = 3; i < 6; ++i) state_server_.state_cov(i, i) = gyro_bias_cov;
  for (int i = 6; i < 9; ++i) state_server_.state_cov(i, i) = velocity_cov;
  for (int i = 9; i < 12; ++i) state_server_.state_cov(i, i) = acc_bias_cov;
  for (int i = 15; i < 18; ++i) state_server_.state_cov(i, i) = extrinsic_rotation_cov;
  for (int i = 18; i < 21; ++i) state_server_.state_cov(i, i) = extrinsic_translation_cov;

  // Extrinsics
  try {
    Eigen::Isometry3d T_imu_cam0 = utils::getTransformEigen(*nh_, "cam0/T_cam_imu");
    Eigen::Isometry3d T_cam0_imu = T_imu_cam0.inverse();
    state_server_.imu_state.R_imu_cam0 = T_cam0_imu.linear().transpose();
    state_server_.imu_state.t_cam0_imu = T_cam0_imu.translation();
    CAMState::T_cam0_cam1 = utils::getTransformEigen(*nh_, "cam1/T_cn_cnm1");
    IMUState::T_imu_body = utils::getTransformEigen(*nh_, "T_imu_body").inverse();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(nh_->get_logger(), "Failed to load extrinsics: %s", e.what());
    return false;
  }

  max_cam_state_size_ = nh_->declare_parameter<int>("max_cam_state_size", 30);
  RCLCPP_INFO(nh_->get_logger(), "MSCKF VIO parameters loaded.");
  return true;
}

bool MsckfVio::createRosIO() {
  odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);

  const auto qos = rclcpp::SensorDataQoS();
  std::string imu_topic = nh_->declare_parameter<std::string>("imu_topic", "/imu0");
  imu_sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, qos, std::bind(&MsckfVio::imuCallback, this, std::placeholders::_1));

  // Frontend publishes CameraMeasurement on "features" (default: /features)
  std::string feature_topic = nh_->declare_parameter<std::string>("feature_topic", "/features");
  feature_sub_ = nh_->create_subscription<svo_msgs::msg::CameraMeasurement>(
      feature_topic, rclcpp::QoS(40), std::bind(&MsckfVio::featureCallback, this, std::placeholders::_1));

  reset_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
      "reset", std::bind(&MsckfVio::resetCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(nh_->get_logger(), "ROS IO created.");
  return true;
}

void MsckfVio::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  processImuMsg(*msg);
}

void MsckfVio::featureCallback(const svo_msgs::msg::CameraMeasurement::ConstSharedPtr& msg) {
  processFeatureMsg(*msg);
}

void MsckfVio::processImuMsg(const sensor_msgs::msg::Imu& msg) {
  imu_msg_buffer_.push_back(msg);
  if (!is_gravity_set_) {
    if (imu_msg_buffer_.size() < gravity_init_imu_buffer_size_) return;

    if (initializeGravityAndBias()) {
      is_gravity_set_ = true;
      const double imu_time = rclcpp::Time(msg.header.stamp).seconds();
      RCLCPP_INFO(nh_->get_logger(), "Gravity/bias initialized at t=%.6f with window=%zu", imu_time, imu_msg_buffer_.size());
    } else {
      // Slide the window and keep trying until we find a static segment.
      imu_msg_buffer_.pop_front();
    }
  }
}

bool MsckfVio::initializeGravityAndBias() {
  Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();

  if (imu_msg_buffer_.empty()) return false;

  for (const auto& imu_msg : imu_msg_buffer_) {
    sum_gyro.x() += imu_msg.angular_velocity.x;
    sum_gyro.y() += imu_msg.angular_velocity.y;
    sum_gyro.z() += imu_msg.angular_velocity.z;
    sum_acc.x() += imu_msg.linear_acceleration.x;
    sum_acc.y() += imu_msg.linear_acceleration.y;
    sum_acc.z() += imu_msg.linear_acceleration.z;
  }

  // Compute mean gyro/acc.
  const Eigen::Vector3d gyro_avg = sum_gyro / static_cast<double>(imu_msg_buffer_.size());
  const Eigen::Vector3d acc_avg = sum_acc / static_cast<double>(imu_msg_buffer_.size());

  // Compute stddev for static detection.
  Eigen::Vector3d var_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d var_acc = Eigen::Vector3d::Zero();
  for (const auto& imu_msg : imu_msg_buffer_) {
    const Eigen::Vector3d g(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
    const Eigen::Vector3d a(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
    var_gyro += (g - gyro_avg).cwiseProduct(g - gyro_avg);
    var_acc += (a - acc_avg).cwiseProduct(a - acc_avg);
  }
  var_gyro /= static_cast<double>(imu_msg_buffer_.size());
  var_acc /= static_cast<double>(imu_msg_buffer_.size());
  const Eigen::Vector3d gyro_std = var_gyro.cwiseSqrt();
  const Eigen::Vector3d acc_std = var_acc.cwiseSqrt();

  const double acc_norm = acc_avg.norm();

  const double gyro_mean_norm = gyro_avg.norm();

  const bool is_static =
      (acc_std.maxCoeff() < gravity_init_acc_std_threshold_) &&
      (gyro_std.maxCoeff() < gravity_init_gyro_std_threshold_) &&
      (gyro_mean_norm < gravity_init_gyro_mean_threshold_) &&
      (acc_norm > gravity_init_gravity_norm_min_) &&
      (acc_norm < gravity_init_gravity_norm_max_);

  if (!is_static) {
    static int warn_decimator = 0;
    if ((warn_decimator++ % 20) == 0) { // Log more frequently
      RCLCPP_WARN(nh_->get_logger(),
                  "IMU init waiting: acc_norm=%.2f acc_std=%.3f gyro_mean=%.3f gyro_std=%.3f (Thresh: |g| in [%.1f,%.1f], acc_std<%.2f, gyro_mean<%.2f, gyro_std<%.2f)",
                  acc_norm, acc_std.maxCoeff(), gyro_mean_norm, gyro_std.maxCoeff(),
                  gravity_init_gravity_norm_min_, gravity_init_gravity_norm_max_,
                  gravity_init_acc_std_threshold_, gravity_init_gyro_mean_threshold_, gravity_init_gyro_std_threshold_);
    }

    return false;
  }

  // Accept: set biases and gravity.
  state_server_.imu_state.gyro_bias = gyro_avg;

  // Gravity magnitude from the average accelerometer measurement.
  const double gravity_norm = acc_norm;
  IMUState::gravity = Eigen::Vector3d(0.0, 0.0, -gravity_norm);

  // Estimate initial orientation from measured gravity direction.
  const Eigen::Quaterniond q0_i_w = Eigen::Quaterniond::FromTwoVectors(acc_avg, -IMUState::gravity);
  state_server_.imu_state.orientation = rotationToQuaternion(q0_i_w.toRotationMatrix().transpose());

  // For robust initialization, set acc_bias to zero and let the filter estimate it.
  // Estimating it here requires a very clean static segment, which is not always available.
  state_server_.imu_state.acc_bias = Eigen::Vector3d::Zero();

  RCLCPP_INFO(nh_->get_logger(),
              "Init gravity/bias: acc_avg=[%.4f %.4f %.4f] acc_std=%.4f gyro_mean=%.4f gyro_std=%.4f |g|=%.4f acc_bias=[%.4f %.4f %.4f] (forced zero) gyro_bias=[%.4f %.4f %.4f]",
              acc_avg.x(), acc_avg.y(), acc_avg.z(),
              acc_std.maxCoeff(), gyro_mean_norm, gyro_std.maxCoeff(), gravity_norm,
              state_server_.imu_state.acc_bias.x(), state_server_.imu_state.acc_bias.y(), state_server_.imu_state.acc_bias.z(),
              state_server_.imu_state.gyro_bias.x(), state_server_.imu_state.gyro_bias.y(), state_server_.imu_state.gyro_bias.z());
  return true;
}

void MsckfVio::processFeatureMsg(const svo_msgs::msg::CameraMeasurement& msg) {
  if (!is_gravity_set_) return;

  double time = rclcpp::Time(msg.header.stamp).seconds();
  if (is_first_img_) {
    is_first_img_ = false;
    state_server_.imu_state.time = time;
  }

  batchImuProcessing(time);
  stateAugmentation(time);
  addFeatureObservations(msg);
  // Perform measurement update steps
  removeLostFeatures(std::set<FeatureIDType>());  // will be replaced by real lost-feature logic
  pruneCamStateBuffer();

  // Publish odometry + TF
  publish(rclcpp::Time(msg.header.stamp));

  onlineReset();
}

void MsckfVio::batchImuProcessing(const double& time_bound) {
  auto it = imu_msg_buffer_.begin();
  while (it != imu_msg_buffer_.end()) {
    double imu_time = rclcpp::Time(it->header.stamp).seconds();
    if (imu_time < state_server_.imu_state.time) {
      it = imu_msg_buffer_.erase(it);
      continue;
    }
    if (imu_time > time_bound) break;

    Eigen::Vector3d m_gyro(it->angular_velocity.x, it->angular_velocity.y, it->angular_velocity.z);
    Eigen::Vector3d m_acc(it->linear_acceleration.x, it->linear_acceleration.y, it->linear_acceleration.z);

    processModel(imu_time, m_gyro, m_acc);
    it = imu_msg_buffer_.erase(it);
  }
  state_server_.imu_state.id = IMUState::next_id++;
}

void MsckfVio::processModel(const double& time, const Eigen::Vector3d& m_gyro, const Eigen::Vector3d& m_acc) {
  IMUState& imu_state = state_server_.imu_state;
  Eigen::Vector3d gyro = m_gyro - imu_state.gyro_bias;
  Eigen::Vector3d acc = m_acc - imu_state.acc_bias;
  double dtime = time - imu_state.time;

  // Scheme B (21-state error): [theta, bg, v, ba, p, theta_ex, p_ex]
  // Only the first 15 states participate in IMU error-state dynamics; extrinsics are modeled constant here.
  const Eigen::Matrix3d R_w_i = quaternionToRotation(imu_state.orientation);

  Eigen::Matrix<double, 21, 21> F = Eigen::Matrix<double, 21, 21>::Zero();
  // d(theta)/dt
  F.block<3, 3>(0, 0) = -skewSymmetric(gyro);
  F.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
  // d(v)/dt
  F.block<3, 3>(6, 0) = -R_w_i.transpose() * skewSymmetric(acc);
  F.block<3, 3>(6, 9) = -R_w_i.transpose();
  // d(p)/dt
  F.block<3, 3>(12, 6) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 21, 12> G = Eigen::Matrix<double, 21, 12>::Zero();
  G.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();          // gyro noise
  G.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();           // gyro bias noise
  G.block<3, 3>(6, 6) = -R_w_i.transpose();                    // acc noise
  G.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();           // acc bias noise

  // Approximate matrix exponential to the 3rd order (same strategy as upstream msckf_vio)
  Eigen::Matrix<double, 21, 21> Fdt = F * dtime;
  Eigen::Matrix<double, 21, 21> Fdt_square = Fdt * Fdt;
  Eigen::Matrix<double, 21, 21> Fdt_cube = Fdt_square * Fdt;
  Eigen::Matrix<double, 21, 21> Phi = Eigen::Matrix<double, 21, 21>::Identity() +
      Fdt + 0.5 * Fdt_square + (1.0 / 6.0) * Fdt_cube;

  predictNewState(dtime, gyro, acc);

  Eigen::Matrix<double, 21, 21> Q = Phi * G * state_server_.continuous_noise_cov * G.transpose() * Phi.transpose() * dtime;
  state_server_.state_cov.block<21, 21>(0, 0) = Phi * state_server_.state_cov.block<21, 21>(0, 0) * Phi.transpose() + Q;

  if (state_server_.cam_states.size() > 0) {
    state_server_.state_cov.block(0, 21, 21, state_server_.state_cov.cols() - 21) =
        Phi * state_server_.state_cov.block(0, 21, 21, state_server_.state_cov.cols() - 21);
    state_server_.state_cov.block(21, 0, state_server_.state_cov.rows() - 21, 21) =
        state_server_.state_cov.block(21, 0, state_server_.state_cov.rows() - 21, 21) * Phi.transpose();
  }

  state_server_.state_cov = (state_server_.state_cov + state_server_.state_cov.transpose()) / 2.0;
  imu_state.time = time;
}

void MsckfVio::predictNewState(const double& dt, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc) {
  Eigen::Vector4d& q = state_server_.imu_state.orientation;
  Eigen::Vector3d& v = state_server_.imu_state.velocity;
  Eigen::Vector3d& p = state_server_.imu_state.position;

  double gyro_norm = gyro.norm();
  Eigen::Matrix4d Omega = Eigen::Matrix4d::Zero();
  Omega.block<3, 3>(0, 0) = -skewSymmetric(gyro);
  Omega.block<3, 1>(0, 3) = gyro;
  Omega.block<1, 3>(3, 0) = -gyro;

  Eigen::Vector4d dq_dt;
  if (gyro_norm > 1e-5) {
      dq_dt = (cos(gyro_norm*dt*0.5)*Eigen::Matrix4d::Identity() + 1/gyro_norm*sin(gyro_norm*dt*0.5)*Omega) * q;
  } else {
      dq_dt = (Eigen::Matrix4d::Identity()+0.5*dt*Omega) * cos(gyro_norm*dt*0.5) * q;
  }

  Eigen::Vector3d k1_v_dot = quaternionToRotation(q).transpose()*acc + IMUState::gravity;
  Eigen::Vector3d k1_p_dot = v;
  Eigen::Vector4d q_mid = (Eigen::Matrix4d::Identity()+0.25*dt*Omega) * q;
  Eigen::Vector3d k2_v_dot = quaternionToRotation(q_mid).transpose()*acc + IMUState::gravity;
  Eigen::Vector3d k2_p_dot = v + k1_v_dot*dt/2;
  Eigen::Vector3d k3_v_dot = quaternionToRotation(q_mid).transpose()*acc + IMUState::gravity;
  Eigen::Vector3d k3_p_dot = v + k2_v_dot*dt/2;
  Eigen::Vector3d k4_v_dot = quaternionToRotation(dq_dt).transpose()*acc + IMUState::gravity;
  Eigen::Vector3d k4_p_dot = v + k3_v_dot*dt;

  q = dq_dt;
  quaternionNormalize(q);
  v += dt/6.0*(k1_v_dot + 2*k2_v_dot + 2*k3_v_dot + k4_v_dot);
  p += dt/6.0*(k1_p_dot + 2*k2_p_dot + 2*k3_p_dot + k4_p_dot);
}

void MsckfVio::stateAugmentation(const double& time) {
  const Eigen::Matrix3d& R_i_c = state_server_.imu_state.R_imu_cam0;
  const Eigen::Vector3d& t_c_i = state_server_.imu_state.t_cam0_imu;

  Eigen::Matrix3d R_w_i = quaternionToRotation(state_server_.imu_state.orientation);
  Eigen::Matrix3d R_w_c = R_i_c * R_w_i;
  Eigen::Vector3d t_c_w = state_server_.imu_state.position + R_w_i.transpose() * t_c_i;

  state_server_.cam_states[state_server_.next_cam_state_id] = CAMState(state_server_.next_cam_state_id);
  CAMState& cam_state = state_server_.cam_states[state_server_.next_cam_state_id];
  state_server_.next_cam_state_id++;

  cam_state.time = time;
  cam_state.orientation = rotationToQuaternion(R_w_c);
  cam_state.position = t_c_w;

  size_t old_size = state_server_.state_cov.rows();
  state_server_.state_cov.conservativeResize(old_size + 6, old_size + 6);

  Eigen::Matrix<double, 6, 21> J = Eigen::Matrix<double, 6, 21>::Zero();
  J.block<3, 3>(0, 0) = R_i_c;
  J.block<3, 3>(3, 0) = skewSymmetric(R_w_i.transpose() * t_c_i);
  J.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity();

  // Scheme B: include extrinsic (imu<->cam0) error states in augmentation Jacobian.
  // Rotation part couples to cam orientation.
  J.block<3, 3>(0, 15) = Eigen::Matrix3d::Identity();
  // Translation part couples to cam position.
  J.block<3, 3>(3, 18) = Eigen::Matrix3d::Identity();

  state_server_.state_cov.block(old_size, 0, 6, old_size).setZero();
  state_server_.state_cov.block(0, old_size, old_size, 6).setZero();

  const Eigen::Matrix<double, 21, 21>& P_II = state_server_.state_cov.block<21, 21>(0, 0);
  state_server_.state_cov.block(old_size, 0, 6, 21) = J * P_II;
  state_server_.state_cov.block(0, old_size, 21, 6) = state_server_.state_cov.block(old_size, 0, 6, 21).transpose();
  state_server_.state_cov.block<6, 6>(old_size, old_size) = J * P_II * J.transpose();

  state_server_.state_cov = (state_server_.state_cov + state_server_.state_cov.transpose()) / 2.0;
}

void MsckfVio::addFeatureObservations(const svo_msgs::msg::CameraMeasurement& msg) {
  StateIDType state_id = state_server_.cam_states.rbegin()->first;
  int curr_feature_num = map_server_.size();
  int tracked_feature_num = 0;

  // Diagnostics: infer whether frontend publishes pixel coords or normalized coords.
  // Throttled to 1Hz.
  static rclcpp::Time last_feat_stats_time(0, 0, RCL_ROS_TIME);
  const rclcpp::Time now = nh_->get_clock()->now();
  double u0_min = std::numeric_limits<double>::infinity();
  double v0_min = std::numeric_limits<double>::infinity();
  double u0_max = -std::numeric_limits<double>::infinity();
  double v0_max = -std::numeric_limits<double>::infinity();

  for (const auto& feature : msg.features) {
    // Update min/max for quick range check.
    u0_min = std::min(u0_min, static_cast<double>(feature.u0));
    v0_min = std::min(v0_min, static_cast<double>(feature.v0));
    u0_max = std::max(u0_max, static_cast<double>(feature.u0));
    v0_max = std::max(v0_max, static_cast<double>(feature.v0));

    auto it = map_server_.find(feature.id);
    if (it == map_server_.end()) {
      map_server_[feature.id] = Feature(feature.id);
      map_server_[feature.id].observations[state_id] = Eigen::Vector4d(
          static_cast<double>(feature.u0), static_cast<double>(feature.v0),
          static_cast<double>(feature.u1), static_cast<double>(feature.v1));
    } else {
      it->second.observations[state_id] = Eigen::Vector4d(
          static_cast<double>(feature.u0), static_cast<double>(feature.v0),
          static_cast<double>(feature.u1), static_cast<double>(feature.v1));
      tracked_feature_num++;
    }
  }
  if ((now - last_feat_stats_time).seconds() > 1.0) {
    last_feat_stats_time = now;
    if (!msg.features.empty()) {
      RCLCPP_INFO(nh_->get_logger(),
                  "[feat_stats] N=%zu u0_range=[%.3f, %.3f] v0_range=[%.3f, %.3f] (if pixel, expect ~[0,%d]/[0,%d]; if norm, expect ~[-1,1])",
                  msg.features.size(), u0_min, u0_max, v0_min, v0_max, 752, 480);
    } else {
      RCLCPP_INFO(nh_->get_logger(), "[feat_stats] N=0");
    }
  }

  if (curr_feature_num > 0) {
    tracking_rate_ = static_cast<double>(tracked_feature_num) / curr_feature_num;
  }
}

void MsckfVio::removeLostFeatures(const std::set<FeatureIDType>& /*current_feature_ids*/) {
  // Ported from upstream msckf_vio: process features which are no longer tracked.
  // A feature is considered "lost" if it does NOT have an observation at the latest camera state.

  // Remove the features that lost track.
  // BTW, find the size the final Jacobian matrix and residual vector.
  int jacobian_row_size = 0;
  std::vector<FeatureIDType> invalid_feature_ids;
  std::vector<FeatureIDType> processed_feature_ids;

  // Use the latest camera state id as the "current" state for tracking/lost decision.
  // This matches the upstream logic: if a feature has no observation in the latest cam state, it is lost.
  const StateIDType curr_state_id = state_server_.cam_states.empty() ? state_server_.imu_state.id
                                                                     : state_server_.cam_states.rbegin()->first;

  size_t initialized_now = 0;
  size_t used_initialized = 0;

  for (auto iter = map_server_.begin(); iter != map_server_.end(); ++iter) {
    auto& feature = iter->second;

    // Pass the features that are still being tracked.
    if (feature.observations.find(curr_state_id) != feature.observations.end()) {
      continue;
    }

    if (feature.observations.size() < 3) {
      invalid_feature_ids.push_back(feature.id);
      continue;
    }

    // If the feature has not been initialized yet, try initializing it,
    // but DO NOT use it for MSCKF update in the same iteration.
    // Otherwise the residual will be near zero (since the point is optimized from the same measurements).
    if (!feature.is_initialized) {
      if (!feature.checkMotion(state_server_.cam_states)) {
        invalid_feature_ids.push_back(feature.id);
        continue;
      }
      if (!feature.initializePosition(state_server_.cam_states)) {
        invalid_feature_ids.push_back(feature.id);
        continue;
      }
      initialized_now++;
      continue;
    }

    used_initialized++;

    // jacobian_row_size is just a rough upper bound now. The actual size is
    // determined by the number of valid observations in featureJacobian.
    if (feature.observations.size() >= 3) {
      jacobian_row_size += static_cast<int>(2 * feature.observations.size() - 3);
    }
    processed_feature_ids.push_back(feature.id);
  }

  for (const auto& feature_id : invalid_feature_ids) {
    map_server_.erase(feature_id);
  }

  if (processed_feature_ids.empty()) {
    if (initialized_now > 0) {
      RCLCPP_INFO(nh_->get_logger(),
                  "MSCKF update skipped: newly initialized %zu features (kept), used_initialized=%zu (none used), cam_states=%zu map=%zu",
                  initialized_now, used_initialized, state_server_.cam_states.size(), map_server_.size());
    }
    return;
  }

  Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(
      jacobian_row_size, 21 + 6 * static_cast<int>(state_server_.cam_states.size()));
  Eigen::VectorXd r = Eigen::VectorXd::Zero(jacobian_row_size);
  int stack_cntr = 0;

  for (const auto& feature_id : processed_feature_ids) {
    auto& feature = map_server_.at(feature_id);

    std::vector<StateIDType> cam_state_ids;
    cam_state_ids.reserve(feature.observations.size());
    for (const auto& measurement : feature.observations) {
      cam_state_ids.push_back(measurement.first);
    }

    Eigen::MatrixXd H_xj;
    Eigen::VectorXd r_j;
    featureJacobian(feature.id, cam_state_ids, H_xj, r_j);

    // dof = (#cam_states-1) in upstream
    const int dof = static_cast<int>(cam_state_ids.size()) - 1;
    if (dof > 0 && gatingTest(H_xj, r_j, dof)) {
      H_x.block(stack_cntr, 0, H_xj.rows(), H_xj.cols()) = H_xj;
      r.segment(stack_cntr, r_j.rows()) = r_j;
      stack_cntr += static_cast<int>(H_xj.rows());
    }

    // Put an upper bound on the row size of measurement Jacobian.
    if (stack_cntr > 1500) {
      break;
    }
  }

  H_x.conservativeResize(stack_cntr, H_x.cols());
  r.conservativeResize(stack_cntr);

  // No valid rows after filtering -> skip update.
  if (stack_cntr == 0) {
    for (const auto& feature_id : processed_feature_ids) {
      map_server_.erase(feature_id);
    }
    return;
  }

  // Debug: verify update is actually triggered and check measurement quality.
  RCLCPP_INFO(nh_->get_logger(),
              "MSCKF update: initialized_now=%zu used_initialized=%zu processed=%zu gated_rows=%d jac_rows=%ld jac_cols=%ld r_norm=%.6f r_min=%.6e r_max=%.6e tracking_rate=%.3f cam_states=%zu map=%zu",
              initialized_now, used_initialized, processed_feature_ids.size(), stack_cntr,
              static_cast<long>(H_x.rows()), static_cast<long>(H_x.cols()),
              r.norm(), r.size() > 0 ? r.minCoeff() : 0.0, r.size() > 0 ? r.maxCoeff() : 0.0,
              tracking_rate_, state_server_.cam_states.size(), map_server_.size());

  measurementUpdate(H_x, r);

  for (const auto& feature_id : processed_feature_ids) {
    map_server_.erase(feature_id);
  }
}

void MsckfVio::pruneCamStateBuffer() {
  if (state_server_.cam_states.size() < static_cast<size_t>(max_cam_state_size_)) return;

  // Find the oldest cam state to be removed
  auto oldest_cam_state_it = state_server_.cam_states.begin();
  StateIDType oldest_cam_id = oldest_cam_state_it->first;

  // TODO: Perform measurement update before removing the state

  // Remove the state from the state server
  state_server_.cam_states.erase(oldest_cam_state_it);

  // Remove the corresponding block in the covariance matrix
  int cam_state_start_idx = 21; // IMU error-state size (scheme B)
  int cam_state_idx = 0;
  auto it = state_server_.cam_states.begin();
  while (it != state_server_.cam_states.end() && it->first != oldest_cam_id) {
      cam_state_idx++;
      ++it;
  }

  int idx_to_remove = cam_state_start_idx + cam_state_idx * 6;

  if (idx_to_remove > 0) { // Should always be true
      int remaining_rows = state_server_.state_cov.rows() - idx_to_remove - 6;
      state_server_.state_cov.block(idx_to_remove, 0, remaining_rows, state_server_.state_cov.cols()) = 
          state_server_.state_cov.block(idx_to_remove + 6, 0, remaining_rows, state_server_.state_cov.cols());
      state_server_.state_cov.block(0, idx_to_remove, state_server_.state_cov.rows(), remaining_rows) = 
          state_server_.state_cov.block(0, idx_to_remove + 6, state_server_.state_cov.rows(), remaining_rows);
      state_server_.state_cov.conservativeResize(state_server_.state_cov.rows() - 6, state_server_.state_cov.cols() - 6);
  }

  // Remove observations associated with the removed cam state
  for (auto& feature_pair : map_server_) {
    feature_pair.second.observations.erase(oldest_cam_id);
  }
}

void MsckfVio::onlineReset() {
  // TODO: check position uncertainty and reset if needed
}

bool MsckfVio::resetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  (void)req; // unused
  reset();
  res->success = true;
  res->message = "MSCKF VIO reset.";
  return true;
}

void MsckfVio::reset() {
  RCLCPP_WARN(nh_->get_logger(), "MSCKF VIO Resetting...");
  is_gravity_set_ = false;
  is_first_img_ = true;
  imu_msg_buffer_.clear();
  map_server_.clear();
  state_server_.cam_states.clear();
  state_server_.next_cam_state_id = 0;
  state_server_.imu_state = IMUState();
  loadParameters(); // Reload initial covariance
  RCLCPP_WARN(nh_->get_logger(), "MSCKF VIO Reset complete.");
}

// ------------------ Missing functions (Scheme B stubs) ------------------

void MsckfVio::publish(const rclcpp::Time& time) {
  // Minimal publish: output current IMU pose/vel as odom, and TF (world->robot) if enabled.
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = fixed_frame_id_;
  odom.child_frame_id = child_frame_id_;

  odom.pose.pose.position.x = state_server_.imu_state.position.x();
  odom.pose.pose.position.y = state_server_.imu_state.position.y();
  odom.pose.pose.position.z = state_server_.imu_state.position.z();

  // IMUState::orientation is [x y z w]
  odom.pose.pose.orientation.x = state_server_.imu_state.orientation.x();
  odom.pose.pose.orientation.y = state_server_.imu_state.orientation.y();
  odom.pose.pose.orientation.z = state_server_.imu_state.orientation.z();
  odom.pose.pose.orientation.w = state_server_.imu_state.orientation.w();

  odom.twist.twist.linear.x = state_server_.imu_state.velocity.x();
  odom.twist.twist.linear.y = state_server_.imu_state.velocity.y();
  odom.twist.twist.linear.z = state_server_.imu_state.velocity.z();

  odom_pub_->publish(odom);

  if (publish_tf_ && tf_pub_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = time;
    tf.header.frame_id = fixed_frame_id_;
    tf.child_frame_id = child_frame_id_;
    tf.transform.translation.x = state_server_.imu_state.position.x();
    tf.transform.translation.y = state_server_.imu_state.position.y();
    tf.transform.translation.z = state_server_.imu_state.position.z();
    tf.transform.rotation.x = state_server_.imu_state.orientation.x();
    tf.transform.rotation.y = state_server_.imu_state.orientation.y();
    tf.transform.rotation.z = state_server_.imu_state.orientation.z();
    tf.transform.rotation.w = state_server_.imu_state.orientation.w();
    tf_pub_->sendTransform(tf);
  }
}

bool MsckfVio::gatingTest(const Eigen::MatrixXd& H, const Eigen::VectorXd& r, const int& dof) {
  if (H.rows() == 0) return false;
  if (dof <= 0) return false;

  // Use only the state portion currently represented in P.
  const Eigen::MatrixXd& P = state_server_.state_cov;

  Eigen::MatrixXd S = H * P * H.transpose();
  S.diagonal().array() += Feature::observation_noise;

  double gamma = r.transpose() * S.ldlt().solve(r);

  auto it = chi_squared_test_table.find(dof);
  if (it == chi_squared_test_table.end()) {
    return true;
  }
  // Chi-square gating with configurable multiplier.
  return gamma < gating_chi2_multiplier_ * it->second;
}

void MsckfVio::measurementUpdate(const Eigen::MatrixXd& H, const Eigen::VectorXd& r) {
  if (H.rows() == 0) return;

  // Hard safety gate: skip update if residual is too large (likely outlier).
  const double r_norm = r.norm();

  // 1Hz summary of residual-gate behavior.
  static rclcpp::Time last_gate_report(0, 0, RCL_ROS_TIME);
  static int gate_total = 0;
  static int gate_skipped = 0;
  static double gate_max_seen = 0.0;
  static double gate_max_skipped = 0.0;

  gate_total++;
  gate_max_seen = std::max(gate_max_seen, r_norm);

  const rclcpp::Time now = nh_->get_clock()->now();
  if ((now - last_gate_report).seconds() > 1.0) {
    last_gate_report = now;
    RCLCPP_INFO(nh_->get_logger(),
                "[res_gate] total=%d skipped=%d max_seen=%.3f max_skipped=%.3f max_allowed=%.3f",
                gate_total, gate_skipped, gate_max_seen, gate_max_skipped, max_update_residual_norm_);
    gate_total = 0;
    gate_skipped = 0;
    gate_max_seen = 0.0;
    gate_max_skipped = 0.0;
  }

  if (max_update_residual_norm_ > 0.0 && r_norm > max_update_residual_norm_) {
    gate_skipped++;
    gate_max_skipped = std::max(gate_max_skipped, r_norm);

    // Still print a per-event warning, but at a low rate.
    static int skip_cnt = 0;
    if ((skip_cnt++ % 20) == 0) {
      RCLCPP_WARN(nh_->get_logger(),
                  "MSCKF update skipped by residual norm gate: r_norm=%.3f > max=%.3f (rows=%ld)",
                  r_norm, max_update_residual_norm_, static_cast<long>(r.size()));
    }
    return;
  }

  Eigen::MatrixXd& P = state_server_.state_cov;

  Eigen::MatrixXd S = H * P * H.transpose();
  S.diagonal().array() += Feature::observation_noise;

  Eigen::MatrixXd K = P * H.transpose() * S.ldlt().solve(Eigen::MatrixXd::Identity(S.rows(), S.rows()));
  Eigen::VectorXd delta_x = K * r;

  // ---- Apply correction to IMU nominal state (minimal; extrinsics not yet applied to nominal) ----
  // IMU part indices in error state:
  // [0:3] theta, [3:6] bg, [6:9] v, [9:12] ba, [12:15] p, [15:18] theta_ex, [18:21] p_ex
  if (delta_x.size() >= 15) {
    Eigen::Vector3d dtheta = delta_x.segment<3>(0);
    Eigen::Vector4d dq = smallAngleQuaternion(dtheta);
    Eigen::Vector4d q = state_server_.imu_state.orientation;
    state_server_.imu_state.orientation = quaternionMultiplication(dq, q);
    quaternionNormalize(state_server_.imu_state.orientation);

    state_server_.imu_state.gyro_bias += delta_x.segment<3>(3);
    state_server_.imu_state.velocity += delta_x.segment<3>(6);
    state_server_.imu_state.acc_bias += delta_x.segment<3>(9);
    state_server_.imu_state.position += delta_x.segment<3>(12);
  }

  // Camera states (6 each: theta, p) start at 21
  int cam_base = 21;
  int idx = cam_base;
  for (auto& kv : state_server_.cam_states) {
    if (idx + 6 > delta_x.size()) break;
    Eigen::Vector3d dtheta = delta_x.segment<3>(idx);
    Eigen::Vector3d dp = delta_x.segment<3>(idx + 3);

    Eigen::Vector4d dq = smallAngleQuaternion(dtheta);
    kv.second.orientation = quaternionMultiplication(dq, kv.second.orientation);
    quaternionNormalize(kv.second.orientation);
    kv.second.position += dp;

    idx += 6;
  }

  // Joseph form update for numerical stability
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(P.rows(), P.cols());
  Eigen::MatrixXd KH = K * H;
  P = (I - KH) * P * (I - KH).transpose() + K * (Feature::observation_noise * Eigen::MatrixXd::Identity(S.rows(), S.rows())) * K.transpose();
  P = 0.5 * (P + P.transpose());
}

bool MsckfVio::measurementJacobian(
    const StateIDType& cam_state_id,
    const FeatureIDType& feature_id,
    Eigen::Matrix<double, 4, 6>& H_x,
    Eigen::Matrix<double, 4, 3>& H_f,
    Eigen::Vector4d& r) {
  // Monocular implementation: we only use (u0, v0). (u1, v1) rows are set to zero.
  // Measurement model in normalized coordinates:
  // z = [x/z, y/z]^T where [x y z]^T = p_f^c = R_c_w (p_f^w - p_c^w)

  H_x.setZero();
  H_f.setZero();
  r.setZero();

  const auto cam_it = state_server_.cam_states.find(cam_state_id);
  if (cam_it == state_server_.cam_states.end()) {
    static int dbg_missing_cam = 0;
    if ((dbg_missing_cam++ % 200) == 0) {
      RCLCPP_WARN(nh_->get_logger(), "[measJac dbg] missing cam_state id=%lld", static_cast<long long>(cam_state_id));
    }
    return false;
  }
  const auto feat_it = map_server_.find(feature_id);
  if (feat_it == map_server_.end()) {
    static int dbg_missing_feat = 0;
    if ((dbg_missing_feat++ % 200) == 0) {
      RCLCPP_WARN(nh_->get_logger(), "[measJac dbg] missing feature id=%lld", static_cast<long long>(feature_id));
    }
    return false;
  }

  const CAMState& cam_state = cam_it->second;
  const Feature& feature = feat_it->second;
  const auto obs_it = feature.observations.find(cam_state_id);
  if (obs_it == feature.observations.end()) {
    static int dbg_missing_obs = 0;
    if ((dbg_missing_obs++ % 200) == 0) {
      RCLCPP_WARN(nh_->get_logger(), "[measJac dbg] missing observation feat=%lld cam=%lld (obs_size=%zu)",
                  static_cast<long long>(feature_id), static_cast<long long>(cam_state_id), feature.observations.size());
    }
    return false;
  }

  const Eigen::Vector2d z(obs_it->second(0), obs_it->second(1));

  const Eigen::Matrix3d R_w_c = quaternionToRotation(cam_state.orientation);
  const Eigen::Matrix3d R_c_w = R_w_c.transpose();
  const Eigen::Vector3d p_c_w = cam_state.position;
  const Eigen::Vector3d p_f_w = feature.position;

  const Eigen::Vector3d p_f_c = R_c_w * (p_f_w - p_c_w);
  const double X = p_f_c(0);
  const double Y = p_f_c(1);
  const double Z = p_f_c(2);
  if (Z < 1e-6) return false;

  Eigen::Vector2d z_hat(X / Z, Y / Z);
  Eigen::Vector2d res = z_hat - z;

  // dh/dp_f_c
  Eigen::Matrix<double, 2, 3> J_h;
  J_h << 1.0 / Z, 0.0, -X / (Z * Z),
         0.0, 1.0 / Z, -Y / (Z * Z);

  // dp_f_c / dtheta_c  (theta is small angle of world->cam)
  Eigen::Matrix3d skew_pf_c = skewSymmetric(p_f_c);
  // dp_f_c / dp_c_w
  Eigen::Matrix3d dp_dpcw = -R_c_w;
  // dp_f_c / dp_f_w
  Eigen::Matrix3d dp_dpfw = R_c_w;

  // Measurement jacobian w.r.t cam state (theta_c, p_c)
  Eigen::Matrix<double, 2, 6> H_x_mono;
  H_x_mono.block<2, 3>(0, 0) = J_h * skew_pf_c;        // note sign: d(R^T (pf-pc))/dtheta = skew(pf_c)
  H_x_mono.block<2, 3>(0, 3) = J_h * dp_dpcw;

  // Measurement jacobian w.r.t feature position in world
  Eigen::Matrix<double, 2, 3> H_f_mono = J_h * dp_dpfw;

  // Fill into 4x? outputs: first two rows are mono, last two rows remain zero.
  H_x.block<2, 6>(0, 0) = H_x_mono;
  H_f.block<2, 3>(0, 0) = H_f_mono;

  r.head<2>() = res;
  r.tail<2>().setZero();
  return true;
}

void MsckfVio::featureJacobian(
    const FeatureIDType& feature_id,
    const std::vector<StateIDType>& cam_state_ids,
    Eigen::MatrixXd& H_x,
    Eigen::VectorXd& r) {
  // Monocular MSCKF feature jacobian with nullspace projection.
  // We use only (u0, v0) per observation.

  H_x.resize(0, 0);
  r.resize(0);

  auto feat_it = map_server_.find(feature_id);
  if (feat_it == map_server_.end()) return;
  Feature& feature = feat_it->second;

  if (cam_state_ids.size() < 3) return;

  // Ensure feature is initialized (triangulated) using mono observations.
  if (!feature.is_initialized) {
    if (!feature.checkMotion(state_server_.cam_states)) return;
    if (!feature.initializePosition(state_server_.cam_states)) return;
  }

  // Collect only valid observations (measurementJacobian may early-return).
  struct ObsBlock {
    int cam_index;
    Eigen::Matrix<double, 2, 6> H_x;
    Eigen::Matrix<double, 2, 3> H_f;
    Eigen::Vector2d r;
  };

  std::vector<ObsBlock> blocks;
  blocks.reserve(cam_state_ids.size());

  for (const auto& cam_id : cam_state_ids) {
    Eigen::Matrix<double, 4, 6> H_xi;
    Eigen::Matrix<double, 4, 3> H_fi;
    Eigen::Vector4d r_i;

    const bool valid = measurementJacobian(cam_id, feature_id, H_xi, H_fi, r_i);
    if (!valid) {
      continue;
    }

    // Find cam index in sliding window.
    int cam_index = 0;
    for (auto it = state_server_.cam_states.begin(); it != state_server_.cam_states.end(); ++it, ++cam_index) {
      if (it->first == cam_id) break;
    }

    ObsBlock b;
    b.cam_index = cam_index;
    b.H_x = H_xi.block<2, 6>(0, 0);
    b.H_f = H_fi.block<2, 3>(0, 0);
    b.r = r_i.head<2>();
    blocks.push_back(b);
  }

  const int m = static_cast<int>(blocks.size());
  if (m < 3) {
    // Not enough valid observations to form an MSCKF constraint.
    H_x.resize(0, 0);
    r.resize(0);
    return;
  }

  Eigen::MatrixXd H_xj = Eigen::MatrixXd::Zero(2 * m, 21 + 6 * static_cast<int>(state_server_.cam_states.size()));
  Eigen::MatrixXd H_fj = Eigen::MatrixXd::Zero(2 * m, 3);
  Eigen::VectorXd r_j = Eigen::VectorXd::Zero(2 * m);

  int row = 0;
  for (const auto& b : blocks) {
    const int col = 21 + 6 * b.cam_index;
    H_xj.block(row, col, 2, 6) = b.H_x;
    H_fj.block(row, 0, 2, 3) = b.H_f;
    r_j.segment(row, 2) = b.r;
    row += 2;
  }

  // Nullspace projection: eliminate feature -> project onto left nullspace of H_fj.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(H_fj, Eigen::ComputeFullU);
  Eigen::MatrixXd A = svd.matrixU().rightCols(2 * m - 3).transpose();

  H_x = A * H_xj;
  r = A * r_j;

  // --- Debugging for r_norm=0 issue ---
  static rclcpp::Time last_debug_time(0, 0, RCL_ROS_TIME);
  rclcpp::Time current_time = nh_->get_clock()->now();
  if ((current_time - last_debug_time).seconds() > 1.0) {  // Throttle to 1Hz
    last_debug_time = current_time;
    
    RCLCPP_DEBUG(nh_->get_logger(),
        "[r_norm debug] Feature %lld: obs=%zu, r_j_norm=%.6f, r_norm=%.6f, svd_rank=%ld/%ld, svd_singular_values=[%.3e, %.3e, %.3e]",
        static_cast<long long>(feature_id),
        cam_state_ids.size(),
        r_j.norm(),
        r.norm(),
        static_cast<long>(svd.rank()),
        static_cast<long>(svd.singularValues().size()),
        svd.singularValues().size() > 0 ? svd.singularValues()(0) : 0.0,
        svd.singularValues().size() > 1 ? svd.singularValues()(1) : 0.0,
        svd.singularValues().size() > 2 ? svd.singularValues()(2) : 0.0
    );
    
    // Log first few elements of r_j and r for diagnosis
    const int max_elements = 4;  // Log first 4 elements of each
    std::stringstream ss_r_j, ss_r;
    for (int i = 0; i < std::min(max_elements, static_cast<int>(r_j.size())); ++i) {
        ss_r_j << r_j(i) << " ";
    }
    for (int i = 0; i < std::min(max_elements, static_cast<int>(r.size())); ++i) {
        ss_r << r(i) << " ";
    }
    
    RCLCPP_INFO(nh_->get_logger(),
        "[r_norm debug] First few residuals - r_j: [%s...]  r: [%s...]",
        ss_r_j.str().c_str(), ss_r.str().c_str());
    
    // Log feature position and some observations
    RCLCPP_INFO(nh_->get_logger(),
        "[r_norm debug] Feature position: [%.3f, %.3f, %.3f], #observations: %zu",
        feature.position.x(), feature.position.y(), feature.position.z(),
        feature.observations.size());
    
    // Log first observation
    if (!feature.observations.empty()) {
        const auto& first_obs = feature.observations.begin()->second;
        RCLCPP_INFO(nh_->get_logger(),
            "[r_norm debug] First observation - cam_id: %lld, u0: %.1f, v0: %.1f",
            static_cast<long long>(feature.observations.begin()->first), first_obs(0), first_obs(1));
    }
  }
}

void MsckfVio::findRedundantCamStates(std::vector<StateIDType>& rm_cam_state_ids) {
  // TODO: implement ROS1 logic. For now no-op.
  rm_cam_state_ids.clear();
}

}  // namespace msckf_vio
