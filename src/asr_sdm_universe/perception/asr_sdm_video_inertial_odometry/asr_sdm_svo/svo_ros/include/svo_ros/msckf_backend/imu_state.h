/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef SVO_ROS_MSCKF_BACKEND_IMU_STATE_H
#define SVO_ROS_MSCKF_BACKEND_IMU_STATE_H

#include <map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#define GRAVITY_ACCELERATION 9.81

namespace msckf_vio {

/*
 * @brief IMUState stores the IMU state used by MSCKF.
 */
struct IMUState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using StateIDType = long long int;

  // Unique identifier for the IMU state.
  StateIDType id;

  // id for next IMU state
  static StateIDType next_id;

  // Time when the state is recorded.
  double time;

  // Orientation quaternion [x y z w].
  // Takes a vector from the world frame to the IMU (body) frame.
  Eigen::Vector4d orientation;

  // Position of the IMU (body) frame in the world frame.
  Eigen::Vector3d position;

  // Velocity of the IMU (body) frame in the world frame.
  Eigen::Vector3d velocity;

  // Bias for measured angular velocity and acceleration.
  Eigen::Vector3d gyro_bias;
  Eigen::Vector3d acc_bias;

  // Transformation between the IMU and the left camera (cam0)
  Eigen::Matrix3d R_imu_cam0;
  Eigen::Vector3d t_cam0_imu;

  // Null-space copies for observability constraint.
  Eigen::Vector4d orientation_null;
  Eigen::Vector3d position_null;
  Eigen::Vector3d velocity_null;

  // Process noise.
  static double gyro_noise;
  static double acc_noise;
  static double gyro_bias_noise;
  static double acc_bias_noise;

  // Gravity vector in the world frame.
  static Eigen::Vector3d gravity;

  // Transformation offset from the IMU frame to the body frame.
  // The transformation takes a vector from the IMU frame to the body frame.
  static Eigen::Isometry3d T_imu_body;

  IMUState()
    : id(0),
      time(0),
      orientation(Eigen::Vector4d(0, 0, 0, 1)),
      position(Eigen::Vector3d::Zero()),
      velocity(Eigen::Vector3d::Zero()),
      gyro_bias(Eigen::Vector3d::Zero()),
      acc_bias(Eigen::Vector3d::Zero()),
      orientation_null(Eigen::Vector4d(0, 0, 0, 1)),
      position_null(Eigen::Vector3d::Zero()),
      velocity_null(Eigen::Vector3d::Zero()) {}

  explicit IMUState(const StateIDType& new_id)
    : id(new_id),
      time(0),
      orientation(Eigen::Vector4d(0, 0, 0, 1)),
      position(Eigen::Vector3d::Zero()),
      velocity(Eigen::Vector3d::Zero()),
      gyro_bias(Eigen::Vector3d::Zero()),
      acc_bias(Eigen::Vector3d::Zero()),
      orientation_null(Eigen::Vector4d(0, 0, 0, 1)),
      position_null(Eigen::Vector3d::Zero()),
      velocity_null(Eigen::Vector3d::Zero()) {}
};

using StateIDType = IMUState::StateIDType;

}  // namespace msckf_vio

#endif  // SVO_ROS_MSCKF_BACKEND_IMU_STATE_H

