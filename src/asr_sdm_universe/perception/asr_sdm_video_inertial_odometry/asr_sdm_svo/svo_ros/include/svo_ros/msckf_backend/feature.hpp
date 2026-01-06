/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef SVO_ROS_MSCKF_BACKEND_FEATURE_H
#define SVO_ROS_MSCKF_BACKEND_FEATURE_H

#include <iostream>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "math_utils.hpp"
#include "imu_state.h"
#include "cam_state.h"

namespace msckf_vio {

/*
 * @brief Feature represents a salient part of an image.
 * See the Appendix of "A Multi-State Constraint Kalman Filter for Vision-aided
 * Inertial Navigation" for feature initialization details.
 */
struct Feature {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using FeatureIDType = long long int;

  /*
   * @brief OptimizationConfig configuration parameters for 3D feature position
   * optimization.
   */
  struct OptimizationConfig {
    double translation_threshold;
    double huber_epsilon;
    double estimation_precision;
    double initial_damping;
    int outer_loop_max_iteration;
    int inner_loop_max_iteration;

    // Sanity checks (in camera frame) to reject bad triangulations.
    double min_depth;
    double max_depth;
    // RMS reprojection error threshold in normalized coords.
    double max_rms_reproj_error;

    OptimizationConfig()
      : translation_threshold(0.2),
        huber_epsilon(0.01),
        estimation_precision(5e-7),
        initial_damping(1e-3),
        outer_loop_max_iteration(10),
        inner_loop_max_iteration(10),
        min_depth(0.1),
        max_depth(150.0),
        max_rms_reproj_error(0.2) {}
  };

  // Constructors
  Feature() : id(0), position(Eigen::Vector3d::Zero()), is_initialized(false) {}

  explicit Feature(const FeatureIDType& new_id)
    : id(new_id), position(Eigen::Vector3d::Zero()), is_initialized(false) {}

  /*
   * @brief cost Compute cost of a camera observation.
   */
  inline void cost(
    const Eigen::Isometry3d& T_c0_ci,
    const Eigen::Vector3d& x,
    const Eigen::Vector2d& z,
    double& e) const;

  /*
   * @brief jacobian Compute Jacobian and residual of a camera observation.
   */
  inline void jacobian(
    const Eigen::Isometry3d& T_c0_ci,
    const Eigen::Vector3d& x,
    const Eigen::Vector2d& z,
    Eigen::Matrix<double, 2, 3>& J,
    Eigen::Vector2d& r,
    double& w) const;

  /*
   * @brief generateInitialGuess Compute an initial guess using two views.
   */
  inline void generateInitialGuess(
    const Eigen::Isometry3d& T_c1_c2,
    const Eigen::Vector2d& z1,
    const Eigen::Vector2d& z2,
    Eigen::Vector3d& p) const;

  /*
   * @brief checkMotion Check if the motion is sufficient for triangulation.
   */
  inline bool checkMotion(const CamStateServer& cam_states) const;

  /*
   * @brief initializePosition Initialize feature 3D position from measurements.
   * The resulted position is in world frame.
   */
  inline bool initializePosition(const CamStateServer& cam_states);

  // Unique identifier
  FeatureIDType id;

  // id for next feature
  static FeatureIDType next_id;

  // Observations: state_id -> [u0 v0 u1 v1]
  std::map<StateIDType, Eigen::Vector4d, std::less<StateIDType>,
    Eigen::aligned_allocator<std::pair<const StateIDType, Eigen::Vector4d>>> observations;

  // 3D position in world frame
  Eigen::Vector3d position;

  // Whether the feature has been initialized
  bool is_initialized;

  // Noise for a normalized feature measurement.
  static double observation_noise;

  // Optimization configuration for solving the 3D position.
  static OptimizationConfig optimization_config;
};

using FeatureIDType = Feature::FeatureIDType;

using MapServer = std::map<FeatureIDType, Feature, std::less<int>,
  Eigen::aligned_allocator<std::pair<const FeatureIDType, Feature>>>;


inline void Feature::cost(
    const Eigen::Isometry3d& T_c0_ci,
    const Eigen::Vector3d& x,
    const Eigen::Vector2d& z,
    double& e) const
{
  // Compute h as Equation (37).
  const double& alpha = x(0);
  const double& beta = x(1);
  const double& rho = x(2);

  Eigen::Vector3d h = T_c0_ci.linear() * Eigen::Vector3d(alpha, beta, 1.0) +
    rho * T_c0_ci.translation();
  const double& h1 = h(0);
  const double& h2 = h(1);
  const double& h3 = h(2);

  Eigen::Vector2d z_hat(h1 / h3, h2 / h3);
  e = (z_hat - z).squaredNorm();
}

inline void Feature::jacobian(
    const Eigen::Isometry3d& T_c0_ci,
    const Eigen::Vector3d& x,
    const Eigen::Vector2d& z,
    Eigen::Matrix<double, 2, 3>& J,
    Eigen::Vector2d& r,
    double& w) const
{
  const double& alpha = x(0);
  const double& beta = x(1);
  const double& rho = x(2);

  Eigen::Vector3d h = T_c0_ci.linear() * Eigen::Vector3d(alpha, beta, 1.0) +
    rho * T_c0_ci.translation();
  const double& h1 = h(0);
  const double& h2 = h(1);
  const double& h3 = h(2);

  Eigen::Matrix3d W;
  W.leftCols<2>() = T_c0_ci.linear().leftCols<2>();
  W.rightCols<1>() = T_c0_ci.translation();

  J.row(0) = 1.0 / h3 * W.row(0) - h1 / (h3 * h3) * W.row(2);
  J.row(1) = 1.0 / h3 * W.row(1) - h2 / (h3 * h3) * W.row(2);

  Eigen::Vector2d z_hat(h1 / h3, h2 / h3);
  r = z_hat - z;

  const double e = r.norm();
  if (e <= optimization_config.huber_epsilon) {
    w = 1.0;
  } else {
    w = std::sqrt(2.0 * optimization_config.huber_epsilon / e);
  }
}

inline void Feature::generateInitialGuess(
    const Eigen::Isometry3d& T_c1_c2,
    const Eigen::Vector2d& z1,
    const Eigen::Vector2d& z2,
    Eigen::Vector3d& p) const
{
  Eigen::Vector3d m = T_c1_c2.linear() * Eigen::Vector3d(z1(0), z1(1), 1.0);

  Eigen::Vector2d A;
  A(0) = m(0) - z2(0) * m(2);
  A(1) = m(1) - z2(1) * m(2);

  Eigen::Vector2d b;
  b(0) = z2(0) * T_c1_c2.translation()(2) - T_c1_c2.translation()(0);
  b(1) = z2(1) * T_c1_c2.translation()(2) - T_c1_c2.translation()(1);

  const double depth = (A.transpose() * A).inverse() * A.transpose() * b;
  p(0) = z1(0) * depth;
  p(1) = z1(1) * depth;
  p(2) = depth;
}

inline bool Feature::checkMotion(const CamStateServer& cam_states) const
{
  const StateIDType& first_cam_id = observations.begin()->first;
  const StateIDType& last_cam_id = (--observations.end())->first;

  Eigen::Isometry3d first_cam_pose;
  first_cam_pose.linear() = quaternionToRotation(
      cam_states.find(first_cam_id)->second.orientation).transpose();
  first_cam_pose.translation() = cam_states.find(first_cam_id)->second.position;

  Eigen::Isometry3d last_cam_pose;
  last_cam_pose.linear() = quaternionToRotation(
      cam_states.find(last_cam_id)->second.orientation).transpose();
  last_cam_pose.translation() = cam_states.find(last_cam_id)->second.position;

  // Direction of the feature when it is first observed (in world frame).
  Eigen::Vector3d feature_direction(
      observations.begin()->second(0),
      observations.begin()->second(1),
      1.0);
  feature_direction.normalize();
  feature_direction = first_cam_pose.linear() * feature_direction;

  const Eigen::Vector3d translation =
    last_cam_pose.translation() - first_cam_pose.translation();

  const double parallel_translation = translation.transpose() * feature_direction;
  const Eigen::Vector3d orthogonal_translation =
    translation - parallel_translation * feature_direction;

  return orthogonal_translation.norm() > optimization_config.translation_threshold;
}

inline bool Feature::initializePosition(const CamStateServer& cam_states)
{
  // Organize camera poses and feature observations.
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> cam_poses;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> measurements;

  cam_poses.reserve(observations.size() * 2);
  measurements.reserve(observations.size() * 2);

  for (auto& m : observations) {
    auto cam_state_iter = cam_states.find(m.first);
    if (cam_state_iter == cam_states.end()) {
      continue;
    }

    // Monocular mode: only cam0 normalized coordinates (u0, v0).
    // NOTE: cam1 (u1, v1) may be absent / all zeros in your current frontend.
    measurements.push_back(m.second.head<2>());

    Eigen::Isometry3d cam0_pose;
    cam0_pose.linear() = quaternionToRotation(
        cam_state_iter->second.orientation).transpose();
    cam0_pose.translation() = cam_state_iter->second.position;

    cam_poses.push_back(cam0_pose);
  }

  if (cam_poses.size() < 2) {
    return false;
  }

  // Normalize poses relative to the first camera.
  const Eigen::Isometry3d T_w_c0 = cam_poses[0].inverse();
  for (auto& pose : cam_poses) {
    pose = pose * T_w_c0;
  }

  // Generate initial guess.
  Eigen::Vector3d initial_position(0.0, 0.0, 0.0);
  generateInitialGuess(cam_poses.back(), measurements.front(), measurements.back(), initial_position);

  Eigen::Vector3d solution(
      initial_position(0) / initial_position(2),
      initial_position(1) / initial_position(2),
      1.0 / initial_position(2));

  // Levenberg-Marquardt.
  double lambda = optimization_config.initial_damping;
  int inner_loop_cntr = 0;
  int outer_loop_cntr = 0;
  bool is_cost_reduced = false;
  double delta_norm = 0.0;

  double total_cost = 0.0;
  for (size_t i = 0; i < cam_poses.size(); ++i) {
    double this_cost = 0.0;
    cost(cam_poses[i], solution, measurements[i], this_cost);
    total_cost += this_cost;
  }

  do {
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < cam_poses.size(); ++i) {
      Eigen::Matrix<double, 2, 3> J;
      Eigen::Vector2d r;
      double w;
      jacobian(cam_poses[i], solution, measurements[i], J, r, w);

      if (w == 1.0) {
        A += J.transpose() * J;
        b += J.transpose() * r;
      } else {
        const double w_square = w * w;
        A += w_square * J.transpose() * J;
        b += w_square * J.transpose() * r;
      }
    }

    do {
      const Eigen::Matrix3d damper = lambda * Eigen::Matrix3d::Identity();
      const Eigen::Vector3d delta = (A + damper).ldlt().solve(b);
      const Eigen::Vector3d new_solution = solution - delta;
      delta_norm = delta.norm();

      double new_cost = 0.0;
      for (size_t i = 0; i < cam_poses.size(); ++i) {
        double this_cost = 0.0;
        cost(cam_poses[i], new_solution, measurements[i], this_cost);
        new_cost += this_cost;
      }

      if (new_cost < total_cost) {
        is_cost_reduced = true;
        solution = new_solution;
        total_cost = new_cost;
        lambda = (lambda / 10.0 > 1e-10) ? lambda / 10.0 : 1e-10;
      } else {
        is_cost_reduced = false;
        lambda = (lambda * 10.0 < 1e12) ? lambda * 10.0 : 1e12;
      }

    } while (inner_loop_cntr++ < optimization_config.inner_loop_max_iteration && !is_cost_reduced);

    inner_loop_cntr = 0;

  } while (outer_loop_cntr++ < optimization_config.outer_loop_max_iteration &&
           delta_norm > optimization_config.estimation_precision);

  // Convert from inverse depth to 3D.
  const Eigen::Vector3d final_position(
      solution(0) / solution(2),
      solution(1) / solution(2),
      1.0 / solution(2));

  // Validate: feature must be in front of each observing camera.
  bool is_valid_solution = true;
  for (const auto& pose : cam_poses) {
    const Eigen::Vector3d p_c = pose.linear() * final_position + pose.translation();
    // Require feature to be sufficiently in front of the camera and not too far.
    if (p_c(2) <= optimization_config.min_depth || p_c(2) >= optimization_config.max_depth) {
      is_valid_solution = false;
      break;
    }
  }

  // Additional reprojection sanity check.
  // total_cost is sum of squared reprojection errors across all observations.
  const double rms_reproj = std::sqrt(total_cost / static_cast<double>(cam_poses.size()));
  if (rms_reproj > optimization_config.max_rms_reproj_error) {
    is_valid_solution = false;
  }

  // Convert to world frame.
  // NOTE: cam_poses[0] is T_c0_w.
  const Eigen::Isometry3d& T_c0_w = cam_poses[0];
  position = T_c0_w.linear() * final_position + T_c0_w.translation();
  if (is_valid_solution) {
    is_initialized = true;
  }
  return is_valid_solution;
}

}  // namespace msckf_vio

#endif  // SVO_ROS_MSCKF_BACKEND_FEATURE_H

