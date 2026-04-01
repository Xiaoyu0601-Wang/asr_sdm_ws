// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/**
 * @file pose_optimizer.cpp
 * @brief Camera pose optimization using Gauss-Newton with optional IMU rotation prior.
 *
 * This module refines the camera pose by minimizing reprojection errors
 * of 3D points observed in the frame. Optionally, an IMU rotation prior
 * can be applied to constrain the camera orientation using gyroscope data.
 *
 * The pose is represented in SE(3) (6 DOF: rotation + translation).
 * The optimization solves for incremental updates in the tangent space (se3).
 */

#include <stdexcept>
#include <svo/pose_optimizer.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <vikit/robust_cost.h>
#include <vikit/math_utils.h>

namespace svo {
namespace pose_optimizer {

namespace {

/// Compute skew-symmetric matrix from 3-vector (for cross product).
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d S;
  S <<  0.0, -v(2),  v(1),
        v(2),    0.0, -v(0),
       -v(1),   v(0),    0.0;
  return S;
}

/// IMU rotation prior regularization: adds a soft constraint on camera rotation
/// to be consistent with IMU-predicted orientation.
/// Uses block-diagonal separation: only affects the rotation block (top-left 3x3)
/// of the full 6x6 se(3) Hessian, leaving translation optimization unaffected.
///
/// Parameters:
///   - R_cam_world_prior: camera rotation in world frame (prior estimate)
///   - R_imu_world_mat: IMU orientation in world frame (3x3 matrix)
///   - R_imu_last_from_imu_cur: relative rotation of IMU between last and current frame
///   - lambda: regularization strength (scaled by lambda^2)
///
/// Residual: theta = log(R_imu_world * R_imu_last_from_imu_cur * R_cam_world^{-1})
/// Cost: J = theta^T * theta
/// Gradient (w.r.t. se3 tangent): g_rot = w^2 * theta
/// Hessian approximation: H_rot = w^2 * I_3 (Gauss-Newton)
inline void computeImuPriorTerm(
    const Eigen::Matrix3d& R_cam_world_prior,
    const Eigen::Matrix3d& R_imu_world_mat,
    const Eigen::Matrix3d& R_imu_last_from_imu_cur,
    double lambda,
    Vector6d* g_prior,
    Matrix6d* H_prior)
{
  // Target rotation: R_target = R_imu_world * R_imu_last_from_imu_cur
  const Eigen::Matrix3d R_target = R_imu_world_mat * R_imu_last_from_imu_cur;

  // Error rotation: R_err = R_target * R_cam_world_prior^{-1}
  const Eigen::Matrix3d R_err = R_target * R_cam_world_prior.transpose();

  // Axis-angle from SO(3) logarithm
  Eigen::AngleAxisd aa(R_err);
  double angle = aa.angle();
  if (angle > M_PI)  // Unwrap to [-pi, pi]
    angle -= 2 * M_PI;
  const Eigen::Vector3d theta = aa.axis() * angle;

  // Gauss-Newton: g = J^T * r, H ≈ J^T * J
  // For rotation-only residual: r(theta) = theta, J = I
  const double w_sq = lambda * lambda;
  g_prior->setZero();
  g_prior->tail<3>() = w_sq * theta;

  H_prior->setZero();
  // Block-diagonal: only affect rotation block (top-left 3x3)
  // Translation block (bottom-right 3x3) remains zero — IMU prior constrains
  // orientation, not position. Off-diagonal blocks also zero.
  H_prior->topLeftCorner<3, 3>() = w_sq * Eigen::Matrix3d::Identity();
}

}  // anonymous namespace

/**
 * @brief Optimizes camera pose using Gauss-Newton with robust weighting.
 *
 * Algorithm:
 * 1. Compute initial error scale using MAD estimator
 * 2. Iterate Gauss-Newton updates with Tukey robust weights
 * 3. After iteration 5, switch to tighter threshold
 * 4. Remove outliers exceeding reprojection threshold
 * 5. Compute pose covariance
 */
void optimizeGaussNewton(
    const double reproj_thresh,
    const size_t n_iter,
    const bool verbose,
    FramePtr& frame,
    double& estimated_scale,
    double& error_init,
    double& error_final,
    size_t& num_obs)
{
  double chi2(0.0);
  vector<double> chi2_vec_init, chi2_vec_final;
  vk::robust_cost::TukeyWeightFunction weight_function;
  SE3 T_old(frame->T_f_w_);
  Matrix6d A;
  Vector6d b;

  std::vector<float> errors;
  errors.reserve(frame->fts_.size());
  for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
  {
    if ((*it)->point == NULL)
      continue;
    Vector2d e = vk::project2d((*it)->f)
               - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    e *= 1.0 / (1 << (*it)->level);
    errors.push_back(e.norm());
  }
  if (errors.empty())
    return;

  vk::robust_cost::MADScaleEstimator scale_estimator;
  estimated_scale = scale_estimator.compute(errors);
  num_obs = errors.size();
  chi2_vec_init.reserve(num_obs);
  chi2_vec_final.reserve(num_obs);
  double scale = estimated_scale;

  for (size_t iter = 0; iter < n_iter; ++iter)
  {
    if (iter == 5)
      scale = 0.85 / frame->cam_->errorMultiplier2();

    b.setZero();
    A.setZero();
    double new_chi2(0.0);

    for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
    {
      if ((*it)->point == NULL)
        continue;
      Matrix26d J;
      Vector3d xyz_f(frame->T_f_w_ * (*it)->point->pos_);
      Frame::jacobian_xyz2uv(xyz_f, J);
      Vector2d e = vk::project2d((*it)->f) - vk::project2d(xyz_f);
      double sqrt_inv_cov = 1.0 / (1 << (*it)->level);
      e *= sqrt_inv_cov;
      if (iter == 0)
        chi2_vec_init.push_back(e.squaredNorm());
      J *= sqrt_inv_cov;
      double weight = weight_function.value(e.norm() / scale);
      A.noalias() += J.transpose() * J * weight;
      b.noalias() -= J.transpose() * e * weight;
      new_chi2 += e.squaredNorm() * weight;
    }

    const Vector6d dT(A.ldlt().solve(b));

    if ((iter > 0 && new_chi2 > chi2) || (bool)std::isnan((double)dT[0]))
    {
      if (verbose)
        std::cout << "it " << iter
                  << "\t FAILURE \t new_chi2 = " << new_chi2 << std::endl;
      frame->T_f_w_ = T_old;
      break;
    }

    SE3 T_new = SE3::exp(dT) * frame->T_f_w_;
    T_old = frame->T_f_w_;
    frame->T_f_w_ = T_new;
    chi2 = new_chi2;

    if (verbose)
      std::cout << "it " << iter
                << "\t Success \t new_chi2 = " << new_chi2
                << "\t norm(dT) = " << vk::norm_max(dT) << std::endl;

    if (vk::norm_max(dT) <= EPS)
      break;
  }

  const double pixel_variance = 1.0;
  frame->Cov_ = pixel_variance * (A * std::pow(frame->cam_->errorMultiplier2(), 2)).inverse();

  double reproj_thresh_scaled = reproj_thresh / frame->cam_->errorMultiplier2();
  size_t n_deleted_refs = 0;
  for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
  {
    if ((*it)->point == NULL)
      continue;
    Vector2d e = vk::project2d((*it)->f) - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    double sqrt_inv_cov = 1.0 / (1 << (*it)->level);
    e *= sqrt_inv_cov;
    chi2_vec_final.push_back(e.squaredNorm());
    if (e.norm() > reproj_thresh_scaled)
    {
      (*it)->point = NULL;
      ++n_deleted_refs;
    }
  }

  error_init = 0.0;
  error_final = 0.0;
  if (!chi2_vec_init.empty())
    error_init = sqrt(vk::getMedian(chi2_vec_init)) * frame->cam_->errorMultiplier2();
  if (!chi2_vec_final.empty())
    error_final = sqrt(vk::getMedian(chi2_vec_final)) * frame->cam_->errorMultiplier2();
  estimated_scale *= frame->cam_->errorMultiplier2();

  if (verbose)
    std::cout << "n deleted obs = " << n_deleted_refs
              << "\t scale = " << estimated_scale
              << "\t error init = " << error_init
              << "\t error end = " << error_final << std::endl;

  num_obs -= n_deleted_refs;
}

// =============================================================================
// POSE OPTIMIZER WITH IMU ROTATION PRIOR
// =============================================================================

void optimizeGaussNewtonWithImuPrior(
    const double reproj_thresh,
    const size_t n_iter,
    const bool verbose,
    FramePtr& frame,
    const Quaterniond& R_world_from_imu,
    const Quaterniond& R_imu_last_from_imu_cur,
    double lambda,
    double& estimated_scale,
    double& error_init,
    double& error_final,
    size_t& num_obs)
{
  if (lambda <= 0.0)
  {
    optimizeGaussNewton(reproj_thresh, n_iter, verbose, frame,
                        estimated_scale, error_init, error_final, num_obs);
    return;
  }

  double chi2(0.0);
  vector<double> chi2_vec_init, chi2_vec_final;
  vk::robust_cost::TukeyWeightFunction weight_function;
  SE3 T_old(frame->T_f_w_);
  Matrix6d A;
  Vector6d b;

  // Error scale
  std::vector<float> errors;
  errors.reserve(frame->fts_.size());
  for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
  {
    if ((*it)->point == NULL)
      continue;
    Vector2d e = vk::project2d((*it)->f)
               - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    e *= 1.0 / (1 << (*it)->level);
    errors.push_back(e.norm());
  }
  if (errors.empty())
    return;
  vk::robust_cost::MADScaleEstimator scale_estimator;
  estimated_scale = scale_estimator.compute(errors);
  num_obs = errors.size();
  chi2_vec_init.reserve(num_obs);
  chi2_vec_final.reserve(num_obs);
  double scale = estimated_scale;

  // Compute IMU-related rotation matrices.
  // R_imu_world: IMU orientation in world frame = R_world_from_imu^{-1}
  const Eigen::Matrix3d R_imu_world_mat = R_world_from_imu.toRotationMatrix();
  // Target camera rotation consistent with IMU = R_imu_world * R_imu_last_from_imu_cur
  // R_imu_last_from_imu_cur comes from gyroscope preintegration between frames.
  // R_cam_world_prior is the current (pre-optimization) camera rotation estimate.
  const Eigen::Matrix3d R_imu_last_from_imu_cur_mat = R_imu_last_from_imu_cur.toRotationMatrix();
  const Eigen::Matrix3d R_cam_world_prior =
      R_imu_world_mat * R_imu_last_from_imu_cur_mat;

  for (size_t iter = 0; iter < n_iter; ++iter)
  {
    if (iter == 5)
      scale = 0.85 / frame->cam_->errorMultiplier2();

    b.setZero();
    A.setZero();
    double new_chi2(0.0);

    // Accumulate reprojection residuals
    for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
    {
      if ((*it)->point == NULL)
        continue;
      Matrix26d J;
      Vector3d xyz_f(frame->T_f_w_ * (*it)->point->pos_);
      Frame::jacobian_xyz2uv(xyz_f, J);
      Vector2d e = vk::project2d((*it)->f) - vk::project2d(xyz_f);
      double sqrt_inv_cov = 1.0 / (1 << (*it)->level);
      e *= sqrt_inv_cov;
      if (iter == 0)
        chi2_vec_init.push_back(e.squaredNorm());
      J *= sqrt_inv_cov;
      double weight = weight_function.value(e.norm() / scale);
      A.noalias() += J.transpose() * J * weight;
      b.noalias() -= J.transpose() * e * weight;
      new_chi2 += e.squaredNorm() * weight;
    }

    // === Add IMU Rotation Prior ===
    if (iter == 0)
    {
      Vector6d g_prior;
      Matrix6d H_prior;
      computeImuPriorTerm(
          R_cam_world_prior, R_imu_world_mat, R_imu_last_from_imu_cur_mat,
          lambda, &g_prior, &H_prior);
      A += H_prior;
      b += g_prior;
    }

    const Vector6d dT(A.ldlt().solve(b));

    if ((iter > 0 && new_chi2 > chi2) || (bool)std::isnan((double)dT[0]))
    {
      if (verbose)
        std::cout << "it " << iter
                  << "\t FAILURE \t new_chi2 = " << new_chi2 << std::endl;
      frame->T_f_w_ = T_old;
      break;
    }

    SE3 T_new = SE3::exp(dT) * frame->T_f_w_;
    T_old = frame->T_f_w_;
    frame->T_f_w_ = T_new;
    chi2 = new_chi2;

    if (verbose)
      std::cout << "it " << iter
                << "\t Success \t new_chi2 = " << new_chi2
                << "\t norm(dT) = " << vk::norm_max(dT) << std::endl;

    if (vk::norm_max(dT) <= EPS)
      break;
  }

  const double pixel_variance = 1.0;
  frame->Cov_ = pixel_variance * (A * std::pow(frame->cam_->errorMultiplier2(), 2)).inverse();

  double reproj_thresh_scaled = reproj_thresh / frame->cam_->errorMultiplier2();
  size_t n_deleted_refs = 0;
  for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
  {
    if ((*it)->point == NULL)
      continue;
    Vector2d e = vk::project2d((*it)->f) - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    double sqrt_inv_cov = 1.0 / (1 << (*it)->level);
    e *= sqrt_inv_cov;
    chi2_vec_final.push_back(e.squaredNorm());
    if (e.norm() > reproj_thresh_scaled)
    {
      (*it)->point = NULL;
      ++n_deleted_refs;
    }
  }

  error_init = 0.0;
  error_final = 0.0;
  if (!chi2_vec_init.empty())
    error_init = sqrt(vk::getMedian(chi2_vec_init)) * frame->cam_->errorMultiplier2();
  if (!chi2_vec_final.empty())
    error_final = sqrt(vk::getMedian(chi2_vec_final)) * frame->cam_->errorMultiplier2();
  estimated_scale *= frame->cam_->errorMultiplier2();

  if (verbose)
    std::cout << "n deleted obs = " << n_deleted_refs
              << "\t scale = " << estimated_scale
              << "\t error init = " << error_init
              << "\t error end = " << error_final << std::endl;

  num_obs -= n_deleted_refs;
}

}  // namespace pose_optimizer
}  // namespace svo
