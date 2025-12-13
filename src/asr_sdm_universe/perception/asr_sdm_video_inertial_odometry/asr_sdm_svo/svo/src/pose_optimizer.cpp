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
 * @brief Camera pose optimization using Gauss-Newton.
 * 
 * This module refines the camera pose by minimizing reprojection errors
 * of 3D points observed in the frame. The optimization uses:
 * 
 * - Gauss-Newton algorithm with robust Tukey weight function
 * - Pyramid level-dependent covariance (lower weight at coarser levels)
 * - MAD (Median Absolute Deviation) scale estimator for robustness
 * - Two-phase approach: initial robust estimation, then tighter threshold
 * 
 * The pose is represented in SE3 (6 DOF: rotation + translation).
 * The optimization solves for incremental updates in the tangent space (se3).
 * 
 * Outlier handling:
 * - Robust Tukey weight function downweights large residuals
 * - After optimization, measurements exceeding threshold are removed
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

/**
 * @brief Optimizes camera pose using Gauss-Newton with robust weighting.
 * 
 * Algorithm:
 * 1. Compute initial error scale using MAD estimator
 * 2. Iterate Gauss-Newton updates with Tukey robust weights
 * 3. After iteration 5, switch to tighter threshold
 * 4. Remove outliers exceeding reprojection threshold
 * 5. Compute pose covariance
 * 
 * @param reproj_thresh Reprojection threshold for outlier removal (pixels)
 * @param n_iter Maximum number of iterations
 * @param verbose Print debug information
 * @param frame Frame whose pose to optimize (updated in place)
 * @param estimated_scale Output: estimated error scale
 * @param error_init Output: initial median error (pixels)
 * @param error_final Output: final median error (pixels)
 * @param num_obs Output: number of inlier observations after optimization
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
  // Initialization
  double chi2(0.0);
  vector<double> chi2_vec_init, chi2_vec_final;
  vk::robust_cost::TukeyWeightFunction weight_function;
  SE3 T_old(frame->T_f_w_);  // Save for potential rollback
  Matrix6d A;  // Hessian approximation (J^T * W * J)
  Vector6d b;  // Weighted gradient (-J^T * W * e)

  // === Compute Error Scale ===
  // Use MAD (Median Absolute Deviation) for robust scale estimation
  std::vector<float> errors; 
  errors.reserve(frame->fts_.size());
  
  for(auto it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point == NULL)
      continue;
    // Reprojection error on unit plane
    Vector2d e = vk::project2d((*it)->f)
               - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    e *= 1.0 / (1<<(*it)->level);  // Scale by pyramid level
    errors.push_back(e.norm());
  }
  
  if(errors.empty())
    return;
    
  vk::robust_cost::MADScaleEstimator scale_estimator;
  estimated_scale = scale_estimator.compute(errors);

  num_obs = errors.size();
  chi2_vec_init.reserve(num_obs);
  chi2_vec_final.reserve(num_obs);
  double scale = estimated_scale;
  
  // === Gauss-Newton Iterations ===
  for(size_t iter=0; iter<n_iter; iter++)
  {
    // After iteration 5, switch to tighter scale for fine refinement
    if(iter == 5)
      scale = 0.85/frame->cam_->errorMultiplier2();

    b.setZero();
    A.setZero();
    double new_chi2(0.0);

    // Accumulate residuals and Jacobians
    for(auto it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
    {
      if((*it)->point == NULL)
        continue;
        
      Matrix26d J;  // Jacobian of projection w.r.t. pose
      Vector3d xyz_f(frame->T_f_w_ * (*it)->point->pos_);
      Frame::jacobian_xyz2uv(xyz_f, J);
      
      // Reprojection error (measured - predicted)
      Vector2d e = vk::project2d((*it)->f) - vk::project2d(xyz_f);
      
      // Scale error by inverse covariance (pyramid level dependent)
      double sqrt_inv_cov = 1.0 / (1<<(*it)->level);
      e *= sqrt_inv_cov;
      
      if(iter == 0)
        chi2_vec_init.push_back(e.squaredNorm());  // For statistics
      
      J *= sqrt_inv_cov;
      
      // Robust weight (Tukey function downweights outliers)
      double weight = weight_function.value(e.norm()/scale);
      
      // Gauss-Newton update: H = J^T * W * J, b = -J^T * W * e
      A.noalias() += J.transpose()*J*weight;
      b.noalias() -= J.transpose()*e*weight;
      new_chi2 += e.squaredNorm()*weight;
    }

    // Solve linear system for pose update (6 DOF)
    const Vector6d dT(A.ldlt().solve(b));

    // Check for error increase or NaN
    if((iter > 0 && new_chi2 > chi2) || (bool) std::isnan((double)dT[0]))
    {
      if(verbose)
        std::cout << "it " << iter
                  << "\t FAILURE \t new_chi2 = " << new_chi2 << std::endl;
      frame->T_f_w_ = T_old;  // Roll back
      break;
    }

    // Apply SE3 update using exponential map
    SE3 T_new = SE3::exp(dT)*frame->T_f_w_;
    T_old = frame->T_f_w_;
    frame->T_f_w_ = T_new;
    chi2 = new_chi2;
    
    if(verbose)
      std::cout << "it " << iter
                << "\t Success \t new_chi2 = " << new_chi2
                << "\t norm(dT) = " << vk::norm_max(dT) << std::endl;

    // Check for convergence
    if(vk::norm_max(dT) <= EPS)
      break;
  }

  // === Compute Pose Covariance ===
  // Covariance = inverse of Fisher information matrix
  // This is an optimistic estimate (assumes all measurements are inliers)
  const double pixel_variance=1.0;
  frame->Cov_ = pixel_variance*(A*std::pow(frame->cam_->errorMultiplier2(),2)).inverse();

  // === Remove Outliers ===
  // Remove features with reprojection error exceeding threshold
  double reproj_thresh_scaled = reproj_thresh / frame->cam_->errorMultiplier2();
  size_t n_deleted_refs = 0;
  
  for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point == NULL)
      continue;
      
    Vector2d e = vk::project2d((*it)->f) - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    double sqrt_inv_cov = 1.0 / (1<<(*it)->level);
    e *= sqrt_inv_cov;
    chi2_vec_final.push_back(e.squaredNorm());
    
    if(e.norm() > reproj_thresh_scaled)
    {
      // Remove reference (point link will be established only if keyframe)
      (*it)->point = NULL;
      ++n_deleted_refs;
    }
  }

  // === Compute Error Statistics ===
  error_init=0.0;
  error_final=0.0;
  if(!chi2_vec_init.empty())
    error_init = sqrt(vk::getMedian(chi2_vec_init))*frame->cam_->errorMultiplier2();
  if(!chi2_vec_final.empty())
    error_final = sqrt(vk::getMedian(chi2_vec_final))*frame->cam_->errorMultiplier2();

  // Scale estimated_scale to pixel units
  estimated_scale *= frame->cam_->errorMultiplier2();
  
  if(verbose)
    std::cout << "n deleted obs = " << n_deleted_refs
              << "\t scale = " << estimated_scale
              << "\t error init = " << error_init
              << "\t error end = " << error_final << std::endl;
              
  num_obs -= n_deleted_refs;
}

} // namespace pose_optimizer
} // namespace svo
