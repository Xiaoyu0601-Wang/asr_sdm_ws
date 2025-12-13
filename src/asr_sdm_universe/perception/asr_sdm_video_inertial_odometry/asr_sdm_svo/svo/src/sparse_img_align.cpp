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
 * @file sparse_img_align.cpp
 * @brief Direct sparse image alignment for camera motion estimation.
 * 
 * This module implements the "semi-direct" approach of SVO: estimating
 * camera motion by minimizing photometric error on sparse features.
 * 
 * Key concepts:
 * - Direct method: Optimizes photometric error (pixel intensity difference)
 *   instead of geometric error (reprojection distance)
 * - Sparse: Only considers pixels around detected features, not dense
 * - Multi-scale: Uses image pyramid for coarse-to-fine optimization
 * - Inverse compositional: Precomputes Jacobians on reference image
 * 
 * The optimization minimizes:
 *   E(T) = Σ || I_cur(π(T * X_i)) - I_ref(π(X_i)) ||²
 * 
 * where:
 * - T is the relative pose (SE3) from reference to current
 * - X_i are 3D points in reference frame
 * - π is the camera projection function
 * - I_ref, I_cur are reference and current images
 * 
 * Reference: Forster et al., "SVO: Fast Semi-Direct Monocular Visual Odometry"
 */

#include <algorithm>
#include <svo/sparse_img_align.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/config.h>
#include <svo/point.h>
#include <vikit/abstract_camera.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>

namespace svo {

/**
 * @brief Constructs sparse image alignment optimizer.
 * 
 * @param max_level Coarsest pyramid level to start optimization
 * @param min_level Finest pyramid level to end optimization
 * @param n_iter Maximum iterations per pyramid level
 * @param method Optimization method (GaussNewton or LevenbergMarquardt)
 * @param display Whether to display residual images
 * @param verbose Whether to print debug output
 */
SparseImgAlign::SparseImgAlign(
    int max_level, int min_level, int n_iter,
    Method method, bool display, bool verbose) :
        display_(display),
        max_level_(max_level),
        min_level_(min_level)
{
  n_iter_ = n_iter;
  n_iter_init_ = n_iter_;
  method_ = method;
  verbose_ = verbose;
  eps_ = 0.000001;  // Convergence threshold
}

/**
 * @brief Runs the sparse image alignment optimization.
 * 
 * Performs coarse-to-fine optimization across pyramid levels:
 * 1. Start at coarsest level (max_level)
 * 2. Optimize pose at each level
 * 3. Propagate result to finer level
 * 4. End at finest level (min_level)
 * 
 * @param ref_frame Reference frame with features
 * @param cur_frame Current frame (pose updated in place)
 * @return Number of features successfully tracked
 */
size_t SparseImgAlign::run(FramePtr ref_frame, FramePtr cur_frame)
{
  reset();

  if(ref_frame->fts_.empty())
  {
    SVO_WARN_STREAM("SparseImgAlign: no features to track!");
    return 0;
  }

  ref_frame_ = ref_frame;
  cur_frame_ = cur_frame;
  
  // Allocate cache for reference patches (one row per feature, patch_area columns)
  ref_patch_cache_ = cv::Mat(ref_frame_->fts_.size(), patch_area_, CV_32F);
  
  // Allocate cache for Jacobians (6 rows for SE3, n_features * patch_area columns)
  jacobian_cache_.resize(Eigen::NoChange, ref_patch_cache_.rows*patch_area_);
  
  // Visibility flags for each feature
  visible_fts_.resize(ref_patch_cache_.rows, false);

  // Initial relative pose estimate
  SE3 T_cur_from_ref(cur_frame_->T_f_w_ * ref_frame_->T_f_w_.inverse());

  // Coarse-to-fine optimization
  for(level_=max_level_; level_>=min_level_; --level_)
  {
    mu_ = 0.1;  // Levenberg-Marquardt damping
    jacobian_cache_.setZero();
    have_ref_patch_cache_ = false;  // Recompute at each level
    
    if(verbose_)
      printf("\nPYRAMID LEVEL %i\n---------------\n", level_);
      
    optimize(T_cur_from_ref);  // Run optimization at this level
  }
  
  // Update current frame pose
  cur_frame_->T_f_w_ = T_cur_from_ref * ref_frame_->T_f_w_;

  return n_meas_/patch_area_;  // Return number of features (not pixels)
}

/**
 * @brief Computes the Fisher information matrix.
 * 
 * The Fisher information is the expected value of the Hessian,
 * useful for uncertainty estimation.
 * 
 * @return 6x6 Fisher information matrix
 */
Eigen::Matrix<double, 6, 6> SparseImgAlign::getFisherInformation()
{
  double sigma_i_sq = 5e-4*255*255;  // Assumed image noise variance
  Eigen::Matrix<double,6,6> I = H_/sigma_i_sq;
  return I;
}

/**
 * @brief Precomputes reference patches and Jacobians.
 * 
 * For inverse compositional approach, the Jacobian is computed on
 * the reference image and cached. This avoids recomputing gradients
 * at each iteration.
 * 
 * For each visible feature:
 * 1. Extract patch around feature (with bilinear interpolation)
 * 2. Compute image gradients (dx, dy)
 * 3. Compute projection Jacobian (d_pixel / d_pose)
 * 4. Chain rule: d_intensity / d_pose = (dx, dy) * d_pixel / d_pose
 */
void SparseImgAlign::precomputeReferencePatches()
{
  const int border = patch_halfsize_+1;
  const cv::Mat& ref_img = ref_frame_->img_pyr_.at(level_);
  const int stride = ref_img.cols;
  const float scale = 1.0f/(1<<level_);  // Scale factor for this pyramid level
  const Vector3d ref_pos = ref_frame_->pos();
  const double focal_length = ref_frame_->cam_->errorMultiplier2();
  
  size_t feature_counter = 0;
  std::vector<bool>::iterator visiblity_it = visible_fts_.begin();
  
  for(auto it=ref_frame_->fts_.begin(), ite=ref_frame_->fts_.end();
      it!=ite; ++it, ++feature_counter, ++visiblity_it)
  {
    // Check if feature with patch is within image bounds
    const float u_ref = (*it)->px[0]*scale;
    const float v_ref = (*it)->px[1]*scale;
    const int u_ref_i = floorf(u_ref);
    const int v_ref_i = floorf(v_ref);
    
    if((*it)->point == NULL || u_ref_i-border < 0 || v_ref_i-border < 0 || 
       u_ref_i+border >= ref_img.cols || v_ref_i+border >= ref_img.rows)
      continue;
      
    *visiblity_it = true;

    // Compute 3D point in reference frame (using feature depth)
    const double depth(((*it)->point->pos_ - ref_pos).norm());
    const Vector3d xyz_ref((*it)->f*depth);

    // Jacobian of projection w.r.t. pose: d(uv) / d(se3)
    Eigen::Matrix<double,2,6> frame_jac;
    Frame::jacobian_xyz2uv(xyz_ref, frame_jac);

    // Bilinear interpolation weights
    const float subpix_u_ref = u_ref-u_ref_i;
    const float subpix_v_ref = v_ref-v_ref_i;
    const float w_ref_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
    const float w_ref_tr = subpix_u_ref * (1.0-subpix_v_ref);
    const float w_ref_bl = (1.0-subpix_u_ref) * subpix_v_ref;
    const float w_ref_br = subpix_u_ref * subpix_v_ref;
    
    size_t pixel_counter = 0;
    float* cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_*feature_counter;
    
    for(int y=0; y<patch_size_; ++y)
    {
      uint8_t* ref_img_ptr = (uint8_t*) ref_img.data + (v_ref_i+y-patch_halfsize_)*stride + (u_ref_i-patch_halfsize_);
      
      for(int x=0; x<patch_size_; ++x, ++ref_img_ptr, ++cache_ptr, ++pixel_counter)
      {
        // Bilinear interpolation of reference patch
        *cache_ptr = w_ref_tl*ref_img_ptr[0] + w_ref_tr*ref_img_ptr[1] + 
                     w_ref_bl*ref_img_ptr[stride] + w_ref_br*ref_img_ptr[stride+1];

        // Compute image gradient using bilinear interpolation
        float dx = 0.5f * ((w_ref_tl*ref_img_ptr[1] + w_ref_tr*ref_img_ptr[2] + 
                           w_ref_bl*ref_img_ptr[stride+1] + w_ref_br*ref_img_ptr[stride+2])
                          -(w_ref_tl*ref_img_ptr[-1] + w_ref_tr*ref_img_ptr[0] + 
                           w_ref_bl*ref_img_ptr[stride-1] + w_ref_br*ref_img_ptr[stride]));
        float dy = 0.5f * ((w_ref_tl*ref_img_ptr[stride] + w_ref_tr*ref_img_ptr[1+stride] + 
                           w_ref_bl*ref_img_ptr[stride*2] + w_ref_br*ref_img_ptr[stride*2+1])
                          -(w_ref_tl*ref_img_ptr[-stride] + w_ref_tr*ref_img_ptr[1-stride] + 
                           w_ref_bl*ref_img_ptr[0] + w_ref_br*ref_img_ptr[1]));

        // Cache the full Jacobian: d_intensity / d_pose
        // = (d_intensity / d_uv) * (d_uv / d_pose) * scale
        jacobian_cache_.col(feature_counter*patch_area_ + pixel_counter) =
            (dx*frame_jac.row(0) + dy*frame_jac.row(1))*(focal_length / (1<<level_));
      }
    }
  }
  have_ref_patch_cache_ = true;
}

/**
 * @brief Computes residuals and optionally linearizes the system.
 * 
 * For each visible feature:
 * 1. Project 3D point to current image using T_cur_from_ref
 * 2. Extract current patch with bilinear interpolation
 * 3. Compute pixel-wise residuals (I_cur - I_ref)
 * 4. Optionally accumulate Hessian and gradient for optimization
 * 
 * @param T_cur_from_ref Current relative pose estimate
 * @param linearize_system Whether to build the linear system (H, Jres)
 * @param compute_weight_scale Whether to compute robust weight scale
 * @return Mean chi-squared error
 */
double SparseImgAlign::computeResiduals(
    const SE3& T_cur_from_ref,
    bool linearize_system,
    bool compute_weight_scale)
{
  const cv::Mat& cur_img = cur_frame_->img_pyr_.at(level_);

  // Optionally create residual image for visualization
  if(linearize_system && display_)
    resimg_ = cv::Mat(cur_img.size(), CV_32F, cv::Scalar(0));

  // Ensure reference patches are computed
  if(have_ref_patch_cache_ == false)
    precomputeReferencePatches();

  // Collect errors for robust weight estimation
  std::vector<float> errors;
  if(compute_weight_scale)
    errors.reserve(visible_fts_.size());
    
  const int stride = cur_img.cols;
  const int border = patch_halfsize_+1;
  const float scale = 1.0f/(1<<level_);
  const Vector3d ref_pos(ref_frame_->pos());
  float chi2 = 0.0;
  
  size_t feature_counter = 0;
  std::vector<bool>::iterator visiblity_it = visible_fts_.begin();
  
  for(auto it=ref_frame_->fts_.begin(); it!=ref_frame_->fts_.end();
      ++it, ++feature_counter, ++visiblity_it)
  {
    if(!*visiblity_it)
      continue;

    // Project 3D point to current image
    const double depth = ((*it)->point->pos_ - ref_pos).norm();
    const Vector3d xyz_ref((*it)->f*depth);
    const Vector3d xyz_cur(T_cur_from_ref * xyz_ref);
    const Vector2f uv_cur_pyr(cur_frame_->cam_->world2cam(xyz_cur).cast<float>() * scale);
    
    const float u_cur = uv_cur_pyr[0];
    const float v_cur = uv_cur_pyr[1];
    const int u_cur_i = floorf(u_cur);
    const int v_cur_i = floorf(v_cur);

    // Check if projection is within image bounds
    if(u_cur_i < 0 || v_cur_i < 0 || u_cur_i-border < 0 || v_cur_i-border < 0 || 
       u_cur_i+border >= cur_img.cols || v_cur_i+border >= cur_img.rows)
      continue;

    // Bilinear interpolation weights for current image
    const float subpix_u_cur = u_cur-u_cur_i;
    const float subpix_v_cur = v_cur-v_cur_i;
    const float w_cur_tl = (1.0-subpix_u_cur) * (1.0-subpix_v_cur);
    const float w_cur_tr = subpix_u_cur * (1.0-subpix_v_cur);
    const float w_cur_bl = (1.0-subpix_u_cur) * subpix_v_cur;
    const float w_cur_br = subpix_u_cur * subpix_v_cur;
    
    float* ref_patch_cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_*feature_counter;
    size_t pixel_counter = 0;
    
    for(int y=0; y<patch_size_; ++y)
    {
      uint8_t* cur_img_ptr = (uint8_t*) cur_img.data + (v_cur_i+y-patch_halfsize_)*stride + (u_cur_i-patch_halfsize_);

      for(int x=0; x<patch_size_; ++x, ++pixel_counter, ++cur_img_ptr, ++ref_patch_cache_ptr)
      {
        // Bilinear interpolation of current patch
        const float intensity_cur = w_cur_tl*cur_img_ptr[0] + w_cur_tr*cur_img_ptr[1] + 
                                    w_cur_bl*cur_img_ptr[stride] + w_cur_br*cur_img_ptr[stride+1];
        
        // Photometric residual
        const float res = intensity_cur - (*ref_patch_cache_ptr);

        // Collect for robust scale estimation
        if(compute_weight_scale)
          errors.push_back(fabsf(res));

        // Apply robust weighting if enabled
        float weight = 1.0;
        if(use_weights_) {
          weight = weight_function_->value(res/scale_);
        }

        chi2 += res*res*weight;
        n_meas_++;

        if(linearize_system)
        {
          // Accumulate Gauss-Newton system: H = J^T * W * J, Jres = -J^T * W * r
          const Eigen::Matrix<double,6,1> J(jacobian_cache_.col(feature_counter*patch_area_ + pixel_counter));
          H_.noalias() += J*J.transpose()*weight;
          Jres_.noalias() -= J*res*weight;
          
          if(display_)
            resimg_.at<float>((int) v_cur+y-patch_halfsize_, (int) u_cur+x-patch_halfsize_) = res/255.0;
        }
      }
    }
  }

  // Compute robust scale on first iteration
  if(compute_weight_scale && iter_ == 0)
    scale_ = scale_estimator_->compute(errors);

  return chi2/n_meas_;
}

/**
 * @brief Solves the linear system for pose update.
 * 
 * Solves H * x = Jres using LDLT decomposition.
 * 
 * @return 1 on success, 0 if solution is NaN
 */
int SparseImgAlign::solve()
{
  x_ = H_.ldlt().solve(Jres_);
  if((bool) std::isnan((double) x_[0]))
    return 0;
  return 1;
}

/**
 * @brief Applies the computed update to the pose.
 * 
 * Uses inverse compositional update: T_new = T_old * exp(-x)
 * 
 * @param T_curold_from_ref Old pose estimate
 * @param T_curnew_from_ref Output: updated pose estimate
 */
void SparseImgAlign::update(
    const ModelType& T_curold_from_ref,
    ModelType& T_curnew_from_ref)
{
  T_curnew_from_ref =  T_curold_from_ref * SE3::exp(-x_);
}

/**
 * @brief Called at the start of each iteration.
 */
void SparseImgAlign::startIteration()
{}

/**
 * @brief Called at the end of each iteration.
 * 
 * Optionally displays residual image for debugging.
 */
void SparseImgAlign::finishIteration()
{
  if(display_)
  {
    cv::namedWindow("residuals", cv::WINDOW_AUTOSIZE);
    cv::imshow("residuals", resimg_*10);
    cv::waitKey(0);
  }
}

} // namespace svo
