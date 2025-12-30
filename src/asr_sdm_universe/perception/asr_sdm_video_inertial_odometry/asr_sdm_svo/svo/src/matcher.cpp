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
 * @file matcher.cpp
 * @brief Feature matching using affine warping and epipolar search.
 * 
 * This module provides two matching strategies:
 * 
 * 1. Direct matching (findMatchDirect): For reprojected map points
 *    - Warp reference patch to current view using affine transformation
 *    - Align warped patch using inverse compositional Lucas-Kanade
 *    - Handles perspective distortion due to viewpoint change
 * 
 * 2. Epipolar matching (findEpipolarMatchDirect): For depth estimation
 *    - Search along epipolar line in current image
 *    - Uses ZMSSD (Zero-Mean Sum of Squared Differences) for matching
 *    - Subpixel refinement after coarse match
 * 
 * Affine warp model handles:
 * - Scale changes (distance to point)
 * - Rotation (camera orientation change)
 * - Perspective foreshortening (viewing angle change)
 */

#include <cstdlib>
#include <vikit/abstract_camera.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>
#include <vikit/patch_score.h>
#include <svo/matcher.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/config.h>
#include <svo/feature_alignment.h>

namespace svo {

namespace warp {

/**
 * @brief Computes affine warp matrix from reference to current view.
 * 
 * The affine warp approximates the perspective transformation of a patch
 * when observed from a different viewpoint. It's computed by:
 * 1. Taking the reference patch center and two neighbors
 * 2. Projecting all three points into the current image
 * 3. Computing the affine transformation from the displacement vectors
 * 
 * @param cam_ref Reference camera model
 * @param cam_cur Current camera model
 * @param px_ref Reference pixel position
 * @param f_ref Reference bearing vector
 * @param depth_ref Depth of the point in reference frame
 * @param T_cur_ref Transformation from reference to current frame
 * @param level_ref Pyramid level of reference feature
 * @param A_cur_ref Output: 2x2 affine warp matrix
 */
void getWarpMatrixAffine(
    const vk::AbstractCamera& cam_ref,
    const vk::AbstractCamera& cam_cur,
    const Vector2d& px_ref,
    const Vector3d& f_ref,
    const double depth_ref,
    const SE3& T_cur_ref,
    const int level_ref,
    Matrix2d& A_cur_ref)
{
  // Half patch size for computing derivatives (in pixels at reference level)
  const int halfpatch_size = 5;
  
  // 3D position of patch center in reference frame
  const Vector3d xyz_ref(f_ref*depth_ref);
  
  // Compute 3D positions of horizontal and vertical neighbors
  // Scale by pyramid level to get correct pixel offset
  Vector3d xyz_du_ref(cam_ref.cam2world(px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref)));
  Vector3d xyz_dv_ref(cam_ref.cam2world(px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)));
  
  // Scale neighbors to same depth as center point
  xyz_du_ref *= xyz_ref[2]/xyz_du_ref[2];
  xyz_dv_ref *= xyz_ref[2]/xyz_dv_ref[2];
  
  // Project all three points to current image
  const Vector2d px_cur(cam_cur.world2cam(T_cur_ref*(xyz_ref)));
  const Vector2d px_du(cam_cur.world2cam(T_cur_ref*(xyz_du_ref)));
  const Vector2d px_dv(cam_cur.world2cam(T_cur_ref*(xyz_dv_ref)));
  
  // Affine warp columns are normalized displacement vectors
  A_cur_ref.col(0) = (px_du - px_cur)/halfpatch_size;
  A_cur_ref.col(1) = (px_dv - px_cur)/halfpatch_size;
}

/**
 * @brief Determines optimal pyramid level for matching.
 * 
 * Higher levels (coarser resolution) are preferred when the affine warp
 * has large scale changes, as this keeps patch sampling consistent.
 * 
 * @param A_cur_ref Affine warp matrix
 * @param max_level Maximum allowed pyramid level
 * @return Optimal pyramid level
 */
int getBestSearchLevel(
    const Matrix2d& A_cur_ref,
    const int max_level)
{
  int search_level = 0;
  double D = A_cur_ref.determinant();  // Determinant indicates scale change
  
  // Each pyramid level halves resolution, reducing effective determinant by 4
  while(D > 3.0 && search_level < max_level)
  {
    search_level += 1;
    D *= 0.25;
  }
  return search_level;
}

/**
 * @brief Applies affine warp to create a warped patch.
 * 
 * Samples the reference image at positions computed by the inverse
 * affine transformation to create a warped patch that should match
 * the appearance in the current image.
 * 
 * @param A_cur_ref Affine warp matrix (cur to ref)
 * @param img_ref Reference image at appropriate pyramid level
 * @param px_ref Reference pixel position (at level 0)
 * @param level_ref Pyramid level of reference image
 * @param search_level Pyramid level for search
 * @param halfpatch_size Half the patch size
 * @param patch Output: warped patch data
 */
void warpAffine(
    const Matrix2d& A_cur_ref,
    const cv::Mat& img_ref,
    const Vector2d& px_ref,
    const int level_ref,
    const int search_level,
    const int halfpatch_size,
    uint8_t* patch)
{
  const int patch_size = halfpatch_size*2 ;
  const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
  
  if(isnan(A_ref_cur(0,0)))
  {
    printf("Affine warp is NaN, probably camera has no translation\n");
    return;
  }

  // Sample reference image using inverse warp
  uint8_t* patch_ptr = patch;
  const Vector2f px_ref_pyr = px_ref.cast<float>() / (1<<level_ref);
  
  for (int y=0; y<patch_size; ++y)
  {
    for (int x=0; x<patch_size; ++x, ++patch_ptr)
    {
      // Compute position in reference image for this patch pixel
      Vector2f px_patch(x-halfpatch_size, y-halfpatch_size);
      px_patch *= (1<<search_level);  // Scale for search level
      const Vector2f px(A_ref_cur*px_patch + px_ref_pyr);
      
      // Bilinear interpolation with boundary check
      if (px[0]<0 || px[1]<0 || px[0]>=img_ref.cols-1 || px[1]>=img_ref.rows-1)
        *patch_ptr = 0;
      else
        *patch_ptr = (uint8_t) vk::interpolateMat_8u(img_ref, px[0], px[1]);
    }
  }
}

} // namespace warp

/**
 * @brief Triangulates depth from two bearing vectors.
 * 
 * Given a relative pose and two bearing vectors pointing to the same
 * 3D point, computes the depth of the point in the reference frame.
 * 
 * Uses linear least squares: solve [R*f_ref, f_cur] * [d_ref, -d_cur]^T = t
 * 
 * @param T_search_ref Transformation from reference to search frame
 * @param f_ref Bearing vector in reference frame
 * @param f_cur Bearing vector in current/search frame
 * @param depth Output: depth in reference frame
 * @return true if triangulation succeeded (sufficient baseline)
 */
bool depthFromTriangulation(
    const SE3& T_search_ref,
    const Vector3d& f_ref,
    const Vector3d& f_cur,
    double& depth)
{
  Eigen::Matrix<double,3,2> A; A << T_search_ref.rotationMatrix() * f_ref, f_cur;
  const Matrix2d AtA = A.transpose()*A;
  
  // Check for degenerate configuration (parallel rays)
  if(AtA.determinant() < 0.000001)
    return false;
    
  const Vector2d depth2 = - AtA.inverse()*A.transpose()*T_search_ref.translation();
  depth = fabs(depth2[0]);
  return true;
}

/**
 * @brief Extracts patch without border from patch with border.
 * 
 * The bordered patch is used for gradient computation; the inner
 * patch is used for matching.
 */
void Matcher::createPatchFromPatchWithBorder()
{
  uint8_t* ref_patch_ptr = patch_;
  for(int y=1; y<patch_size_+1; ++y, ref_patch_ptr += patch_size_)
  {
    uint8_t* ref_patch_border_ptr = patch_with_border_ + y*(patch_size_+2) + 1;
    for(int x=0; x<patch_size_; ++x)
      ref_patch_ptr[x] = ref_patch_border_ptr[x];
  }
}

/**
 * @brief Finds a match for a 3D point in the current frame.
 * 
 * For reprojection-based tracking:
 * 1. Find the best reference observation of the point
 * 2. Compute affine warp from reference to current view
 * 3. Warp reference patch
 * 4. Align warped patch in current image
 * 
 * @param pt 3D point to match
 * @param cur_frame Current frame
 * @param px_cur Input: initial estimate; Output: refined position
 * @return true if match found
 */
bool Matcher::findMatchDirect(
    const Point& pt,
    const Frame& cur_frame,
    Vector2d& px_cur)
{
  // Get observation with most similar viewing angle
  if(!pt.getCloseViewObs(cur_frame.pos(), ref_ftr_))
    return false;

  // Check reference patch is within image bounds
  if(!ref_ftr_->frame->cam_->isInFrame(
      ref_ftr_->px.cast<int>()/(1<<ref_ftr_->level), halfpatch_size_+2, ref_ftr_->level))
    return false;

  // Compute affine warp
  warp::getWarpMatrixAffine(
      *ref_ftr_->frame->cam_, *cur_frame.cam_, ref_ftr_->px, ref_ftr_->f,
      (ref_ftr_->frame->pos() - pt.pos_).norm(),
      cur_frame.T_f_w_ * ref_ftr_->frame->T_f_w_.inverse(), ref_ftr_->level, A_cur_ref_);
  
  // Select optimal pyramid level
  search_level_ = warp::getBestSearchLevel(A_cur_ref_, Config::nPyrLevels()-1);
  
  // Warp reference patch
  warp::warpAffine(A_cur_ref_, ref_ftr_->frame->img_pyr_[ref_ftr_->level], ref_ftr_->px,
                   ref_ftr_->level, search_level_, halfpatch_size_+1, patch_with_border_);
  createPatchFromPatchWithBorder();

  // Align patch in current image
  Vector2d px_scaled(px_cur/(1<<search_level_));

  bool success = false;
  if(ref_ftr_->type == Feature::EDGELET)
  {
    // 1D alignment for edgelets (along gradient direction)
    Vector2d dir_cur(A_cur_ref_*ref_ftr_->grad);
    dir_cur.normalize();
    success = feature_alignment::align1D(
          cur_frame.img_pyr_[search_level_], dir_cur.cast<float>(),
          patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
  }
  else
  {
    // 2D alignment for corners
    success = feature_alignment::align2D(
      cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
      options_.align_max_iter, px_scaled);
  }
  
  // Scale result back to level 0
  px_cur = px_scaled * (1<<search_level_);
  return success;
}

/**
 * @brief Finds a match along the epipolar line for depth estimation.
 * 
 * For depth filter updates:
 * 1. Compute epipolar line in current image
 * 2. Search along line using ZMSSD patch matching
 * 3. Subpixel refinement at best match
 * 4. Triangulate depth from matched position
 * 
 * @param ref_frame Reference frame containing the feature
 * @param cur_frame Current frame to search in
 * @param ref_ftr Reference feature
 * @param d_estimate Estimated depth (inverse of seed's mu)
 * @param d_min Minimum depth to search
 * @param d_max Maximum depth to search
 * @param depth Output: triangulated depth
 * @return true if match found
 */
bool Matcher::findEpipolarMatchDirect(
    const Frame& ref_frame,
    const Frame& cur_frame,
    const Feature& ref_ftr,
    const double d_estimate,
    const double d_min,
    const double d_max,
    double& depth)
{
  SE3 T_cur_ref = cur_frame.T_f_w_ * ref_frame.T_f_w_.inverse();

  // 【深度范围护栏】对深度搜索区间做 clamp（单位：米）。
  // 背景：depth filter 会把 seed 的深度区间 (d_min/d_max) 传进来做极线搜索；
  // 若该区间发散（例如 d_max ~ 1e8），会导致 epipolar line 极长、n_steps 爆炸，
  // 触发 max_epi_search_steps 后整段更新被跳过 => depth filter 无法获得观测更新 => 地图崩溃/频繁重定位。
  // 这里对 EuRoC MH_01 采用 0.2~30m 的室内合理范围作为工程护栏，让更新至少“可执行”。
  constexpr double kDepthMinClamp = 0.2;
  constexpr double kDepthMaxClamp = 30.0;
  const double d_min_c = std::max(d_min, kDepthMinClamp);
  const double d_max_c = std::min(d_max, kDepthMaxClamp);
  if (!(d_max_c > d_min_c)) {
    return false;
  }

  const int zmssd_threshold = static_cast<int>(PatchScore::threshold() * Config::patchMatchThresholdFactor());
  int zmssd_best = zmssd_threshold;  // Best ZMSSD score found along epipolar line
  Vector2d uv_best;

  // Compute epipolar line endpoints on unit plane
  Vector2d A = vk::project2d(T_cur_ref * (ref_ftr.f*d_min_c));
  Vector2d B = vk::project2d(T_cur_ref * (ref_ftr.f*d_max_c));
  epi_dir_ = A - B;

  // Compute affine warp at estimated depth
  warp::getWarpMatrixAffine(
      *ref_frame.cam_, *cur_frame.cam_, ref_ftr.px, ref_ftr.f,
      d_estimate, T_cur_ref, ref_ftr.level, A_cur_ref_);

  // Check for edgelet-epipolar angle (reject if perpendicular to epipolar line)
  reject_ = false;
  if(ref_ftr.type == Feature::EDGELET && options_.epi_search_edgelet_filtering)
  {
    const Vector2d grad_cur = (A_cur_ref_ * ref_ftr.grad).normalized();
    const double cosangle = fabs(grad_cur.dot(epi_dir_.normalized()));
    if(cosangle < options_.epi_search_edgelet_max_angle) {
      reject_ = true;
      return false;
    }
  }

  search_level_ = warp::getBestSearchLevel(A_cur_ref_, Config::nPyrLevels()-1);

  // Compute epipolar line length in pixels
  Vector2d px_A(cur_frame.cam_->world2cam(A));
  Vector2d px_B(cur_frame.cam_->world2cam(B));
  epi_length_ = (px_A-px_B).norm() / (1<<search_level_);

  // Warp reference patch
  warp::warpAffine(A_cur_ref_, ref_frame.img_pyr_[ref_ftr.level], ref_ftr.px,
                   ref_ftr.level, search_level_, halfpatch_size_+1, patch_with_border_);
  createPatchFromPatchWithBorder();

  // Short epipolar line: direct alignment without search
  if(epi_length_ < 2.0)
  {
    px_cur_ = (px_A+px_B)/2.0;
    Vector2d px_scaled(px_cur_/(1<<search_level_));
    bool res;
    if(options_.align_1d)
      res = feature_alignment::align1D(
          cur_frame.img_pyr_[search_level_], (px_A-px_B).cast<float>().normalized(),
          patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
    else
      res = feature_alignment::align2D(
          cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
          options_.align_max_iter, px_scaled);
    if(res)
    {
      px_cur_ = px_scaled*(1<<search_level_);
      if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px_cur_), depth))
        return true;
    }
    return false;
  }

  // Long epipolar line: exhaustive search
  size_t n_steps = epi_length_/0.7;  // One step per pixel
  Vector2d step = epi_dir_/n_steps;

  // 【保护】限制极线搜索的最大步数，避免出现超大计算量。
  // 当频繁触发时，通常说明 seed 的深度区间 (d_min/d_max) 已经不合理（常见原因：尺度/深度先验发散）。
  // 此时选择跳过本次更新，避免卡死。配合上面的深度 clamp，可显著降低触发频率。
  if(n_steps > options_.max_epi_search_steps)
  {
    printf(
      "WARNING: skip epipolar search: %zu evaluations (limit=%zu), px_lenght=%f, d_min=%f, d_max=%f.\n",
      n_steps, options_.max_epi_search_steps, epi_length_, d_min, d_max);
    return false;
  }

  // Precompute patch statistics for ZMSSD
  int pixel_sum = 0;
  int pixel_sum_square = 0;
  PatchScore patch_score(patch_);

  // Search along epipolar line
  Vector2d uv = B-step;
  Vector2i last_checked_pxi(0,0);
  ++n_steps;
  
  for(size_t i=0; i<n_steps; ++i, uv+=step)
  {
    Vector2d px(cur_frame.cam_->world2cam(uv));
    Vector2i pxi(px[0]/(1<<search_level_)+0.5,
                 px[1]/(1<<search_level_)+0.5);

    // Skip if same pixel as last check
    if(pxi == last_checked_pxi)
      continue;
    last_checked_pxi = pxi;

    // Boundary check
    if(!cur_frame.cam_->isInFrame(pxi, patch_size_, search_level_))
      continue;

    // Compute ZMSSD score
    uint8_t* cur_patch_ptr = cur_frame.img_pyr_[search_level_].data
                             + (pxi[1]-halfpatch_size_)*cur_frame.img_pyr_[search_level_].cols
                             + (pxi[0]-halfpatch_size_);
    int zmssd = patch_score.computeScore(cur_patch_ptr, cur_frame.img_pyr_[search_level_].cols);

    if(zmssd < zmssd_best) {
      zmssd_best = zmssd;
      uv_best = uv;
    }
  }

  // Refine best match with subpixel alignment
  if(zmssd_best < PatchScore::threshold())
  {
    if(options_.subpix_refinement)
    {
      px_cur_ = cur_frame.cam_->world2cam(uv_best);
      Vector2d px_scaled(px_cur_/(1<<search_level_));
      bool res;
      if(options_.align_1d)
        res = feature_alignment::align1D(
            cur_frame.img_pyr_[search_level_], (px_A-px_B).cast<float>().normalized(),
            patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
      else
        res = feature_alignment::align2D(
            cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
            options_.align_max_iter, px_scaled);
      if(res)
      {
        px_cur_ = px_scaled*(1<<search_level_);
        if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px_cur_), depth))
          return true;
      }
      return false;
    }
    
    px_cur_ = cur_frame.cam_->world2cam(uv_best);
    if(depthFromTriangulation(T_cur_ref, ref_ftr.f, vk::unproject2d(uv_best).normalized(), depth))
      return true;
  }
  return false;
}

} // namespace svo
