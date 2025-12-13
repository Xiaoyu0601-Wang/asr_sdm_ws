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
 * @file initialization.cpp
 * @brief Two-view initialization using KLT tracking and homography decomposition.
 * 
 * This module handles the critical initialization phase of monocular visual
 * odometry. The process involves:
 * 
 * 1. First frame: Detect features using FAST corner detector
 * 2. Subsequent frames: Track features using KLT optical flow
 * 3. When sufficient disparity is achieved:
 *    a. Estimate homography using RANSAC
 *    b. Decompose homography to recover relative pose
 *    c. Triangulate 3D points
 *    d. Scale the map to desired mean depth
 * 
 * The homography model is used because:
 * - Works well for planar scenes (common in initialization)
 * - Degrades gracefully for non-planar scenes
 * - More robust than essential matrix for small baselines
 */

#include <svo/config.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/initialization.h>
#include <svo/feature_detection.h>
#include <vikit/math_utils.h>
#include <vikit/homography.h>

namespace svo {
namespace initialization {

/**
 * @brief Processes the first frame for initialization.
 * 
 * Detects features in the first frame and stores them for tracking.
 * Requires at least 100 features for robust initialization.
 * 
 * @param frame_ref First frame (becomes reference keyframe)
 * @return SUCCESS if enough features, FAILURE otherwise
 */
InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)
{
  reset();
  
  // Detect features in first frame
  detectFeatures(frame_ref, px_ref_, f_ref_);
  
  if(px_ref_.size() < 100)
  {
    SVO_WARN_STREAM_THROTTLE(2.0, "First image has less than 100 features. Retry in more textured environment.");
    return FAILURE;
  }
  
  frame_ref_ = frame_ref;
  // Initialize current positions with reference positions for KLT
  px_cur_.insert(px_cur_.begin(), px_ref_.begin(), px_ref_.end());
  return SUCCESS;
}

/**
 * @brief Processes subsequent frames for initialization.
 * 
 * Tracks features from the reference frame using KLT and checks:
 * 1. Minimum number of tracked features
 * 2. Minimum average disparity (baseline)
 * 3. Minimum inliers after RANSAC homography estimation
 * 
 * On success, triangulates initial map points and computes relative pose.
 * 
 * @param frame_cur Current frame candidate
 * @return SUCCESS if initialization complete, NO_KEYFRAME if not enough disparity,
 *         FAILURE if tracking failed
 */
InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)
{
  // Track features using KLT optical flow
  trackKlt(frame_ref_, frame_cur, px_ref_, px_cur_, f_ref_, f_cur_, disparities_);
  SVO_INFO_STREAM("Init: KLT tracked "<< disparities_.size() <<" features");

  // Check minimum tracked features
  if(disparities_.size() < Config::initMinTracked())
    return FAILURE;

  // Check minimum disparity (need baseline for triangulation)
  double disparity = vk::getMedian(disparities_);
  SVO_INFO_STREAM("Init: KLT "<<disparity<<"px average disparity.");
  if(disparity < Config::initMinDisparity())
    return NO_KEYFRAME;  // Keep waiting for more motion

  // Estimate homography and recover pose using RANSAC
  computeHomography(
      f_ref_, f_cur_,
      frame_ref_->cam_->errorMultiplier2(), Config::poseOptimThresh(),
      inliers_, xyz_in_cur_, T_cur_from_ref_);
  SVO_INFO_STREAM("Init: Homography RANSAC "<<inliers_.size()<<" inliers.");

  // Check minimum inliers
  if(inliers_.size() < Config::initMinInliers())
  {
    SVO_WARN_STREAM("Init WARNING: "<<Config::initMinInliers()<<" inliers minimum required.");
    return FAILURE;
  }

  // === Scale Normalization ===
  // Rescale map so mean scene depth equals the configured scale
  vector<double> depth_vec;
  for(size_t i=0; i<xyz_in_cur_.size(); ++i)
    depth_vec.push_back((xyz_in_cur_[i]).z());
  double scene_depth_median = vk::getMedian(depth_vec);
  double scale = Config::mapScale()/scene_depth_median;
  
  // Set current frame pose (scaled)
  frame_cur->T_f_w_ = T_cur_from_ref_ * frame_ref_->T_f_w_;
  frame_cur->T_f_w_.translation() =
      -frame_cur->T_f_w_.rotationMatrix()*(frame_ref_->pos() + scale*(frame_cur->pos() - frame_ref_->pos()));

  // === Triangulation ===
  // Create 3D points and features for each inlier
  SE3 T_world_cur = frame_cur->T_f_w_.inverse();
  for(vector<int>::iterator it=inliers_.begin(); it!=inliers_.end(); ++it)
  {
    Vector2d px_cur(px_cur_[*it].x, px_cur_[*it].y);
    Vector2d px_ref(px_ref_[*it].x, px_ref_[*it].y);
    
    // Verify point is in valid image region and has positive depth
    if(frame_ref_->cam_->isInFrame(px_cur.cast<int>(), 10) && frame_ref_->cam_->isInFrame(px_ref.cast<int>(), 10) && xyz_in_cur_[*it].z() > 0)
    {
      // Transform triangulated point to world frame
      Vector3d pos = T_world_cur * (xyz_in_cur_[*it]*scale);
      Point* new_point = new Point(pos);

      // Create feature in current frame
      Feature* ftr_cur(new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0));
      frame_cur->addFeature(ftr_cur);
      new_point->addFrameRef(ftr_cur);

      // Create feature in reference frame
      Feature* ftr_ref(new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it], 0));
      frame_ref_->addFeature(ftr_ref);
      new_point->addFrameRef(ftr_ref);
    }
  }
  return SUCCESS;
}

/**
 * @brief Resets initialization state.
 */
void KltHomographyInit::reset()
{
  px_cur_.clear();
  frame_ref_.reset();
}

/**
 * @brief Detects features in a frame using FAST corner detector.
 * 
 * @param frame Frame to detect features in
 * @param px_vec Output: 2D pixel positions
 * @param f_vec Output: 3D bearing vectors
 */
void detectFeatures(
    FramePtr frame,
    vector<cv::Point2f>& px_vec,
    vector<Vector3d>& f_vec)
{
  Features new_features;
  feature_detection::FastDetector detector(
      frame->img().cols, frame->img().rows, Config::gridSize(), Config::nPyrLevels());
  detector.detect(frame.get(), frame->img_pyr_, Config::triangMinCornerScore(), new_features);

  // Convert to OpenCV format and compute bearing vectors
  px_vec.clear(); px_vec.reserve(new_features.size());
  f_vec.clear(); f_vec.reserve(new_features.size());
  std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
    px_vec.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
    f_vec.push_back(ftr->f);
    delete ftr;  // Features not needed - just extracting positions
  });
}

/**
 * @brief Tracks features using KLT optical flow.
 * 
 * Uses OpenCV's pyramidal Lucas-Kanade tracker with:
 * - 30x30 window size
 * - 4 pyramid levels
 * - 30 iterations max
 * 
 * Failed tracks are removed from all vectors.
 * 
 * @param frame_ref Reference frame
 * @param frame_cur Current frame
 * @param px_ref Reference pixel positions (updated to remove failures)
 * @param px_cur Current pixel positions (updated with tracked positions)
 * @param f_ref Reference bearing vectors (updated to remove failures)
 * @param f_cur Output: current bearing vectors
 * @param disparities Output: disparity (motion) for each tracked point
 */
void trackKlt(
    FramePtr frame_ref,
    FramePtr frame_cur,
    vector<cv::Point2f>& px_ref,
    vector<cv::Point2f>& px_cur,
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<double>& disparities)
{
  // KLT parameters
  const double klt_win_size = 30.0;
  const int klt_max_iter = 30;
  const double klt_eps = 0.001;
  
  vector<uchar> status;
  vector<float> error;
  vector<float> min_eig_vec;
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
  
  // Run KLT tracker
  cv::calcOpticalFlowPyrLK(frame_ref->img_pyr_[0], frame_cur->img_pyr_[0],
                           px_ref, px_cur,
                           status, error,
                           cv::Size2i(klt_win_size, klt_win_size),
                           4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);

  // Remove failed tracks
  vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
  vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
  vector<Vector3d>::iterator f_ref_it = f_ref.begin();
  f_cur.clear(); f_cur.reserve(px_cur.size());
  disparities.clear(); disparities.reserve(px_cur.size());
  
  for(size_t i=0; px_ref_it != px_ref.end(); ++i)
  {
    if(!status[i])
    {
      // Tracking failed - remove from all vectors
      px_ref_it = px_ref.erase(px_ref_it);
      px_cur_it = px_cur.erase(px_cur_it);
      f_ref_it = f_ref.erase(f_ref_it);
      continue;
    }
    
    // Compute bearing vector and disparity
    f_cur.push_back(frame_cur->c2f(px_cur_it->x, px_cur_it->y));
    disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
    ++px_ref_it;
    ++px_cur_it;
    ++f_ref_it;
  }
}

/**
 * @brief Estimates homography and recovers relative pose.
 * 
 * Uses RANSAC to robustly estimate the homography matrix between
 * reference and current views, then decomposes it to extract
 * relative rotation, translation (up to scale), and triangulated points.
 * 
 * @param f_ref Reference bearing vectors
 * @param f_cur Current bearing vectors
 * @param focal_length Camera focal length for error scaling
 * @param reprojection_threshold RANSAC inlier threshold
 * @param inliers Output: indices of inlier correspondences
 * @param xyz_in_cur Output: triangulated 3D points in current frame
 * @param T_cur_from_ref Output: relative pose transformation
 */
void computeHomography(
    const vector<Vector3d>& f_ref,
    const vector<Vector3d>& f_cur,
    double focal_length,
    double reprojection_threshold,
    vector<int>& inliers,
    vector<Vector3d>& xyz_in_cur,
    SE3& T_cur_from_ref)
{
  // Project bearing vectors to normalized image plane
  vector<Vector2d > uv_ref(f_ref.size());
  vector<Vector2d > uv_cur(f_cur.size());
  for(size_t i=0, i_max=f_ref.size(); i<i_max; ++i)
  {
    uv_ref[i] = vk::project2d(f_ref[i]);
    uv_cur[i] = vk::project2d(f_cur[i]);
  }
  
  // Estimate homography with RANSAC
  vk::Homography Homography(uv_ref, uv_cur, focal_length, reprojection_threshold);
  Homography.computeSE3fromMatches();
  
  // Compute inliers and triangulate 3D points
  vector<int> outliers;
  vk::computeInliers(f_cur, f_ref,
                     Homography.T_c2_from_c1.rotationMatrix(), Homography.T_c2_from_c1.translation(),
                     reprojection_threshold, focal_length,
                     xyz_in_cur, inliers, outliers);
  
  T_cur_from_ref = Homography.T_c2_from_c1;
}


} // namespace initialization
} // namespace svo
