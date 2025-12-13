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
 * @file frame.cpp
 * @brief Camera frame representation with image pyramid and features.
 * 
 * A Frame represents a single camera observation including:
 * - Grayscale image pyramid for multi-scale processing
 * - Camera intrinsics and extrinsics (pose T_f_w_)
 * - Detected features and their associated 3D points
 * - Key points for fast visibility checking
 * 
 * Key points are strategically selected features (center, corners) that
 * provide quick field-of-view overlap tests between keyframes.
 */

#include <stdexcept>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/config.h>
#include <boost/bind.hpp>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/performance_monitor.h>
#include <fast/fast.h>

namespace svo {

/// Static frame counter for unique IDs
int Frame::frame_counter_ = 0;

/**
 * @brief Constructs a frame from camera model and image.
 * 
 * @param cam Camera model for projection/unprojection
 * @param img Grayscale image (CV_8UC1)
 * @param timestamp Frame timestamp
 */
Frame::Frame(vk::AbstractCamera* cam, const cv::Mat& img, double timestamp) :
    id_(frame_counter_++),           // Unique frame ID
    timestamp_(timestamp),
    cam_(cam),
    key_pts_(5),                     // 5 key points: center + 4 quadrants
    is_keyframe_(false),
    v_kf_(NULL)                      // g2o vertex (used during BA)
{
  initFrame(img);
}

/**
 * @brief Destructor - cleans up all features.
 */
Frame::~Frame()
{
  std::for_each(fts_.begin(), fts_.end(), [&](Feature* i){delete i;});
}

/**
 * @brief Initializes frame with image pyramid.
 * 
 * Validates image format and builds multi-scale image pyramid
 * for feature detection and tracking.
 * 
 * @param img Grayscale image
 * @throws std::runtime_error if image format is invalid
 */
void Frame::initFrame(const cv::Mat& img)
{
  // Validate image format
  if(img.empty() || img.type() != CV_8UC1 || img.cols != cam_->width() || img.rows != cam_->height())
    throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");

  // Initialize key points to NULL
  std::for_each(key_pts_.begin(), key_pts_.end(), [&](Feature* ftr){ ftr=NULL; });

  // Build image pyramid for multi-scale processing
  frame_utils::createImgPyramid(img, max(Config::nPyrLevels(), Config::kltMaxLevel()+1), img_pyr_);
}

/**
 * @brief Marks this frame as a keyframe.
 * 
 * Keyframes are retained in the map and used for tracking.
 * Also triggers selection of key points for visibility testing.
 */
void Frame::setKeyframe()
{
  is_keyframe_ = true;
  setKeyPoints();
}

/**
 * @brief Adds a feature to this frame's feature list.
 * 
 * @param ftr Feature to add (frame takes ownership)
 */
void Frame::addFeature(Feature* ftr)
{
  fts_.push_back(ftr);
}

/**
 * @brief Selects key points for fast field-of-view testing.
 * 
 * Key points are strategically located features that enable quick
 * visibility checks between frames:
 * - key_pts_[0]: Feature closest to image center
 * - key_pts_[1-4]: Features in each image quadrant
 * 
 * These are updated when the feature set changes.
 */
void Frame::setKeyPoints()
{
  // Clear key points with deleted 3D points
  for(size_t i = 0; i < 5; ++i)
    if(key_pts_[i] != NULL)
      if(key_pts_[i]->point == NULL)
        key_pts_[i] = NULL;

  // Select best features for each key point position
  std::for_each(fts_.begin(), fts_.end(), [&](Feature* ftr){ if(ftr->point != NULL) checkKeyPoints(ftr); });
}

/**
 * @brief Checks if a feature should replace a current key point.
 * 
 * Key point selection criteria:
 * - key_pts_[0]: Closest to image center
 * - key_pts_[1-4]: Furthest from center in each quadrant
 * 
 * @param ftr Feature to check
 */
void Frame::checkKeyPoints(Feature* ftr)
{
  const int cu = cam_->width()/2;   // Image center u
  const int cv = cam_->height()/2;  // Image center v

  // Key point 0: Feature closest to image center
  if(key_pts_[0] == NULL)
    key_pts_[0] = ftr;
  else if(std::max(std::fabs(ftr->px[0]-cu), std::fabs(ftr->px[1]-cv))
        < std::max(std::fabs(key_pts_[0]->px[0]-cu), std::fabs(key_pts_[0]->px[1]-cv)))
    key_pts_[0] = ftr;

  // Key point 1: Bottom-right quadrant (u >= cu, v >= cv)
  if(ftr->px[0] >= cu && ftr->px[1] >= cv)
  {
    if(key_pts_[1] == NULL)
      key_pts_[1] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[1]->px[0]-cu) * (key_pts_[1]->px[1]-cv))
      key_pts_[1] = ftr;
  }
  
  // Key point 2: Top-right quadrant (u >= cu, v < cv)
  if(ftr->px[0] >= cu && ftr->px[1] < cv)
  {
    if(key_pts_[2] == NULL)
      key_pts_[2] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[2]->px[0]-cu) * (key_pts_[2]->px[1]-cv))
      key_pts_[2] = ftr;
  }
  
  // Key point 3: Top-left quadrant (u < cu, v < cv)
  if(ftr->px[0] < cv && ftr->px[1] < cv)
  {
    if(key_pts_[3] == NULL)
      key_pts_[3] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[3]->px[0]-cu) * (key_pts_[3]->px[1]-cv))
      key_pts_[3] = ftr;
  }
  
  // Key point 4: Bottom-left quadrant (u < cu, v >= cv)
  if(ftr->px[0] < cv && ftr->px[1] >= cv)
  {
    if(key_pts_[4] == NULL)
      key_pts_[4] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[4]->px[0]-cu) * (key_pts_[4]->px[1]-cv))
      key_pts_[4] = ftr;
  }
}

/**
 * @brief Removes a feature from the key points list.
 * 
 * Called when a key point's 3D point is deleted. Triggers
 * re-selection of key points from remaining features.
 * 
 * @param ftr Feature being removed
 */
void Frame::removeKeyPoint(Feature* ftr)
{
  bool found = false;
  std::for_each(key_pts_.begin(), key_pts_.end(), [&](Feature*& i){
    if(i == ftr) {
      i = NULL;
      found = true;
    }
  });
  if(found)
    setKeyPoints();  // Re-select key points
}

/**
 * @brief Checks if a 3D point is visible in this frame.
 * 
 * A point is visible if:
 * 1. It is in front of the camera (positive depth)
 * 2. It projects within the image bounds
 * 
 * @param xyz_w 3D point in world coordinates
 * @return true if point is visible
 */
bool Frame::isVisible(const Vector3d& xyz_w) const
{
  Vector3d xyz_f = T_f_w_*xyz_w;  // Transform to frame coordinates
  if(xyz_f.z() < 0.0)
    return false;  // Behind camera
  Vector2d px = f2c(xyz_f);  // Project to image
  if(px[0] >= 0.0 && px[1] >= 0.0 && px[0] < cam_->width() && px[1] < cam_->height())
    return true;
  return false;
}


/// Utility functions for the Frame class
namespace frame_utils {

/**
 * @brief Creates an image pyramid by successive half-sampling.
 * 
 * @param img_level_0 Full resolution image
 * @param n_levels Number of pyramid levels to create
 * @param pyr Output image pyramid
 */
void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr)
{
  pyr.resize(n_levels);
  pyr[0] = img_level_0;
  for(int i=1; i<n_levels; ++i)
  {
    pyr[i] = cv::Mat(pyr[i-1].rows/2, pyr[i-1].cols/2, CV_8U);
    vk::halfSample(pyr[i-1], pyr[i]);  // 2x2 block averaging
  }
}

/**
 * @brief Computes scene depth statistics from frame features.
 * 
 * Used for depth filter initialization and keyframe selection.
 * 
 * @param frame Frame with features
 * @param depth_mean Output: median depth
 * @param depth_min Output: minimum depth
 * @return true if depths could be computed
 */
bool getSceneDepth(const Frame& frame, double& depth_mean, double& depth_min)
{
  vector<double> depth_vec;
  depth_vec.reserve(frame.fts_.size());
  depth_min = std::numeric_limits<double>::max();
  
  for(auto it=frame.fts_.begin(), ite=frame.fts_.end(); it!=ite; ++it)
  {
    if((*it)->point != NULL)
    {
      // Compute depth: z-coordinate in frame coordinates
      const double z = frame.w2f((*it)->point->pos_).z();
      depth_vec.push_back(z);
      depth_min = fmin(z, depth_min);
    }
  }
  
  if(depth_vec.empty())
  {
    SVO_WARN_STREAM("Cannot set scene depth. Frame has no point-observations!");
    return false;
  }
  
  // Use median for robustness to outliers
  depth_mean = vk::getMedian(depth_vec);
  return true;
}

} // namespace frame_utils
} // namespace svo
