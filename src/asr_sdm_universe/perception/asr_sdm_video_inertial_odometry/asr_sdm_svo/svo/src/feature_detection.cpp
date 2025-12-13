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
 * @file feature_detection.cpp
 * @brief Multi-scale FAST corner detection with spatial bucketing.
 * 
 * This module implements feature detection for visual odometry using FAST
 * corners detected across an image pyramid. Key design choices:
 * 
 * - Grid-based bucketing: Image is divided into cells, with at most one
 *   feature per cell. This ensures uniform spatial distribution.
 * 
 * - Multi-scale detection: FAST corners are detected at multiple pyramid
 *   levels to handle features of different sizes/scales.
 * 
 * - Shi-Tomasi scoring: FAST corners are ranked by Shi-Tomasi score
 *   (minimum eigenvalue of structure tensor) for better corner quality.
 * 
 * - SIMD acceleration: Uses SSE2 or NEON optimized FAST implementations.
 */

#include <svo/feature_detection.h>
#include <svo/feature.h>
#include <fast/fast.h>
#include <vikit/vision.h>

namespace svo {
namespace feature_detection {

/**
 * @brief Constructs the abstract detector base class.
 * 
 * Initializes the grid structure for spatial bucketing. Each grid cell
 * is sized according to cell_size, and the grid covers the full image.
 * 
 * @param img_width Image width in pixels
 * @param img_height Image height in pixels
 * @param cell_size Size of each grid cell in pixels
 * @param n_pyr_levels Number of pyramid levels for detection
 */
AbstractDetector::AbstractDetector(
    const int img_width,
    const int img_height,
    const int cell_size,
    const int n_pyr_levels) :
        cell_size_(cell_size),
        n_pyr_levels_(n_pyr_levels),
        grid_n_cols_(ceil(static_cast<double>(img_width)/cell_size_)),
        grid_n_rows_(ceil(static_cast<double>(img_height)/cell_size_)),
        grid_occupancy_(grid_n_cols_*grid_n_rows_, false)  // All cells initially empty
{}

/**
 * @brief Resets the grid occupancy to all empty.
 * 
 * Called before each detection pass to allow new features in all cells.
 */
void AbstractDetector::resetGrid()
{
  std::fill(grid_occupancy_.begin(), grid_occupancy_.end(), false);
}

/**
 * @brief Marks grid cells as occupied for existing features.
 * 
 * This prevents detecting new features too close to existing ones,
 * maintaining good spatial distribution.
 * 
 * @param fts List of existing features to mark
 */
void AbstractDetector::setExistingFeatures(const Features& fts)
{
  std::for_each(fts.begin(), fts.end(), [&](Feature* i){
    grid_occupancy_.at(
        static_cast<int>(i->px[1]/cell_size_)*grid_n_cols_
        + static_cast<int>(i->px[0]/cell_size_)) = true;
  });
}

/**
 * @brief Marks a single grid cell as occupied.
 * 
 * Used to mark cells where depth filter seeds are being tracked,
 * preventing detection of overlapping features.
 * 
 * @param px Pixel location to mark
 */
void AbstractDetector::setGridOccpuancy(const Vector2d& px)
{
  grid_occupancy_.at(
      static_cast<int>(px[1]/cell_size_)*grid_n_cols_
    + static_cast<int>(px[0]/cell_size_)) = true;
}

/**
 * @brief Constructs a FAST corner detector.
 * 
 * Inherits grid-based bucketing from AbstractDetector.
 * 
 * @param img_width Image width
 * @param img_height Image height
 * @param cell_size Grid cell size for bucketing
 * @param n_pyr_levels Number of pyramid levels
 */
FastDetector::FastDetector(
    const int img_width,
    const int img_height,
    const int cell_size,
    const int n_pyr_levels) :
        AbstractDetector(img_width, img_height, cell_size, n_pyr_levels)
{}

/**
 * @brief Detects FAST corners across the image pyramid with bucketing.
 * 
 * Detection algorithm:
 * 1. For each pyramid level (coarse to fine):
 *    a. Run FAST corner detection (with SIMD if available)
 *    b. Compute Shi-Tomasi score for each corner
 *    c. Apply non-maximum suppression in 3x3 neighborhood
 *    d. For each corner, compute its grid cell and update if it has
 *       higher score than existing corner in that cell
 * 
 * 2. Create Feature objects for corners exceeding detection threshold
 * 
 * @param frame Frame to detect features in
 * @param img_pyr Image pyramid
 * @param detection_threshold Minimum corner score threshold
 * @param fts Output: detected features
 */
void FastDetector::detect(
    Frame* frame,
    const ImgPyr& img_pyr,
    const double detection_threshold,
    Features& fts)
{
  // Initialize corner candidates with threshold as default score
  Corners corners(grid_n_cols_*grid_n_rows_, Corner(0,0,detection_threshold,0,0.0f));
  
  // Process each pyramid level
  for(int L=0; L<n_pyr_levels_; ++L)
  {
    const int scale = (1<<L);  // Scale factor: 1, 2, 4, 8...
    vector<fast::fast_xy> fast_corners;
    
    // FAST corner detection with architecture-specific SIMD
#if __SSE2__
      // SSE2-optimized FAST-10 for x86
      fast::fast_corner_detect_10_sse2(
          (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
          img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#elif HAVE_FAST_NEON
      // NEON-optimized FAST-9 for ARM
      fast::fast_corner_detect_9_neon(
          (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
          img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#else
      // Scalar FAST-10 fallback
      fast::fast_corner_detect_10(
          (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
          img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#endif
    
    // Compute FAST scores for non-maximum suppression
    vector<int> scores, nm_corners;
    fast::fast_corner_score_10((fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols, fast_corners, 20, scores);
    fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

    // Process non-maximum suppressed corners
    for(auto it=nm_corners.begin(), ite=nm_corners.end(); it!=ite; ++it)
    {
      fast::fast_xy& xy = fast_corners.at(*it);
      
      // Compute grid cell index (scale coordinates to level 0)
      const int k = static_cast<int>((xy.y*scale)/cell_size_)*grid_n_cols_
                  + static_cast<int>((xy.x*scale)/cell_size_);
      
      // Skip if cell is already occupied by existing feature
      if(grid_occupancy_[k])
        continue;
      
      // Compute Shi-Tomasi score (more reliable than FAST score)
      // This is the minimum eigenvalue of the structure tensor
      const float score = vk::shiTomasiScore(img_pyr[L], xy.x, xy.y);
      
      // Update if this corner has higher score than current best in cell
      if(score > corners.at(k).score)
        corners.at(k) = Corner(xy.x*scale, xy.y*scale, score, L, 0.0f);
    }
  }

  // Create Feature objects for corners exceeding threshold
  std::for_each(corners.begin(), corners.end(), [&](Corner& c) {
    if(c.score > detection_threshold)
      fts.push_back(new Feature(frame, Vector2d(c.x, c.y), c.level));
  });

  // Reset grid for next detection
  resetGrid();
}

} // namespace feature_detection
} // namespace svo
