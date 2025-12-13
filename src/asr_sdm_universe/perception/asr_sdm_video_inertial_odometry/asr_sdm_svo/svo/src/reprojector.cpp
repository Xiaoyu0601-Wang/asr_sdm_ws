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
 * @file reprojector.cpp
 * @brief Map point reprojection and feature alignment.
 * 
 * The Reprojector is responsible for finding correspondences between
 * the current frame and the map. It works by:
 * 
 * 1. Finding keyframes with overlapping field of view
 * 2. Reprojecting their observed 3D points into the current frame
 * 3. Aligning features at reprojected locations
 * 
 * Grid-based selection ensures uniform spatial distribution:
 * - Image is divided into cells
 * - Each cell stores candidate points
 * - One point per cell is selected for alignment
 * - Points are prioritized by quality (good > unknown > candidate)
 * 
 * This approach provides:
 * - Efficient O(n) point selection instead of O(n²) matching
 * - Uniform feature distribution for pose estimation
 * - Quality-based prioritization for robustness
 */

#include <algorithm>
#include <stdexcept>
#include <svo/reprojector.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/map.h>
#include <svo/config.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <vikit/abstract_camera.h>
#include <vikit/math_utils.h>
#include <vikit/timer.h>

namespace svo {

/**
 * @brief Constructs reprojector with camera model and map reference.
 * 
 * @param cam Camera model for projection
 * @param map Reference to the map
 */
Reprojector::Reprojector(vk::AbstractCamera* cam, Map& map) :
    map_(map)
{
  initializeGrid(cam);
}

/**
 * @brief Destructor - cleans up grid cells.
 */
Reprojector::~Reprojector()
{
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ delete c; });
}

/**
 * @brief Initializes the grid structure for spatial bucketing.
 * 
 * Creates a grid of cells covering the image. Cell order is randomized
 * to prevent systematic bias in feature selection.
 * 
 * @param cam Camera model for image dimensions
 */
void Reprojector::initializeGrid(vk::AbstractCamera* cam)
{
  grid_.cell_size = Config::gridSize();
  grid_.grid_n_cols = ceil(static_cast<double>(cam->width())/grid_.cell_size);
  grid_.grid_n_rows = ceil(static_cast<double>(cam->height())/grid_.cell_size);
  grid_.cells.resize(grid_.grid_n_cols*grid_.grid_n_rows);
  
  // Allocate cells
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell*& c){ c = new Cell; });
  
  // Randomize cell processing order to avoid bias
  grid_.cell_order.resize(grid_.cells.size());
  for(size_t i=0; i<grid_.cells.size(); ++i)
    grid_.cell_order[i] = i;
  random_shuffle(grid_.cell_order.begin(), grid_.cell_order.end());
}

/**
 * @brief Resets grid for new frame processing.
 * 
 * Clears all cells and resets match counters.
 */
void Reprojector::resetGrid()
{
  n_matches_ = 0;
  n_trials_ = 0;
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ c->clear(); });
}

/**
 * @brief Main function: reprojects map points and aligns features.
 * 
 * Algorithm:
 * 1. Find keyframes with overlapping field of view
 * 2. Reproject points from overlapping keyframes into grid cells
 * 3. Reproject candidate points from depth filter
 * 4. For each cell, select best point and align feature
 * 
 * @param frame Current frame to find matches in
 * @param overlap_kfs Output: keyframes with overlap and their match counts
 */
void Reprojector::reprojectMap(
    FramePtr frame,
    std::vector< std::pair<FramePtr,std::size_t> >& overlap_kfs)
{
  resetGrid();

  // === Step 1: Find Overlapping Keyframes ===
  SVO_START_TIMER("reproject_kfs");
  list< pair<FramePtr,double> > close_kfs;
  map_.getCloseKeyframes(frame, close_kfs);

  // Sort by distance (closest first)
  close_kfs.sort(boost::bind(&std::pair<FramePtr, double>::second, _1) <
                 boost::bind(&std::pair<FramePtr, double>::second, _2));

  // === Step 2: Reproject Points from Overlapping Keyframes ===
  // Limit to max_n_kfs to bound computation
  size_t n = 0;
  overlap_kfs.reserve(options_.max_n_kfs);
  
  for(auto it_frame=close_kfs.begin(), ite_frame=close_kfs.end();
      it_frame!=ite_frame && n<options_.max_n_kfs; ++it_frame, ++n)
  {
    FramePtr ref_frame = it_frame->first;
    overlap_kfs.push_back(pair<FramePtr,size_t>(ref_frame,0));

    // Reproject each map point observed by this keyframe
    for(auto it_ftr=ref_frame->fts_.begin(), ite_ftr=ref_frame->fts_.end();
        it_ftr!=ite_ftr; ++it_ftr)
    {
      // Skip features without associated 3D point
      if((*it_ftr)->point == NULL)
        continue;

      // Avoid projecting same point multiple times
      if((*it_ftr)->point->last_projected_kf_id_ == frame->id_)
        continue;
      (*it_ftr)->point->last_projected_kf_id_ = frame->id_;
      
      if(reprojectPoint(frame, (*it_ftr)->point))
        overlap_kfs.back().second++;  // Count successful projections
    }
  }
  SVO_STOP_TIMER("reproject_kfs");

  // === Step 3: Reproject Candidate Points ===
  // Candidates are from depth filter, not yet validated
  SVO_START_TIMER("reproject_candidates");
  {
    boost::unique_lock<boost::mutex> lock(map_.point_candidates_.mut_);
    auto it=map_.point_candidates_.candidates_.begin();
    while(it!=map_.point_candidates_.candidates_.end())
    {
      if(!reprojectPoint(frame, it->first))
      {
        // Track failed reprojections for candidate quality
        it->first->n_failed_reproj_ += 3;
        if(it->first->n_failed_reproj_ > 30)
        {
          // Too many failures - delete candidate
          map_.point_candidates_.deleteCandidate(*it);
          it = map_.point_candidates_.candidates_.erase(it);
          continue;
        }
      }
      ++it;
    }
  }
  SVO_STOP_TIMER("reproject_candidates");

  // === Step 4: Process Grid Cells ===
  // Select one point per cell and align feature
  SVO_START_TIMER("feature_align");
  for(size_t i=0; i<grid_.cells.size(); ++i)
  {
    // Process cells in randomized order
    if(reprojectCell(*grid_.cells.at(grid_.cell_order[i]), frame))
      ++n_matches_;
      
    // Stop if we have enough features
    if(n_matches_ > (size_t) Config::maxFts())
      break;
  }
  SVO_STOP_TIMER("feature_align");
}

/**
 * @brief Comparator for sorting points by quality.
 * 
 * Priority: TYPE_GOOD > TYPE_UNKNOWN > TYPE_CANDIDATE
 */
bool Reprojector::pointQualityComparator(Candidate& lhs, Candidate& rhs)
{
  if(lhs.pt->type_ > rhs.pt->type_)
    return true;
  return false;
}

/**
 * @brief Processes a single grid cell to find a feature match.
 * 
 * Algorithm:
 * 1. Sort candidates by quality (good > unknown > candidate)
 * 2. Try to align each candidate until one succeeds
 * 3. Create feature for successful match
 * 
 * @param cell Grid cell with candidate points
 * @param frame Current frame
 * @return true if a match was found
 */
bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)
{
  // Sort by quality (best first)
  cell.sort(boost::bind(&Reprojector::pointQualityComparator, _1, _2));
  
  Cell::iterator it=cell.begin();
  while(it!=cell.end())
  {
    ++n_trials_;

    // Skip deleted points
    if(it->pt->type_ == Point::TYPE_DELETED)
    {
      it = cell.erase(it);
      continue;
    }

    // Try to align feature at reprojected position
    bool found_match = true;
    if(options_.find_match_direct)
      found_match = matcher_.findMatchDirect(*it->pt, *frame, it->px);
      
    if(!found_match)
    {
      // Alignment failed - update point quality
      it->pt->n_failed_reproj_++;
      
      // Delete low-quality points with too many failures
      if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_failed_reproj_ > 15)
        map_.safeDeletePoint(it->pt);
      if(it->pt->type_ == Point::TYPE_CANDIDATE  && it->pt->n_failed_reproj_ > 30)
        map_.point_candidates_.deleteCandidatePoint(it->pt);
        
      it = cell.erase(it);
      continue;
    }
    
    // Match successful - update point quality
    it->pt->n_succeeded_reproj_++;
    if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_succeeded_reproj_ > 10)
      it->pt->type_ = Point::TYPE_GOOD;  // Promote to good quality

    // Create feature for this match
    Feature* new_feature = new Feature(frame.get(), it->px, matcher_.search_level_);
    frame->addFeature(new_feature);

    // Link feature to 3D point (reverse link only added if frame becomes keyframe)
    new_feature->point = it->pt;

    // Copy edgelet gradient direction if applicable
    if(matcher_.ref_ftr_->type == Feature::EDGELET)
    {
      new_feature->type = Feature::EDGELET;
      new_feature->grad = matcher_.A_cur_ref_*matcher_.ref_ftr_->grad;
      new_feature->grad.normalize();
    }

    // Remove from cell (point successfully matched)
    it = cell.erase(it);

    // One match per cell
    return true;
  }
  return false;
}

/**
 * @brief Projects a 3D point into the frame's grid.
 * 
 * Checks if the point projects within the image and adds it
 * to the appropriate grid cell.
 * 
 * @param frame Current frame
 * @param point 3D point to project
 * @return true if point projects within image
 */
bool Reprojector::reprojectPoint(FramePtr frame, Point* point)
{
  // Project point to image
  Vector2d px(frame->w2c(point->pos_));
  
  // Check if within image (with margin for patch size)
  if(frame->cam_->isInFrame(px.cast<int>(), 8))
  {
    // Compute grid cell index
    const int k = static_cast<int>(px[1]/grid_.cell_size)*grid_.grid_n_cols
                + static_cast<int>(px[0]/grid_.cell_size);
    
    // Add to cell's candidate list
    grid_.cells.at(k)->push_back(Candidate(point, px));
    return true;
  }
  return false;
}

} // namespace svo
