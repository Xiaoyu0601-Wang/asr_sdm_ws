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
 * @file frame_handler_base.cpp
 * @brief Base class for visual odometry frame processing.
 * 
 * This module provides the common infrastructure for frame-by-frame visual
 * odometry processing, including:
 * 
 * - State machine: Manages pipeline stages (paused, first frame, second frame,
 *   default tracking, relocalization)
 * 
 * - Performance monitoring: Tracks frame processing time and observation counts
 * 
 * - Quality monitoring: Assesses tracking quality based on feature counts
 * 
 * - Structure optimization: Refines 3D point positions using Gauss-Newton
 * 
 * The base class is extended by FrameHandlerMono for monocular operation.
 */

#include <vikit/abstract_camera.h>
#include <stdlib.h>
#include <Eigen/StdVector>
#include <boost/bind.hpp>
#include <fstream>
#include <svo/frame_handler_base.h>
#include <svo/config.h>
#include <svo/feature.h>
#include <svo/matcher.h>
#include <svo/map.h>
#include <svo/point.h>

namespace svo
{

// Global performance monitor for tracing (when SVO_TRACE is defined)
#ifdef SVO_TRACE
vk::PerformanceMonitor* g_permon = NULL;
#endif

/**
 * @brief Default constructor initializing state machine and performance tracking.
 * 
 * Sets up:
 * - Initial state: PAUSED (waiting for start command)
 * - Frame timing accumulator (10 samples for FPS averaging)
 * - Observation count accumulator
 * - Tracking quality: INSUFFICIENT (until tracking starts)
 * 
 * When SVO_TRACE is defined, also initializes the performance monitor
 * with various timers and logging channels.
 */
FrameHandlerBase::FrameHandlerBase() :
  stage_(STAGE_PAUSED),              // Start in paused state
  set_reset_(false),                 // No pending reset
  set_start_(false),                 // No pending start
  acc_frame_timings_(10),            // Rolling average of last 10 frame times
  acc_num_obs_(10),                  // Rolling average of last 10 observation counts
  num_obs_last_(0),                  // Last frame's observation count
  tracking_quality_(TRACKING_INSUFFICIENT)
{
#ifdef SVO_TRACE
  // Initialize Performance Monitor for detailed timing analysis
  g_permon = new vk::PerformanceMonitor();
  
  // Timers for each pipeline stage
  g_permon->addTimer("pyramid_creation");      // Image pyramid construction
  g_permon->addTimer("sparse_img_align");      // Direct image alignment
  g_permon->addTimer("reproject");             // Map point reprojection
  g_permon->addTimer("reproject_kfs");         // Keyframe-based reprojection
  g_permon->addTimer("reproject_candidates");  // Candidate point reprojection
  g_permon->addTimer("feature_align");         // Subpixel feature alignment
  g_permon->addTimer("pose_optimizer");        // Pose refinement
  g_permon->addTimer("point_optimizer");       // Structure refinement
  g_permon->addTimer("local_ba");              // Local bundle adjustment
  g_permon->addTimer("tot_time");              // Total frame processing time
  
  // Logging channels for statistics
  g_permon->addLog("timestamp");
  g_permon->addLog("img_align_n_tracked");     // Features tracked in alignment
  g_permon->addLog("repr_n_mps");              // Map points reprojected
  g_permon->addLog("repr_n_new_references");   // New references found
  g_permon->addLog("sfba_thresh");             // Pose optimizer threshold
  g_permon->addLog("sfba_error_init");         // Initial pose error
  g_permon->addLog("sfba_error_final");        // Final pose error
  g_permon->addLog("sfba_n_edges_final");      // Final edge count
  g_permon->addLog("loba_n_erredges_init");    // Local BA initial outliers
  g_permon->addLog("loba_n_erredges_fin");     // Local BA final outliers
  g_permon->addLog("loba_err_init");           // Local BA initial error
  g_permon->addLog("loba_err_fin");            // Local BA final error
  g_permon->addLog("n_candidates");            // Candidate point count
  g_permon->addLog("dropout");                 // Tracking failures
  g_permon->init(Config::traceName(), Config::traceDir());
#endif

  SVO_INFO_STREAM("SVO initialized");
}

/**
 * @brief Destructor - cleans up performance monitor if used.
 */
FrameHandlerBase::~FrameHandlerBase()
{
  SVO_INFO_STREAM("SVO destructor invoked");
#ifdef SVO_TRACE
  delete g_permon;
#endif
}

/**
 * @brief Common processing at the start of each frame.
 * 
 * Handles:
 * - Start command: Resets system and transitions to FIRST_FRAME stage
 * - Paused state: Returns false to skip processing
 * - Normal operation: Starts timing, cleans up map trash
 * 
 * @param timestamp Frame timestamp
 * @return true if processing should continue, false if paused
 */
bool FrameHandlerBase::startFrameProcessingCommon(const double timestamp)
{
  // Handle pending start command
  if(set_start_)
  {
    resetAll();  // Virtual function - resets derived class state
    stage_ = STAGE_FIRST_FRAME;
  }

  // Skip processing if paused
  if(stage_ == STAGE_PAUSED)
    return false;

  // Begin frame processing
  SVO_LOG(timestamp);
  SVO_DEBUG_STREAM("New Frame");
  SVO_START_TIMER("tot_time");
  timer_.start();

  // Clean up deleted points from previous iteration
  // (can't do earlier due to visualization requirements)
  map_.emptyTrash();
  return true;
}

/**
 * @brief Common processing at the end of each frame.
 * 
 * Handles:
 * - Performance logging and FPS calculation
 * - Tracking failure detection and recovery
 * - Reset handling
 * 
 * @param update_id Frame ID for logging
 * @param dropout Result of frame processing (success/failure/keyframe)
 * @param num_observations Number of features successfully tracked
 * @return 0 (reserved for future use)
 */
int FrameHandlerBase::finishFrameProcessingCommon(
    const size_t update_id,
    const UpdateResult dropout,
    const size_t num_observations)
{
  SVO_DEBUG_STREAM("Frame: "<<update_id<<"\t fps-avg = "<< 1.0/acc_frame_timings_.getMean()<<"\t nObs = "<<acc_num_obs_.getMean());
  SVO_LOG(dropout);

  // Update timing statistics
  acc_frame_timings_.push_back(timer_.stop());
  if(stage_ == STAGE_DEFAULT_FRAME)
    acc_num_obs_.push_back(num_observations);
  num_obs_last_ = num_observations;
  SVO_STOP_TIMER("tot_time");

#ifdef SVO_TRACE
  // Write trace data to file
  g_permon->writeToFile();
  {
    boost::unique_lock<boost::mutex> lock(map_.point_candidates_.mut_);
    size_t n_candidates = map_.point_candidates_.candidates_.size();
    SVO_LOG(n_candidates);
  }
#endif

  // Handle tracking failure
  if(dropout == RESULT_FAILURE &&
      (stage_ == STAGE_DEFAULT_FRAME || stage_ == STAGE_RELOCALIZING ))
  {
    // Attempt relocalization
    stage_ = STAGE_RELOCALIZING;
    tracking_quality_ = TRACKING_INSUFFICIENT;
  }
  else if (dropout == RESULT_FAILURE)
    resetAll();  // Complete failure - reset everything
  
  // Handle pending reset command
  if(set_reset_)
    resetAll();

  return 0;
}

/**
 * @brief Common reset functionality.
 * 
 * Resets map, state machine, and tracking quality.
 * Called by derived class resetAll() implementations.
 */
void FrameHandlerBase::resetCommon()
{
  map_.reset();
  stage_ = STAGE_PAUSED;
  set_reset_ = false;
  set_start_ = false;
  tracking_quality_ = TRACKING_INSUFFICIENT;
  num_obs_last_ = 0;
  SVO_INFO_STREAM("RESET");
}

/**
 * @brief Assesses tracking quality based on observation count.
 * 
 * Sets tracking_quality_ to:
 * - GOOD: Sufficient features and no large drop
 * - INSUFFICIENT: Too few features or large sudden drop
 * 
 * @param num_observations Number of successfully tracked features
 */
void FrameHandlerBase::setTrackingQuality(const size_t num_observations)
{
  tracking_quality_ = TRACKING_GOOD;
  
  // Check minimum feature count
  if(num_observations < Config::qualityMinFts())
  {
    SVO_WARN_STREAM_THROTTLE(0.5, "Tracking less than "<< Config::qualityMinFts() <<" features!");
    tracking_quality_ = TRACKING_INSUFFICIENT;
  }
  
  // Check for sudden feature drop (indicates tracking failure)
  const int feature_drop = static_cast<int>(std::min(num_obs_last_, Config::maxFts())) - num_observations;
  if(feature_drop > Config::qualityMaxFtsDrop())
  {
    SVO_WARN_STREAM("Lost "<< feature_drop <<" features!");
    tracking_quality_ = TRACKING_INSUFFICIENT;
  }
}

/**
 * @brief Comparator for sorting points by last optimization time.
 * 
 * Used to prioritize points that haven't been optimized recently.
 */
bool ptLastOptimComparator(Point* lhs, Point* rhs)
{
  return (lhs->last_structure_optim_ < rhs->last_structure_optim_);
}

/**
 * @brief Optimizes 3D point structure observed from a frame.
 * 
 * Selects up to max_n_pts points that haven't been optimized recently
 * and refines their positions using Gauss-Newton optimization.
 * 
 * @param frame Frame whose observed points to optimize
 * @param max_n_pts Maximum number of points to optimize
 * @param max_iter Maximum iterations per point optimization
 */
void FrameHandlerBase::optimizeStructure(
    FramePtr frame,
    size_t max_n_pts,
    int max_iter)
{
  // Collect all points observed in this frame
  deque<Point*> pts;
  for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point != NULL)
      pts.push_back((*it)->point);
  }
  
  // Select points that haven't been optimized recently
  // nth_element partitions: oldest optimized points come first
  max_n_pts = min(max_n_pts, pts.size());
  nth_element(pts.begin(), pts.begin() + max_n_pts, pts.end(), ptLastOptimComparator);
  
  // Optimize selected points
  for(deque<Point*>::iterator it=pts.begin(); it!=pts.begin()+max_n_pts; ++it)
  {
    (*it)->optimize(max_iter);
    (*it)->last_structure_optim_ = frame->id_;  // Mark as recently optimized
  }
}


} // namespace svo
