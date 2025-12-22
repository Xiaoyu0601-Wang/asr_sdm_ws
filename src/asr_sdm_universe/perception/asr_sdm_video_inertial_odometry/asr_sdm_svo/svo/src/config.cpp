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
 * @file config.cpp
 * @brief Configuration singleton containing all SVO algorithm parameters.
 * 
 * This module implements a singleton pattern for storing and accessing all
 * configuration parameters used throughout the SVO pipeline. The singleton
 * ensures consistent configuration across all modules.
 * 
 * In ROS2, these parameters can be overridden through launch files or runtime
 * parameter configuration using the ROS2 parameter system.
 * 
 * Parameter Categories:
 * - Tracing: Debug output file paths
 * - Image Processing: Pyramid levels, grid size
 * - Initialization: Minimum features, disparity, inliers
 * - Feature Tracking: KLT pyramid levels, iterations
 * - Pose Optimization: Thresholds, iterations
 * - Bundle Adjustment: Local BA settings
 * - Keyframe Selection: Distance thresholds, maximum keyframes
 * - Quality Monitoring: Minimum features for good tracking
 */

#include <svo/config.h>

namespace svo {

/**
 * @brief Default constructor initializing all parameters with default values.
 * 
 * These default values are suitable for general use cases. For specific
 * applications (e.g., fast motion, low-texture environments), parameters
 * should be tuned accordingly.
 */
Config::Config() :
    // === Tracing Parameters ===
    trace_name("svo"),              // Base name for trace files
    trace_dir("/tmp"),              // Directory for trace output
    
    // === Image Pyramid Parameters ===
    n_pyr_levels(3),                // Number of pyramid levels for feature detection
    
    // === IMU Integration ===
    use_imu(false),                 // Whether to use IMU data (not implemented)
    
    // === Local Bundle Adjustment ===
    core_n_kfs(3),                  // Number of keyframes in local BA window
    
    // === Map Scale ===
    map_scale(1.0),                 // Initial map scale (depth normalization)
    
    // === Feature Detection Grid ===
    grid_size(25),                  // Grid cell size for uniform feature distribution (pixels)
    
    // === Initialization Parameters ===
    init_min_disparity(50.0),       // Minimum disparity for initialization (pixels)
    init_min_tracked(50),           // Minimum tracked features for initialization
    init_min_inliers(40),           // Minimum inliers after RANSAC for initialization
    
    // === KLT Tracker Parameters ===
    klt_max_level(4),               // Maximum pyramid level for KLT tracking
    klt_min_level(2),               // Minimum pyramid level for KLT tracking
    
    // === Reprojection and Pose Optimization ===
    reproj_thresh(2.0),             // Reprojection threshold for outlier removal (pixels)
    poseoptim_thresh(2.0),          // Threshold for pose optimization outlier removal (pixels)
    poseoptim_num_iter(10),         // Number of pose optimization iterations
    
    // === Structure Optimization ===
    structureoptim_max_pts(20),     // Maximum points to optimize per frame
    structureoptim_num_iter(5),     // Number of structure optimization iterations
    
    // === Local Bundle Adjustment Parameters ===
    loba_thresh(2.0),               // Reprojection threshold for local BA (pixels)
    loba_robust_huber_width(1.0),   // Huber kernel width for robust estimation
    loba_num_iter(0),               // Local BA iterations (0 = disabled)
    
    // === Keyframe Selection ===
    kfselect_mindist(0.12),         // Minimum distance ratio for new keyframe selection
    
    // === Triangulation Parameters ===
    triang_min_corner_score(20.0),  // Minimum corner score for triangulation
    triang_half_patch_size(4),      // Half patch size for triangulation matching
    
    // === Subpixel Refinement ===
    subpix_n_iter(10),              // Number of subpixel refinement iterations
    
    // === Threading Options ===
    use_threaded_depthfilter(true), // Run depth filter in a background thread by default
    
    // === Map Size Limits ===
    max_n_kfs(0),                   // Maximum keyframes in map (0 = unlimited)
    
    // === Sensor Synchronization ===
    img_imu_delay(0.0),             // Time delay between image and IMU (seconds)
    
    // === Feature Limits ===
    max_fts(120),                   // Maximum features per frame
    
    // === Tracking Quality Monitoring ===
    quality_min_fts(50),            // Minimum features for good tracking quality
    quality_max_drop_fts(40),       // Maximum feature drop indicating tracking failure
    patch_match_thresh_factor(1.0)  // Scale factor for ZMSSD acceptance threshold (>=1.0 more permissive)
{}

/**
 * @brief Returns the singleton instance of the Config class.
 * 
 * Thread-safe in C++11 and later due to static local variable initialization
 * guarantees. The instance is created on first use and destroyed automatically
 * at program termination.
 * 
 * @return Reference to the singleton Config instance
 */
Config& Config::getInstance()
{
  static Config instance; // Instantiated on first use and guaranteed to be destroyed
  return instance;
}

} // namespace svo
