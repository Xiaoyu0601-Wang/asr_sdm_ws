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
 * @file depth_filter.cpp
 * @brief Probabilistic depth estimation using a Bayesian depth filter.
 * 
 * This module implements a probabilistic depth filter based on the paper:
 * "Video-based, Real-Time Multi View Stereo" by G. Vogiatzis and C. Hernández.
 * 
 * The depth filter estimates the depth of image features using multiple
 * observations from different viewpoints. It maintains a probability
 * distribution over inverse depth for each feature (seed), modeled as a
 * mixture of a Gaussian (for inliers) and a uniform distribution (for outliers).
 * 
 * Key components:
 * - Seed: Represents a feature with uncertain depth, stores Beta distribution
 *   parameters (a, b) for inlier/outlier probability, and Gaussian parameters
 *   (mu, sigma) for inverse depth
 * - DepthFilter: Manages seeds, runs epipolar search, and updates depth estimates
 * 
 * The filter runs in a separate thread for real-time performance.
 */

#include <algorithm>
#include <vikit/math_utils.h>
#include <vikit/abstract_camera.h>
#include <vikit/vision.h>
#include <boost/bind.hpp>
#include <boost/math/distributions/normal.hpp>
#include <svo/global.h>
#include <svo/depth_filter.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/matcher.h>
#include <svo/config.h>
#include <svo/feature_detection.h>

namespace svo {

/// Static counters for seed batch and individual seed identification
int Seed::batch_counter = 0;
int Seed::seed_counter = 0;

/**
 * @brief Constructs a new depth seed from a feature.
 * 
 * Initializes the probabilistic depth model with:
 * - Inverse depth mu = 1/depth_mean (mean inverse depth)
 * - Depth range z_range = 1/depth_min (maximum inverse depth)
 * - Initial variance sigma² = (z_range)²/36 (covers ~3 sigma of range)
 * - Beta parameters a=b=10 (uniform prior on inlier probability)
 * 
 * @param ftr Feature to track (contains 2D position and bearing vector)
 * @param depth_mean Mean scene depth for initialization
 * @param depth_min Minimum scene depth (used for range calculation)
 */
Seed::Seed(Feature* ftr, float depth_mean, float depth_min) :
    batch_id(batch_counter),        // Batch ID for age tracking
    id(seed_counter++),             // Unique seed identifier
    ftr(ftr),                       // Associated feature
    a(10),                          // Beta distribution parameter (inlier count + prior)
    b(10),                          // Beta distribution parameter (outlier count + prior)
    mu(1.0/depth_mean),             // Mean inverse depth
    z_range(1.0/depth_min),         // Inverse depth range (max inverse depth)
    sigma2(z_range*z_range/36)      // Variance of inverse depth distribution
{}

/**
 * @brief Constructs the depth filter with feature detector and callback.
 * 
 * @param feature_detector Detector for finding new features to track
 * @param seed_converged_cb Callback when a seed's depth converges (adds to map)
 */
DepthFilter::DepthFilter(feature_detection::DetectorPtr feature_detector, callback_t seed_converged_cb) :
    feature_detector_(feature_detector),
    seed_converged_cb_(seed_converged_cb),
    seeds_updating_halt_(false),     // Flag to pause seed updates
    thread_(NULL),                   // Worker thread pointer
    new_keyframe_set_(false),        // Flag indicating new keyframe available
    new_keyframe_min_depth_(0.0),
    new_keyframe_mean_depth_(0.0)
{}

/**
 * @brief Destructor - stops the worker thread and cleans up.
 */
DepthFilter::~DepthFilter()
{
  stopThread();
  SVO_INFO_STREAM("DepthFilter destructed.");
}

/**
 * @brief Starts the background thread for seed updates.
 * 
 * The thread continuously processes incoming frames and updates seed depths.
 */
void DepthFilter::startThread()
{
  thread_ = new boost::thread(&DepthFilter::updateSeedsLoop, this);
}

/**
 * @brief Stops the background thread gracefully.
 * 
 * Signals the thread to stop and waits for it to complete.
 */
void DepthFilter::stopThread()
{
  SVO_INFO_STREAM("DepthFilter stop thread invoked.");
  if(thread_ != NULL)
  {
    SVO_INFO_STREAM("DepthFilter interrupt and join thread... ");
    seeds_updating_halt_ = true;
    thread_->interrupt();
    thread_->join();
    thread_ = NULL;
  }
}

/**
 * @brief Adds a regular frame for depth seed updates.
 * 
 * Frames are added to a queue for processing. If the queue is too long,
 * old frames are dropped to maintain real-time performance.
 * 
 * @param frame Frame to process for seed updates
 */
void DepthFilter::addFrame(FramePtr frame)
{
  if(thread_ != NULL)
  {
    {
      lock_t lock(frame_queue_mut_);
      if(frame_queue_.size() > 2)  // Limit queue size to prevent lag
        frame_queue_.pop();
      frame_queue_.push(frame);
    }
    seeds_updating_halt_ = false;
    frame_queue_cond_.notify_one();  // Wake up worker thread
  }
  else
    updateSeeds(frame);  // No thread - process synchronously
}

/**
 * @brief Adds a keyframe and initializes new seeds.
 * 
 * Keyframes trigger initialization of new depth seeds for features not
 * yet tracked. This is called when a new keyframe is selected.
 * 
 * @param frame New keyframe
 * @param depth_mean Mean scene depth for seed initialization
 * @param depth_min Minimum scene depth for depth range
 */
void DepthFilter::addKeyframe(FramePtr frame, double depth_mean, double depth_min)
{
  new_keyframe_min_depth_ = depth_min;
  new_keyframe_mean_depth_ = depth_mean;
  if(thread_ != NULL)
  {
    new_keyframe_ = frame;
    new_keyframe_set_ = true;
    seeds_updating_halt_ = true;  // Pause regular updates
    frame_queue_cond_.notify_one();
  }
  else
    initializeSeeds(frame);
}

/**
 * @brief Initializes new seeds from features in a keyframe.
 * 
 * Detects new features in the frame (avoiding existing feature locations)
 * and creates a seed for each new feature with the probabilistic depth model.
 * 
 * @param frame Keyframe to initialize seeds from
 */
void DepthFilter::initializeSeeds(FramePtr frame)
{
  Features new_features;
  feature_detector_->setExistingFeatures(frame->fts_);  // Avoid existing features
  feature_detector_->detect(frame.get(), frame->img_pyr_,
                            Config::triangMinCornerScore(), new_features);

  // Create seeds for all new features
  seeds_updating_halt_ = true;
  lock_t lock(seeds_mut_);  // Lock prevents updateSeeds from running
  ++Seed::batch_counter;    // New batch for age tracking
  std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
    seeds_.push_back(Seed(ftr, new_keyframe_mean_depth_, new_keyframe_min_depth_));
  });

  if(options_.verbose)
    SVO_INFO_STREAM("DepthFilter: Initialized "<<new_features.size()<<" new seeds");
  seeds_updating_halt_ = false;
}

/**
 * @brief Removes all seeds associated with a keyframe being deleted.
 * 
 * Called when a keyframe is removed from the map to clean up orphaned seeds.
 * 
 * @param frame Keyframe being removed
 */
void DepthFilter::removeKeyframe(FramePtr frame)
{
  seeds_updating_halt_ = true;
  lock_t lock(seeds_mut_);
  std::list<Seed>::iterator it=seeds_.begin();
  size_t n_removed = 0;
  while(it!=seeds_.end())
  {
    if(it->ftr->frame == frame.get())
    {
      it = seeds_.erase(it);
      ++n_removed;
    }
    else
      ++it;
  }
  seeds_updating_halt_ = false;
}

/**
 * @brief Resets the depth filter, clearing all seeds and queued frames.
 */
void DepthFilter::reset()
{
  seeds_updating_halt_ = true;
  {
    lock_t lock(seeds_mut_);
    seeds_.clear();
  }
  lock_t lock();
  while(!frame_queue_.empty())
    frame_queue_.pop();
  seeds_updating_halt_ = false;

  if(options_.verbose)
    SVO_INFO_STREAM("DepthFilter: RESET.");
}

/**
 * @brief Main loop for the background thread.
 * 
 * Waits for frames or keyframes and processes them. Keyframes trigger
 * seed initialization, regular frames trigger seed updates.
 */
void DepthFilter::updateSeedsLoop()
{
  while(!boost::this_thread::interruption_requested())
  {
    FramePtr frame;
    {
      lock_t lock(frame_queue_mut_);
      // Wait until there's work to do
      while(frame_queue_.empty() && new_keyframe_set_ == false)
        frame_queue_cond_.wait(lock);
      
      if(new_keyframe_set_)
      {
        // Keyframe has priority - clear queue and process it
        new_keyframe_set_ = false;
        seeds_updating_halt_ = false;
        clearFrameQueue();
        frame = new_keyframe_;
      }
      else
      {
        // Process next frame from queue
        frame = frame_queue_.front();
        frame_queue_.pop();
      }
    }
    updateSeeds(frame);
    if(frame->isKeyframe())
      initializeSeeds(frame);
  }
}

/**
 * @brief Updates all seeds using observations from a new frame.
 * 
 * For each seed, performs epipolar line search to find a match in the new
 * frame, then updates the depth probability distribution using Bayesian
 * inference. Seeds that converge (low variance) are converted to 3D points.
 * 
 * @param frame Frame to use for seed updates
 */
void DepthFilter::updateSeeds(FramePtr frame)
{
  // Track statistics
  size_t n_updates=0, n_failed_matches=0, n_seeds = seeds_.size();
  lock_t lock(seeds_mut_);
  std::list<Seed>::iterator it=seeds_.begin();

  // Compute pixel noise in angular terms for uncertainty estimation
  const double focal_length = frame->cam_->errorMultiplier2();
  double px_noise = 1.0;
  // Law of chords: convert pixel error to angular error
  double px_error_angle = atan(px_noise/(2.0*focal_length))*2.0;

  while( it!=seeds_.end())
  {
    // Check for halt signal (e.g., new keyframe incoming)
    if(seeds_updating_halt_)
      return;

    // Remove seeds that are too old (exceeded maximum keyframe age)
    if((Seed::batch_counter - it->batch_id) > options_.max_n_kfs) {
      it = seeds_.erase(it);
      continue;
    }

    // Check if the predicted point is in front of camera and projects into image
    SE3 T_ref_cur = it->ftr->frame->T_f_w_ * frame->T_f_w_.inverse();
    const Vector3d xyz_f(T_ref_cur.inverse()*(1.0/it->mu * it->ftr->f) );
    if(xyz_f.z() < 0.0)  {
      ++it;  // Point is behind camera
      continue;
    }
    if(!frame->cam_->isInFrame(frame->f2c(xyz_f).cast<int>())) {
      ++it;  // Point doesn't project into image
      continue;
    }

    // Compute inverse depth search range (mu ± sqrt(sigma²))
    float z_inv_min = it->mu + sqrt(it->sigma2);
    float z_inv_max = max(it->mu - sqrt(it->sigma2), 0.00000001f);
    
    // Search along epipolar line for a match
    double z;
    if(!matcher_.findEpipolarMatchDirect(
        *it->ftr->frame, *frame, *it->ftr, 1.0/it->mu, 1.0/z_inv_min, 1.0/z_inv_max, z))
    {
      it->b++;  // Increase outlier count (Beta distribution update)
      ++it;
      ++n_failed_matches;
      continue;
    }

    // Compute depth uncertainty tau from geometric triangulation error
    double tau = computeTau(T_ref_cur, it->ftr->f, z, px_error_angle);
    double tau_inverse = 0.5 * (1.0/max(0.0000001, z-tau) - 1.0/(z+tau));

    // Bayesian update of the inverse depth distribution
    updateSeed(1./z, tau_inverse*tau_inverse, &*it);
    ++n_updates;

    if(frame->isKeyframe())
    {
      // Mark grid cell as occupied to prevent detecting features at same location
      feature_detector_->setGridOccpuancy(matcher_.px_cur_);
    }

    // Check for convergence: variance below threshold relative to range
    if(sqrt(it->sigma2) < it->z_range/options_.seed_convergence_sigma2_thresh)
    {
      assert(it->ftr->point == NULL);
      // Create 3D point at converged depth
      Vector3d xyz_world(it->ftr->frame->T_f_w_.inverse() * (it->ftr->f * (1.0/it->mu)));
      Point* point = new Point(xyz_world, it->ftr);
      it->ftr->point = point;

      // Add to candidate points for potential map inclusion
      seed_converged_cb_(point, it->sigma2);
      it = seeds_.erase(it);
    }
    else if(isnan(z_inv_min))
    {
      SVO_WARN_STREAM("z_min is NaN");
      it = seeds_.erase(it);
    }
    else
      ++it;
  }
}

/**
 * @brief Clears the frame queue.
 */
void DepthFilter::clearFrameQueue()
{
  while(!frame_queue_.empty())
    frame_queue_.pop();
}

/**
 * @brief Gets a copy of seeds associated with a specific frame.
 * 
 * @param frame Frame to get seeds for
 * @param seeds Output list of seeds
 */
void DepthFilter::getSeedsCopy(const FramePtr& frame, std::list<Seed>& seeds)
{
  lock_t lock(seeds_mut_);
  for(std::list<Seed>::iterator it=seeds_.begin(); it!=seeds_.end(); ++it)
  {
    if (it->ftr->frame == frame.get())
      seeds.push_back(*it);
  }
}

/**
 * @brief Updates the seed's depth distribution using a new measurement.
 * 
 * Implements the Bayesian update from Vogiatzis & Hernández. The distribution
 * is a mixture of:
 * - Gaussian: for valid depth measurements (inliers)
 * - Uniform: for outliers/noise
 * 
 * The update computes the posterior distribution parameters based on
 * the likelihood of the measurement under both hypotheses.
 * 
 * @param x New inverse depth measurement
 * @param tau2 Variance of the measurement
 * @param seed Seed to update
 */
void DepthFilter::updateSeed(const float x, const float tau2, Seed* seed)
{
  // Compute normalization scale for the likelihood
  float norm_scale = sqrt(seed->sigma2 + tau2);
  if(std::isnan(norm_scale))
    return;
  
  // Likelihood under Gaussian (inlier) hypothesis
  boost::math::normal_distribution<float> nd(seed->mu, norm_scale);
  
  // Compute posterior variance and mean for Gaussian hypothesis
  float s2 = 1./(1./seed->sigma2 + 1./tau2);  // Posterior variance
  float m = s2*(seed->mu/seed->sigma2 + x/tau2);  // Posterior mean
  
  // Compute mixture weights C1 (inlier) and C2 (outlier)
  float C1 = seed->a/(seed->a+seed->b) * boost::math::pdf(nd, x);  // Prior * likelihood (inlier)
  float C2 = seed->b/(seed->a+seed->b) * 1./seed->z_range;         // Prior * likelihood (outlier)
  float normalization_constant = C1 + C2;
  C1 /= normalization_constant;  // Posterior probability of inlier
  C2 /= normalization_constant;  // Posterior probability of outlier
  
  // Update Beta distribution parameters for inlier/outlier tracking
  float f = C1*(seed->a+1.)/(seed->a+seed->b+1.) + C2*seed->a/(seed->a+seed->b+1.);
  float e = C1*(seed->a+1.)*(seed->a+2.)/((seed->a+seed->b+1.)*(seed->a+seed->b+2.))
          + C2*seed->a*(seed->a+1.0f)/((seed->a+seed->b+1.0f)*(seed->a+seed->b+2.0f));

  // Update inverse depth distribution parameters
  float mu_new = C1*m+C2*seed->mu;
  seed->sigma2 = C1*(s2 + m*m) + C2*(seed->sigma2 + seed->mu*seed->mu) - mu_new*mu_new;
  seed->mu = mu_new;
  seed->a = (e-f)/(f-e/f);  // Update Beta parameter a
  seed->b = seed->a*(1.0f-f)/f;  // Update Beta parameter b
}

/**
 * @brief Computes depth uncertainty tau from geometric triangulation.
 * 
 * Uses the law of sines to propagate pixel uncertainty to depth uncertainty
 * based on the triangulation geometry between reference and current frame.
 * 
 * @param T_ref_cur Transformation from current to reference frame
 * @param f Bearing vector in reference frame
 * @param z Estimated depth
 * @param px_error_angle Angular uncertainty from pixel noise
 * @return Depth uncertainty tau
 */
double DepthFilter::computeTau(
      const SE3& T_ref_cur,
      const Vector3d& f,
      const double z,
      const double px_error_angle)
{
  Vector3d t(T_ref_cur.translation());
  Vector3d a = f*z-t;  // Vector from current camera to 3D point
  double t_norm = t.norm();
  double a_norm = a.norm();
  
  // Angles in the triangulation triangle
  double alpha = acos(f.dot(t)/t_norm);  // Angle at reference camera
  double beta = acos(a.dot(-t)/(t_norm*a_norm));  // Angle at 3D point
  
  // Perturb beta by pixel error angle
  double beta_plus = beta + px_error_angle;
  double gamma_plus = PI-alpha-beta_plus;  // Triangle angles sum to PI
  
  // Law of sines: compute perturbed depth
  double z_plus = t_norm*sin(beta_plus)/sin(gamma_plus);
  
  return (z_plus - z);  // Depth uncertainty
}

} // namespace svo
