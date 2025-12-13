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
 * @file point.cpp
 * @brief 3D map point with multi-view observations.
 * 
 * A Point represents a 3D landmark in the map. Each point maintains:
 * - 3D position in world coordinates
 * - List of feature observations from different keyframes
 * - Quality tracking (type, failed/successful reprojection counts)
 * - Optional surface normal for filtering
 * 
 * Points can have different types:
 * - TYPE_CANDIDATE: From depth filter, not yet validated
 * - TYPE_UNKNOWN: Regular point, quality unknown
 * - TYPE_GOOD: Successfully tracked multiple times
 * - TYPE_DELETED: Marked for garbage collection
 */

#include <stdexcept>
#include <vikit/math_utils.h>
#include <svo/point.h>
#include <svo/frame.h>
#include <svo/feature.h>
 
namespace svo {

/// Static counter for unique point IDs
int Point::point_counter_ = 0;

/**
 * @brief Constructs a point at the given 3D position.
 * 
 * @param pos 3D position in world coordinates
 */
Point::Point(const Vector3d& pos) :
  id_(point_counter_++),             // Unique point ID
  pos_(pos),                         // 3D position
  normal_set_(false),                // Surface normal not computed
  n_obs_(0),                         // No observations yet
  v_pt_(NULL),                       // g2o vertex (for BA)
  last_published_ts_(0),             // Last visualization timestamp
  last_projected_kf_id_(-1),         // Last keyframe this was projected to
  type_(TYPE_UNKNOWN),               // Default type
  n_failed_reproj_(0),               // Failed reprojection counter
  n_succeeded_reproj_(0),            // Successful reprojection counter
  last_structure_optim_(0)           // Last optimization frame ID
{}

/**
 * @brief Constructs a point with an initial observation.
 * 
 * @param pos 3D position in world coordinates
 * @param ftr Initial feature observation
 */
Point::Point(const Vector3d& pos, Feature* ftr) :
  id_(point_counter_++),
  pos_(pos),
  normal_set_(false),
  n_obs_(1),
  v_pt_(NULL),
  last_published_ts_(0),
  last_projected_kf_id_(-1),
  type_(TYPE_UNKNOWN),
  n_failed_reproj_(0),
  n_succeeded_reproj_(0),
  last_structure_optim_(0)
{
  obs_.push_front(ftr);  // Add initial observation
}

/**
 * @brief Destructor.
 */
Point::~Point()
{}

/**
 * @brief Adds a new frame observation of this point.
 * 
 * @param ftr Feature observing this point
 */
void Point::addFrameRef(Feature* ftr)
{
  obs_.push_front(ftr);
  ++n_obs_;
}

/**
 * @brief Finds the feature in a specific frame that observes this point.
 * 
 * @param frame Frame to search in
 * @return Feature pointer, or NULL if not found
 */
Feature* Point::findFrameRef(Frame* frame)
{
  for(auto it=obs_.begin(), ite=obs_.end(); it!=ite; ++it)
    if((*it)->frame == frame)
      return *it;
  return NULL;
}

/**
 * @brief Removes the observation from a specific frame.
 * 
 * @param frame Frame to remove observation from
 * @return true if observation was found and removed
 */
bool Point::deleteFrameRef(Frame* frame)
{
  for(auto it=obs_.begin(), ite=obs_.end(); it!=ite; ++it)
  {
    if((*it)->frame == frame)
    {
      obs_.erase(it);
      return true;
    }
  }
  return false;
}

/**
 * @brief Initializes the surface normal from the first observation.
 * 
 * The normal points from the point toward the first observing camera.
 * This can be used for visibility checking in future observations.
 */
void Point::initNormal()
{
  assert(!obs_.empty());
  const Feature* ftr = obs_.back();
  assert(ftr->frame != NULL);
  
  // Normal points toward camera, transformed to world frame
  normal_ = ftr->frame->T_f_w_.rotationMatrix().transpose()*(-ftr->f);
  
  // Information matrix for normal: high confidence along view direction
  normal_information_ = Eigen::DiagonalMatrix<double,3,3>(pow(20/(pos_-ftr->frame->pos()).norm(),2), 1.0, 1.0);
  normal_set_ = true;
}

/**
 * @brief Finds the observation with viewing angle closest to current view.
 * 
 * Used to select the best reference patch for matching. Observations
 * from similar viewpoints produce better matches due to less
 * perspective distortion.
 * 
 * @param framepos Position of current camera
 * @param ftr Output: best reference feature
 * @return true if a suitable observation found (angle < 60°)
 */
bool Point::getCloseViewObs(const Vector3d& framepos, Feature*& ftr) const
{
  Vector3d obs_dir(framepos - pos_); 
  obs_dir.normalize();
  
  auto min_it=obs_.begin();
  double min_cos_angle = 0;
  
  for(auto it=obs_.begin(), ite=obs_.end(); it!=ite; ++it)
  {
    Vector3d dir((*it)->frame->pos() - pos_); 
    dir.normalize();
    double cos_angle = obs_dir.dot(dir);
    
    if(cos_angle > min_cos_angle)
    {
      min_cos_angle = cos_angle;
      min_it = it;
    }
  }
  
  ftr = *min_it;
  
  // Reject if viewing angle too different (> 60°)
  if(min_cos_angle < 0.5)
    return false;
  return true;
}

/**
 * @brief Optimizes the 3D point position using all observations.
 * 
 * Uses Gauss-Newton optimization to minimize the sum of squared
 * reprojection errors across all observing frames.
 * 
 * The Jacobian of the reprojection w.r.t. 3D point position is:
 * J = d(project(T*p))/dp where T is frame pose, p is 3D point
 * 
 * @param n_iter Maximum number of iterations
 */
void Point::optimize(const size_t n_iter)
{
  Vector3d old_point = pos_;
  double chi2 = 0.0;
  Matrix3d A;   // Hessian approximation (J^T * J)
  Vector3d b;   // Gradient (J^T * residual)

  for(size_t i=0; i<n_iter; i++)
  {
    A.setZero();
    b.setZero();
    double new_chi2 = 0.0;

    // Accumulate residuals and Jacobians from all observations
    for(auto it=obs_.begin(); it!=obs_.end(); ++it)
    {
      Matrix23d J;
      const Vector3d p_in_f((*it)->frame->T_f_w_ * pos_);
      
      // Jacobian of projection w.r.t. 3D point
      Point::jacobian_xyz2uv(p_in_f, (*it)->frame->T_f_w_.rotationMatrix(), J);
      
      // Reprojection error on unit plane
      const Vector2d e(vk::project2d((*it)->f) - vk::project2d(p_in_f));
      new_chi2 += e.squaredNorm();
      
      // Gauss-Newton: H = J^T * J, b = -J^T * e
      A.noalias() += J.transpose() * J;
      b.noalias() -= J.transpose() * e;
    }

    // Solve linear system using LDLT decomposition
    const Vector3d dp(A.ldlt().solve(b));

    // Check for error increase or NaN
    if((i > 0 && new_chi2 > chi2) || (bool) std::isnan((double)dp[0]))
    {
#ifdef POINT_OPTIMIZER_DEBUG
      cout << "it " << i
           << "\t FAILURE \t new_chi2 = " << new_chi2 << endl;
#endif
      pos_ = old_point;  // Roll back
      break;
    }

    // Apply update
    Vector3d new_point = pos_ + dp;
    old_point = pos_;
    pos_ = new_point;
    chi2 = new_chi2;
    
#ifdef POINT_OPTIMIZER_DEBUG
    cout << "it " << i
         << "\t Success \t new_chi2 = " << new_chi2
         << "\t norm(b) = " << vk::norm_max(b)
         << endl;
#endif

    // Check for convergence
    if(vk::norm_max(dp) <= EPS)
      break;
  }
#ifdef POINT_OPTIMIZER_DEBUG
  cout << endl;
#endif
}

} // namespace svo
