#ifndef SVO_POSE_OPTIMIZER_H_
#define SVO_POSE_OPTIMIZER_H_

#include <svo/global.h>

namespace svo
{

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class Point;

/// Motion-only bundle adjustment. Minimize the reprojection error of a single frame.
namespace pose_optimizer
{

/// Standard version without IMU prior (backward compatible).
void optimizeGaussNewton(
  const double reproj_thresh, const size_t n_iter, const bool verbose, FramePtr & frame,
  double & estimated_scale, double & error_init, double & error_final, size_t & num_obs);

/// Version with IMU rotation prior:
///   - R_world_from_imu: initial gravity-aligned orientation of world w.r.t. IMU
///   - R_imu_last_from_imu_cur: relative rotation of IMU between frames (from gyroscope)
///   - lambda: regularization strength (0 = no prior, >0 = stronger prior)
void optimizeGaussNewtonWithImuPrior(
  const double reproj_thresh, const size_t n_iter, const bool verbose, FramePtr & frame,
  const Quaterniond& R_world_from_imu, const Quaterniond& R_imu_last_from_imu_cur,
  double lambda,
  double & estimated_scale, double & error_init, double & error_final, size_t & num_obs);

}  // namespace pose_optimizer
}  

#endif
