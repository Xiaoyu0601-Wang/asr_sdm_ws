/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef SVO_ROS_MSCKF_BACKEND_MATH_UTILS_HPP
#define SVO_ROS_MSCKF_BACKEND_MATH_UTILS_HPP

#include <cmath>

#include <Eigen/Dense>

namespace msckf_vio {

/*
 *  @brief Create a skew-symmetric matrix from a 3-element vector.
 *  @note Performs the operation:
 *  w   ->  [  0 -w3  w2]
 *          [ w3   0 -w1]
 *          [-w2  w1   0]
 */
inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& w) {
  Eigen::Matrix3d w_hat;
  w_hat(0, 0) = 0;
  w_hat(0, 1) = -w(2);
  w_hat(0, 2) = w(1);
  w_hat(1, 0) = w(2);
  w_hat(1, 1) = 0;
  w_hat(1, 2) = -w(0);
  w_hat(2, 0) = -w(1);
  w_hat(2, 1) = w(0);
  w_hat(2, 2) = 0;
  return w_hat;
}

/*
 * @brief Normalize the given quaternion to unit quaternion.
 */
inline void quaternionNormalize(Eigen::Vector4d& q) {
  const double norm = q.norm();
  q = q / norm;
}

/*
 * @brief Perform q1 * q2
 */
inline Eigen::Vector4d quaternionMultiplication(
    const Eigen::Vector4d& q1,
    const Eigen::Vector4d& q2) {
  Eigen::Matrix4d L;
  L(0, 0) =  q1(3); L(0, 1) =  q1(2); L(0, 2) = -q1(1); L(0, 3) =  q1(0);
  L(1, 0) = -q1(2); L(1, 1) =  q1(3); L(1, 2) =  q1(0); L(1, 3) =  q1(1);
  L(2, 0) =  q1(1); L(2, 1) = -q1(0); L(2, 2) =  q1(3); L(2, 3) =  q1(2);
  L(3, 0) = -q1(0); L(3, 1) = -q1(1); L(3, 2) = -q1(2); L(3, 3) =  q1(3);

  Eigen::Vector4d q = L * q2;
  quaternionNormalize(q);
  return q;
}

/*
 * @brief Convert the vector part of a quaternion to a full quaternion.
 */
inline Eigen::Vector4d smallAngleQuaternion(const Eigen::Vector3d& dtheta) {
  const Eigen::Vector3d dq = dtheta / 2.0;
  Eigen::Vector4d q;
  const double dq_square_norm = dq.squaredNorm();

  if (dq_square_norm <= 1) {
    q.head<3>() = dq;
    q(3) = std::sqrt(1 - dq_square_norm);
  } else {
    q.head<3>() = dq;
    q(3) = 1;
    q = q / std::sqrt(1 + dq_square_norm);
  }

  return q;
}

/*
 * @brief Convert a quaternion to the corresponding rotation matrix.
 */
inline Eigen::Matrix3d quaternionToRotation(const Eigen::Vector4d& q) {
  const Eigen::Vector3d& q_vec = q.block(0, 0, 3, 1);
  const double& q4 = q(3);
  const Eigen::Matrix3d R =
    (2 * q4 * q4 - 1) * Eigen::Matrix3d::Identity() -
    2 * q4 * skewSymmetric(q_vec) +
    2 * q_vec * q_vec.transpose();
  return R;
}

/*
 * @brief Convert a rotation matrix to a quaternion.
 */
inline Eigen::Vector4d rotationToQuaternion(const Eigen::Matrix3d& R) {
  Eigen::Vector4d score;
  score(0) = R(0, 0);
  score(1) = R(1, 1);
  score(2) = R(2, 2);
  score(3) = R.trace();

  int max_row = 0, max_col = 0;
  score.maxCoeff(&max_row, &max_col);

  Eigen::Vector4d q = Eigen::Vector4d::Zero();
  if (max_row == 0) {
    q(0) = std::sqrt(1 + 2 * R(0, 0) - R.trace()) / 2.0;
    q(1) = (R(0, 1) + R(1, 0)) / (4 * q(0));
    q(2) = (R(0, 2) + R(2, 0)) / (4 * q(0));
    q(3) = (R(1, 2) - R(2, 1)) / (4 * q(0));
  } else if (max_row == 1) {
    q(1) = std::sqrt(1 + 2 * R(1, 1) - R.trace()) / 2.0;
    q(0) = (R(0, 1) + R(1, 0)) / (4 * q(1));
    q(2) = (R(1, 2) + R(2, 1)) / (4 * q(1));
    q(3) = (R(2, 0) - R(0, 2)) / (4 * q(1));
  } else if (max_row == 2) {
    q(2) = std::sqrt(1 + 2 * R(2, 2) - R.trace()) / 2.0;
    q(0) = (R(0, 2) + R(2, 0)) / (4 * q(2));
    q(1) = (R(1, 2) + R(2, 1)) / (4 * q(2));
    q(3) = (R(0, 1) - R(1, 0)) / (4 * q(2));
  } else {
    q(3) = std::sqrt(1 + R.trace()) / 2.0;
    q(0) = (R(1, 2) - R(2, 1)) / (4 * q(3));
    q(1) = (R(2, 0) - R(0, 2)) / (4 * q(3));
    q(2) = (R(0, 1) - R(1, 0)) / (4 * q(3));
  }

  if (q(3) < 0) {
    q = -q;
  }
  quaternionNormalize(q);
  return q;
}

}  // namespace msckf_vio

#endif  // SVO_ROS_MSCKF_BACKEND_MATH_UTILS_HPP

