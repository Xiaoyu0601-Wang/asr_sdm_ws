/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef SVO_ROS_MSCKF_BACKEND_CAM_STATE_H
#define SVO_ROS_MSCKF_BACKEND_CAM_STATE_H

#include <map>
#include <vector>

#include <Eigen/Dense>

#include "imu_state.h"

namespace msckf_vio {

/*
 * @brief CAMState stores a camera state used to form measurement models.
 */
struct CAMState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // An unique identifier for the CAM state.
  StateIDType id;

  // Time when the state is recorded.
  double time;

  // Orientation (quaternion as [x y z w]).
  // Takes a vector from the world frame to the camera frame.
  Eigen::Vector4d orientation;

  // Position of the camera frame in the world frame.
  Eigen::Vector3d position;

  // Null-space copies for observability constraint.
  Eigen::Vector4d orientation_null;
  Eigen::Vector3d position_null;

  // Takes a vector from the cam0 frame to the cam1 frame.
  static Eigen::Isometry3d T_cam0_cam1;

  CAMState()
    : id(0),
      time(0),
      orientation(Eigen::Vector4d(0, 0, 0, 1)),
      position(Eigen::Vector3d::Zero()),
      orientation_null(Eigen::Vector4d(0, 0, 0, 1)),
      position_null(Eigen::Vector3d::Zero()) {}

  explicit CAMState(const StateIDType& new_id)
    : id(new_id),
      time(0),
      orientation(Eigen::Vector4d(0, 0, 0, 1)),
      position(Eigen::Vector3d::Zero()),
      orientation_null(Eigen::Vector4d(0, 0, 0, 1)),
      position_null(Eigen::Vector3d::Zero()) {}
};

using CamStateServer = std::map<StateIDType, CAMState, std::less<int>,
  Eigen::aligned_allocator<std::pair<const StateIDType, CAMState>>>;

}  // namespace msckf_vio

#endif  // SVO_ROS_MSCKF_BACKEND_CAM_STATE_H

