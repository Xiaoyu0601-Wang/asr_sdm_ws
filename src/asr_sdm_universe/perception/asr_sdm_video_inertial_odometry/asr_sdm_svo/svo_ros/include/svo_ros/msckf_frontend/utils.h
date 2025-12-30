/*
 * ROS2 utilities for MSCKF_VIO within svo_ros
 */

#ifndef SVO_ROS_MSCKF_VIO_UTILS_H
#define SVO_ROS_MSCKF_VIO_UTILS_H

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>

namespace msckf_vio
{
namespace utils
{

/// Read a 4x4 transform matrix from a ROS2 parameter and return as Eigen::Isometry3d.
/// Parameter is expected as std::vector<double> of length 16, row-major.
Eigen::Isometry3d getTransformEigen(rclcpp::Node& node, const std::string& field);

/// Same as getTransformEigen but returned as cv::Mat (4x4, CV_64F).
cv::Mat getTransformCV(rclcpp::Node& node, const std::string& field);

/// Get raw 4x4 transform as cv::Mat from a std::vector<double> ROS2 parameter.
cv::Mat getVec16Transform(rclcpp::Node& node, const std::string& field);

/// Alias of getVec16Transform for compatibility with Kalibr-style naming.
cv::Mat getKalibrStyleTransform(rclcpp::Node& node, const std::string& field);

}  // namespace utils
}  // namespace msckf_vio

#endif  // SVO_ROS_MSCKF_VIO_UTILS_H
