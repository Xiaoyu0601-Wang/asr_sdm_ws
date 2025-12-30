#include <svo_ros/msckf_frontend/utils.h>

#include <cstring>

namespace msckf_vio
{
namespace utils
{

static cv::Mat getTransformMat(rclcpp::Node& node, const std::string& field)
{
  std::vector<double> tf_vec;
  if (!node.has_parameter(field)) {
    // Declare with identity by default.
    node.declare_parameter<std::vector<double>>(field, std::vector<double>{
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0
    });
  }
  node.get_parameter(field, tf_vec);

  cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
  if (tf_vec.size() == 16) {
    std::memcpy(T.ptr<double>(), tf_vec.data(), 16 * sizeof(double));
  }
  return T;
}

Eigen::Isometry3d getTransformEigen(rclcpp::Node& node, const std::string& field)
{
  cv::Mat T_cv = getTransformMat(node, field);
  Eigen::Matrix4d T_eig = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      T_eig(r,c) = T_cv.at<double>(r,c);

  Eigen::Isometry3d T_iso(T_eig);
  return T_iso;
}

cv::Mat getTransformCV(rclcpp::Node& node, const std::string& field)
{
  return getTransformMat(node, field);
}

cv::Mat getVec16Transform(rclcpp::Node& node, const std::string& field)
{
  return getTransformMat(node, field);
}

cv::Mat getKalibrStyleTransform(rclcpp::Node& node, const std::string& field)
{
  return getTransformMat(node, field);
}

}  // namespace utils
}  // namespace msckf_vio
