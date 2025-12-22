#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <rclcpp/rclcpp.hpp>

#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/params_helper.h>
#include <vikit/pinhole_camera.h>

#include <memory>
#include <string>

namespace vk
{
namespace camera_loader
{

/// Load from ROS2 Node parameters with a prefix
bool loadFromRosNode(
  rclcpp::Node::SharedPtr node, const std::string & prefix, vk::AbstractCamera *& cam)
{
  bool res = true;
  std::string cam_model = getParam<std::string>(node, prefix + "cam_model", "");
  if (cam_model == "Ocam") {
    cam = new vk::OmniCamera(getParam<std::string>(node, prefix + "cam_calib_file", ""));
  } else if (cam_model == "Pinhole") {
    cam = new vk::PinholeCamera(
      getParam<int>(node, prefix + "cam_width", 640),
      getParam<int>(node, prefix + "cam_height", 480),
      getParam<double>(node, prefix + "cam_fx", 0.0),
      getParam<double>(node, prefix + "cam_fy", 0.0),
      getParam<double>(node, prefix + "cam_cx", 0.0),
      getParam<double>(node, prefix + "cam_cy", 0.0),
      getParam<double>(node, prefix + "cam_d0", 0.0),
      getParam<double>(node, prefix + "cam_d1", 0.0),
      getParam<double>(node, prefix + "cam_d2", 0.0),
      getParam<double>(node, prefix + "cam_d3", 0.0));
  } else if (cam_model == "ATAN") {
    cam = new vk::ATANCamera(
      getParam<int>(node, prefix + "cam_width", 640),
      getParam<int>(node, prefix + "cam_height", 480),
      getParam<double>(node, prefix + "cam_fx", 0.0),
      getParam<double>(node, prefix + "cam_fy", 0.0),
      getParam<double>(node, prefix + "cam_cx", 0.0),
      getParam<double>(node, prefix + "cam_cy", 0.0),
      getParam<double>(node, prefix + "cam_d0", 0.0));
  } else {
    cam = nullptr;
    res = false;
  }
  return res;
}

// Overload for raw node pointer
bool loadFromRosNode(rclcpp::Node * node, const std::string & prefix, vk::AbstractCamera *& cam)
{
  bool res = true;
  std::string cam_model = getParam<std::string>(node, prefix + "cam_model", "");
  if (cam_model == "Ocam") {
    cam = new vk::OmniCamera(getParam<std::string>(node, prefix + "cam_calib_file", ""));
  } else if (cam_model == "Pinhole") {
    cam = new vk::PinholeCamera(
      getParam<int>(node, prefix + "cam_width", 640),
      getParam<int>(node, prefix + "cam_height", 480),
      getParam<double>(node, prefix + "cam_fx", 0.0),
      getParam<double>(node, prefix + "cam_fy", 0.0),
      getParam<double>(node, prefix + "cam_cx", 0.0),
      getParam<double>(node, prefix + "cam_cy", 0.0),
      getParam<double>(node, prefix + "cam_d0", 0.0),
      getParam<double>(node, prefix + "cam_d1", 0.0),
      getParam<double>(node, prefix + "cam_d2", 0.0),
      getParam<double>(node, prefix + "cam_d3", 0.0));
  } else if (cam_model == "ATAN") {
    cam = new vk::ATANCamera(
      getParam<int>(node, prefix + "cam_width", 640),
      getParam<int>(node, prefix + "cam_height", 480),
      getParam<double>(node, prefix + "cam_fx", 0.0),
      getParam<double>(node, prefix + "cam_fy", 0.0),
      getParam<double>(node, prefix + "cam_cx", 0.0),
      getParam<double>(node, prefix + "cam_cy", 0.0),
      getParam<double>(node, prefix + "cam_d0", 0.0));
  } else {
    cam = nullptr;
    res = false;
  }
  return res;
}

}  // namespace camera_loader
}  // namespace vk

#endif  // VIKIT_CAMERA_LOADER_H_
