#pragma once

#include "asr_sdm_ros2/svo_interface.h"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stdexcept>

namespace svo_ros
{

class SvoNodeBase
{
public:
  // Initializes glog, gflags and ROS2.
  static void initThirdParty(int argc, char ** argv);

  explicit SvoNodeBase(const rclcpp::Node::SharedPtr & node, int argc, char** argv);

  void run();

private:
  rclcpp::Node::SharedPtr node_;
  svo::PipelineType type_;

public:
  std::unique_ptr<svo::SvoInterface> svo_interface_;
};

}  // namespace svo_ros
