#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace svo {

// forward declarations
class BackendInterface;

namespace backend_factory {

// ROS2: use rclcpp::Node for parameter access
std::shared_ptr<BackendInterface> makeBackend(const rclcpp::Node::SharedPtr& node);

} // namespace backend_factory
} // namespace svo
