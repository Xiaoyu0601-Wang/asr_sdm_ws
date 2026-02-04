#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace svo {

class SvoInterface;

// ROS2 Component equivalent of ROS1 Nodelet.
class SvoNodelet : public rclcpp::Node
{
public:
  explicit SvoNodelet(const rclcpp::NodeOptions& options);
  ~SvoNodelet() override;

 private:
  std::unique_ptr<SvoInterface> svo_interface_;
};

} // namespace svo
