#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <svo_ros/msckf_backend/msckf_vio.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("msckf_backend_node");

  auto vio = std::make_shared<msckf_vio::MsckfVio>(node);
  (void)vio->initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

