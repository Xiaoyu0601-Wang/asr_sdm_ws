#include <rclcpp/rclcpp.hpp>

#include <svo_ros/msckf_frontend/image_processor.h>

namespace msckf_vio
{

// Simple ROS2 node that owns an ImageProcessor frontend.
class VioNode : public rclcpp::Node
{
public:
  VioNode() : rclcpp::Node("svo_vio")
  {
    // Construct ImageProcessor with this node handle.
    image_processor_ = std::make_unique<ImageProcessor>(this);
    if (!image_processor_->initialize()) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize ImageProcessor");
      throw std::runtime_error("ImageProcessor initialization failed");
    }

    RCLCPP_INFO(get_logger(), "svo_vio node started (ImageProcessor frontend active)");
  }

private:
  std::unique_ptr<ImageProcessor> image_processor_;
};

}  // namespace msckf_vio

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<msckf_vio::VioNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
