#include "asr_sdm_ros2/svo_node_base.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("svo");
  svo_ros::SvoNodeBase svo_node(node, argc, argv);
  svo_node.run();
  rclcpp::shutdown();
  return 0;
}
