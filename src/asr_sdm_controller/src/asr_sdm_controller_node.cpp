/* standard headers */
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>

/* ROS2 headers */
#include <rclcpp/rclcpp.hpp>

#include "asr_sdm_control_msgs/msg/robot_cmd.hpp"
#include "asr_sdm_hardware_msgs/msg/can_frame.hpp"
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class AsrSdmControllerNode : public rclcpp::Node
{
public:
  AsrSdmControllerNode() : Node("asr_sdm_controller"), count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Start");
    pub_ros_info_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_hardware_ =
      this->create_wall_timer(1000ms, std::bind(&AsrSdmControllerNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Publishing");
    //      auto message = std_msgs::msg::String();
    //      message.data = "Hello, world! " + std::to_string(count_++);
    //      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //      publisher_->publish(message);
  }

  size_t count_;

  rclcpp::TimerBase::SharedPtr timer_hardware_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ros_info_;
  rclcpp::Publisher<asr_sdm_hardware_msgs::msg::CanFrame>::SharedPtr pub_can_interface_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsrSdmControllerNode>());
  rclcpp::shutdown();
  return 0;
}
