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
    pub_heartbeat_ =
      this->create_publisher<std_msgs::msg::String>("~/output/controller/heartbeat", 1);
    pub_robot_cmd_ =
      this->create_publisher<asr_sdm_control_msgs::msg::RobotCmd>("~/output/robot_cmd", 1);
    timer_heartbeat_ =
      this->create_wall_timer(1500ms, std::bind(&AsrSdmControllerNode::timer_heartbeat, this));
    timer_robot_control_ =
      this->create_wall_timer(100ms, std::bind(&AsrSdmControllerNode::timer_controller, this));
  }

private:
  void timer_heartbeat() { RCLCPP_INFO(this->get_logger(), "Control Node Heartbeat"); }

  void timer_controller()
  {
    RCLCPP_INFO(this->get_logger(), "Control Node Publishing");
    //      auto message = std_msgs::msg::String();
    //      message.data = "Hello, world! " + std::to_string(count_++);
    //      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //      publisher_->publish(message);
  }

  size_t count_;

  rclcpp::TimerBase::SharedPtr timer_heartbeat_;
  rclcpp::TimerBase::SharedPtr timer_robot_control_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_heartbeat_;
  rclcpp::Publisher<asr_sdm_control_msgs::msg::RobotCmd>::SharedPtr pub_robot_cmd_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsrSdmControllerNode>());
  rclcpp::shutdown();
  return 0;
}
