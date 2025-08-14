/* standard headers */
#include <fcntl.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <string>

/* ROS2 headers */
#include "asr_sdm_hardware/uart2can.hpp"

#include <rclcpp/rclcpp.hpp>

#include "asr_sdm_control_msgs/msg/robot_cmd.hpp"
#include "asr_sdm_hardware_msgs/msg/can_frame.hpp"
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class AsrSdmHardwareNode : public rclcpp::Node
{
public:
  AsrSdmHardwareNode() : Node("asrsdm_hardware"), count_(0)
  {
    // Declare parameters with default values
    this->declare_parameter("uart_can.uart_port", "/dev/ttyACM0");
    this->declare_parameter("uart_can.uart_baudrate", 57600);
    // Get parameters
    const std::string uart_port =
      this->get_parameter("uart_can.uart_port").get_parameter_value().get<std::string>();
    const uint32_t uart_baudrate =
      this->get_parameter("uart_can.uart_baudrate").get_parameter_value().get<uint32_t>();
    RCLCPP_INFO(
      this->get_logger(), "UART Port: %s, Baudrate: %u", uart_port.c_str(), uart_baudrate);
    // Initialize UART_CAN instance
    uart_can_ = std::make_unique<amp::UART_CAN>(uart_port, uart_baudrate);
    RCLCPP_INFO(this->get_logger(), "UART_CAN initialized");

    // Declare the topic name parameter with default value
    this->declare_parameter<std::string>("topic_asr_sdm_cmd", "~/input/asr_sdm_cmd");
    // Get the topic name parameter
    std::string topic_asr_sdm_cmd =
      this->get_parameter("topic_asr_sdm_cmd").get_parameter_value().get<std::string>();
    // Initialize publishers and subscribers
    pub_heartbeat_ = this->create_publisher<std_msgs::msg::String>("~/hardware/heartbeat", 1);
    sub_robot_cmd_ = this->create_subscription<asr_sdm_control_msgs::msg::RobotCmd>(
      topic_asr_sdm_cmd, rclcpp::SensorDataQoS{}.keep_last(1),
      std::bind(&AsrSdmHardwareNode::hardware_control, this, std::placeholders::_1));

    // Initialize timers
    timer_heartbeat_ =
      this->create_wall_timer(1000ms, std::bind(&AsrSdmHardwareNode::timer_heartbeat, this));
    timer_imu_ =
      this->create_wall_timer(500ms, std::bind(&AsrSdmHardwareNode::timer_hardware_ctrl, this));
  }

private:
  void hardware_control(const asr_sdm_control_msgs::msg::RobotCmd::SharedPtr msg)
  {
    msg_robot_cmd_ = *msg;
  }

  void timer_heartbeat()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello ROS2: " + std::to_string(1);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    pub_heartbeat_->publish(message);
    // RCLCPP_INFO(this->get_logger(), "Hardware Node Heartbeat");
  }

  void timer_hardware_ctrl() {}

  size_t count_;

  rclcpp::TimerBase::SharedPtr timer_heartbeat_;
  rclcpp::TimerBase::SharedPtr timer_imu_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_heartbeat_;
  rclcpp::Subscription<asr_sdm_control_msgs::msg::RobotCmd>::SharedPtr sub_robot_cmd_;

  amp::UART_CAN::Ptr uart_can_;
  asr_sdm_control_msgs::msg::RobotCmd msg_robot_cmd_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AsrSdmHardwareNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
