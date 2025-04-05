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
#include "asr_sdm_hardware/msg/can_frame.hpp"
#include "asr_sdm_hardware/uart_can.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

// #define GPIO_CHIP "/dev/gpiochip0"
// #define GPIO_PIN 24

using namespace std::chrono_literals;

class AsrSdmHardwareNode : public rclcpp::Node
{
public:
  AsrSdmHardwareNode() : Node("asrsdm_hardware"), count_(0)
  {
    this->declare_parameter("uart_can.uart_port", "/dev/ttyS3");
    const std::string uart_port =
      this->get_parameter("uart_can.uart_port").get_parameter_value().get<std::string>();
    this->declare_parameter("uart_can.uart_baudrate", 115200);
    const uint32_t uart_baudrate =
      this->get_parameter("uart_can.uart_port").get_parameter_value().get<uint32_t>();

    // const std::string uart_port =
    // this->declare_parameter<std::string>("uart_can.uart_port", "/dev/ttyS3");
    // const uint32_t uart_baudrate =
    //   this->declare_parameter<uint32_t>("uart_can.uart_baudrate", 115200);

    uart_can_ = std::make_unique<amp::UART_CAN>(uart_port, uart_baudrate);

    pub_ros_info_ = this->create_publisher<std_msgs::msg::String>("~/output/topic", 10);
    sub_can_interface_ = this->create_subscription<asr_sdm_hardware::msg::CanFrame>(
      "~/input/can_frame", rclcpp::SensorDataQoS{}.keep_last(1),
      std::bind(&AsrSdmHardwareNode::canFrameCallback, this, std::placeholders::_1));

    timer_hardware_ =
      this->create_wall_timer(1000ms, std::bind(&AsrSdmHardwareNode::timerPublishCallback, this));
    timer_imu_ =
      this->create_wall_timer(500ms, std::bind(&AsrSdmHardwareNode::timerHardwareCallback, this));
  }

private:
  void canFrameCallback(const asr_sdm_hardware::msg::CanFrame::SharedPtr msg)
  {
    msg_can_frame_ = *msg;
    uart_can_->sendMsg(msg->id, msg->rtr, msg->ext, msg->dlc, msg->data.data());
    RCLCPP_INFO(
      this->get_logger(), "Received CAN frame: id=%d, data=%d", msg_can_frame_.id,
      msg_can_frame_.data[0]);
  }

  void timerPublishCallback()
  {
    // RCLCPP_INFO("Publishing");
    RCLCPP_INFO(this->get_logger(), "Publishing");
    //      auto message = std_msgs::msg::String();
    //      message.data = "Hello, world! " + std::to_string(count_++);
    //      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //      publisher_->publish(message);
  }

  void timerHardwareCallback() {}

  size_t count_;

  rclcpp::TimerBase::SharedPtr timer_hardware_;
  rclcpp::TimerBase::SharedPtr timer_imu_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ros_info_;
  rclcpp::Subscription<asr_sdm_hardware::msg::CanFrame>::SharedPtr sub_can_interface_;

  amp::UART_CAN::Ptr uart_can_;
  asr_sdm_hardware::msg::CanFrame msg_can_frame_;
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
