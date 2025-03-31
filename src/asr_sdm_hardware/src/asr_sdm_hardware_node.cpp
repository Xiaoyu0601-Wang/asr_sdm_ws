/* standard headers */
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <string>

/* ROS2 headers */
#include "asr_sdm_hardware/msg/CANFrame.hpp"
#include "periphery/spi.h"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

// #define SPI_PORT 3
// #define SPI_CS   0
// #define SPI_FREQUENCY   1000000

// #define SPI_DEVICE "/dev/spidev3.0"
// // #define SPI_SPEED 500000  // 500kHz
// #define GPIO_CHIP "/dev/gpiochip0"
// #define GPIO_PIN 24

using namespace std::chrono_literals;

class AsrSdmHardwareNode : public rclcpp::Node
{
public:
  AsrSdmHardwareNode() : Node("asrsdm_hardware"), count_(0)
  {
    const std::string uart_port =
      this->declare_parameter<std::string>("uart_can.uart_port", "/dev/ttyS3");
    const uint32_t uart_baudrate =
      this->declare_parameter<uint32_t>("uart_can.uart_baudrate", 115200);

    uart_can_ = std::make_unique<amp::UART_CAN>();
    // uart_can_->initCAN(MCP_ANY, CAN_250KBPS, MCP_8MHZ);

    pub_ros_info_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    sub_can_interface_ = this->create_subscription<asr_sdm_hardware::msg::CANFrame>(
      "~/input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
      std::bind(&AsrSdmHardwareNode::topic_callback, this, std::placeholders::_1));

    timer_hardware_ =
      this->create_wall_timer(1000ms, std::bind(&AsrSdmHardwareNode::timer_callback, this));
    timer_imu_ =
      this->create_wall_timer(500ms, std::bind(&AsrSdmHardwareNode::timer_imu_callback, this));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  void timer_callback()
  {
    // RCLCPP_INFO("Publishing");
    RCLCPP_INFO(this->get_logger(), "Publishing");
    //      auto message = std_msgs::msg::String();
    //      message.data = "Hello, world! " + std::to_string(count_++);
    //      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //      publisher_->publish(message);
  }

  void timer_imu_callback() {}

  size_t count_;

  rclcpp::TimerBase::SharedPtr timer_hardware_;
  rclcpp::TimerBase::SharedPtr timer_imu_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ros_info_;
  rclcpp::Subscription<asr_sdm_hardware::msg::CANFrame>::SharedPtr sub_can_interface_;

  amp::UART_CAN::Ptr uart_can_;
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
