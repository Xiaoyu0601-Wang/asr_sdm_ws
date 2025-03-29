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
    callback_group_cf_cmd_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_cf_cmd = rclcpp::SubscriptionOptions();
    sub_opt_cf_cmd.callback_group = callback_group_cf_cmd_;

    pub_ros_info_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    sub_can_interface_ = this->create_subscription<asr_sdm_hardware::msg::CANFrame>(
      "topic", rclcpp::SystemDefaultsQoS(),
      std::bind(&AsrSdmHardwareNode::topic_callback, this, _1), sub_opt_cf_cmd);

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

  // multithreading
  rclcpp::CallbackGroup::SharedPtr callback_group_cf_cmd_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsrSdmHardwareNode>());
  rclcpp::shutdown();
  return 0;
}
