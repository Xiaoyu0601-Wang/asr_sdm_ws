/* standard headers */
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <string>

/* ROS2 headers */
#include "asr_sdm_controller/mcp_can.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

// #include <fcntl.h>
// #include <linux/spi/spidev.h>
// #include <sys/ioctl.h>
// #include <unistd.h>

#include "periphery/serial.h"

#include <cstring>
#include <iostream>

using namespace std::chrono_literals;

class AsrSdmControllerNode : public rclcpp::Node
{
public:
  AsrSdmControllerNode() : Node("asr_sdm_controller"), count_(0)
  {
    serial_ = serial_new();
    uint8_t s_buf[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    /* Open /dev/ttyUSB0 with baudrate 115200, and defaults of 8N1, no flow control */
    if (serial_open(serial_, "/dev/ttyS3", 115200) < 0) {
      fprintf(stderr, "serial_open(): %s\n", serial_errmsg(serial_));
      // exit(1);
    }
    RCLCPP_INFO(this->get_logger(), "send");
    /* Write to the serial port */
    if (serial_write(serial_, s_buf, sizeof(s_buf)) < 0) {
      fprintf(stderr, "serial_write(): %s\n", serial_errmsg(serial_));
      // exit(1);
    }
    RCLCPP_INFO(this->get_logger(), "Start");
    pub_ros_info_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_hardware_ =
      this->create_wall_timer(1000ms, std::bind(&AsrSdmControllerNode::timer_callback, this));
    timer_imu_ =
      this->create_wall_timer(5ms, std::bind(&AsrSdmControllerNode::timer_imu_callback, this));
  }

private:
  void timer_callback()
  {
    // RCLCPP_INFO("Publishing");
    RCLCPP_INFO(this->get_logger(), "Publishing");
    // const uint8_t len = 8;
    // uint8_t buf[len] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    // can_->sendMsgBuf(1, 0, len, buf);
    // can_->mcp2515_send(0x100, buf, len);
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

  // amp::MCP_CAN::Ptr can_;
  serial_t * serial_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsrSdmControllerNode>());
  rclcpp::shutdown();
  return 0;
}
