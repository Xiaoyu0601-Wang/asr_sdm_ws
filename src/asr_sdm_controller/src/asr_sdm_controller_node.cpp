/* standard headers */
#include <chrono>
#include <string>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

/* ROS2 headers */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "asr_sdm_controller/mcp_can.hpp"

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>

// #define SPI_PORT 3
// #define SPI_CS   0
// #define SPI_FREQUENCY   1000000

// #define SPI_DEVICE "/dev/spidev3.0"
// // #define SPI_SPEED 500000  // 500kHz
// #define GPIO_CHIP "/dev/gpiochip0"
// #define GPIO_PIN 24

using namespace std::chrono_literals;

class AsrSdmControllerNode : public rclcpp::Node
{
  public:
  AsrSdmControllerNode()
    : Node("asr_sdm_controller")
    , count_(0)
    {
    	can_.reset(new amp::MCP_CAN);
    	can_->initCAN(MCP_ANY, CAN_500KBPS, MCP_8MHZ);

    	pub_ros_info_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    	timer_hardware_ = this->create_wall_timer(1000ms, std::bind(&AsrSdmControllerNode::timer_callback, this));
    	timer_imu_ = this->create_wall_timer(5ms, std::bind(&AsrSdmControllerNode::timer_imu_callback, this));
    }

  private:
    void timer_callback()
    {
      // RCLCPP_INFO("Publishing");
    	RCLCPP_INFO(this->get_logger(), "Publishing");
//      auto message = std_msgs::msg::String();
//      message.data = "Hello, world! " + std::to_string(count_++);
//      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//      publisher_->publish(message);
    }

    void timer_imu_callback()
    {
      
    }

    size_t count_;

    rclcpp::TimerBase::SharedPtr timer_hardware_;
    rclcpp::TimerBase::SharedPtr timer_imu_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ros_info_;
    
    amp::MCP_CAN::Ptr can_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsrSdmControllerNode>());
  rclcpp::shutdown();
  return 0;
}
