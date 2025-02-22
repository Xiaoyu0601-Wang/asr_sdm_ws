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

/* mraa headers */
#include "mraa/common.hpp"
#include "mraa/spi.hpp"

// #include "usr_sdm_controller/mcp_can.hpp"

// #define SPI_PORT 3
// #define SPI_CS   0
// #define SPI_FREQUENCY   1000000

using namespace std::chrono_literals;

class ASRSDM : public rclcpp::Node
{
  public:
    ASRSDM()
    : Node("usr_sdm_controller")
    , count_(0)
    // , spi_(SPI_PORT, SPI_CS)
    {
      // can.reset(new amp::ASRSDM::CANProtocol);
      // can->interfaceSetup();can_
    	// can_.reset(new MCP_CAN);
    	// can_->initCAN(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
      spi_.frequency(SPI_FREQUENCY);

    	pub_ros_info_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    	timer_hardware_ = this->create_wall_timer(1000ms, std::bind(&ASRSDM::timer_callback, this));
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

    rclcpp::TimerBase::SharedPtr timer_hardware_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ros_info_;
    size_t count_;

    // mraa::Spi spi_;

    // amp::ASRSDM::CANProtocol::Ptr can;
    // amp::MCP_CAN::Ptr can_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // mraa::Spi spi_(3, 0);

  rclcpp::spin(std::make_shared<ASRSDM>());
  rclcpp::shutdown();
  return 0;
}
