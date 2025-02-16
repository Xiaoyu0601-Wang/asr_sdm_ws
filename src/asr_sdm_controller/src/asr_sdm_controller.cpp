/* standard headers */
#include <chrono>
#include <string>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

/* mraa headers */
#include "mraa/common.hpp"
#include "mraa/spi.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// #include "usr_sdm_controller/can_protocol.hpp"

#define SPI_PORT 3

using namespace std::chrono_literals;

class ASRSDM : public rclcpp::Node
{
  public:
    ASRSDM()
    : Node("usr_sdm_controller")
    , count_(0)
    , spi_(SPI_PORT)
    {
    	// can.reset(new amp::ASRSDM::CANProtocol);
    	// can->interfaceSetup();

    	publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    	timer_ = this->create_wall_timer(1000ms, std::bind(&ASRSDM::timer_callback, this));
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

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    mraa::Spi spi_;

    // amp::ASRSDM::CANProtocol::Ptr can;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ASRSDM>());
  rclcpp::shutdown();
  return 0;
}
