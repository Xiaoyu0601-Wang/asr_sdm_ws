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

// #include "asr_sdm_hardware/mcp_can.hpp"

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
// #include <linux/spi/spidev.h>
#include <cstring>

#include "periphery/spi.h"

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
  AsrSdmHardwareNode()
    : Node("asrsdm_hardware")
    , count_(0)
    , spi_()
    {
      spi_ = spi_new();

      /* Open spidev3.0 with mode 0 and max speed 1MHz */
      if (spi_open(spi_, "/dev/spidev3.0", 0, 100000) < 0)
      {
          fprintf(stderr, "spi_open(): %s\n", spi_errmsg(spi_));
      }
      uint8_t tx_buffer[] = {0x03, 0x0F, 0x00};
      uint8_t rx_buffer[3] = {0};
      if (spi_transfer(spi_, tx_buffer, rx_buffer, sizeof(tx_buffer)) < 0)
      {
        fprintf(stderr, "spi_transfer(): %s\n", spi_errmsg(spi_));
      }
      std::cout << "Received Data: 0x" << std::hex << (int)rx_buffer[0]
                                                   << (int)rx_buffer[1]
                                                   << (int)rx_buffer[2]
                                                   << std::endl;
      spi_close(spi_);
      spi_free(spi_);

    	pub_ros_info_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    	timer_hardware_ = this->create_wall_timer(1000ms, std::bind(&AsrSdmHardwareNode::timer_callback, this));
    	timer_imu_ = this->create_wall_timer(500ms, std::bind(&AsrSdmHardwareNode::timer_imu_callback, this));
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
    // uint8_t spi_tx_data[8];
    // uint8_t spi_rx_data[8];
    // int spi_fd;
    spi_t *spi_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsrSdmHardwareNode>());
  rclcpp::shutdown();
  return 0;
}
