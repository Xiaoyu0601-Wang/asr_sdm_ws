/* standard headers */
#include <fcntl.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

/* ROS2 headers */
#include "asr_sdm_hardware/comm_protocol.hpp"
#include "asr_sdm_hardware/uart2can.hpp"

#include <rclcpp/rclcpp.hpp>

#include "asr_sdm_control_msgs/msg/control_cmd.hpp"
#include "asr_sdm_hardware_msgs/msg/can_frame.hpp"
#include "asr_sdm_hardware_msgs/msg/hardware_cmd.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class AsrSdmHardwareNode : public rclcpp::Node
{
public:
  AsrSdmHardwareNode() : Node("asrsdm_hardware"), control_cmd_received_status_(false)
  {
    // Declare parameters with default values
    this->declare_parameter("uart2can.uart_port", "/tty0");
    this->declare_parameter("uart2can.uart_baudrate", 57600);
    this->declare_parameter("uart2can.can_id_mask", 0x100100);
    this->declare_parameter("uart2can.can_frame_length", 8);
    this->declare_parameter("uart2can.uart_frame_head", 0xAA);
    this->declare_parameter("uart2can.uart_frame_tail", 0xBB);
    this->declare_parameter("imu_wheeltec_n100.uart_port", "/tty1");
    this->declare_parameter("imu_wheeltec_n100.uart_baudrate", 57600);
    // Get parameters
    const std::string uart2can_port =
      this->get_parameter("uart2can.uart_port").get_parameter_value().get<std::string>();
    const uint32_t uart2can_baudrate =
      this->get_parameter("uart2can.uart_baudrate").get_parameter_value().get<uint32_t>();
    const uint32_t uart2can_can_id_mask =
      this->get_parameter("uart2can.can_id_mask").get_parameter_value().get<uint32_t>();
    const uint8_t uart2can_can_frame_length =
      this->get_parameter("uart2can.can_frame_length").get_parameter_value().get<uint8_t>();
    const uint8_t uart2can_uart_frame_head =
      this->get_parameter("uart2can.uart_frame_head").get_parameter_value().get<uint8_t>();
    const uint8_t uart2can_uart_frame_tail =
      this->get_parameter("uart2can.uart_frame_tail").get_parameter_value().get<uint8_t>();
    RCLCPP_INFO(
      this->get_logger(), "UART2CAN Port: %s, Baudrate: %u, CAN_ID: %x", uart2can_port.c_str(),
      uart2can_baudrate, uart2can_can_id_mask);

    const std::string imu_wheeltec_n100_port =
      this->get_parameter("imu_wheeltec_n100.uart_port").get_parameter_value().get<std::string>();
    const uint32_t imu_wheeltec_n100_baudrate =
      this->get_parameter("imu_wheeltec_n100.uart_baudrate").get_parameter_value().get<uint32_t>();
    RCLCPP_INFO(
      this->get_logger(), "IMU Wheeltec N100 Port: %s, Baudrate: %u",
      imu_wheeltec_n100_port.c_str(), imu_wheeltec_n100_baudrate);

    // Initialize UART2CAN instance
    uart2can_.reset(new amp::UART2CAN);
    uart2can_->initModules(
      uart2can_port, uart2can_baudrate, uart2can_can_id_mask, uart2can_can_frame_length,
      uart2can_uart_frame_head, uart2can_uart_frame_tail);

    comm_protocol_ = std::make_unique<amp::CommProtocol>(uart2can_);

    // Declare the topic name parameter with default value
    this->declare_parameter<std::string>("topic_asr_sdm_cmd", "~/input/asr_sdm_cmd");
    // Get the topic name parameter
    std::string topic_asr_sdm_cmd =
      this->get_parameter("topic_asr_sdm_cmd").get_parameter_value().get<std::string>();
    // Initialize publishers and subscribers
    pub_heartbeat_ = this->create_publisher<std_msgs::msg::String>("~/hardware/heartbeat", 1);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("~/hardware/imu", 1);
    sub_control_cmd_ = this->create_subscription<asr_sdm_control_msgs::msg::ControlCmd>(
      topic_asr_sdm_cmd, rclcpp::SensorDataQoS{}.keep_last(1),
      std::bind(&AsrSdmHardwareNode::hardware_control, this, std::placeholders::_1));

    // Initialize timers
    timer_heartbeat_ =
      this->create_wall_timer(1000ms, std::bind(&AsrSdmHardwareNode::timer_heartbeat, this));
    timer_hardware_ctrl_ =
      this->create_wall_timer(1000ms, std::bind(&AsrSdmHardwareNode::timer_hardware_ctrl, this));

    // // CAN test
    // std::vector<uint8_t> can_data(8);
    // for (size_t i = 0; i < 8; ++i) {
    //   can_data[i] = static_cast<uint8_t>(i);
    // }
    // uart2can_->sendMsg(0x100101, 0, 0x80, 8, can_data.data());
  }

private:
  void hardware_control(const asr_sdm_control_msgs::msg::ControlCmd::SharedPtr msg)
  {
    auto msg_control_cmd = *msg;
    control_cmd_received_status_ = true;

    for (uint8_t i = 0; i < msg_control_cmd.units_cmd.size(); i++) {
      std::vector<int32_t> unit_cmd;
      unit_cmd.push_back(msg_control_cmd.units_cmd[i].unit_id);
      unit_cmd.push_back(msg_control_cmd.units_cmd[i].screw1_vel);
      unit_cmd.push_back(msg_control_cmd.units_cmd[i].screw2_vel);
      unit_cmd.push_back(msg_control_cmd.units_cmd[i].joint1_angle);
      unit_cmd.push_back(msg_control_cmd.units_cmd[i].joint2_angle);
      proceed_control_cmd_.push_back(unit_cmd);
    }
  }

  void timer_heartbeat()
  {
    /** Test */
    // control_cmd_received_status_ = true;

    // std::vector<int32_t> unit_cmd;
    // unit_cmd.push_back(0x00010101);
    // unit_cmd.push_back(0x00000102);
    // unit_cmd.push_back(0x00000304);
    // unit_cmd.push_back(0x00000506);
    // unit_cmd.push_back(0x00000708);
    // proceed_control_cmd_.push_back(unit_cmd);

    auto message = std_msgs::msg::String();
    message.data = "Hello ROS2: " + std::to_string(1);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    pub_heartbeat_->publish(message);
  }

  void timer_imu()
  {
    auto message = sensor_msgs::msg::Imu();
    // Fill the IMU message with data
    pub_imu_->publish(message);
  }

  void timer_hardware_ctrl()
  {
    /* Send commands to hardware */
    if (control_cmd_received_status_ == true) {
      // Send commands to hardware
      for (uint8_t i = 0; i < proceed_control_cmd_.size(); ++i) {
        // std::cout << "proceed_control_cmd_[" << i << "]: ";
        std::vector<uint8_t> uart2can_msg;
        comm_protocol_->setActuatorCMD(
          amp::CommProtocol::REGISTER_SCREW_VEL, amp::CommProtocol::WRITE, proceed_control_cmd_,
          &uart2can_msg);
        // uart2can_->sendMsg(uart2can_msg.data(), 0, amp::UART2CAN::CAN_EXT_FRAME, 8);
      }

      // Clear the command buffer and reset the flag
      proceed_control_cmd_.clear();
      control_cmd_received_status_ = false;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_heartbeat_;
  rclcpp::TimerBase::SharedPtr timer_imu_;
  rclcpp::TimerBase::SharedPtr timer_hardware_ctrl_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_heartbeat_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Subscription<asr_sdm_control_msgs::msg::ControlCmd>::SharedPtr sub_control_cmd_;

  amp::UART2CAN::Ptr uart2can_;
  // asr_sdm_control_msgs::msg::ControlCmd msg_robot_cmd_;
  std::vector<std::vector<int32_t>> proceed_control_cmd_;
  // asr_sdm_hardware_msgs::msg::HardwareCmd msg_hardware_cmd_;
  bool control_cmd_received_status_;
  std::unique_ptr<amp::CommProtocol> comm_protocol_;
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
