#ifndef WHEELTEC_N100_HPP_
#define WHEELTEC_N100_HPP_

#include "serial/serial.h"
#include "tf2/transform_datatypes.h"
#include "wheeltec_n100_imu/fdilink_data_struct.h"

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define IMU_LEN 0x38     // 56
#define AHRS_LEN 0x30    // 48
#define INSGPS_LEN 0x54  // 84
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295
// Sample covariance
#define IMU_MAG_COV {0.01, 0.01, 0.01}
#define IMU_GYRO_COV {0.01, 0.01, 0.01}
#define IMU_ACCEL_COV {0.05, 0.05, 0.05}

namespace amp
{
struct WheeltecN100
{
  uint8_t m_nExtFlg;  // Identifier Type
                      // Extended (29 bit) or Standard (11 bit)
  uint32_t m_nID;     // CAN ID
  uint8_t m_nDlc;     // Data Length Code
  uint8_t tx_buf[8];
  uint8_t rx_buf[8];
  uint8_t m_nDta[FRAME_HEAD_LENGTH + MAX_FRAME_DATA_LENGTH];
  uint8_t m_nRtr;
  uint8_t frame_head;
  uint8_t frame_tail;
};

class WheeltecN100
{
public:
  WheeltecN100()
  {
    // Declare parameters
    this->declare_parameter("debug", false);
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("serial_baud", 921600);
    this->declare_parameter("serial_timeout", 20);
    this->declare_parameter("device_type", 1);
    this->declare_parameter("frist_sn", false);
    this->declare_parameter("imu_topic", "imu");
    this->declare_parameter("mag_pose_2d_topic", "magnetic_pose_2d");
    this->declare_parameter("imu_trueEast_topic", "imu_trueEast");
    this->declare_parameter("mag_topic", "magnetic_field");
    this->declare_parameter("imu_frame", "imu");
    this->declare_parameter("yaw_offset", -2.094);
    this->declare_parameter("mag_offset_x", 0.0);
    this->declare_parameter("mag_offset_y", 0.0);
    this->declare_parameter("mag_offset_z", 0.0);
    this->declare_parameter("imu_mag_covVec", std::vector<double>(IMU_MAG_COV));
    this->declare_parameter("imu_gyro_covVec", std::vector<double>(IMU_GYRO_COV));
    this->declare_parameter("imu_accel_covVec", std::vector<double>(IMU_ACCEL_COV));
    // Get parameters
    if_debug_ = this->get_parameter("debug").as_bool();
    serial_port_ = this->get_parameter("serial_port").as_string();
    serial_baud_ = this->get_parameter("serial_baud").as_int();
    serial_timeout_ = this->get_parameter("serial_timeout").as_int();
    device_type_ = this->get_parameter("device_type").as_int();
    frist_sn_ = this->get_parameter("frist_sn").as_bool();
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    mag_pose_2d_topic_ = this->get_parameter("mag_pose_2d_topic").as_string();
    imu_trueEast_topic_ = this->get_parameter("imu_trueEast_topic").as_string();
    mag_topic_ = this->get_parameter("mag_topic").as_string();
    imu_frame_id_ = this->get_parameter("imu_frame").as_string();
    yaw_offset_ = this->get_parameter("yaw_offset").as_double();
    mag_offset_x_ = this->get_parameter("mag_offset_x").as_double();
    mag_offset_y_ = this->get_parameter("mag_offset_y").as_double();
    mag_offset_z_ = this->get_parameter("mag_offset_z").as_double();
    imu_mag_cov = this->get_parameter("imu_mag_covVec").as_double_array();
    imu_gyro_cov = this->get_parameter("imu_gyro_covVec").as_double_array();
    imu_accel_cov = this->get_parameter("imu_accel_covVec").as_double_array();
    q_rot.setRPY(0, 0, yaw_offset_);
    // Initialize serial port
    serial_.setPort(serial_port_);
    serial_.setBaudrate(serial_baud_);
    serial_.setTimeout(serial::Timeout::simpleTimeout(serial_timeout_));
    try {
      serial_.open();
    } catch (serial::IOException & e) {
      RCLCPP_ERROR_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Unable to open serial port " << serial_port_.c_str());
      exit(0);
    }
    if (serial_.isOpen()) {
      RCLCPP_INFO_STREAM_ONCE(
        this->get_logger(), "Initialized Serial port " << serial_port_.c_str());
    } else {
      RCLCPP_ERROR_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Unable to initialize serial port " << serial_port_.c_str());
    }
    // Initialize subscribers and publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    imu_trueEast_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_trueEast_topic_, 10);
    mag_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>(mag_pose_2d_topic_, 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_, 10);
    // Initialize timer
    timer_ = this->create_wall_timer(10ms, std::bind(&WheeltecN100::read_imu, this));
    // Initialize data
    imu_frame_.frame.header.serial_num = 0;
    ahrs_frame_.frame.header.serial_num = 0;
    insgps_frame_.frame.header.serial_num = 0;
    imu_frame_.read_buf.read_msg.resize(IMU_LEN + 1);
    ahrs_frame_.read_buf.read_msg.resize(AHRS_LEN + 1);
    insgps_frame_.read_buf.read_msg.resize(INSGPS_LEN + 1);
    imu_frame_.frame.data.data_buff.resize(IMU_LEN);
    ahrs_frame_.frame.data.data_buff.resize(AHRS_LEN);
    insgps_frame_.frame.data.data_buff.resize(INSGPS_LEN);
    imu_frame_.frame.header.header_crc16_l = 0;
    imu_frame_.frame.header.header_crc16_h = 0;
    ahrs_frame_.frame.header.header_crc16_l = 0;
    ahrs_frame_.frame.header.header_crc16_h = 0;
    insgps_frame_.frame.header.header_crc16_l = 0;
    insgps_frame_.frame.header.header_crc16_h = 0;
    imu_frame_.frame.frame_end = FRAME_END;
    ahrs_frame_.frame.frame_end = FRAME_END;
    insgps_frame_.frame.frame_end = FRAME_END;
    read_sn_ = 0;
    sn_lost_ = 0;
    crc_error_ = 0;
  }
  ~WheeltecN100()
  {
    if (serial_.isOpen()) {
      serial_.close();
      RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Closed Serial port " << serial_port_.c_str());
    }
  }
  // Read IMU data
  void read_imu();
  // CRC16 calculation
  uint16_t CRC16_Table(const uint8_t * data, size_t len);
}

}  // namespace amp

#endif  // WHEELTEC_N100_HPP_
