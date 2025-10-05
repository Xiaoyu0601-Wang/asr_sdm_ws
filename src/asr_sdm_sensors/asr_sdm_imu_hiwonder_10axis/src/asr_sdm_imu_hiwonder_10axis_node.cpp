#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <serial/serial.h>  // Use https://github.com/wjwwood/serial or your preferred serial lib

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class IMUDriverNode : public rclcpp::Node
{
public:
  IMUDriverNode(const std::string & port_name) : Node("imu_driver_node"), port_name_(port_name)
  {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    imu_msg_.header.frame_id = "imu_link";
    driver_thread_ = std::thread(&IMUDriverNode::driver_loop, this);
  }

  ~IMUDriverNode() override
  {
    running_ = false;
    if (driver_thread_.joinable()) driver_thread_.join();
  }

private:
  void driver_loop()
  {
    serial::Serial ser;
    try {
      ser.setPort(port_name_);
      ser.setBaudrate(9600);
      serial::Timeout to = serial::Timeout::simpleTimeout(500);
      ser.setTimeout(to);
      ser.open();
      if (ser.isOpen()) {
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully...");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Serial port opening failure: %s", e.what());
      return;
    }

    std::vector<uint8_t> buff;
    while (rclcpp::ok() && running_) {
      if (ser.available()) {
        size_t n = ser.available();
        std::vector<uint8_t> data(n);
        ser.read(data, n);
        for (size_t i = 0; i < n; ++i) {
          if (handle_serial_data(data[i])) {
            publish_imu();
          }
        }
      }
      std::this_thread::sleep_for(1ms);
    }
    ser.close();
  }

  bool handle_serial_data(uint8_t raw_data)
  {
    static int key = 0;
    static std::vector<uint8_t> buff(11, 0);
    static std::vector<double> acceleration(3, 0.0);
    static std::vector<double> angularVelocity(3, 0.0);
    static std::vector<double> angle_degree(3, 0.0);
    bool angle_flag = false;

    buff[key] = raw_data;
    key++;
    if (buff[0] != 0x55) {
      key = 0;
      return false;
    }
    if (key < 11) {
      return false;
    } else {
      if (buff[1] == 0x51 && check_sum(buff, 10)) {
        auto acc = hex_to_short(buff, 2);
        for (int i = 0; i < 3; ++i) acceleration[i] = acc[i] / 32768.0 * 16 * 9.8;
      } else if (buff[1] == 0x52 && check_sum(buff, 10)) {
        auto gyro = hex_to_short(buff, 2);
        for (int i = 0; i < 3; ++i) angularVelocity[i] = gyro[i] / 32768.0 * 2000 * M_PI / 180.0;
      } else if (buff[1] == 0x53 && check_sum(buff, 10)) {
        auto angle = hex_to_short(buff, 2);
        for (int i = 0; i < 3; ++i) angle_degree[i] = angle[i] / 32768.0 * 180.0;
        angle_flag = true;
      }
      key = 0;
      if (angle_flag) {
        accel_ = acceleration;
        gyro_ = angularVelocity;
        angle_ = angle_degree;
      }
      return angle_flag;
    }
  }

  std::vector<int16_t> hex_to_short(const std::vector<uint8_t> & data, size_t start)
  {
    std::vector<int16_t> out(4);
    for (size_t i = 0; i < 4; ++i) {
      out[i] = static_cast<int16_t>(data[start + 2 * i] | (data[start + 2 * i + 1] << 8));
    }
    return out;
  }

  bool check_sum(const std::vector<uint8_t> & data, size_t check_index)
  {
    uint8_t sum = 0;
    for (size_t i = 0; i < check_index; ++i) sum += data[i];
    return (sum & 0xFF) == data[check_index];
  }

  void publish_imu()
  {
    imu_msg_.header.stamp = this->get_clock()->now();
    imu_msg_.linear_acceleration.x = accel_[0];
    imu_msg_.linear_acceleration.y = accel_[1];
    imu_msg_.linear_acceleration.z = accel_[2];
    imu_msg_.angular_velocity.x = gyro_[0];
    imu_msg_.angular_velocity.y = gyro_[1];
    imu_msg_.angular_velocity.z = gyro_[2];

    auto angle_radian = angle_;
    for (auto & v : angle_radian) v = v * M_PI / 180.0;
    auto q = get_quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2]);
    imu_msg_.orientation.x = q[0];
    imu_msg_.orientation.y = q[1];
    imu_msg_.orientation.z = q[2];
    imu_msg_.orientation.w = q[3];

    imu_pub_->publish(imu_msg_);
  }

  std::vector<double> get_quaternion_from_euler(double roll, double pitch, double yaw)
  {
    double qx =
      sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    double qy =
      cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    double qz =
      cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    double qw =
      cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    return {qx, qy, qz, qw};
  }

  std::string port_name_;
  std::thread driver_thread_;
  std::atomic<bool> running_{true};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  sensor_msgs::msg::Imu imu_msg_;
  std::vector<double> accel_{0, 0, 0};
  std::vector<double> gyro_{0, 0, 0};
  std::vector<double> angle_{0, 0, 0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMUDriverNode>("/dev/imu_usb");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}