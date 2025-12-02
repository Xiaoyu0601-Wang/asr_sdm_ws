#include <chrono>
#include <memory>
#include <string>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

class MinimalVioFrontend : public rclcpp::Node {
public:
  MinimalVioFrontend() : rclcpp::Node("minimal_vio_frontend"), img_count_(0), imu_count_(0) {
    // Parameters
    img_topic_ = this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu/data");
    report_period_ = this->declare_parameter<double>("report_period", 1.0);

    // Subscriptions
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      img_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr) { img_count_++; });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr) { imu_count_++; });

    // Periodic report
    const auto period = std::chrono::duration<double>(report_period_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MinimalVioFrontend::report, this));

    RCLCPP_INFO(get_logger(), "Minimal VIO Frontend started. Subscribing to %s and %s",
                img_topic_.c_str(), imu_topic_.c_str());
  }

private:
  void report() {
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                         "Image msgs: %zu, IMU msgs: %zu (last 1s approx)",
                         img_count_.exchange(0), imu_count_.exchange(0));
  }

  // Params
  std::string img_topic_;
  std::string imu_topic_;
  double report_period_;

  // State
  std::atomic<size_t> img_count_;
  std::atomic<size_t> imu_count_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalVioFrontend>());
  rclcpp::shutdown();
  return 0;
}

