#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "asr_sdm_vio/msg/camera_measurement.hpp"

using std::placeholders::_1;

class MsckfVioNode : public rclcpp::Node {
public:
  MsckfVioNode() : rclcpp::Node("msckf_vio") {
    // Parameters (will be expanded)
    this->declare_parameter<bool>("publish_tf", true);

    // Subscribers (ROS2 msgs)
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(),
      std::bind(&MsckfVioNode::imuCallback, this, _1));

    feature_sub_ = this->create_subscription<asr_sdm_vio::msg::CameraMeasurement>(
      "features", rclcpp::SensorDataQoS(),
      std::bind(&MsckfVioNode::featureCallback, this, _1));

    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    feature_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "feature_point_cloud", 10);

    RCLCPP_INFO(get_logger(), "MsckfVioNode (ROS2 skeleton) started");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // TODO: Feed into MSCKF when core is ported
    (void)msg;
  }

  void featureCallback(const asr_sdm_vio::msg::CameraMeasurement::SharedPtr msg) {
    // TODO: Feed into MSCKF when core is ported
    (void)msg;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<asr_sdm_vio::msg::CameraMeasurement>::SharedPtr feature_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr feature_cloud_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MsckfVioNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

