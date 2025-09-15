#include "rclcpp/rclcpp.hpp"

#include "asr_sdm_control_msgs/msg/control_cmd.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

class JoystickTeleop : public rclcpp::Node
{
public:
  JoystickTeleop() : Node("joystick_teleop")
  {
    // Declare parameters
    this->declare_parameter<int>("linear_axis", 1);   // left joystick up/down
    this->declare_parameter<int>("angular_axis", 0);  // left joystick left/right
    this->declare_parameter<double>("linear_scale", 0.5);
    this->declare_parameter<double>("angular_scale", 1.0);

    // Create subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoystickTeleop::joy_callback, this, std::placeholders::_1));

    // Create publisher
    pub_control_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Joystick Teleop Node started.");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto twist = geometry_msgs::msg::Twist();

    int linear_axis = this->get_parameter("linear_axis").as_int();
    int angular_axis = this->get_parameter("angular_axis").as_int();
    double linear_scale = this->get_parameter("linear_scale").as_double();
    double angular_scale = this->get_parameter("angular_scale").as_double();

    if (msg->axes.size() > static_cast<size_t>(linear_axis)) {
      twist.linear.x = msg->axes[linear_axis] * linear_scale;
    }
    if (msg->axes.size() > static_cast<size_t>(angular_axis)) {
      twist.angular.z = msg->axes[angular_axis] * angular_scale;
    }

    pub_control_cmd_->publish(twist);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_control_cmd_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoystickTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}