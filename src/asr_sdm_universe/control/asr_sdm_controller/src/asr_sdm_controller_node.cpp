/**
 * @file asr_sdm_controller_node.cpp
 * @brief 基于论文“基于螺旋驱动机构的蛇形机器人建模与控制”的头部跟踪控制实现。
 *
 * 该节点订阅前端操作指令（Twist），在内部利用论文第4、5节推导的
 * 运动学与前单元跟随控制律，递推计算关节角速度、各螺旋单元中心速度，
 * 再结合被动轮速度约束求解螺旋角速度 θ̇_i，最终发布 `asr_sdm_control_msgs::msg::ControlCmd`。
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include "asr_sdm_control_msgs/msg/control_cmd.hpp"
#include "asr_sdm_control_msgs/msg/unit_cmd.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.141592653589793238462643383279502884;

double clamp(double value, double min_value, double max_value)
{
  return std::min(std::max(value, min_value), max_value);
}

double clamp_abs(double value, double max_abs)
{
  return clamp(value, -max_abs, max_abs);
}

double wrap_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

struct Vec2
{
  double x{0.0};
  double y{0.0};

  Vec2 & operator+=(const Vec2 & other)
  {
    x += other.x;
    y += other.y;
    return *this;
  }
};

Vec2 operator+(Vec2 lhs, const Vec2 & rhs)
{
  lhs += rhs;
  return lhs;
}

Vec2 operator*(double scalar, const Vec2 & v)
{
  return Vec2{scalar * v.x, scalar * v.y};
}

Vec2 orientation_vector(double angle)
{
  return {std::cos(angle), std::sin(angle)};
}

Vec2 orientation_derivative(double angle, double angle_rate)
{
  return {-std::sin(angle) * angle_rate, std::cos(angle) * angle_rate};
}
}  // namespace

class AsrSdmControllerNode : public rclcpp::Node
{
public:
  AsrSdmControllerNode()
  : Node("asr_sdm_controller"),
    link_front_length_(declare_parameter<double>("link_front_length", 0.103)),
    link_rear_length_(declare_parameter<double>("link_rear_length", 0.123)),
    screw_radius_(declare_parameter<double>("screw_radius", 0.075)),
    joint_limit_(declare_parameter<double>("joint_limit", kPi / 2.0)),
    control_rate_hz_(declare_parameter<double>("control_rate_hz", 50.0)),
    cmd_timeout_(declare_parameter<double>("cmd_timeout", 0.5)),
    max_linear_speed_(declare_parameter<double>("max_linear_speed", 0.2)),
    max_angular_speed_(declare_parameter<double>("max_angular_speed", 0.5)),
    screw_velocity_scale_(declare_parameter<double>("screw_velocity_scale", 60.0 / (2.0 * kPi))),
    max_screw_command_(declare_parameter<double>("max_screw_command", 1500.0)),
    joint_angle_scale_(declare_parameter<double>("joint_angle_scale", 1000.0)),
    heartbeat_period_ms_(declare_parameter<int>("heartbeat_period_ms", 1500))
  {
    RCLCPP_INFO(this->get_logger(), "启动 ASR SDM 头部跟踪控制器");

    link_length_ = link_front_length_ + link_rear_length_;

    auto alpha_param = declare_parameter<std::vector<double>>(
      "helix_alpha", {-kPi / 4.0, kPi / 4.0, -kPi / 4.0, kPi / 4.0});
    if (alpha_param.size() != helix_alpha_.size()) {
      RCLCPP_FATAL(
        this->get_logger(),
        "参数 helix_alpha 长度应为 %zu，实际为 %zu，无法启动控制器", helix_alpha_.size(),
        alpha_param.size());
      throw std::runtime_error("helix_alpha size mismatch");
    }
    std::copy(alpha_param.begin(), alpha_param.end(), helix_alpha_.begin());

    auto unit_ids_param =
      declare_parameter<std::vector<int64_t>>("unit_ids", {1, 2, 3, 4});
    if (unit_ids_param.size() != unit_ids_.size()) {
      RCLCPP_FATAL(
        this->get_logger(), "unit_ids 长度应为 %zu，实际为 %zu", unit_ids_.size(),
        unit_ids_param.size());
      throw std::runtime_error("unit_ids size mismatch");
    }
    for (size_t i = 0; i < unit_ids_param.size(); ++i) {
      unit_ids_[i] = static_cast<int32_t>(unit_ids_param[i]);
    }

    auto joint_map_param =
      declare_parameter<std::vector<int64_t>>("joint_index_map", {0, 1, 2, -1});
    if (joint_map_param.size() != joint_index_map_.size()) {
      RCLCPP_FATAL(
        this->get_logger(), "joint_index_map 长度应为 %zu，实际为 %zu", joint_index_map_.size(),
        joint_map_param.size());
      throw std::runtime_error("joint_index_map size mismatch");
    }
    for (size_t i = 0; i < joint_map_param.size(); ++i) {
      joint_index_map_[i] = static_cast<int>(joint_map_param[i]);
    }

    pub_heartbeat_ =
      this->create_publisher<std_msgs::msg::String>("~/output/controller/heartbeat", 1);
    pub_control_cmd_ = this->create_publisher<asr_sdm_control_msgs::msg::ControlCmd>(
      "~/output/control_cmd", rclcpp::SensorDataQoS{});

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "~/input/cmd_vel", rclcpp::SystemDefaultsQoS(),
      std::bind(&AsrSdmControllerNode::cmd_vel_callback, this, std::placeholders::_1));

    auto heartbeat_period = std::chrono::milliseconds(heartbeat_period_ms_);
    timer_heartbeat_ =
      this->create_wall_timer(heartbeat_period, std::bind(&AsrSdmControllerNode::timer_heartbeat, this));

    auto control_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / control_rate_hz_));
    last_update_time_ = this->now();
    timer_robot_control_ = this->create_wall_timer(
      control_period, std::bind(&AsrSdmControllerNode::timer_controller, this));
  }

private:
  struct TwistCommand
  {
    double linear{0.0};
    double angular{0.0};
    rclcpp::Time stamp;
  };

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_cmd_.linear = clamp_abs(msg->linear.x, max_linear_speed_);
    latest_cmd_.angular = clamp_abs(msg->angular.z, max_angular_speed_);
    latest_cmd_.stamp = this->now();
  }

  TwistCommand get_active_command(const rclcpp::Time & now) const
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    TwistCommand cmd = latest_cmd_;
    if (!cmd.stamp.nanoseconds() ||
        (now - cmd.stamp).seconds() > cmd_timeout_) {
      cmd.linear = 0.0;
      cmd.angular = 0.0;
    }
    return cmd;
  }

  void timer_heartbeat()
  {
    auto message = std_msgs::msg::String();
    message.data = "asr_sdm_controller alive";
    pub_heartbeat_->publish(message);
  }

  void timer_controller()
  {
    const auto now = this->now();
    const double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;
    if (dt <= 0.0) {
      return;
    }

    const TwistCommand cmd = get_active_command(now);
    integrate_state(cmd.linear, cmd.angular, dt);
    publish_control(now);
  }

  void integrate_state(double v1, double omega1, double dt)
  {
    // 头部位置与姿态（式(7)）
    x_p_ += (-v1 * std::cos(psi_p_)) * dt;
    y_p_ += (-v1 * std::sin(psi_p_)) * dt;
    psi_p_ = wrap_angle(psi_p_ + omega1 * dt);

    // 前单元跟随控制律（式(21)-(23)）
    double phi_dot1 =
      -(2.0 / link_length_) * (v1 * std::sin(phi_[0]) + link_length_ * omega1 * std::cos(phi_[0])) -
      omega1;
    double bar_v2 = v1 * std::cos(phi_[0]) - link_length_ * omega1 * std::sin(phi_[0]);

    double phi_dot2 =
      -(2.0 / link_length_) * bar_v2 * std::sin(phi_[1]) -
      (omega1 + phi_dot1) * (std::cos(phi_[1]) + 1.0);
    double bar_v3 =
      bar_v2 * std::cos(phi_[1]) -
      0.5 * link_length_ * (omega1 + phi_dot1) * std::sin(phi_[1]);

    double phi_dot3 =
      -(2.0 / link_length_) * bar_v3 * std::sin(phi_[2]) -
      (omega1 + phi_dot1 + phi_dot2) * (std::cos(phi_[2]) + 1.0);

    phi_[0] = clamp(phi_[0] + phi_dot1 * dt, -joint_limit_, joint_limit_);
    phi_[1] = clamp(phi_[1] + phi_dot2 * dt, -joint_limit_, joint_limit_);
    phi_[2] = clamp(phi_[2] + phi_dot3 * dt, -joint_limit_, joint_limit_);

    last_phi_dot_ = {phi_dot1, phi_dot2, phi_dot3};
    latest_command_ = {v1, omega1};
  }

  void publish_control(const rclcpp::Time & stamp)
  {
    const size_t num_units = unit_ids_.size();
    std::array<double, 4> psi_units{};
    std::array<double, 4> psi_dot_units{};

    psi_units[0] = psi_p_;
    psi_dot_units[0] = latest_command_.second;
    for (size_t i = 1; i < num_units; ++i) {
      psi_units[i] = wrap_angle(psi_units[i - 1] + phi_[i - 1]);
      psi_dot_units[i] = psi_dot_units[i - 1] + last_phi_dot_[i - 1];
    }

    std::array<Vec2, 5> joint_pos{};
    std::array<Vec2, 5> joint_vel{};
    std::array<Vec2, 4> center_pos{};
    std::array<Vec2, 4> center_vel{};

    joint_pos[0] = {x_p_, y_p_};
    joint_vel[0] = {-latest_command_.first * std::cos(psi_p_),
                    -latest_command_.first * std::sin(psi_p_)};

    for (size_t i = 0; i < num_units; ++i) {
      const Vec2 dir = orientation_vector(psi_units[i]);
      const Vec2 dir_dot = orientation_derivative(psi_units[i], psi_dot_units[i]);

      center_pos[i] = joint_pos[i] + link_front_length_ * dir;
      center_vel[i] = joint_vel[i] + link_front_length_ * dir_dot;

      joint_pos[i + 1] = joint_pos[i] + link_length_ * dir;
      joint_vel[i + 1] = joint_vel[i] + link_length_ * dir_dot;
    }

    std::array<double, 4> theta_dot{};
    for (size_t i = 0; i < num_units; ++i) {
      const double denom = screw_radius_ * std::sin(helix_alpha_[i]);
      if (std::abs(denom) < 1e-6) {
        theta_dot[i] = 0.0;
        continue;
      }
      theta_dot[i] = -(
                        center_vel[i].x * std::cos(helix_alpha_[i] + psi_units[i]) +
                        center_vel[i].y * std::sin(helix_alpha_[i] + psi_units[i])) /
                     denom;
    }

    asr_sdm_control_msgs::msg::ControlCmd cmd_msg;
    cmd_msg.header.stamp = stamp;
    cmd_msg.header.frame_id = "asr_sdm/base";

    for (size_t i = 0; i < num_units; ++i) {
      asr_sdm_control_msgs::msg::UnitCmd unit_cmd;
      unit_cmd.unit_id = unit_ids_[i];

      const double screw_cmd =
        clamp_abs(theta_dot[i] * screw_velocity_scale_, max_screw_command_);
      unit_cmd.screw1_vel = static_cast<int32_t>(std::round(screw_cmd));
      unit_cmd.screw2_vel = static_cast<int32_t>(std::round(screw_cmd));

      const int joint_index = joint_index_map_[i];
      double joint_angle = 0.0;
      if (joint_index >= 0 && static_cast<size_t>(joint_index) < phi_.size()) {
        joint_angle = phi_[joint_index];
      }
      const double joint_cmd =
        clamp_abs(joint_angle * joint_angle_scale_, joint_limit_ * joint_angle_scale_);
      unit_cmd.joint1_angle = static_cast<int32_t>(std::round(joint_cmd));
      unit_cmd.joint2_angle = 0;

      cmd_msg.units_cmd.push_back(unit_cmd);
    }

    pub_control_cmd_->publish(cmd_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_heartbeat_;
  rclcpp::TimerBase::SharedPtr timer_robot_control_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_heartbeat_;
  rclcpp::Publisher<asr_sdm_control_msgs::msg::ControlCmd>::SharedPtr pub_control_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  mutable std::mutex cmd_mutex_;
  mutable TwistCommand latest_cmd_;

  double link_front_length_{0.0};
  double link_rear_length_{0.0};
  double link_length_{0.0};
  double screw_radius_{0.0};
  double joint_limit_{0.0};
  double control_rate_hz_{0.0};
  double cmd_timeout_{0.0};
  double max_linear_speed_{0.0};
  double max_angular_speed_{0.0};
  double screw_velocity_scale_{0.0};
  double max_screw_command_{0.0};
  double joint_angle_scale_{0.0};
  int heartbeat_period_ms_{0};

  std::array<double, 4> helix_alpha_{};
  std::array<int32_t, 4> unit_ids_{};
  std::array<int, 4> joint_index_map_{};

  double x_p_{0.0};
  double y_p_{0.0};
  double psi_p_{0.0};
  std::array<double, 3> phi_{{0.0, 0.0, 0.0}};
  std::array<double, 3> last_phi_dot_{{0.0, 0.0, 0.0}};
  std::pair<double, double> latest_command_{0.0, 0.0};
  rclcpp::Time last_update_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsrSdmControllerNode>());
  rclcpp::shutdown();
  return 0;
}
