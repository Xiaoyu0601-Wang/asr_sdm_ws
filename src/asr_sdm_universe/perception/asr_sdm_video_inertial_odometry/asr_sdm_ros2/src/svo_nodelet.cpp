#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <asr_sdm_ros2/svo_interface.h>
#include <asr_sdm_ros2/svo_nodelet.h>
#include <yaml-cpp/yaml.h>

namespace
{

inline void set_param_from_yaml(rclcpp::Node & node, const std::string & key, const YAML::Node & v)
{
  if (!v.IsScalar()) {
    // Only scalar params are used in current ROS1-style yaml files in this repo.
    return;
  }

  // Try bool
  try {
    const bool b = v.as<bool>();
    node.declare_parameter<bool>(key, b);
    node.set_parameter(rclcpp::Parameter(key, b));
    return;
  } catch (...) {
  }

  // Try int
  try {
    const int i = v.as<int>();
    node.declare_parameter<int>(key, i);
    node.set_parameter(rclcpp::Parameter(key, i));
    return;
  } catch (...) {
  }

  // Try double
  try {
    const double d = v.as<double>();
    node.declare_parameter<double>(key, d);
    node.set_parameter(rclcpp::Parameter(key, d));
    return;
  } catch (...) {
  }

  // Fallback string
  const std::string s = v.as<std::string>();
  node.declare_parameter<std::string>(key, s);
  node.set_parameter(rclcpp::Parameter(key, s));
}

inline void load_ros1_style_yaml_params(rclcpp::Node & node, const std::string & yaml_path)
{
  if (yaml_path.empty()) return;

  YAML::Node root = YAML::LoadFile(yaml_path);
  if (!root.IsMap()) return;

  for (auto it = root.begin(); it != root.end(); ++it) {
    const std::string key = it->first.as<std::string>();
    const YAML::Node val = it->second;
    set_param_from_yaml(node, key, val);
  }
}

}  // namespace

namespace svo
{

SvoNodelet::SvoNodelet(const rclcpp::NodeOptions & options) : rclcpp::Node("svo", options)
{
  RCLCPP_INFO(get_logger(), "Initialized %s component.", get_name());

  const std::string default_param_file =
    ament_index_cpp::get_package_share_directory("asr_sdm_ros2") + "/param/vio_mono.yaml";

  const std::string param_file =
    this->declare_parameter<std::string>("param_file", default_param_file);

  // Load ROS1-style YAML into ROS2 parameter system (strictly keep original key names).
  try {
    load_ros1_style_yaml_params(*this, param_file);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load param_file '%s': %s", param_file.c_str(), e.what());
  }

  // Keep original parameter name/semantics.
  const bool pipeline_is_stereo = this->declare_parameter<bool>("pipeline_is_stereo", false);
  const svo::PipelineType type =
    pipeline_is_stereo ? svo::PipelineType::kStereo : svo::PipelineType::kMono;

  // Preserve original control flow (nodelet onInit -> ctor).
  svo_interface_.reset(new SvoInterface(type, this->shared_from_this()));

  if (svo_interface_->imu_handler_) svo_interface_->subscribeImu();
  svo_interface_->subscribeImage();
  svo_interface_->subscribeRemoteKey();
}

SvoNodelet::~SvoNodelet()
{
  RCLCPP_INFO(get_logger(), "SVO quit");
  if (svo_interface_) svo_interface_->quit_ = true;
}

}  // namespace svo

RCLCPP_COMPONENTS_REGISTER_NODE(svo::SvoNodelet)
