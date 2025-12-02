#include <asr_sdm_vio_ros2/backend_factory.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <svo/imu_handler.h>
#include <svo/backend/backend_optimizer.h>
#include <svo/backend/backend_interface.h>
#include <svo/backend/graph_manager.h>

namespace svo {
namespace backend_factory {

namespace {
  template<typename T>
  T declare_and_get(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& default_value) {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    T v{};
    node->get_parameter(name, v);
    if constexpr (std::is_same<T, std::string>::value) {
      if (v.empty()) return default_value;
    }
    return v;
  }
}

BackendInterface::Ptr makeBackend(const rclcpp::Node::SharedPtr& node)
{
  GraphManagerOptions graph_manager_options;
  graph_manager_options.reproj_error_noise =
      declare_and_get<double>(node, "ba_reproj_error_noise", 1.0/370.0);
  graph_manager_options.smart_reproj_threshold =
      declare_and_get<double>(node, "ba_smart_reproj_threshold", 3.0/370.0);
  graph_manager_options.min_parallax_thresh =
      declare_and_get<double>(node, "ba_min_parallax_deg", 5.0)/180.0*M_PI;
  graph_manager_options.trace_tracks =
      declare_and_get<bool>(node, "ba_trace_tracks", false);
  graph_manager_options.use_robust_px_noise =
      declare_and_get<bool>(node, "ba_use_robust_px_noise", false);
  graph_manager_options.init_pos_sigma =
      declare_and_get<double>(node, "ba_init_pos_sigma_meter", 0.001);
  graph_manager_options.init_roll_pitch_sigma =
      declare_and_get<double>(node, "ba_init_roll_pitch_sigma_degree", 45.0)/180.0*M_PI;
  graph_manager_options.init_yaw_sigma =
      declare_and_get<double>(node, "ba_init_yaw_sigma_degree", 2.0)/180.0*M_PI;

  OptimizerBackendOptions backend_options;
  backend_options.verbose =
      declare_and_get<bool>(node, "ba_verbose", false);
  backend_options.max_iterations_per_update =
      declare_and_get<int>(node, "ba_max_iterations_per_update", 10);
  backend_options.output_errors =
      declare_and_get<bool>(node, "ba_output_errors", false);
  std::string default_trace_dir;
  try {
    default_trace_dir = ament_index_cpp::get_package_share_directory("asr_sdm_vio_ros2") + std::string("/trace");
  } catch (...) {
    default_trace_dir = std::string("/tmp/asr_sdm_vio_trace");
  }
  backend_options.trace_dir =
      declare_and_get<std::string>(node, "trace_dir", default_trace_dir);
  backend_options.isam_relinearize_thresh =
      declare_and_get<double>(node, "isam_relinearize_thresh", 0.1);
  backend_options.isam_relinearize_skip =
      declare_and_get<int>(node, "isam_relinearize_skip", 1);
  backend_options.isam_wildfire_thresh =
      declare_and_get<double>(node, "isam_wildfire_thresh", 0.001);
  backend_options.isam_detailed_results =
      declare_and_get<bool>(node, "isam_detailed_results", false);

  BackendInterfaceOptions ba_interface_options;
  ba_interface_options.trace_dir = backend_options.trace_dir;
  ba_interface_options.isam_wait_time_ms =
      declare_and_get<int>(node, "isam_wait_time_ms", 2);
  ba_interface_options.use_smart_factors =
      declare_and_get<bool>(node, "ba_use_smart_factors", false);
  ba_interface_options.add_imu_factors =
      declare_and_get<bool>(node, "ba_add_imu_factors", false);
  ba_interface_options.min_num_obs =
      declare_and_get<int>(node, "ba_min_num_obs", 2);
  ba_interface_options.n_frames_in_init_ba =
      declare_and_get<int>(node, "ba_n_frames_in_init_ba", 8);

  BackendInterface::Ptr ba_interface =
      std::make_shared<BackendInterface>(
        ba_interface_options, graph_manager_options, backend_options);

  if (declare_and_get<bool>(node, "ba_parallelized", false))
    ba_interface->startThread();

  return ba_interface;
}

} // namespace backend_factory
} // namespace svo
