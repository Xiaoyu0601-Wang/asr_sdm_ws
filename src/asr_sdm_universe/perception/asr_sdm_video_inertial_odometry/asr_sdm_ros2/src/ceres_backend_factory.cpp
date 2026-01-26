#include <asr_sdm_ros2/ceres_backend_factory.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <asr_sdm_vio/imu_handler.h>
#include <asr_sdm_vio/motion_detector.hpp>

namespace svo {
namespace ceres_backend_factory {

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

MotionDetectorOptions loadMotionDetectorOptions(const rclcpp::Node::SharedPtr& node)
{
  MotionDetectorOptions o;
  o.px_diff_threshold = declare_and_get<double>(node, "zero_motion_px_diff_threshold", 0.5);
  o.ratio_moving_pixels_threshold = declare_and_get<double>(node, "zero_motion_ratio_moving_pixels_threshold", 0.1);
  o.min_number_correspondences = declare_and_get<int>(node, "zero_motion_min_number_correspondences", 5);
  o.max_features_to_check = declare_and_get<int>(node, "zero_motion_max_features_to_check", 100);
  DEBUG_CHECK(o.max_features_to_check>0) << "max_features_to_check must be > 0";
  o.sigma = declare_and_get<double>(node, "zero_motion_sigma", 0.05);
  return o;
}

CeresBackendInterface::Ptr makeBackend(const rclcpp::Node::SharedPtr& node,
                                       const CameraBundlePtr& camera_bundle)
{
  VLOG(1) << "Initialize Backend.";
  CeresBackendOptions backend_options;
  backend_options.verbose = declare_and_get<bool>(node, "ceres_verbose", false);
  backend_options.marginalize = declare_and_get<bool>(node, "ceres_marginalize", true);
  backend_options.num_iterations = static_cast<size_t>(declare_and_get<int>(node, "ceres_num_iterations", 3));
  backend_options.num_threads = static_cast<size_t>(declare_and_get<int>(node, "ceres_num_threads", 2));
  backend_options.num_imu_frames = static_cast<size_t>(declare_and_get<int>(node, "ceres_num_imu_frames", 3));
  backend_options.num_keyframes = static_cast<size_t>(declare_and_get<int>(node, "max_n_kfs", 5));
  backend_options.max_iteration_time = static_cast<double>(declare_and_get<double>(node, "ceres_max_iteration_time", -1.0));
  backend_options.remove_marginalization_term_after_correction_ = declare_and_get<bool>(node, "ceres_remove_marginalization_term_after_loop", false);
  backend_options.recalculate_imu_terms_after_loop = declare_and_get<bool>(node, "ceres_recalculate_imu_terms_after_loop", false);
  backend_options.remove_fixation_min_num_fixed_landmarks_ = static_cast<size_t>(declare_and_get<int>(node, "ceres_remove_fixation_min_num_fixed_points", 10));
  backend_options.max_fixed_lm_in_ceres_  = static_cast<size_t>(declare_and_get<int>(node, "ceres_max_fixed_landmarks", 50));

  CeresBackendInterfaceOptions ba_interface_options;
  ba_interface_options.min_num_obs = static_cast<size_t>(declare_and_get<int>(node, "backend_min_num_obs", 2));
  ba_interface_options.min_parallax_thresh = declare_and_get<double>(node, "backend_min_parallax_deg", 2.0)/180.0*M_PI;
  ba_interface_options.only_use_corners = declare_and_get<bool>(node, "backend_only_use_corners", false);
  ba_interface_options.use_zero_motion_detection = declare_and_get<bool>(node, "backend_use_zero_motion_detection", true);
  ba_interface_options.backend_zero_motion_check_n_frames = static_cast<int>(declare_and_get<int>(node, "backend_zero_motion_check_n_frames", 5));
  ba_interface_options.use_outlier_rejection = declare_and_get<bool>(node, "use_outlier_rejection", true);
  ba_interface_options.outlier_rejection_px_threshold = declare_and_get<double>(node, "outlier_rejection_px_threshold", 2.0);
  ba_interface_options.skip_optimization_when_tracking_bad = declare_and_get<bool>(node, "skip_optimization_when_tracking_bad", false);
  ba_interface_options.min_added_measurements = declare_and_get<int>(node, "skip_optim_min_obs", 20);
  ba_interface_options.refine_extrinsics = declare_and_get<bool>(node, "backend_refine_extrinsics", false);
  ba_interface_options.extrinsics_pos_sigma_meter = declare_and_get<double>(node, "backend_extrinsics_pos_sigma_meter", 0.05);
  ba_interface_options.extrinsics_pos_sigma_meter = declare_and_get<double>(node, "backend_extrinsics_rot_sigma_deg", 5.0) / 180.0 * M_PI;

  CeresBackendInterface::Ptr ba_interface =
      std::allocate_shared<CeresBackendInterface>(Eigen::aligned_allocator<CeresBackendInterface>{},
                                                  ba_interface_options,
                                                  backend_options,
                                                  loadMotionDetectorOptions(node),
                                                  camera_bundle);

  if (declare_and_get<bool>(node, "ba_parallelized", true))
    ba_interface->startThread();

  return ba_interface;
}

} // namespace ceres_backend_factory
} // namespace svo

