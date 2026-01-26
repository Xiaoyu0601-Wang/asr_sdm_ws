#include <asr_sdm_ros2/svo_factory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <asr_sdm_vio/svo.h>
#include <asr_sdm_vio/common/imu_calibration.h>
#include <asr_sdm_vio/frame_handler_mono.h>
#include <asr_sdm_vio/frame_handler_stereo.h>

#include <vikit/params_helper.h>

#ifdef SVO_LOOP_CLOSING
#include <asr_sdm_vio/online_loopclosing/loop_closing.h>
#include <asr_sdm_vio/online_loopclosing/map_alignment.h>
#endif

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

#ifdef SVO_USE_VIN_BACKEND
#include <svo_gtsam/backend_optimizer.h>
#include <svo_gtsam/backend_interface.h>
#include <svo_gtsam/graph_manager.h>
#endif

#ifdef SVO_GLOBAL_MAP
#include <svo/global_map.h>
#include <svo/gtsam/graph_manager.h>
#include <svo/gtsam/gtsam_optimizer.h>
#endif

namespace svo {
namespace factory {

BaseOptions loadBaseOptions(const rclcpp::Node::SharedPtr& node, bool forward_default)
{
  BaseOptions o;
  o.max_n_kfs = vk::param<int>(node, "max_n_kfs", 5);
  o.use_imu = vk::param<bool>(node, "use_imu", false);

  const std::string trace_dir_default =
      ament_index_cpp::get_package_share_directory("asr_sdm_svo2") + "/trace";
  o.trace_dir = vk::param<std::string>(node, "trace_dir", trace_dir_default);

  o.quality_min_fts = vk::param<int>(node, "quality_min_fts", 50);
  o.quality_max_fts_drop = vk::param<int>(node, "quality_max_drop_fts", 40);
  o.relocalization_max_trials = vk::param<int>(node, "relocalization_max_trials", 50);
  o.poseoptim_prior_lambda = vk::param<double>(node, "poseoptim_prior_lambda", 0.0);
  o.poseoptim_using_unit_sphere = vk::param<bool>(node, "poseoptim_using_unit_sphere", false);
  o.img_align_prior_lambda_rot = vk::param<double>(node, "img_align_prior_lambda_rot", 0.0);
  o.img_align_prior_lambda_trans = vk::param<double>(node, "img_align_prior_lambda_trans", 0.0);
  o.structure_optimization_max_pts = vk::param<int>(node, "structure_optimization_max_pts", 20);
  o.init_map_scale = vk::param<double>(node, "map_scale", 1.0);

  const std::string default_kf_criterion = forward_default ? "FORWARD" : "DOWNLOOKING";
  if (vk::param<std::string>(node, "kfselect_criterion", default_kf_criterion) == "FORWARD")
    o.kfselect_criterion = KeyframeCriterion::FORWARD;
  else
    o.kfselect_criterion = KeyframeCriterion::DOWNLOOKING;

  o.kfselect_min_dist = vk::param<double>(node, "kfselect_min_dist", 0.12);
  o.kfselect_numkfs_upper_thresh = vk::param<int>(node, "kfselect_numkfs_upper_thresh", 120);
  o.kfselect_numkfs_lower_thresh = vk::param<double>(node, "kfselect_numkfs_lower_thresh", 70);
  o.kfselect_min_dist_metric = vk::param<double>(node, "kfselect_min_dist_metric", 0.01);
  o.kfselect_min_angle = vk::param<double>(node, "kfselect_min_angle", 20);
  o.kfselect_min_disparity = vk::param<double>(node, "kfselect_min_disparity", 40);
  o.kfselect_min_num_frames_between_kfs =
      vk::param<int>(node, "kfselect_min_num_frames_between_kfs", 2);
  o.kfselect_backend_max_time_sec = vk::param<double>(node, "kfselect_backend_max_time_sec", 3.0);

  o.img_align_max_level = vk::param<int>(node, "img_align_max_level", 4);
  o.img_align_min_level = vk::param<int>(node, "img_align_min_level", 2);
  o.img_align_robustification = vk::param<bool>(node, "img_align_robustification", false);
  o.img_align_use_distortion_jacobian =
      vk::param<bool>(node, "img_align_use_distortion_jacobian", false);
  o.img_align_est_illumination_gain =
      vk::param<bool>(node, "img_align_est_illumination_gain", false);
  o.img_align_est_illumination_offset =
      vk::param<bool>(node, "img_align_est_illumination_offset", false);

  o.poseoptim_thresh = vk::param<double>(node, "poseoptim_thresh", 2.0);
  o.update_seeds_with_old_keyframes =
      vk::param<bool>(node, "update_seeds_with_old_keyframes", true);
  o.use_async_reprojectors = vk::param<bool>(node, "use_async_reprojectors", false);
  o.trace_statistics = vk::param<bool>(node, "trace_statistics", false);
  o.backend_scale_stable_thresh =
      vk::param<double>(node, "backend_scale_stable_thresh", 0.02);
  o.global_map_lc_timeout_sec_ =
      vk::param<double>(node, "global_map_timeout_sec", 2.0);
  return o;
}

DetectorOptions loadDetectorOptions(const rclcpp::Node::SharedPtr& node)
{
  DetectorOptions o;
  o.cell_size = vk::param<int>(node, "grid_size", 35);
  o.max_level = vk::param<int>(node, "n_pyr_levels", 3) - 1;
  o.threshold_primary = vk::param<int>(node, "detector_threshold_primary", 10);
  o.threshold_secondary = vk::param<int>(node, "detector_threshold_secondary", 200);
  o.threshold_shitomasi = vk::param<int>(node, "detector_threshold_shitomasi", 100);
  if (vk::param<bool>(node, "use_edgelets", true))
    o.detector_type = DetectorType::kFastGrad;
  else
    o.detector_type = DetectorType::kFast;
  return o;
}

DepthFilterOptions loadDepthFilterOptions(const rclcpp::Node::SharedPtr& node)
{
  DepthFilterOptions o;
  o.max_search_level = vk::param<int>(node, "n_pyr_levels", 3) - 1;
  o.use_threaded_depthfilter =
      vk::param<bool>(node, "use_threaded_depthfilter", true);
  o.seed_convergence_sigma2_thresh =
      vk::param<double>(node, "seed_convergence_sigma2_thresh", 200.0);
  o.mappoint_convergence_sigma2_thresh =
      vk::param<double>(node, "mappoint_convergence_sigma2_thresh", 500.0);
  o.scan_epi_unit_sphere = vk::param<bool>(node, "scan_epi_unit_sphere", false);
  o.affine_est_offset = vk::param<bool>(node, "depth_filter_affine_est_offset", true);
  o.affine_est_gain = vk::param<bool>(node, "depth_filter_affine_est_gain", false);

  o.max_n_seeds_per_frame = static_cast<size_t>(
      static_cast<double>(vk::param<int>(node, "max_fts", 120)) *
      vk::param<double>(node, "max_seeds_ratio", 3.0));

  o.max_map_seeds_per_frame = static_cast<size_t>(
      static_cast<double>(vk::param<int>(node, "max_map_fts", 120)));

  o.extra_map_points =
      vk::param<bool>(node, "depth_filter_extra_map_points", false);

  if (vk::param<bool>(node, "runlc", false) && !o.extra_map_points)
  {
    LOG(WARNING) << "Loop closure requires extra map points, "
                 << " but the option is not set, overriding to true.";
    o.extra_map_points = true;
  }
  return o;
}

InitializationOptions loadInitializationOptions(const rclcpp::Node::SharedPtr& node)
{
  InitializationOptions o;
  o.init_min_features = vk::param<int>(node, "init_min_features", 100);
  o.init_min_tracked = vk::param<int>(node, "init_min_tracked", 80);
  o.init_min_inliers = vk::param<int>(node, "init_min_inliers", 70);
  o.init_min_disparity = vk::param<double>(node, "init_min_disparity", 40.0);
  o.init_min_features_factor = vk::param<double>(node, "init_min_features_factor", 2.0);
  o.reproj_error_thresh = vk::param<double>(node, "reproj_err_thresh", 2.0);
  o.init_disparity_pivot_ratio = vk::param<double>(node, "init_disparity_pivot_ratio", 0.5);

  const std::string init_method = vk::param<std::string>(node, "init_method", "FivePoint");
  if (init_method == "Homography")
    o.init_type = InitializerType::kHomography;
  else if (init_method == "TwoPoint")
    o.init_type = InitializerType::kTwoPoint;
  else if (init_method == "FivePoint")
    o.init_type = InitializerType::kFivePoint;
  else if (init_method == "OneShot")
    o.init_type = InitializerType::kOneShot;
  else
    LOG(ERROR) << "Initialization Method not supported: " << init_method;

  return o;
}

FeatureTrackerOptions loadTrackerOptions(const rclcpp::Node::SharedPtr& node)
{
  FeatureTrackerOptions o;
  o.klt_max_level = vk::param<int>(node, "klt_max_level", 4);
  o.klt_min_level = vk::param<int>(node, "klt_min_level", 0.001);
  return o;
}

ReprojectorOptions loadReprojectorOptions(const rclcpp::Node::SharedPtr& node)
{
  ReprojectorOptions o;
  o.max_n_kfs = vk::param<int>(node, "reprojector_max_n_kfs", 5);
  o.max_n_features_per_frame = vk::param<int>(node, "max_fts", 160);
  o.cell_size = vk::param<int>(node, "grid_size", 35);
  o.reproject_unconverged_seeds =
      vk::param<bool>(node, "reproject_unconverged_seeds", true);
  o.max_unconverged_seeds_ratio =
      vk::param<double>(node, "max_unconverged_seeds_ratio", -1.0);
  o.min_required_features = vk::param<int>(node, "quality_min_fts", 50);
  o.seed_sigma2_thresh =
      vk::param<double>(node, "seed_convergence_sigma2_thresh", 200.0);

  o.affine_est_offset =
      vk::param<bool>(node, "reprojector_affine_est_offset", true);
  o.affine_est_gain =
      vk::param<bool>(node, "reprojector_affine_est_gain", false);
  o.max_fixed_landmarks =
      vk::param<int>(node, "reprojector_max_fixed_landmarks", 50);
  o.max_n_global_kfs =
      vk::param<int>(node, "reprojector_max_n_global_kfs", 20);
  o.use_kfs_from_global_map =
      vk::param<bool>(node, "reprojector_use_kfs_from_global_map", false);
  o.fixed_lm_grid_size =
      vk::param<int>(node, "reprojector_fixed_lm_grid_size", 50);

  return o;
}

CameraBundle::Ptr loadCameraFromYaml(const rclcpp::Node::SharedPtr& node)
{
  const std::string calib_file = vk::param<std::string>(node, "calib_file", "~/cam.yaml");
  CameraBundle::Ptr ncam = CameraBundle::loadFromYaml(calib_file);
  std::cout << "loaded " << ncam->numCameras() << " cameras";
  for (const auto& cam : ncam->getCameraVector())
    cam->printParameters(std::cout, "");
  return ncam;
}

StereoTriangulationOptions loadStereoOptions(const rclcpp::Node::SharedPtr& node)
{
  StereoTriangulationOptions o;
  o.triangulate_n_features = vk::param<int>(node, "max_fts", 120);
  o.max_depth_inv = vk::param<double>(node, "max_depth_inv", 1.0 / 50.0);
  o.min_depth_inv = vk::param<double>(node, "min_depth_inv", 1.0 / 0.5);
  o.mean_depth_inv = vk::param<double>(node, "mean_depth_inv", 1.0 / 2.0);
  return o;
}

ImuHandler::Ptr getImuHandler(const rclcpp::Node::SharedPtr& node)
{
  const std::string calib_file = vk::param<std::string>(node, "calib_file", "");
  ImuCalibration imu_calib = ImuHandler::loadCalibrationFromFile(calib_file);
  imu_calib.print("Loaded IMU Calibration");
  ImuInitialization imu_init = ImuHandler::loadInitializationFromFile(calib_file);
  imu_init.print("Loaded IMU Initialization");
  IMUHandlerOptions options;
  options.temporal_stationary_check =
      vk::param<bool>(node, "imu_temporal_stationary_check", false);
  options.temporal_window_length_sec_ =
      vk::param<double>(node, "imu_temporal_window_length_sec", 0.5);
  options.stationary_acc_sigma_thresh_ =
      vk::param<double>(node, "stationary_acc_sigma_thresh", 0.0);
  options.stationary_gyr_sigma_thresh_ =
      vk::param<double>(node, "stationary_gyr_sigma_thresh", 0.0);
  ImuHandler::Ptr imu_handler(new ImuHandler(imu_calib, imu_init, options));
  return imu_handler;
}

void setInitialPose(const rclcpp::Node::SharedPtr& node, FrameHandlerBase& vo)
{
  Transformation T_world_imuinit(
      Quaternion(vk::param<double>(node, "T_world_imuinit/qw", 1.0),
                 vk::param<double>(node, "T_world_imuinit/qx", 0.0),
                 vk::param<double>(node, "T_world_imuinit/qy", 0.0),
                 vk::param<double>(node, "T_world_imuinit/qz", 0.0)),
      Vector3d(vk::param<double>(node, "T_world_imuinit/tx", 0.0),
               vk::param<double>(node, "T_world_imuinit/ty", 0.0),
               vk::param<double>(node, "T_world_imuinit/tz", 0.0)));
  vo.setInitialImuPose(T_world_imuinit);
}

FrameHandlerMono::Ptr makeMono(const rclcpp::Node::SharedPtr& node, const CameraBundlePtr& cam)
{
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(node);
  if (ncam->numCameras() > 1)
  {
    LOG(WARNING) << "Load more cameras than needed, will erase from the end.";
    ncam->keepFirstNCams(1);
  }

  FrameHandlerMono::Ptr vo = std::make_shared<FrameHandlerMono>(
      loadBaseOptions(node, false),
      loadDepthFilterOptions(node),
      loadDetectorOptions(node),
      loadInitializationOptions(node),
      loadReprojectorOptions(node),
      loadTrackerOptions(node),
        ncam);

  setInitialPose(node, *vo);
  return vo;
}

FrameHandlerStereo::Ptr makeStereo(const rclcpp::Node::SharedPtr& node, const CameraBundlePtr& cam)
{
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(node);
  if (ncam->numCameras() > 2)
  {
    LOG(WARNING) << "Load more cameras than needed, will erase from the end.";
    ncam->keepFirstNCams(2);
  }

  InitializationOptions init_options = loadInitializationOptions(node);
  init_options.init_type = InitializerType::kStereo;

  FrameHandlerStereo::Ptr vo = std::make_shared<FrameHandlerStereo>(
      loadBaseOptions(node, true),
      loadDepthFilterOptions(node),
      loadDetectorOptions(node),
        init_options,
      loadStereoOptions(node),
      loadReprojectorOptions(node),
      loadTrackerOptions(node),
        ncam);

  setInitialPose(node, *vo);
  return vo;
}

FrameHandlerArray::Ptr makeArray(const rclcpp::Node::SharedPtr& node, const CameraBundlePtr& cam)
{
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(node);

  InitializationOptions init_options = loadInitializationOptions(node);
  init_options.init_type = InitializerType::kArrayGeometric;
  init_options.init_min_disparity = 25;

  DepthFilterOptions depth_filter_options = loadDepthFilterOptions(node);
  depth_filter_options.verbose = true;

  FrameHandlerArray::Ptr vo = std::make_shared<FrameHandlerArray>(
      loadBaseOptions(node, true),
        depth_filter_options,
      loadDetectorOptions(node),
        init_options,
      loadReprojectorOptions(node),
      loadTrackerOptions(node),
        ncam);

  setInitialPose(node, *vo);
  return vo;
}

#ifdef SVO_LOOP_CLOSING
std::shared_ptr<LoopClosing> getLoopClosingModule(
    const rclcpp::Node::SharedPtr& node,
    const CameraBundlePtr& cam)
{
  const std::string voc_name = vk::param<std::string>(node, "voc_name", "");
  const std::string voc_path = vk::param<std::string>(node, "voc_path", "");

  LoopClosureOptions o;
  o.runlc = vk::param<bool>(node, "runlc", false);
  o.voc_name = voc_name;
  o.voc_path = voc_path;
  o.alpha = vk::param<double>(node, "alpha", 0.0);
  o.beta = vk::param<double>(node, "beta", 0.0);
  o.ignored_past_frames = vk::param<int>(node, "ignored_past_frames", 0);
  o.scale_ret_app = vk::param<std::string>(node, "scale_ret_app", "None");
  o.bowthresh = vk::param<double>(node, "bowthresh", 0.0);
  o.gv_3d_inlier_thresh = vk::param<double>(node, "gv_3d_inlier_thresh", 0.0);
  o.min_num_3d = vk::param<int>(node, "min_num_3d", 0);
  o.orb_dist_thresh = vk::param<int>(node, "orb_dist_thresh", 0);
  o.gv_2d_match_thresh = vk::param<double>(node, "gv_2d_match_thresh", 0.0);
  o.use_opengv = vk::param<bool>(node, "use_opengv", true);
  o.enable_image_logging = vk::param<bool>(node, "enable_image_logging", false);
  o.image_log_base_path = vk::param<std::string>(node, "image_log_base_path", "");
  o.global_map_type = vk::param<std::string>(node, "global_map_type", "None");
  o.force_correction_dist_thresh_meter =
      vk::param<double>(node, "force_correction_dist_thresh_meter", 0.1);

  CameraBundle::Ptr cams = (cam) ? cam : loadCameraFromYaml(node);

  return std::make_shared<LoopClosing>(o, cams);
}
#endif

}  // namespace factory
}  // namespace svo
