#include <memory>
#include <string>
#include <chrono>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <video_kit/cameras/ncamera.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rmw/qos_profiles.h>

// SVO includes
#include <asr_sdm_vio/stereo_triangulation.h>
#include <asr_sdm_vio/global.h>
#include <asr_sdm_vio/initialization.h>
#include <asr_sdm_vio/reprojector.h>
#include <asr_sdm_vio/direct/depth_filter.h>
#include <asr_sdm_vio/direct/feature_detection_types.h>
#include <asr_sdm_vio/tracker/feature_tracking_types.h>
#include <asr_sdm_vio/frame_handler_stereo.h>
#include <asr_sdm_vio/imu_handler.h>
#include <asr_sdm_vio/common/imu_calibration.h>
#include <asr_sdm_vio/ceres_backend_interface.hpp>
#include <asr_sdm_vio/motion_detector.hpp>
#include <glog/logging.h>
#include <cmath>

using namespace std::chrono_literals;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Imu;

class VioFrontendStereoRos2 : public rclcpp::Node {
public:
  VioFrontendStereoRos2(const rclcpp::NodeOptions &options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  : rclcpp::Node("vio_frontend_stereo_ros2", options) {
    // Helpers to read/declare parameters safely (avoid re-declare when overrides already declared)
    auto get_str = [&](const char* name, const std::string& def){
      rclcpp::Parameter p;
      if (this->get_parameter(name, p) && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        return p.as_string();
      }
      std::string v;
      return this->get_parameter(name, v) ? v : def;
    };
    auto get_bool = [&](const char* name, bool def){
      rclcpp::Parameter p;
      if (this->get_parameter(name, p)) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) return p.as_bool();
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) return p.as_int() != 0;
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) return std::fabs(p.as_double()) > 1e-12;
      }
      bool v;
      return this->get_parameter(name, v) ? v : def;
    };
    auto get_double = [&](const char* name, double def){
      rclcpp::Parameter p;
      if (this->get_parameter(name, p)) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) return p.as_double();
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) return static_cast<double>(p.as_int());
      }
      double v;
      return this->get_parameter(name, v) ? v : def;
    };
    auto get_int = [&](const char* name, int def){
      rclcpp::Parameter p;
      if (this->get_parameter(name, p)) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) return static_cast<int>(p.as_int());
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) return static_cast<int>(std::lround(p.as_double()));
      }
      int v;
      return this->get_parameter(name, v) ? v : def;
    };



    // Parameters
    left_topic_  = get_str("left_topic", "/cam0/image_raw");
    right_topic_ = get_str("right_topic", "/cam1/image_raw");
    imu_topic_   = get_str("imu_topic", "/imu0");
    cam_yaml_    = get_str("calib_yaml", ament_index_cpp::get_package_share_directory("asr_sdm_vio_ros2") + "/param/calib/euroc_stereo.yaml");
    imu_calib_yaml_ = get_str("imu_calib_yaml", "/home/lxy/asr_sdm_ws/datasheet/MH_01_easy/mav0/imu0/sensor.yaml");
    imu_init_yaml_ = get_str("imu_init_yaml", "");
    // Controls
    enable_vo_ = get_bool("enable_vo", true);
    publish_passthrough_ = get_bool("publish_passthrough", true);

    // Load camera rig (expect 2 cameras)
    try {
      ncam_ = vk::cameras::NCamera::loadFromYaml(cam_yaml_.c_str());
      if (!ncam_) {
        RCLCPP_ERROR(get_logger(), "Failed to load camera rig from %s", cam_yaml_.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Loaded camera rig from %s (num_cams=%zu)", cam_yaml_.c_str(), ncam_->numCameras());
        if (ncam_->numCameras() < 2) {
          RCLCPP_WARN(get_logger(), "Stereo frontend expects >=2 cameras in rig");
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Exception while loading camera: %s", e.what());
    }

    // Init SVO stereo pipeline + IMU接入
    {
      // Map ROS2 params -> SVO options
      svo::BaseOptions base_opts;
      base_opts.use_imu = get_bool("use_imu", true);
      base_opts.max_n_kfs = get_int("max_n_kfs", 10);
      base_opts.quality_min_fts = get_int("quality_min_fts", 40);
      base_opts.img_align_max_level = get_int("img_align_max_level", 4);
      base_opts.img_align_min_level = get_int("img_align_min_level", 2);
      base_opts.kfselect_numkfs_upper_thresh = get_int("kfselect_numkfs_upper_thresh", 140);
      base_opts.kfselect_numkfs_lower_thresh = get_int("kfselect_numkfs_lower_thresh", 100);
      base_opts.kfselect_min_disparity = get_double("kfselect_min_disparity", 40.0);
      base_opts.kfselect_min_angle = get_double("kfselect_min_angle", 20.0);
      base_opts.kfselect_min_dist_metric = get_double("kfselect_min_dist_metric", 0.1);
      base_opts.update_seeds_with_old_keyframes = get_bool("update_seeds_with_old_keyframes", true);
      base_opts.poseoptim_thresh = get_double("poseoptim_thresh", 2.0);
      base_opts.img_align_prior_lambda_rot = get_double("img_align_prior_lambda_rot", 0.5);
      base_opts.img_align_prior_lambda_trans = get_double("img_align_prior_lambda_trans", 0.0);
      base_opts.use_async_reprojectors = get_bool("use_async_reprojectors", false);
      base_opts.kfselect_criterion = svo::KeyframeCriterion::FORWARD;  // EuRoC is forward-looking

      svo::DepthFilterOptions depth_opts;
      depth_opts.affine_est_offset = get_bool("depth_filter_affine_est_offset", true);
      depth_opts.affine_est_gain = get_bool("depth_filter_affine_est_gain", false);

      svo::DetectorOptions detector_opts;
      detector_opts.cell_size = get_int("grid_size", 25);
      detector_opts.threshold_primary = get_int("detector_threshold_primary", 10);
      detector_opts.threshold_secondary = get_int("detector_threshold_secondary", 200);
      detector_opts.threshold_shitomasi = get_int("detector_threshold_shitomasi", 50);

      svo::InitializationOptions init_opts;
      init_opts.init_min_features = get_int("init_min_features", 60);
      init_opts.init_min_disparity = get_double("init_min_disparity", 30.0);

      svo::StereoTriangulationOptions stereo_opts; // 默认

      svo::ReprojectorOptions reproj_opts;
      reproj_opts.cell_size = detector_opts.cell_size;
      reproj_opts.max_n_kfs = get_int("reprojector_max_n_kfs", 5);

      svo::FeatureTrackerOptions tracker_opts; // 默认

      try {
        vo_ = std::make_shared<svo::FrameHandlerStereo>(
          base_opts, depth_opts, detector_opts, init_opts,
          stereo_opts, reproj_opts, tracker_opts, ncam_);

        RCLCPP_INFO(get_logger(), "Loading IMU parameters from ROS2 parameter server.");
        svo::ImuCalibration imu_calib;
        // Use EuRoC-like reasonable defaults unless overridden by ROS2 params file
        imu_calib.gyro_noise_density = get_double("gyroscope_noise_density", 1.6968e-04);
        imu_calib.gyro_bias_random_walk_sigma = get_double("gyroscope_random_walk", 1.9393e-05);
        imu_calib.acc_noise_density = get_double("accelerometer_noise_density", 2.0000e-03);
        imu_calib.acc_bias_random_walk_sigma = get_double("accelerometer_random_walk", 3.0000e-03);
        imu_calib.imu_rate = get_double("imu_rate", 200.0);

        svo::ImuInitialization imu_init;

        svo::IMUHandlerOptions imu_opts;
        imu_handler_ = std::make_shared<svo::ImuHandler>(imu_calib, imu_init, imu_opts);
        vo_->imu_handler_ = imu_handler_;
        RCLCPP_INFO(get_logger(), "IMU handler created with parameters from parameter server.");

        // NOTE: 暂不启动 Ceres 后端，先以纯前端流水线验证，避免构造期 shared_from_this/bad_weak_ptr 等潜在问题
        // 如需启用后端，请将下方代码解注释并将其移动到构造完成后的回调中执行。
        // RCLCPP_INFO(get_logger(), "Creating Ceres backend...");
        // svo::CeresBackendInterfaceOptions backend_iface_opts;
        // svo::CeresBackendOptions backend_opts;
        // svo::MotionDetectorOptions motion_detector_opts; // default options
        // backend_ = std::make_shared<svo::CeresBackendInterface>(backend_iface_opts, backend_opts, motion_detector_opts, ncam_);
        // backend_->setImu(imu_handler_);
        // svo::CeresBackendPublisher::Ptr backend_pub;
        // backend_->makePublisher(this->shared_from_this(), backend_pub);
        // backend_->startThread();
        // vo_->setBundleAdjuster(backend_);
        // RCLCPP_INFO(get_logger(), "Ceres backend created and attached to VIO pipeline.");

        vo_->start();
        RCLCPP_INFO(get_logger(), "SVO Stereo pipeline created and started (frontend-only).");
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Exception creating SVO stereo: %s", e.what());
      }
    }

    auto imu_qos = rclcpp::SensorDataQoS();

    left_sub_.subscribe(this, left_topic_, rmw_qos_profile_sensor_data);
    right_sub_.subscribe(this, right_topic_, rmw_qos_profile_sensor_data);

    using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(50), left_sub_, right_sub_);
    sync_->registerCallback(std::bind(&VioFrontendStereoRos2::stereoCb, this, std::placeholders::_1, std::placeholders::_2));

    imu_sub_ = create_subscription<Imu>(
      imu_topic_, imu_qos,
      std::bind(&VioFrontendStereoRos2::imuCb, this, std::placeholders::_1));

    // Optional left image passthrough to /image_with_features regardless of VO state
    left_passthrough_sub_ = create_subscription<Image>(
      left_topic_, rclcpp::SensorDataQoS(),
      [this](const Image::ConstSharedPtr msg){
        if (publish_passthrough_ && image_pub_) {
          image_pub_->publish(*msg);
        }
      });

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("Rig", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/vio/odom", 10);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/image_with_features", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
    path_msg_.header.frame_id = "map";

    status_timer_ = create_wall_timer(1000ms, [this]() {
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                           "rx: stereo_pairs=%zu, imu=%zu", stereo_pairs_.exchange(0), imu_count_.exchange(0));
    });

    RCLCPP_INFO(get_logger(), "Stereo VIO frontend ready. left=%s right=%s imu=%s",
                left_topic_.c_str(), right_topic_.c_str(), imu_topic_.c_str());
  }

public:
  ~VioFrontendStereoRos2() override {
    RCLCPP_INFO(get_logger(), "Shutting down VIO frontend...");
    try {
      if (vo_) {
        /* no explicit stop() available in FrameHandlerStereo */
        vo_.reset();
      }
      backend_.reset();
      imu_handler_.reset();
      path_msg_.poses.clear();
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Exception during shutdown: %s", e.what());
    }
  }
private:
  static inline uint64_t to_ns(const builtin_interfaces::msg::Time& t) {
    return static_cast<uint64_t>(t.sec) * 1000000000ull + static_cast<uint64_t>(t.nanosec);
  }

  void stereoCb(const Image::ConstSharedPtr left, const Image::ConstSharedPtr right) {
    stereo_pairs_++;
    try {
      // Always publish passthrough image if requested (even if VO is disabled/not producing frames)
      if (publish_passthrough_ && image_pub_) {
        image_pub_->publish(*left);
      }

      if (!enable_vo_ || !vo_) return;

      cv_bridge::CvImageConstPtr cv_l = cv_bridge::toCvShare(left, left->encoding);
      cv_bridge::CvImageConstPtr cv_r = cv_bridge::toCvShare(right, right->encoding);
      cv::Mat imgL, imgR;
      if (left->encoding == "mono8") imgL = cv_l->image; else cv::cvtColor(cv_l->image, imgL, cv::COLOR_BGR2GRAY);
      if (right->encoding == "mono8") imgR = cv_r->image; else cv::cvtColor(cv_r->image, imgR, cv::COLOR_BGR2GRAY);

      const uint64_t ts_ns = to_ns(left->header.stamp);
      vo_->addImages(imgL, imgR, ts_ns);

      auto frames = vo_->getLastFrames();
      if (!frames) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 3000,
                             "VO not initialized yet (pairs=%zu, imu=%zu). stage=%s, quality=%s, last_obs=%zu",
                             stereo_pairs_.load(), imu_count_.load(),
                             vo_->stageStr().c_str(), vo_->trackingQualityStr().c_str(), vo_->lastNumObservations());
      }
      if (frames) {
        const auto T_W_B = frames->get_T_W_B();
        const auto q = T_W_B.getRotation().toImplementation();
        const auto p = T_W_B.getPosition();

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = left->header.stamp;
        pose.header.frame_id = "map";
        pose.pose.position.x = p.x();
        pose.pose.position.y = p.y();
        pose.pose.position.z = p.z();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        pose_pub_->publish(pose);

        nav_msgs::msg::Odometry odom;
        odom.header = pose.header;
        odom.child_frame_id = "rig";
        odom.pose.pose = pose.pose;
        odom_pub_->publish(odom);

        // Publish path (limit size to prevent memory bloat)
        geometry_msgs::msg::PoseStamped path_pose = pose;
        path_msg_.header.stamp = pose.header.stamp;
        path_msg_.poses.push_back(path_pose);
        // Keep only last 1000 poses to avoid memory issues
        if (path_msg_.poses.size() > 1000) {
          path_msg_.poses.erase(path_msg_.poses.begin());
        }
        path_pub_->publish(path_msg_);
      }

    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "stereoCb exception: %s", e.what());
    }
  }

  void imuCb(const Imu::ConstSharedPtr msg) {
    imu_count_++;
    if (!imu_handler_) return;
    try {
      const double t = static_cast<double>(msg->header.stamp.sec) + 1e-9 * static_cast<double>(msg->header.stamp.nanosec);
      Eigen::Vector3d omega(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
      Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
      svo::ImuMeasurement m(t, omega, acc);
      imu_handler_->addImuMeasurement(m);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "imuCb exception: %s", e.what());
    }
  }

  // Params
  std::string left_topic_;
  std::string right_topic_;
  std::string imu_topic_;
  std::string cam_yaml_;
  std::string imu_calib_yaml_;
  std::string imu_init_yaml_;

  // Camera rig
  vk::cameras::NCamera::Ptr ncam_;

  // SVO VO pipeline
  std::shared_ptr<svo::FrameHandlerStereo> vo_;
  // IMU handler
  svo::ImuHandler::Ptr imu_handler_;

  // Backend
  svo::CeresBackendInterface::Ptr backend_;

  // State counters
  std::atomic<size_t> stereo_pairs_{0};
  std::atomic<size_t> imu_count_{0};
  bool enable_vo_ = true;
  bool publish_passthrough_ = true;

  // ROS interfaces
  message_filters::Subscriber<Image> left_sub_;
  message_filters::Subscriber<Image> right_sub_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<Image, Image>>> sync_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<Image>::SharedPtr left_passthrough_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VioFrontendStereoRos2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
