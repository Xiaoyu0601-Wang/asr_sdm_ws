#include <memory>
#include <string>
#include <chrono>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <video_kit/cameras/ncamera.h>

// SVO / VIO includes (mono pipeline)
#include <asr_sdm_vio/global.h>
#include <asr_sdm_vio/initialization.h>
#include <asr_sdm_vio/reprojector.h>
#include <asr_sdm_vio/direct/depth_filter.h>
#include <asr_sdm_vio/direct/feature_detection_types.h>
#include <asr_sdm_vio/tracker/feature_tracking_types.h>
#include <asr_sdm_vio/frame_handler_mono.h>
#include <asr_sdm_vio/imu_handler.h>
#include <asr_sdm_vio/common/imu_calibration.h>
#include <asr_sdm_vio/ceres_backend_interface.hpp>
#include <asr_sdm_vio/motion_detector.hpp>
#include <glog/logging.h>

using namespace std::chrono_literals;

class VioFrontendRos2 : public rclcpp::Node {
public:
  VioFrontendRos2() : rclcpp::Node("vio_frontend_ros2") {
    // Parameters
    image_topic_ = declare_parameter<std::string>("image_topic", "/cam0/image_raw");
    imu_topic_   = declare_parameter<std::string>("imu_topic", "/imu0");
    cam_yaml_    = declare_parameter<std::string>(
      "calib_yaml",
      ament_index_cpp::get_package_share_directory("asr_sdm_vio_ros2") + "/param/calib/euroc_mono.yaml");

    // 控制开关与 IMU 噪声参数
    this->declare_parameter<bool>("enable_vo", true);
    this->declare_parameter<bool>("publish_passthrough", true);
    this->declare_parameter<double>("gyroscope_noise_density", 1.6968e-04);
    this->declare_parameter<double>("gyroscope_random_walk", 1.9393e-05);
    this->declare_parameter<double>("accelerometer_noise_density", 2.0000e-03);
    this->declare_parameter<double>("accelerometer_random_walk", 3.0000e-03);
    this->declare_parameter<double>("imu_rate", 200.0);

    // 读取控制参数
    this->get_parameter("enable_vo", enable_vo_);
    this->get_parameter("publish_passthrough", publish_passthrough_);

    // Load camera rig (单目)
    try {
      ncam_ = vk::cameras::NCamera::loadFromYaml(cam_yaml_.c_str());
      if (!ncam_) {
        RCLCPP_ERROR(get_logger(), "Failed to load camera from %s", cam_yaml_.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Loaded camera rig from %s (num_cams=%zu)", cam_yaml_.c_str(), ncam_->numCameras());
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Exception while loading camera: %s", e.what());
    }

    // 提前创建 IMU handler（在首帧图像到来前就接收 IMU）
    {
      svo::ImuCalibration imu_calib;
      this->get_parameter("gyroscope_noise_density", imu_calib.gyro_noise_density);
      this->get_parameter("gyroscope_random_walk", imu_calib.gyro_bias_random_walk_sigma);
      this->get_parameter("accelerometer_noise_density", imu_calib.acc_noise_density);
      this->get_parameter("accelerometer_random_walk", imu_calib.acc_bias_random_walk_sigma);
      this->get_parameter("imu_rate", imu_calib.imu_rate);

      svo::ImuInitialization imu_init;
      svo::IMUHandlerOptions imu_opts;
      imu_handler_ = std::make_shared<svo::ImuHandler>(imu_calib, imu_init, imu_opts);
      RCLCPP_INFO(get_logger(), "IMU handler created (pre-VO) and ready.");
    }

    // 初始化单目 SVO 前端（可禁用以便仅做图像直通可视化与排障）
    if (enable_vo_) {
      init_mono_pipeline();
    } else {
      RCLCPP_WARN(get_logger(), "enable_vo=false: Skip creating VO pipeline (passthrough only)");
    }

    // Subscriptions（与 rosbag2_player 兼容的 QoS）
    auto img_qos = rclcpp::SensorDataQoS();
    auto imu_qos = rclcpp::SensorDataQoS();

    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, img_qos,
      std::bind(&VioFrontendRos2::imageCb, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos,
      std::bind(&VioFrontendRos2::imuCb, this, std::placeholders::_1));

    // Publishers
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("Rig", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/vio/odom", 10);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/image_with_features", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
    path_msg_.header.frame_id = "map";

    // Periodic status
    status_timer_ = create_wall_timer(1000ms, [this]() {
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                           "rx: img=%zu, imu=%zu", img_count_.exchange(0), imu_count_.exchange(0));
    });

    RCLCPP_INFO(get_logger(), "Mono VIO frontend ready. image_topic=%s imu_topic=%s",
                image_topic_.c_str(), imu_topic_.c_str());
  }

private:
  static inline uint64_t to_ns(const builtin_interfaces::msg::Time& t) {
    return static_cast<uint64_t>(t.sec) * 1000000000ull + static_cast<uint64_t>(t.nanosec);
  }

  void init_mono_pipeline() {
    try {
      svo::BaseOptions base_opts;          base_opts.use_imu = true;
      svo::DepthFilterOptions depth_opts;  // 默认
      svo::DetectorOptions detector_opts;  // 默认
      svo::InitializationOptions init_opts;// 默认
      svo::ReprojectorOptions reproj_opts; // 默认
      svo::FeatureTrackerOptions tracker_opts; // 默认

      vo_ = std::make_shared<svo::FrameHandlerMono>(
        base_opts, depth_opts, detector_opts, init_opts,
        reproj_opts, tracker_opts, ncam_);

      // 复用已创建的 IMU handler（避免重复构造导致潜在内存问题）
      if (!imu_handler_) {
        svo::ImuCalibration imu_calib;
        this->get_parameter("gyroscope_noise_density", imu_calib.gyro_noise_density);
        this->get_parameter("gyroscope_random_walk", imu_calib.gyro_bias_random_walk_sigma);
        this->get_parameter("accelerometer_noise_density", imu_calib.acc_noise_density);
        this->get_parameter("accelerometer_random_walk", imu_calib.acc_bias_random_walk_sigma);
        this->get_parameter("imu_rate", imu_calib.imu_rate);
        svo::ImuInitialization imu_init;
        svo::IMUHandlerOptions imu_opts;
        imu_handler_ = std::make_shared<svo::ImuHandler>(imu_calib, imu_init, imu_opts);
        RCLCPP_INFO(get_logger(), "IMU handler created (lazy).");
      }
      vo_->imu_handler_ = imu_handler_;
      RCLCPP_INFO(get_logger(), "IMU handler attached to VO.");

      // Ceres 后端
      // NOTE: 暂不启动 Ceres 后端，先以纯前端流水线验证，避免构造期 shared_from_this 等潜在问题
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

      // 延迟启动：在接收到第一帧图像时再 start()
      RCLCPP_INFO(get_logger(), "SVO Mono pipeline created (frontend-only, lazy start).");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Exception creating SVO mono: %s", e.what());
    }
  }

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    img_count_++;
    try {
      // Always publish passthrough image if requested (helps RViz even if VO not running)
      if (publish_passthrough_ && image_pub_) {
        image_pub_->publish(*msg);
      }

      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
      cv::Mat gray;
      if (msg->encoding == "mono8") {
        gray = cv_ptr->image;
      } else {
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
      }

      const uint64_t ts_ns = to_ns(msg->header.stamp);
      if (vo_ && !vo_started_) {
        vo_->start();
        vo_started_ = true;
        RCLCPP_INFO(get_logger(), "SVO mono frontend started on first image (ts=%lu)", ts_ns);
      }
      if (vo_) {
        vo_->addImage(gray, ts_ns);
      }

      auto frame = vo_ ? vo_->lastFrame() : nullptr;
      if (frame) {
        const auto T_W_B = frame->T_world_imu(); // imu位姿（或 T_world_cam() 亦可）
        const auto q = T_W_B.getRotation().toImplementation();
        const auto p = T_W_B.getPosition();

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = msg->header.stamp;
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

        // Publish path (accumulate)
        geometry_msgs::msg::PoseStamped path_pose = pose;
        path_msg_.header.stamp = pose.header.stamp;
        path_msg_.poses.push_back(path_pose);
        path_pub_->publish(path_msg_);

        // Re-publish image for visualization
        image_pub_->publish(*msg);
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "imageCb exception: %s", e.what());
    }
  }

  void imuCb(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    imu_count_++;
    if (!imu_handler_) return;
    const double t = static_cast<double>(msg->header.stamp.sec) + 1e-9 * static_cast<double>(msg->header.stamp.nanosec);
    Eigen::Vector3d omega(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    svo::ImuMeasurement m(t, omega, acc);
    imu_handler_->addImuMeasurement(m);
  }

  // Params
  std::string image_topic_;
  std::string imu_topic_;
  std::string cam_yaml_;

  // Camera rig
  vk::cameras::NCamera::Ptr ncam_;

  // SVO VO pipeline
  std::shared_ptr<svo::FrameHandlerMono> vo_;
  // IMU handler
  svo::ImuHandler::Ptr imu_handler_;

  // Backend
  svo::CeresBackendInterface::Ptr backend_;

  // State
  std::atomic<size_t> img_count_{0};
  std::atomic<size_t> imu_count_{0};
  bool vo_started_ = false;
  bool enable_vo_ = true;
  bool publish_passthrough_ = true;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char **argv) {
  // Initialize glog to avoid uninitialized usage in underlying libs
  google::InitGoogleLogging("vio_frontend_ros2");
  FLAGS_alsologtostderr = 1;
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<VioFrontendRos2>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    fprintf(stderr, "[vio_frontend_ros2] Unhandled exception: %s\n", e.what());
  }
  rclcpp::shutdown();
  google::ShutdownGoogleLogging();
  return 0;
}
