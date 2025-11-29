#include <memory>
#include <string>
#include <chrono>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <video_kit/cameras/ncamera.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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
#include <glog/logging.h>

using namespace std::chrono_literals;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Imu;

class VioFrontendStereoRos2 : public rclcpp::Node {
public:
  VioFrontendStereoRos2() : rclcpp::Node("vio_frontend_stereo_ros2") {
    // Parameters
    left_topic_  = declare_parameter<std::string>("left_topic", "/cam0/image_raw");
    right_topic_ = declare_parameter<std::string>("right_topic", "/cam1/image_raw");
    imu_topic_   = declare_parameter<std::string>("imu_topic", "/imu0");
    cam_yaml_    = declare_parameter<std::string>(
      "calib_yaml",
      ament_index_cpp::get_package_share_directory("asr_sdm_vio_ros2") + "/param/calib/euroc_stereo.yaml");
    imu_calib_yaml_ = declare_parameter<std::string>(
      "imu_calib_yaml",
      "/home/lxy/asr_sdm_ws/datasheet/MH_01_easy/mav0/imu0/sensor.yaml");
    imu_init_yaml_ = declare_parameter<std::string>(
      "imu_init_yaml",
      "");

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
      svo::BaseOptions base_opts;
      base_opts.use_imu = true; // 开启IMU
      // 其他阈值保持默认或根据需要微调
      svo::DepthFilterOptions depth_opts; // 默认
      svo::DetectorOptions detector_opts; // 默认
      svo::InitializationOptions init_opts; // 默认
      svo::StereoTriangulationOptions stereo_opts; // 默认
      svo::ReprojectorOptions reproj_opts; // 默认
      svo::FeatureTrackerOptions tracker_opts; // 默认

      try {
        vo_ = std::make_shared<svo::FrameHandlerStereo>(
          base_opts, depth_opts, detector_opts, init_opts,
          stereo_opts, reproj_opts, tracker_opts, ncam_);

        // 构造 IMU Handler（从文件加载标定/初始化，若提供）
        try {
          RCLCPP_INFO(get_logger(), "Loading IMU parameters from ROS2 parameter server.");
          svo::ImuCalibration imu_calib;
          // Use the ported vikit params_helper for ROS2
          this->declare_parameter<double>("gyroscope_noise_density", 0.0);
          this->declare_parameter<double>("gyroscope_random_walk", 0.0);
          this->declare_parameter<double>("accelerometer_noise_density", 0.0);
          this->declare_parameter<double>("accelerometer_random_walk", 0.0);
          this->declare_parameter<double>("imu_rate", 200.0);

          this->get_parameter("gyroscope_noise_density", imu_calib.gyro_noise_density);
          this->get_parameter("gyroscope_random_walk", imu_calib.gyro_bias_random_walk_sigma);
          this->get_parameter("accelerometer_noise_density", imu_calib.acc_noise_density);
          this->get_parameter("accelerometer_random_walk", imu_calib.acc_bias_random_walk_sigma);
          this->get_parameter("imu_rate", imu_calib.imu_rate);

          // For now, use default initialization. Can be loaded from params as well.
          svo::ImuInitialization imu_init;

          svo::IMUHandlerOptions imu_opts;
          imu_handler_ = std::make_shared<svo::ImuHandler>(imu_calib, imu_init, imu_opts);
          vo_->imu_handler_ = imu_handler_;
          RCLCPP_INFO(get_logger(), "IMU handler created with parameters from parameter server.");

          // Create and attach backend
          RCLCPP_INFO(get_logger(), "Creating Ceres backend...");
          svo::CeresBackendInterfaceOptions backend_iface_opts;
          svo::CeresBackendOptions backend_opts;
          backend_ = std::make_shared<svo::CeresBackendInterface>(backend_iface_opts, backend_opts, ncam_);
          backend_->setImu(imu_handler_);
          vo_->setBundleAdjuster(backend_);
          RCLCPP_INFO(get_logger(), "Ceres backend created and attached to VIO pipeline.");

        } catch (const std::exception &e) {
          RCLCPP_WARN(get_logger(), "Failed to create IMU handler or Backend: %s", e.what());
        }

        vo_->start();
        RCLCPP_INFO(get_logger(), "SVO Stereo pipeline created and started.");
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Exception creating SVO stereo: %s", e.what());
      }
    }

    // QoS RELIABLE
    auto img_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable();

    // Create raw subscribers for message_filters
    left_sub_.subscribe(this, left_topic_, img_qos.get_rmw_qos_profile());
    right_sub_.subscribe(this, right_topic_, img_qos.get_rmw_qos_profile());

    // Approximate time sync
    using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(10), left_sub_, right_sub_);
    sync_->registerCallback(std::bind(&VioFrontendStereoRos2::stereoCb, this, std::placeholders::_1, std::placeholders::_2));

    // IMU subscription（后续对接 ImuHandler 再启用）
    imu_sub_ = create_subscription<Imu>(
      imu_topic_, imu_qos,
      std::bind(&VioFrontendStereoRos2::imuCb, this, std::placeholders::_1));

    // Publishers
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("Rig", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/vio/odom", 10);

    // Periodic status
    timer_ = create_wall_timer(1000ms, [this]() {
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                           "rx: stereo_pairs=%zu, imu=%zu", stereo_pairs_.exchange(0), imu_count_.exchange(0));
    });

    RCLCPP_INFO(get_logger(), "Stereo VIO frontend ready. left=%s right=%s imu=%s",
                left_topic_.c_str(), right_topic_.c_str(), imu_topic_.c_str());
  }

private:
  static inline uint64_t to_ns(const builtin_interfaces::msg::Time& t) {
    return static_cast<uint64_t>(t.sec) * 1000000000ull + static_cast<uint64_t>(t.nanosec);
  }

  void stereoCb(const Image::ConstSharedPtr left, const Image::ConstSharedPtr right) {
    stereo_pairs_++;
    if (!vo_) return;
    try {
      // Ensure mono8
      cv_bridge::CvImageConstPtr cv_l = cv_bridge::toCvShare(left, left->encoding);
      cv_bridge::CvImageConstPtr cv_r = cv_bridge::toCvShare(right, right->encoding);
      cv::Mat imgL, imgR;
      if (left->encoding == "mono8") imgL = cv_l->image; else cv::cvtColor(cv_l->image, imgL, cv::COLOR_BGR2GRAY);
      if (right->encoding == "mono8") imgR = cv_r->image; else cv::cvtColor(cv_r->image, imgR, cv::COLOR_BGR2GRAY);

      const uint64_t ts_ns = to_ns(left->header.stamp);
      vo_->addImages(imgL, imgR, ts_ns);

      // 若已生成最新帧，尝试发布位姿（简化：仅发布时间戳，不进行姿态解算转换，后续完善）
      auto frames = vo_->getLastFrames();
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
      }

    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "stereoCb exception: %s", e.what());
    }
  }

  void imuCb(const Imu::ConstSharedPtr msg) {
    imu_count_++;
    if (!imu_handler_) return;
    const double t = static_cast<double>(msg->header.stamp.sec) + 1e-9 * static_cast<double>(msg->header.stamp.nanosec);
    Eigen::Vector3d omega(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    svo::ImuMeasurement m(t, omega, acc);
    imu_handler_->addImuMeasurement(m);
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

  // State counters
  std::atomic<size_t> stereo_pairs_{0};
  std::atomic<size_t> imu_count_{0};

  // ROS interfaces
  message_filters::Subscriber<Image> left_sub_;
  message_filters::Subscriber<Image> right_sub_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<Image, Image>>> sync_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VioFrontendStereoRos2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
