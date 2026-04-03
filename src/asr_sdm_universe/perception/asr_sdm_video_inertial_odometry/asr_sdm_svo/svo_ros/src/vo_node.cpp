#include <Eigen/Core>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include <svo/config.h>
#include <svo/frame.h>
#include <svo/frame_handler_mono.h>
#include <svo/imu_types.h>
#include <svo/map.h>
#include <svo_ros/visualizer.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/math_utils.h>
#include <vikit/params_helper.h>
#include <vikit/pinhole_camera.h>
#include <vikit/user_input_thread.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

namespace svo
{

class VoNode : public rclcpp::Node
{
public:
  VoNode();
  ~VoNode();
  void init();
  void run();
  bool shouldQuit() const { return quit_; }

  // Getters needed by main()
  bool& quitFlag() { return quit_; }
  std::condition_variable& imgCv() { return img_cv_; }

private:
  // --- ROS callbacks ---
  void imgCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void imuCb(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  void remoteKeyCb(const std_msgs::msg::String::ConstSharedPtr& msg);
  void processUserActions();

  // --- Core ---
  svo::FrameHandlerMono* vo_ = nullptr;
  vk::AbstractCamera* cam_ = nullptr;
  std::unique_ptr<svo::Visualizer> visualizer_;
  std::shared_ptr<vk::UserInputThread> user_input_thread_;
  bool quit_ = false;

  // --- Image pipeline (producer-consumer, single-frame) ---
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_img_;
  std::mutex img_mutex_;
  std::condition_variable img_cv_;
  bool new_img_ready_ = false;

  // --- Remote key ---
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_remote_key_;
  std::mutex remote_mutex_;
  std::string remote_input_;

  // --- IMU ---
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  std::shared_ptr<svo::ImuHandler> imu_handler_;
  bool use_imu_ = false;
  bool set_initial_attitude_from_gravity_ = true;
  svo::ImuCalibration imu_calib_;
  svo::ImuInitialization imu_init_;
  svo::IMUHandlerOptions imu_options_;
  Eigen::Quaterniond last_imu_rotation_{Eigen::Quaterniond::Identity()};

  // --- IMU health monitoring ---
  int imu_meas_count_ = 0;
  double last_imu_msg_ts_ = 0.0;
  bool imu_timeout_warned_ = false;
  static constexpr double IMU_TIMEOUT_SEC_ = 2.0;

  // --- Publish control ---
  bool publish_markers_ = true;
  bool publish_dense_input_ = false;
  bool enable_visualization_ = false;
  double last_marker_publish_ts_ = 0.0;

  // --- Frame throttling ---
  bool enable_frame_throttle_ = true;
  double target_fps_ = 15.0;
  double last_accepted_ts_ = 0.0;
};

// =============================================================================
// Constructor
// =============================================================================
VoNode::VoNode()
  : Node("svo")
{
  publish_markers_      = vk::getParam<bool>(this, "publish_markers", true);
  publish_dense_input_  = vk::getParam<bool>(this, "publish_dense_input", false);
  enable_visualization_  = vk::getParam<bool>(this, "enable_visualization", false);

  if (vk::getParam<bool>(this, "accept_console_user_input", true))
    user_input_thread_ = std::make_shared<vk::UserInputThread>();

  const std::string dataset_name = vk::getParam<std::string>(this, "dataset_name", "");

  if (dataset_name == "euroc")
  {
    RCLCPP_INFO(this->get_logger(), "Using hardcoded EuRoC camera parameters");
    cam_ = new vk::PinholeCamera(752, 480, 458.654, 457.296, 367.215, 248.375);
  }
  else
  {
    const std::string cam_model = vk::getParam<std::string>(this, "cam_model", "Pinhole");
    bool ok = vk::camera_loader::loadFromRosNode(this, "", cam_);
    if (!ok || cam_ == nullptr)
      throw std::runtime_error("Failed to load camera. cam_model=" + cam_model);
    RCLCPP_INFO(this->get_logger(), "Created '%s' camera model", cam_model.c_str());
  }

  // --- Algorithm parameters ---
  if (dataset_name == "euroc")
  {
    RCLCPP_INFO(this->get_logger(), "Using hardcoded EuRoC algorithm parameters");
    svo::Config::gridSize() = 18;
    svo::Config::maxFts() = 400;
    svo::Config::triangMinCornerScore() = 3.0;
    svo::Config::nPyrLevels() = 4;
    svo::Config::fastType() = 12;
    svo::Config::kltMaxLevel() = 4;
    svo::Config::kltMinLevel() = 0;
    svo::Config::reprojThresh() = 4.0;
    svo::Config::poseOptimThresh() = 4.0;
    svo::Config::poseOptimNumIter() = 10;
    svo::Config::qualityMinFts() = 30;
    svo::Config::qualityMaxFtsDrop() = 80;
    svo::Config::initMinDisparity() = 50.0;
    svo::Config::initMinTracked() = 50;
    svo::Config::initMinInliers() = 40;
    svo::Config::kfSelectMinDist() = 0.001;
    svo::Config::maxNKfs() = 180;
    svo::Config::mapScale() = 5.0;
    svo::Config::useImu() = true;
    svo::Config::useThreadedDepthfilter() = false;
    svo::Config::patchMatchThresholdFactor() = 1.5;
    svo::Config::subpixNIter() = 20;
  }
  else
  {
    svo::Config::gridSize() = static_cast<size_t>(
        vk::getParam<int>(this, "grid_size", static_cast<int>(svo::Config::gridSize())));
    svo::Config::maxFts() = static_cast<size_t>(
        vk::getParam<int>(this, "max_fts", static_cast<int>(svo::Config::maxFts())));
    svo::Config::triangMinCornerScore() = vk::getParam<double>(
        this, "triang_min_corner_score", svo::Config::triangMinCornerScore());
    svo::Config::nPyrLevels() = static_cast<size_t>(
        vk::getParam<int>(this, "n_pyr_levels", static_cast<int>(svo::Config::nPyrLevels())));
    svo::Config::fastType() = static_cast<size_t>(
        vk::getParam<int>(this, "fast_type", static_cast<int>(svo::Config::fastType())));
    svo::Config::reprojThresh() = vk::getParam<double>(
        this, "reproj_thresh", svo::Config::reprojThresh());
    svo::Config::poseOptimThresh() = vk::getParam<double>(
        this, "poseoptim_thresh", svo::Config::poseOptimThresh());
    svo::Config::qualityMinFts() = static_cast<size_t>(vk::getParam<int>(
        this, "quality_min_fts", static_cast<int>(svo::Config::qualityMinFts())));
    svo::Config::qualityMaxFtsDrop() = vk::getParam<int>(
        this, "quality_max_drop_fts", svo::Config::qualityMaxFtsDrop());
    svo::Config::kfSelectMinDist() = vk::getParam<double>(
        this, "kfselect_mindist", svo::Config::kfSelectMinDist());
    svo::Config::maxNKfs() = static_cast<size_t>(vk::getParam<int>(
        this, "max_n_kfs", static_cast<int>(svo::Config::maxNKfs())));
    svo::Config::mapScale() = vk::getParam<double>(
        this, "map_scale", svo::Config::mapScale());
    svo::Config::initMinDisparity() = vk::getParam<double>(
        this, "init_min_disparity", svo::Config::initMinDisparity());
    svo::Config::initMinTracked() = static_cast<size_t>(vk::getParam<int>(
        this, "init_min_tracked", static_cast<int>(svo::Config::initMinTracked())));
    svo::Config::initMinInliers() = static_cast<size_t>(vk::getParam<int>(
        this, "init_min_inliers", static_cast<int>(svo::Config::initMinInliers())));
    svo::Config::patchMatchThresholdFactor() = vk::getParam<double>(
        this, "patch_match_thresh_factor", svo::Config::patchMatchThresholdFactor());
    svo::Config::kltMinLevel() = static_cast<size_t>(vk::getParam<int>(
        this, "klt_min_level", static_cast<int>(svo::Config::kltMinLevel())));
    svo::Config::kltMaxLevel() = static_cast<size_t>(vk::getParam<int>(
        this, "klt_max_level", static_cast<int>(svo::Config::kltMaxLevel())));
    svo::Config::subpixNIter() = static_cast<size_t>(vk::getParam<int>(
        this, "subpix_n_iter", static_cast<int>(svo::Config::subpixNIter())));
    svo::Config::maxEpiSearchSteps() = static_cast<size_t>(vk::getParam<int>(
        this, "max_epi_search_steps", static_cast<int>(svo::Config::maxEpiSearchSteps())));
  }

  // --- Frame throttling ---
  enable_frame_throttle_ = vk::getParam<bool>(this, "enable_frame_throttle", true);
  target_fps_ = vk::getParam<double>(this, "target_fps", 15.0);

  // --- IMU ---
  use_imu_ = vk::getParam<bool>(this, "use_imu", false);
  if (use_imu_)
  {
    RCLCPP_INFO(this->get_logger(), "IMU fusion is ENABLED");
    set_initial_attitude_from_gravity_ =
        vk::getParam<bool>(this, "set_initial_attitude_from_gravity", true);

    imu_calib_.delay_imu_cam = vk::getParam<double>(this, "imu_delay_imu_cam", 0.0);
    imu_calib_.max_imu_delta_t = vk::getParam<double>(this, "imu_max_imu_delta_t", 0.1);
    imu_calib_.saturation_accel_max = vk::getParam<double>(this, "imu_acc_max", 200.0);
    imu_calib_.saturation_omega_max = vk::getParam<double>(this, "imu_omega_max", 20.0);
    imu_calib_.gyro_noise_density = vk::getParam<double>(this, "imu_gyro_noise_density", 1.0e-4);
    imu_calib_.acc_noise_density = vk::getParam<double>(this, "imu_acc_noise_density", 1.0e-3);
    imu_calib_.gyro_bias_random_walk_sigma = vk::getParam<double>(this, "imu_gyro_bias_rw", 1.0e-6);
    imu_calib_.acc_bias_random_walk_sigma = vk::getParam<double>(this, "imu_acc_bias_rw", 1.0e-5);
    imu_calib_.gravity_magnitude = vk::getParam<double>(this, "imu_gravity_magnitude", 9.81);
    imu_calib_.imu_rate = vk::getParam<double>(this, "imu_rate", 200.0);

    imu_init_.velocity.setZero();
    imu_init_.omega_bias.setZero();
    imu_init_.acc_bias.setZero();
    imu_init_.velocity_sigma = vk::getParam<double>(this, "imu_velocity_sigma", 0.1);
    imu_init_.omega_bias_sigma = vk::getParam<double>(this, "imu_omega_bias_sigma", 0.01);
    imu_init_.acc_bias_sigma = vk::getParam<double>(this, "imu_acc_bias_sigma", 0.1);

    imu_options_.temporal_stationary_check = vk::getParam<bool>(
        this, "imu_temporal_stationary_check", false);
    imu_options_.temporal_window_length_sec_ = vk::getParam<double>(
        this, "imu_temporal_window_length_sec", 0.5);
    imu_options_.stationary_acc_sigma_thresh_ = vk::getParam<double>(
        this, "stationary_acc_sigma_thresh", 0.0);
    imu_options_.stationary_gyr_sigma_thresh_ = vk::getParam<double>(
        this, "stationary_gyr_sigma_thresh", 0.0);

    imu_handler_ = std::make_shared<svo::ImuHandler>(imu_calib_, imu_init_, imu_options_);

    RCLCPP_INFO(this->get_logger(),
                "IMU: delay=%.3fs rate=%.0fHz gravity=%.2f",
                imu_calib_.delay_imu_cam, imu_calib_.imu_rate,
                imu_calib_.gravity_magnitude);
  }

  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

// =============================================================================
void VoNode::init()
{
  visualizer_ = std::make_unique<svo::Visualizer>(this->shared_from_this());

  visualizer_->T_world_from_vision_ = Sophus::SE3d(
      vk::rpy2dcm(Eigen::Vector3d(
          vk::getParam<double>(this, "init_rx", 0.0),
          vk::getParam<double>(this, "init_ry", 0.0),
          vk::getParam<double>(this, "init_rz", 0.0))),
      Eigen::Vector3d(
          vk::getParam<double>(this, "init_tx", 0.0),
          vk::getParam<double>(this, "init_ty", 0.0),
          vk::getParam<double>(this, "init_tz", 0.0)));

  sub_remote_key_ = create_subscription<std_msgs::msg::String>(
      "remote_key", 5,
      std::bind(&VoNode::remoteKeyCb, this, std::placeholders::_1));

  const std::string cam_topic = vk::getParam<std::string>(this, "cam_topic", "camera/image_raw");
  sub_img_ = create_subscription<sensor_msgs::msg::Image>(
      cam_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&VoNode::imgCb, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Subscribing to camera: %s", cam_topic.c_str());

  if (use_imu_)
  {
    const std::string imu_topic = vk::getParam<std::string>(this, "imu_topic", "/imu/data");
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&VoNode::imuCb, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribing to IMU: %s", imu_topic.c_str());
  }
}

// =============================================================================
VoNode::~VoNode()
{
  quit_ = true;
  img_cv_.notify_all();
  sub_img_.reset();
  sub_imu_.reset();
  sub_remote_key_.reset();
  visualizer_.reset();
  delete vo_;
  delete cam_;
  if (user_input_thread_ != nullptr)
    user_input_thread_->stop();
}

// =============================================================================
// IMU callback (producer thread — ROS callback executor)
// =============================================================================
void VoNode::imuCb(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
  const double t = rclcpp::Time(msg->header.stamp).seconds();
  ++imu_meas_count_;

  if (last_imu_msg_ts_ > 0.0)
  {
    const double dt = t - last_imu_msg_ts_;
    if (dt > 1.0 / imu_calib_.imu_rate * 100.0)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "IMU gap: %.1f ms", dt * 1000.0);
    }
  }
  last_imu_msg_ts_ = t;

  if (imu_timeout_warned_)
  {
    RCLCPP_INFO(this->get_logger(), "IMU data resumed");
    imu_timeout_warned_ = false;
  }

  const double omega_norm = std::sqrt(
      msg->angular_velocity.x * msg->angular_velocity.x +
      msg->angular_velocity.y * msg->angular_velocity.y +
      msg->angular_velocity.z * msg->angular_velocity.z);
  const double acc_norm = std::sqrt(
      msg->linear_acceleration.x * msg->linear_acceleration.x +
      msg->linear_acceleration.y * msg->linear_acceleration.y +
      msg->linear_acceleration.z * msg->linear_acceleration.z);

  if (omega_norm > imu_calib_.saturation_omega_max ||
      acc_norm > imu_calib_.saturation_accel_max)
    return;

  imu_handler_->addImuMeasurement(
      svo::ImuMeasurement(t,
          Eigen::Vector3d(msg->angular_velocity.x,
                           msg->angular_velocity.y,
                           msg->angular_velocity.z),
          Eigen::Vector3d(msg->linear_acceleration.x,
                           msg->linear_acceleration.y,
                           msg->linear_acceleration.z)));
}

// =============================================================================
// Image callback (producer — throttles, then signals consumer)
// =============================================================================
void VoNode::imgCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  const double timestamp = rclcpp::Time(msg->header.stamp).seconds();

  if (enable_frame_throttle_ && target_fps_ > 0.0)
  {
    const double min_dt = 1.0 / target_fps_;
    if (timestamp - last_accepted_ts_ < min_dt)
      return;
  }
  last_accepted_ts_ = timestamp;

  {
    std::lock_guard<std::mutex> lock(img_mutex_);
    latest_img_ = msg;
    new_img_ready_ = true;
  }
  img_cv_.notify_one();
}

// =============================================================================
// Main processing loop (consumer — waits on condition variable)
// =============================================================================
void VoNode::run()
{
  RCLCPP_INFO(this->get_logger(), "SVO processing loop started");

  while (rclcpp::ok() && !quit_)
  {
    sensor_msgs::msg::Image::ConstSharedPtr img;
    double timestamp = 0.0;

    // Wait for next image
    {
      std::unique_lock<std::mutex> lock(img_mutex_);
      img_cv_.wait_for(lock, std::chrono::milliseconds(100),
                       [this]() { return new_img_ready_ || quit_; });
      if (quit_ || !new_img_ready_)
        continue;
      img = latest_img_;
      timestamp = rclcpp::Time(img->header.stamp).seconds();
      new_img_ready_ = false;
    }

    processUserActions();
    if (quit_)
      break;

    // --- IMU prior ---
    if (use_imu_ && imu_handler_)
    {
      if (!vo_->hasStarted() && set_initial_attitude_from_gravity_)
      {
        Eigen::Quaterniond R_imu_world;
        if (imu_handler_->getInitialAttitude(timestamp, R_imu_world))
        {
          vo_->setRotationPrior(R_imu_world);
          last_imu_rotation_ = R_imu_world;
        }
      }
      else if (vo_->hasStarted() && vo_->lastFrame() != nullptr)
      {
        Eigen::Quaterniond R_imu_last_cur;
        if (imu_handler_->getRelativeRotationPrior(
                vo_->lastFrame()->timestamp_, timestamp, false, R_imu_last_cur))
        {
          vo_->setRotationIncrementPrior(R_imu_last_cur);
          last_imu_rotation_ = R_imu_last_cur * last_imu_rotation_;
        }
      }
    }

    // --- Image conversion ---
    cv::Mat img_cv;
    try
    {
      img_cv = cv_bridge::toCvShare(img, "mono8")->image;
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
      continue;
    }

    // --- SVO ---
    vo_->addImage(img_cv, timestamp);

    // --- Visualization ---
    if (enable_visualization_)
    {
      visualizer_->publishMinimal(img_cv, vo_->lastFrame(), *vo_, timestamp);

      if (publish_markers_ &&
          vo_->stage() != svo::FrameHandlerBase::STAGE_PAUSED &&
          timestamp - last_marker_publish_ts_ > 0.2)
      {
        visualizer_->visualizeMarkers(
            vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());
        last_marker_publish_ts_ = timestamp;
      }

      if (publish_dense_input_)
        visualizer_->exportToDense(vo_->lastFrame());
    }

    if (vo_->stage() == svo::FrameHandlerMono::STAGE_PAUSED)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(this->get_logger(), "SVO processing loop finished");
}

// =============================================================================
void VoNode::processUserActions()
{
  char input = 0;
  {
    std::lock_guard<std::mutex> lock(remote_mutex_);
    if (!remote_input_.empty())
    {
      input = remote_input_[0];
      remote_input_.clear();
    }
  }

  if (user_input_thread_ != nullptr)
  {
    const char c = user_input_thread_->getInput();
    if (c != 0)
      input = c;
  }

  switch (input)
  {
    case 'q':
      quit_ = true;
      break;
    case 'r':
      vo_->reset();
      visualizer_->resetTrajectory();
      if (imu_handler_)
      {
        imu_handler_->reset();
        last_imu_rotation_ = Eigen::Quaterniond::Identity();
      }
      break;
    case 's':
      vo_->start();
      break;
  }
}

void VoNode::remoteKeyCb(const std_msgs::msg::String::ConstSharedPtr& msg)
{
  std::lock_guard<std::mutex> lock(remote_mutex_);
  remote_input_ = msg->data;
}

}  // namespace svo

// =============================================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<svo::VoNode>();
  node->init();

  RCLCPP_INFO(node->get_logger(), "SVO node ready");

  // IMU and remote-key callbacks run in the ROS executor (MultiThreadedExecutor).
  // The SVO processing loop runs in its own thread and waits on a condition
  // variable for each new image. This keeps IMU callbacks non-blocking.
  std::thread svo_thread([&node]() { node->run(); });

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  node->quitFlag() = true;
  node->imgCv().notify_all();

  if (svo_thread.joinable())
    svo_thread.join();

  RCLCPP_INFO(node->get_logger(), "SVO shut down");
  rclcpp::shutdown();
  return 0;
}
