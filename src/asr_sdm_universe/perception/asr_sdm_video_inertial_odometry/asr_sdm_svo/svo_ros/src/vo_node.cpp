#include <Eigen/Core>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo_ros/visualizer.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/math_utils.h>
#include <vikit/params_helper.h>
#include <vikit/pinhole_camera.h>
#include <vikit/user_input_thread.h>

#include <string>

namespace svo
{

/// SVO Interface as ROS2 Node
class VoNode : public rclcpp::Node
{
public:
  svo::FrameHandlerMono * vo_;
  std::unique_ptr<svo::Visualizer> visualizer_;
  bool publish_markers_;
  bool publish_dense_input_;
  std::shared_ptr<vk::UserInputThread> user_input_thread_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_remote_key_;
  std::string remote_input_;
  vk::AbstractCamera * cam_;
  bool quit_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber it_sub_;

  VoNode() : Node("svo"), vo_(nullptr), cam_(nullptr), quit_(false)
  {
    // Declare and get parameters
    publish_markers_ = vk::getParam<bool>(this, "publish_markers", true);
    publish_dense_input_ = vk::getParam<bool>(this, "publish_dense_input", false);
    remote_input_ = "";

    // Start user input thread in parallel thread that listens to console keys
    if (vk::getParam<bool>(this, "accept_console_user_input", true))
      user_input_thread_ = std::make_shared<vk::UserInputThread>();

    // Check if we should use hardcoded dataset parameters
    std::string dataset_name = vk::getParam<std::string>(this, "dataset_name", "");

    // 加载相机参数并创建相机模型
    // 说明：这里使用 vikit::camera_loader（而不是手动 new PinholeCamera），
    // 因为 camera_loader 支持 Pinhole + 径向-切向畸变参数（cam_d0..cam_d3，对应 OpenCV 的 k1,k2,p1,p2）。

    if (dataset_name == "euroc") {
      RCLCPP_INFO(this->get_logger(), "Using hardcoded EuRoC dataset parameters");
      // Keep the old behavior for dataset_name==euroc (no distortion)
      cam_ = new vk::PinholeCamera(752, 480, 458.654, 457.296, 367.215, 248.375);
      RCLCPP_INFO(this->get_logger(), "Successfully created 'PINHOLE' camera model (hardcoded)." );
    } else {
      std::string cam_model = vk::getParam<std::string>(this, "cam_model", "Pinhole");
      // Normalize common variants
      if (cam_model == "PINHOLE") cam_model = "Pinhole";

      bool ok = vk::camera_loader::loadFromRosNode(this, "", cam_);
      if (!ok || cam_ == nullptr) {
        throw std::runtime_error("Failed to load camera from ROS parameters. cam_model=" + cam_model);
      }
      RCLCPP_INFO(this->get_logger(), "Successfully created '%s' camera model.", cam_model.c_str());
    }

    // Override SVO core parameters from ROS2 params before creating VO
    try {
      int grid, maxfts, nlevels;
      double tri;

      if (dataset_name == "euroc") {
        // Hardcoded EuRoC algorithm parameters for optimal performance
        RCLCPP_INFO(this->get_logger(), "Using hardcoded EuRoC algorithm parameters");
        grid = 18;
        maxfts = 400;
        tri = 3.0;
        nlevels = 4;

        // Additional EuRoC-specific tracking parameters
        svo::Config::kltMaxLevel() = 4;
        svo::Config::kltMinLevel() = 0;
        svo::Config::reprojThresh() = 4.0;     // More tolerant reprojection threshold
        svo::Config::poseOptimThresh() = 4.0;  // More tolerant pose optim threshold
        svo::Config::poseOptimNumIter() = 10;
        svo::Config::qualityMinFts() = 40;      // More tolerant quality threshold
        svo::Config::qualityMaxFtsDrop() = 80;  // Tolerate larger drops
        svo::Config::initMinDisparity() = 50.0;
        svo::Config::initMinTracked() = 50;
        svo::Config::initMinInliers() = 40;
        // Keyframe and map scale tuning
        svo::Config::kfSelectMinDist() = 0.001;
        svo::Config::maxNKfs() = 180;
        svo::Config::mapScale() = 5.0;
        // Misc flags
        svo::Config::useImu() = true;  // currently not used in this codebase
        svo::Config::useThreadedDepthfilter() =
          false;  // run depth filter synchronously for bag playback
        svo::Config::patchMatchThresholdFactor() = 1.5;  // relax ZMSSD acceptance
        svo::Config::subpixNIter() = 20;                 // more iterations for subpixel alignment
        // Lower feature quality requirement to avoid frequent relocalization
        svo::Config::qualityMinFts() = 30;

        RCLCPP_INFO(
          this->get_logger(), "EuRoC config: grid=%d, max_fts=%d, triang_score=%.1f, pyr_levels=%d",
          grid, maxfts, tri, nlevels);
        RCLCPP_INFO(
          this->get_logger(),
          "EuRoC tracking: reproj_thresh=%.1f, quality_min_fts=%zu, quality_max_drop=%d, "
          "kf_min_dist=%.4f, max_kfs=%zu, map_scale=%.1f, zmssd_factor=%.2f, subpix_iter=%zu",
          svo::Config::reprojThresh(), svo::Config::qualityMinFts(),
          svo::Config::qualityMaxFtsDrop(), svo::Config::kfSelectMinDist(), svo::Config::maxNKfs(),
          svo::Config::mapScale(), svo::Config::patchMatchThresholdFactor(),
          svo::Config::subpixNIter());
      } else {
        // Load parameters from ROS2 parameter server (default behavior)
        grid = vk::getParam<int>(this, "grid_size", static_cast<int>(svo::Config::gridSize()));
        maxfts = vk::getParam<int>(this, "max_fts", static_cast<int>(svo::Config::maxFts()));
        tri = vk::getParam<double>(
          this, "triang_min_corner_score", svo::Config::triangMinCornerScore());
        nlevels =
          vk::getParam<int>(this, "n_pyr_levels", static_cast<int>(svo::Config::nPyrLevels()));

        // 将关键的鲁棒性参数从 ROS2 参数写回到 SVO 核心单例 Config。
        // 原因：SVO 核心算法使用 svo::Config::* 的静态单例；
        // 仅仅把参数作为 ROS param 传入并不足够，必须显式写回 Config 才会真正生效。
        // 这些参数以前只在 dataset_name=="euroc" 的硬编码分支里设置，导致非 euroc 分支跟踪很不稳定。
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
        // max_n_kfs is a node param already; keep SVO core in sync too.
        svo::Config::maxNKfs() = static_cast<size_t>(vk::getParam<int>(
          this, "max_n_kfs", static_cast<int>(svo::Config::maxNKfs())));
        svo::Config::mapScale() = vk::getParam<double>(
          this, "map_scale", svo::Config::mapScale());
        svo::Config::patchMatchThresholdFactor() = vk::getParam<double>(
          this, "patch_match_thresh_factor", svo::Config::patchMatchThresholdFactor());

        // KLT tracker tuning (for fast motion / larger inter-frame displacement)
        svo::Config::kltMinLevel() = static_cast<size_t>(vk::getParam<int>(
          this, "klt_min_level", static_cast<int>(svo::Config::kltMinLevel())));
        svo::Config::kltMaxLevel() = static_cast<size_t>(vk::getParam<int>(
          this, "klt_max_level", static_cast<int>(svo::Config::kltMaxLevel())));

        // Subpixel refinement iterations
        svo::Config::subpixNIter() = static_cast<size_t>(vk::getParam<int>(
          this, "subpix_n_iter", static_cast<int>(svo::Config::subpixNIter())));
      }

      svo::Config::gridSize() = static_cast<size_t>(grid);
      svo::Config::maxFts() = static_cast<size_t>(maxfts);
      svo::Config::triangMinCornerScore() = tri;
      svo::Config::nPyrLevels() = static_cast<size_t>(nlevels);

      // 打印最终生效的 SVO 核心配置（权威值）。
      // 用途：排查“yaml 改了但没生效 / 参数被覆盖 / 类型不匹配”等问题时非常关键。
      RCLCPP_INFO(
        this->get_logger(),
        "SVO Config(final): grid=%zu max_fts=%zu pyr_levels=%zu tri_score=%.2f | reproj=%.2f poseopt=%.2f | "
        "quality_min=%zu quality_max_drop=%d | kf_min_dist=%.4f max_kfs=%zu map_scale=%.2f | "
        "klt_levels=[%zu..%zu] zmssd_factor=%.2f subpix_iter=%zu",
        svo::Config::gridSize(), svo::Config::maxFts(), svo::Config::nPyrLevels(),
        svo::Config::triangMinCornerScore(), svo::Config::reprojThresh(),
        svo::Config::poseOptimThresh(), svo::Config::qualityMinFts(),
        svo::Config::qualityMaxFtsDrop(), svo::Config::kfSelectMinDist(),
        svo::Config::maxNKfs(), svo::Config::mapScale(), svo::Config::kltMinLevel(),
        svo::Config::kltMaxLevel(), svo::Config::patchMatchThresholdFactor(),
        svo::Config::subpixNIter());
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Parameter override failed: %s", e.what());
    }

    // Init VO and start
    vo_ = new svo::FrameHandlerMono(cam_);
    vo_->start();

    // Subscribe to remote input
    sub_remote_key_ = this->create_subscription<std_msgs::msg::String>(
      "remote_key", 5, std::bind(&VoNode::remoteKeyCb, this, std::placeholders::_1));
  }

  // Initialize components that need shared_from_this() - must be called after construction
  void init()
  {
    // Create visualizer (needs shared_from_this)
    visualizer_ = std::make_unique<svo::Visualizer>(this->shared_from_this());

    // Get initial position and orientation
    visualizer_->T_world_from_vision_ = Sophus::SE3d(
      vk::rpy2dcm(
        Eigen::Vector3d(
          vk::getParam<double>(this, "init_rx", 0.0), vk::getParam<double>(this, "init_ry", 0.0),
          vk::getParam<double>(this, "init_rz", 0.0))),
      Eigen::Vector3d(
        vk::getParam<double>(this, "init_tx", 0.0), vk::getParam<double>(this, "init_ty", 0.0),
        vk::getParam<double>(this, "init_tz", 0.0)));

    // Subscribe to cam msgs (needs shared_from_this)
    std::string cam_topic = vk::getParam<std::string>(this, "cam_topic", "camera/image_raw");
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    it_sub_ = it_->subscribe(cam_topic, 5, std::bind(&VoNode::imgCb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SVO node initialized, subscribing to: %s", cam_topic.c_str());
  }

  ~VoNode()
  {
    delete vo_;
    delete cam_;
    if (user_input_thread_ != nullptr) user_input_thread_->stop();
  }

  void imgCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv::Mat img;
    try {
      img = cv_bridge::toCvShare(msg, "mono8")->image;
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    processUserActions();

    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    vo_->addImage(img, timestamp);
    visualizer_->publishMinimal(img, vo_->lastFrame(), *vo_, timestamp);

    if (publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
      visualizer_->visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

    if (publish_dense_input_) visualizer_->exportToDense(vo_->lastFrame());

    if (vo_->stage() == FrameHandlerMono::STAGE_PAUSED) usleep(100000);
  }

  void processUserActions()
  {
    char input = remote_input_.empty() ? 0 : remote_input_.c_str()[0];
    remote_input_ = "";

    if (user_input_thread_ != nullptr) {
      char console_input = user_input_thread_->getInput();
      if (console_input != 0) input = console_input;
    }

    switch (input) {
      case 'q':
        quit_ = true;
        RCLCPP_INFO(this->get_logger(), "SVO user input: QUIT");
        break;
      case 'r':
        vo_->reset();
        RCLCPP_INFO(this->get_logger(), "SVO user input: RESET");
        break;
      case 's':
        vo_->start();
        RCLCPP_INFO(this->get_logger(), "SVO user input: START");
        break;
      default:;
    }
  }

  void remoteKeyCb(const std_msgs::msg::String::SharedPtr key_input)
  {
    remote_input_ = key_input->data;
  }

  bool shouldQuit() const { return quit_; }
};

}  // namespace svo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<svo::VoNode>();

  // Initialize components that need shared_from_this()
  node->init();

  RCLCPP_INFO(node->get_logger(), "SVO node created");

  rclcpp::Rate rate(100);
  while (rclcpp::ok() && !node->shouldQuit()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "SVO terminated.");
  rclcpp::shutdown();
  return 0;
}
