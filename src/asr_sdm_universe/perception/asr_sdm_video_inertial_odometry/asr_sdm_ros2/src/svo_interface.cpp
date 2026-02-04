#include <asr_sdm_vio/ceres_backend_interface.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include <sensor_msgs/image_encodings.hpp>

#include <asr_sdm_ros2/ceres_backend_factory.h>
#include <asr_sdm_ros2/svo_factory.h>
#include <asr_sdm_ros2/svo_interface.h>
#include <asr_sdm_ros2/visualizer.h>
#include <asr_sdm_vio/common/camera.h>
#include <asr_sdm_vio/common/conversions.h>
#include <asr_sdm_vio/common/frame.h>
#include <asr_sdm_vio/direct/depth_filter.h>
#include <asr_sdm_vio/frame_handler_array.h>
#include <asr_sdm_vio/frame_handler_mono.h>
#include <asr_sdm_vio/frame_handler_stereo.h>
#include <asr_sdm_vio/imu_handler.h>
#include <asr_sdm_vio/initialization.h>
#include <asr_sdm_vio/map.h>
#include <glog/logging.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <vikit/params_helper.h>
#include <vikit/timer.h>

#include <stdexcept>

#ifdef SVO_USE_GTSAM_BACKEND
#include <asr_sdm_ros2/backend_factory.h>
#include <asr_sdm_vio/backend/backend_interface.h>
#include <asr_sdm_vio/backend/backend_optimizer.h>
#endif

#ifdef SVO_LOOP_CLOSING
#include <asr_sdm_vio/online_loopclosing/loop_closing.h>
#endif

#ifdef SVO_GLOBAL_MAP
#include <asr_sdm_vio/global_map.h>
#endif

namespace
{
inline int64_t stampToNs(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}

inline double stampToSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}
}  // namespace

namespace svo
{

SvoInterface::SvoInterface(const PipelineType & pipeline_type, const rclcpp::Node::SharedPtr & node)
: node_(node),
  pipeline_type_(pipeline_type)
{
  if (!node_) {
    throw std::invalid_argument("SvoInterface: node is null");
  }

  set_initial_attitude_from_gravity_ =
    vk::param<bool>(node_, "set_initial_attitude_from_gravity", true);
  automatic_reinitialization_ = vk::param<bool>(node_, "automatic_reinitialization", false);

  imu_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  image_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  key_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  switch (pipeline_type_) {
    case PipelineType::kMono:
      svo_ = factory::makeMono(node_);
      break;
    case PipelineType::kStereo:
      svo_ = factory::makeStereo(node_);
      break;
    case PipelineType::kArray:
      svo_ = factory::makeArray(node_);
      break;
    default:
      LOG(FATAL) << "Unknown pipeline";
      break;
  }
  ncam_ = svo_->getNCamera();

  // Visualizer API in this repo is still ROS1-flavored; it must be ported similarly.
  // For now we keep construction to preserve control flow.
  visualizer_.reset(new Visualizer(svo_->options_.trace_dir, node_, ncam_->getNumCameras()));

  if (vk::param<bool>(node_, "use_imu", false)) {
    imu_handler_ = factory::getImuHandler(node_);
    svo_->imu_handler_ = imu_handler_;
  }

  if (vk::param<bool>(node_, "use_ceres_backend", false)) {
    ceres_backend_interface_ = ceres_backend_factory::makeBackend(node_, ncam_);
    if (imu_handler_) {
      svo_->setBundleAdjuster(ceres_backend_interface_);
      ceres_backend_interface_->setImu(imu_handler_);
      ceres_backend_interface_->setNodeAndCreatePublishers(node_.get());
    } else {
      LOG(ERROR) << "Cannot use ceres backend without using imu";
    }
  }

#ifdef SVO_USE_GTSAM_BACKEND
  if (vk::param<bool>(node_, "use_backend", false)) {
    backend_interface_ = svo::backend_factory::makeBackend(node_);
    ceres_backend_publisher_.reset(new CeresBackendPublisher(svo_->options_.trace_dir, *node_));
    svo_->setBundleAdjuster(backend_interface_);
    backend_interface_->imu_handler_ = imu_handler_;
  }
#endif

  if (vk::param<bool>(node_, "runlc", false)) {
#ifdef SVO_LOOP_CLOSING
    LoopClosingPtr loop_closing_ptr = factory::getLoopClosingModule(node_, svo_->getNCamera());
    svo_->lc_ = std::move(loop_closing_ptr);
    CHECK(svo_->depth_filter_->options_.extra_map_points)
      << "The depth filter seems to be initialized without extra map points.";
#else
    LOG(FATAL) << "You have to enable loop closing in svo_cmake.";
#endif
  }

  if (vk::param<bool>(node_, "use_global_map", false)) {
#ifdef SVO_GLOBAL_MAP
    svo_->global_map_ = factory::getGlobalMap(node_, svo_->getNCamera());
    if (imu_handler_) {
      svo_->global_map_->initializeIMUParams(imu_handler_->imu_calib_, imu_handler_->imu_init_);
    }
#else
    LOG(FATAL) << "You have to enable global map in cmake";
#endif
  }

  svo_->start();
}

SvoInterface::~SvoInterface()
{
  if (imu_thread_) imu_thread_->join();
  if (image_thread_) image_thread_->join();
  VLOG(1) << "Destructed SVO.";
}

void SvoInterface::processImageBundle(
  const std::vector<cv::Mat> & images, const int64_t timestamp_nanoseconds)
{
  if (!svo_->isBackendValid()) {
    if (vk::param<bool>(node_, "use_ceres_backend", false, true)) {
      ceres_backend_interface_ = ceres_backend_factory::makeBackend(node_, ncam_);
      if (imu_handler_) {
        svo_->setBundleAdjuster(ceres_backend_interface_);
        ceres_backend_interface_->setImu(imu_handler_);
        ceres_backend_interface_->setNodeAndCreatePublishers(node_.get());
      } else {
        LOG(ERROR) << "Cannot use ceres backend without using imu";
      }
    }
  }
  svo_->addImageBundle(images, timestamp_nanoseconds);
}

void SvoInterface::publishResults(
  const std::vector<cv::Mat> & images, const int64_t timestamp_nanoseconds)
{
  CHECK_NOTNULL(svo_.get());
  CHECK_NOTNULL(visualizer_.get());

  visualizer_->img_caption_.clear();
  if (svo_->isBackendValid()) {
    std::string static_str = ceres_backend_interface_->getStationaryStatusStr();
    visualizer_->img_caption_ = static_str;
  }

  visualizer_->publishSvoInfo(svo_.get(), timestamp_nanoseconds);
  switch (svo_->stage()) {
    case Stage::kTracking: {
      Eigen::Matrix<double, 6, 6> covariance;
      covariance.setZero();
      visualizer_->publishImuPose(
        svo_->getLastFrames()->get_T_W_B(), covariance, timestamp_nanoseconds);
      visualizer_->publishCameraPoses(svo_->getLastFrames(), timestamp_nanoseconds);
      visualizer_->visualizeMarkers(svo_->getLastFrames(), svo_->closeKeyframes(), svo_->map());
      visualizer_->exportToDense(svo_->getLastFrames());
      bool draw_boundary = false;
      if (svo_->isBackendValid()) {
        draw_boundary = svo_->getBundleAdjuster()->isFixedToGlobalMap();
      }
      visualizer_->publishImagesWithFeatures(
        svo_->getLastFrames(), timestamp_nanoseconds, draw_boundary);
#ifdef SVO_LOOP_CLOSING
      if (svo_->lc_) {
        visualizer_->publishLoopClosureInfo(
          svo_->lc_->cur_loop_check_viz_info_, std::string("loop_query"),
          Eigen::Vector3f(0.0f, 0.0f, 1.0f), 0.5);
        visualizer_->publishLoopClosureInfo(
          svo_->lc_->loop_detect_viz_info_, std::string("loop_detection"),
          Eigen::Vector3f(1.0f, 0.0f, 0.0f), 1.0);
        if (svo_->isBackendValid()) {
          visualizer_->publishLoopClosureInfo(
            svo_->lc_->loop_correction_viz_info_, std::string("loop_correction"),
            Eigen::Vector3f(0.0f, 1.0f, 0.0f), 3.0);
        }
        if (svo_->getLastFrames()->at(0)->isKeyframe()) {
          bool pc_recalculated = visualizer_->publishPoseGraph(
            svo_->lc_->kf_list_, svo_->lc_->need_to_update_pose_graph_viz_,
            static_cast<size_t>(svo_->lc_->options_.ignored_past_frames));
          if (pc_recalculated) {
            svo_->lc_->need_to_update_pose_graph_viz_ = false;
          }
        }
      }
#endif
#ifdef SVO_GLOBAL_MAP
      if (svo_->global_map_) {
        visualizer_->visualizeGlobalMap(
          *(svo_->global_map_), std::string("global_vis"), Eigen::Vector3f(0.0f, 0.0f, 1.0f), 0.3);
        visualizer_->visualizeFixedLandmarks(svo_->getLastFrames()->at(0));
      }
#endif
      break;
    }
    case Stage::kInitializing: {
      visualizer_->publishBundleFeatureTracks(
        svo_->initializer_->frames_ref_, svo_->getLastFrames(), timestamp_nanoseconds);
      break;
    }
    case Stage::kPaused:
    case Stage::kRelocalization:
      visualizer_->publishImages(images, timestamp_nanoseconds);
      break;
    default:
      LOG(FATAL) << "Unknown stage";
      break;
  }

#ifdef SVO_USE_GTSAM_BACKEND
  if (svo_->stage() == Stage::kTracking && backend_interface_) {
    if (svo_->getLastFrames()->isKeyframe()) {
      std::lock_guard<std::mutex> estimate_lock(backend_interface_->optimizer_->estimate_mut_);
      const gtsam::Values & state = backend_interface_->optimizer_->estimate_;
      ceres_backend_publisher_->visualizeFrames(state);
      if (backend_interface_->options_.add_imu_factors)
        ceres_backend_publisher_->visualizeVelocity(state);
      ceres_backend_publisher_->visualizePoints(state);
    }
  }
#endif
}

bool SvoInterface::setImuPrior(const int64_t timestamp_nanoseconds)
{
  if (svo_->getBundleAdjuster()) {
    if (!svo_->hasStarted()) {
      if (imu_handler_->getMeasurementsCopy().size() < 10u) {
        return false;
      }
    }
    return true;
  }

  if (imu_handler_ && !svo_->hasStarted() && set_initial_attitude_from_gravity_) {
    Quaternion R_imu_world;
    if (imu_handler_->getInitialAttitude(
          timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds, R_imu_world)) {
      VLOG(3) << "Set initial orientation from accelerometer measurements.";
      svo_->setRotationPrior(R_imu_world);
    } else {
      return false;
    }
  } else if (imu_handler_ && svo_->getLastFrames()) {
    Quaternion R_lastimu_newimu;
    if (imu_handler_->getRelativeRotationPrior(
          svo_->getLastFrames()->getMinTimestampNanoseconds() *
            common::conversions::kNanoSecondsToSeconds,
          timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds, false,
          R_lastimu_newimu)) {
      VLOG(3) << "Set incremental rotation prior from IMU.";
      svo_->setRotationIncrementPrior(R_lastimu_newimu);
    }
  }
  return true;
}

void SvoInterface::monoCallback(const ImageMsg::ConstSharedPtr & msg)
{
  if (idle_) return;

  cv::Mat image;
  try {
    image = cv_bridge::toCvCopy(msg)->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  std::vector<cv::Mat> images;
  images.push_back(image.clone());

  const int64_t stamp_ns = stampToNs(msg->header.stamp);

  if (!setImuPrior(stamp_ns)) {
    VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
    return;
  }

  imageCallbackPreprocessing(stamp_ns);
  processImageBundle(images, stamp_ns);
  publishResults(images, stamp_ns);

  if (svo_->stage() == Stage::kPaused && automatic_reinitialization_) svo_->start();

  imageCallbackPostprocessing();
}

void SvoInterface::stereoCallback(
  const ImageMsg::ConstSharedPtr & msg0, const ImageMsg::ConstSharedPtr & msg1)
{
  if (idle_) return;

  cv::Mat img0, img1;
  try {
    img0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8)->image;
    img1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8)->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const int64_t stamp_ns = stampToNs(msg0->header.stamp);

  if (!setImuPrior(stamp_ns)) {
    VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
    return;
  }

  imageCallbackPreprocessing(stamp_ns);

  processImageBundle({img0, img1}, stamp_ns);
  publishResults({img0, img1}, stamp_ns);

  if (svo_->stage() == Stage::kPaused && automatic_reinitialization_) svo_->start();

  imageCallbackPostprocessing();
}

void SvoInterface::imuCallback(const ImuMsg::ConstSharedPtr & msg)
{
  const Eigen::Vector3d omega_imu(
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  const Eigen::Vector3d lin_acc_imu(
    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const ImuMeasurement m(stampToSec(msg->header.stamp), omega_imu, lin_acc_imu);
  if (imu_handler_)
    imu_handler_->addImuMeasurement(m);
  else
    LOG(ERROR) << "SvoNode has no ImuHandler";
}

void SvoInterface::inputKeyCallback(const StringMsg::ConstSharedPtr & key_input)
{
  std::string remote_input = key_input->data;
  char input = remote_input.c_str()[0];
  switch (input) {
    case 'q':
      quit_ = true;
      VLOG(1) << "SVO user input: QUIT";
      break;
    case 'r':
      svo_->reset();
      idle_ = true;
      VLOG(1) << "SVO user input: RESET";
      break;
    case 's':
      svo_->start();
      idle_ = false;
      VLOG(1) << "SVO user input: START";
      break;
    case 'c':
      svo_->setCompensation(true);
      VLOG(1) << "Enabled affine compensation.";
      break;
    case 'C':
      svo_->setCompensation(false);
      VLOG(1) << "Disabled affine compensation.";
      break;
    default:
      break;
  }
}

void SvoInterface::subscribeImu()
{
  const std::string imu_topic = vk::param<std::string>(node_, "imu_topic", "imu");
  rclcpp::SubscriptionOptions opt;
  opt.callback_group = imu_cb_group_;
  sub_imu_ = node_->create_subscription<ImuMsg>(
    imu_topic, rclcpp::QoS(200), std::bind(&SvoInterface::imuCallback, this, std::placeholders::_1),
    opt);
}

void SvoInterface::subscribeImage()
{
  const std::string image_topic = vk::param<std::string>(node_, "cam0_topic", "camera/image_raw");

  rclcpp::SubscriptionOptions opt;
  opt.callback_group = image_cb_group_;

  sub_mono_image_ = image_transport::create_subscription(
    node_.get(), image_topic, std::bind(&SvoInterface::monoCallback, this, std::placeholders::_1),
    "raw", rclcpp::QoS(5).get_rmw_qos_profile(), opt);
}

void SvoInterface::subscribeRemoteKey()
{
  const std::string remote_key_topic =
    vk::param<std::string>(node_, "remote_key_topic", "svo/remote_key");

  rclcpp::SubscriptionOptions opt;
  opt.callback_group = key_cb_group_;

  sub_remote_key_ = node_->create_subscription<StringMsg>(
    remote_key_topic, rclcpp::QoS(5),
    std::bind(&SvoInterface::inputKeyCallback, this, std::placeholders::_1), opt);
}

void SvoInterface::imuLoop()
{
  VLOG(1) << "SvoNode: Started IMU loop.";

  // Dedicated executor: spin the callback group without adding the node to multiple executors.
  rclcpp::executors::SingleThreadedExecutor exec;

  const std::string imu_topic = vk::param<std::string>(node_, "imu_topic", "imu");

  rclcpp::SubscriptionOptions opt;
  opt.callback_group = imu_cb_group_;

  auto sub_imu = node_->create_subscription<ImuMsg>(
    imu_topic, rclcpp::QoS(200), std::bind(&SvoInterface::imuCallback, this, std::placeholders::_1),
    opt);

  while (rclcpp::ok() && !quit_) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void SvoInterface::monoLoop()
{
  VLOG(1) << "SvoNode: Started Image loop.";

  rclcpp::executors::SingleThreadedExecutor exec;

  const std::string image_topic = vk::param<std::string>(node_, "cam0_topic", "camera/image_raw");

  rclcpp::SubscriptionOptions opt;
  opt.callback_group = image_cb_group_;

  auto sub = image_transport::create_subscription(
    node_.get(), image_topic, std::bind(&SvoInterface::monoCallback, this, std::placeholders::_1),
    "raw", rclcpp::QoS(5).get_rmw_qos_profile(), opt);

  (void)sub;

  while (rclcpp::ok() && !quit_) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void SvoInterface::stereoLoop()
{
  // Stereo loop is kept for API parity but requires ROS2 message_filters + image_transport filters.
  // It will be enabled when stereo launch is migrated.
  VLOG(1) << "SvoNode: Started Stereo Image loop (not enabled in mono mode).";
}

}  // namespace svo
