#pragma once

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>

#include <asr_sdm_vio/common/types.h>
#include <asr_sdm_vio/common/camera_fwd.h>
#include <asr_sdm_vio/common/transformation.h>

namespace svo {

// forward declarations
class FrameHandlerBase;
class Visualizer;
class ImuHandler;
class BackendInterface;
class CeresBackendInterface;
class CeresBackendPublisher;

enum class PipelineType {
  kMono,
  kStereo,
  kArray
};

/// SVO Interface
class SvoInterface
{
public:
  using ImageMsg = sensor_msgs::msg::Image;
  using ImuMsg = sensor_msgs::msg::Imu;
  using StringMsg = std_msgs::msg::String;

  explicit SvoInterface(const PipelineType& pipeline_type,
                        const rclcpp::Node::SharedPtr& node);

  virtual ~SvoInterface();

  // Processing
  void processImageBundle(
      const std::vector<cv::Mat>& images,
      int64_t timestamp_nanoseconds);

  bool setImuPrior(const int64_t timestamp_nanoseconds);

  void publishResults(
      const std::vector<cv::Mat>& images,
      const int64_t timestamp_nanoseconds);

  // Subscription and callbacks
  void monoCallback(const ImageMsg::ConstSharedPtr& msg);
  void stereoCallback(
      const ImageMsg::ConstSharedPtr& msg0,
      const ImageMsg::ConstSharedPtr& msg1);
  void imuCallback(const ImuMsg::ConstSharedPtr& imu_msg);
  void inputKeyCallback(const StringMsg::ConstSharedPtr& key_input);

  // These functions are called before and after monoCallback or stereoCallback.
  // a derived class can implement some additional logic here.
  virtual void imageCallbackPreprocessing(int64_t timestamp_nanoseconds) {}
  virtual void imageCallbackPostprocessing() {}

  void subscribeImu();
  void subscribeImage();
  void subscribeRemoteKey();

  // ROS2 subscriptions are kept alive by storing them as members.
  rclcpp::Subscription<ImuMsg>::SharedPtr sub_imu_;
  image_transport::Subscriber sub_mono_image_;
  // (stereo subscriptions are not yet migrated)


  const rclcpp::Node::SharedPtr& getNode() const { return node_; }

  // System state.
  bool quit_ = false;
  bool idle_ = false;
  bool automatic_reinitialization_ = false;

  // Parameters
  bool set_initial_attitude_from_gravity_ = true;

  // SVO modules.
  std::shared_ptr<FrameHandlerBase> svo_;
  std::shared_ptr<Visualizer> visualizer_;
  std::shared_ptr<ImuHandler> imu_handler_;
  std::shared_ptr<BackendInterface> backend_interface_;
  std::shared_ptr<CeresBackendInterface> ceres_backend_interface_;
  std::shared_ptr<CeresBackendPublisher> ceres_backend_publisher_;

  CameraBundlePtr ncam_;

private:
  void imuLoop();
  void monoLoop();
  void stereoLoop();

  rclcpp::Node::SharedPtr node_;
  PipelineType pipeline_type_;

  rclcpp::Subscription<StringMsg>::SharedPtr sub_remote_key_;

  std::string remote_input_;

  std::unique_ptr<std::thread> imu_thread_;
  std::unique_ptr<std::thread> image_thread_;

  // Callback groups to mimic original per-thread callback queues.
  rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
  rclcpp::CallbackGroup::SharedPtr image_cb_group_;
  rclcpp::CallbackGroup::SharedPtr key_cb_group_;
};

} // namespace svo
