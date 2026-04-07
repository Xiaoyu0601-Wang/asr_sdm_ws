#ifndef ASR_SDM_VIDEO_ENHANCEMENT_HPP_
#define ASR_SDM_VIDEO_ENHANCEMENT_HPP_

#include "asr_sdm_video_enhancement/enhancement_processor.hpp"

#include <cv_bridge/cv_bridge.hpp>  // ubuntu 24.04 -> cv_bridge.hpp; ubuntu 22.04 -> cv_bridge.h
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <chrono>
#include <string>

namespace asr_sdm_video_enhancement
{
using sensor_msgs::msg::CompressedImage;
using std::placeholders::_1;

class VideoEnhancementNode : public rclcpp::Node
{
public:
  explicit VideoEnhancementNode(const rclcpp::NodeOptions & node_options);

private:
  // Parameter
  EnhancementConfig config_;
  bool overlay_fps_text_ = false;
  EnhancementProcessor processor_;

  // Subscriber
  rclcpp::Subscription<CompressedImage>::SharedPtr sub_image_;

  // Publisher
  image_transport::Publisher pub_image_;

  // Callback
  void onImageCallback(const CompressedImage::SharedPtr msg);

  // FPS
  std::chrono::steady_clock::time_point last_frame_time_;
  double avg_fps_ = 0.0;
  int frame_count_ = 0;
};
}  // namespace asr_sdm_video_enhancement

#endif  // ASR_SDM_VIDEO_ENHANCEMENT_HPP_
