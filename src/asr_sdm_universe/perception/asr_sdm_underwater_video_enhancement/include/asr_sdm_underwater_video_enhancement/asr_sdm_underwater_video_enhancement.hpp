#ifndef ASR_SDM_UNDERWATER_VIDEO_ENHANCEMENT_HPP_
#define ASR_SDM_UNDERWATER_VIDEO_ENHANCEMENT_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#ifdef HAVE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.hpp>  // TODO: Change it back to .h once the issue is resolved.
#endif

namespace asr_sdm_underwater_video_enhancement
{
using namespace cv;
using sensor_msgs::msg::Image;
using std::placeholders::_1;

class UnderWaterVideoEnhancementNode : public rclcpp::Node
{
public:
  explicit UnderWaterVideoEnhancementNode(const rclcpp::NodeOptions & node_options);

private:
  // Parameter
  float airlight;
  float beta_b;
  float beta_g;
  float beta_r;
  bool is_show_fps;
  bool is_resize;
  double resize_scale;

  // Subscriber
  rclcpp::Subscription<Image>::SharedPtr sub_image_;

  // Publisher
  rclcpp::Publisher<Image>::SharedPtr pub_image_;

  // Callback
  void onImageCallback(const Image::SharedPtr msg);

  // Function
  Mat calcBGRChannel(const Mat & src);
  Mat calcTransmissionMap(const Mat & b_channel, float A, float beta_b, float beta_g, float beta_r);
  Mat imageRestoration(const Mat & src, const Mat & t_map, float A);
  Mat processFrame(const Mat & frame, float airlight, float beta_b, float beta_g, float beta_r);

  // FPS
  std::chrono::steady_clock::time_point last_frame_time_;
  double avg_fps_ = 0.0;
  int frame_count_ = 0;
};
}  // namespace asr_sdm_underwater_video_enhancement

#endif  // ASR_SDM_UNDERWATER_VIDEO_ENHANCEMENT_HPP_
