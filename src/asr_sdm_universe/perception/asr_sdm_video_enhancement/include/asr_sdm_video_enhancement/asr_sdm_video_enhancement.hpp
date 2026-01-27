#ifndef ASR_SDM_VIDEO_ENHANCEMENT_HPP_
#define ASR_SDM_VIDEO_ENHANCEMENT_HPP_

#include <cv_bridge/cv_bridge.hpp>  // ubuntu 24.04 -> cv_bridge.hpp; ubuntu 22.04 -> cv_bridge.h
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

namespace asr_sdm_video_enhancement
{
using namespace cv;
using sensor_msgs::msg::Image;
using std::placeholders::_1;

class VideoEnhancementNode : public rclcpp::Node
{
public:
  explicit VideoEnhancementNode(const rclcpp::NodeOptions & node_options);

private:
  // Parameter
  int airlight;
  double scale;

  // Subscriber
  rclcpp::Subscription<Image>::SharedPtr sub_image_;

  // Publisher
  rclcpp::Publisher<Image>::SharedPtr pub_image_;

  // Callback
  void onImageCallback(const Image::SharedPtr msg);

  // Function
  double avePixel(const Mat & src);
  Mat calcYchannel(const Mat & src);
  Mat calcTransmission(const Mat & src, const Mat & Mmed, int airlight);
  Mat dehazing(const Mat & src, const Mat & transmission, int airlight);
  Mat processFrame(const Mat & frame, int airlight);
  void gammaCorrection(const Mat & src, Mat & dst, float gamma);

  // FPS
  std::chrono::steady_clock::time_point last_frame_time_;
  double avg_fps_ = 0.0;
  int frame_count_ = 0;
};
}  // namespace asr_sdm_video_enhancement

#endif  // ASR_SDM_VIDEO_ENHANCEMENT_HPP_
