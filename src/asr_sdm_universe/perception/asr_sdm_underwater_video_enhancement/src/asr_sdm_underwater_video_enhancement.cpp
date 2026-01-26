#include "asr_sdm_underwater_video_enhancement/asr_sdm_underwater_video_enhancement.hpp"

namespace asr_sdm_underwater_video_enhancement
{

UnderWaterVideoEnhancementNode::UnderWaterVideoEnhancementNode(
  const rclcpp::NodeOptions & node_options)
: Node("asr_sdm_underwater_video_enhancement", node_options)
{
  // Parameter
  airlight = static_cast<float>(declare_parameter("airlight", 0.0));
  beta_b = static_cast<float>(declare_parameter("beta_b", 0.0));
  beta_g = static_cast<float>(declare_parameter("beta_g", 0.0));
  beta_r = static_cast<float>(declare_parameter("beta_r", 0.0));
  is_show_fps = static_cast<bool>(declare_parameter("is_show_fps", false));
  is_resize = static_cast<bool>(declare_parameter("is_resize", false));
  resize_scale = static_cast<double>(declare_parameter("resize_scale", 0.5));

  // Subscriber
  sub_image_ = this->create_subscription<Image>(
    "input/image",
    rclcpp::SensorDataQoS(),  // QoS 明确设成 SensorDataQoS，和 RealSense 完全一致
    std::bind(&UnderWaterVideoEnhancementNode::onImageCallback, this, _1));

  // Publisher
  pub_image_ = this->create_publisher<Image>(
    "output/image",
    rclcpp::SensorDataQoS());  // QoS 明确设成 SensorDataQoS，和 RealSense 完全一致
}

// Callback
void UnderWaterVideoEnhancementNode::onImageCallback(const Image::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000, "Got image %ux%u", msg->width,
    msg->height);  // 节流
  try {
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    // Calculate FPS
    if (is_show_fps) {
      auto now = std::chrono::steady_clock::now();
      if (frame_count_ > 0) {
        double elapsed = std::chrono::duration<double>(now - last_frame_time_).count();
        double fps_now = 1.0 / elapsed;
        avg_fps_ = (avg_fps_ * (frame_count_ - 1) + fps_now) / frame_count_;
        std::string text = "FPS: " + std::to_string(fps_now).substr(0, 5) +
                           " Avg: " + std::to_string(avg_fps_).substr(0, 5);
        putText(
          cv_ptr->image, text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
          cv::Scalar(0, 255, 0), 2);
      }
      last_frame_time_ = now;
      frame_count_++;
    }

    // If processing 360p images directly, this step can be skipped.
    if (is_resize) {
      resize(
        cv_ptr->image, cv_ptr->image, cv::Size(), resize_scale, resize_scale, cv::INTER_LINEAR);
    }

    Mat processed = processFrame(cv_ptr->image, airlight, beta_b, beta_g, beta_r);
    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", processed).toImageMsg();
    pub_image_->publish(*out_msg);

  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

// Function
Mat UnderWaterVideoEnhancementNode::calcBGRChannel(const Mat & src)
{
  Mat b_channel;
  extractChannel(src, b_channel, 0);
  return b_channel;
}

Mat UnderWaterVideoEnhancementNode::calcTransmissionMap(
  const Mat & b_channel, float A, float beta_b, float beta_g, float beta_r)
{
  Mat b;
  b_channel.convertTo(b, CV_32F);

  Mat d = 1.0f - b / A;
  d = max(d, 0);
  d = min(d, 1);

  Mat t_b = -beta_b * d;
  exp(t_b, t_b);

  Mat t_g = -beta_g * d;
  exp(t_g, t_g);

  Mat t_r = -beta_r * d;
  exp(t_r, t_r);

  std::vector<Mat> t_channels = {t_b, t_g, t_r};
  Mat t_bgr;
  merge(t_channels, t_bgr);

  float t0 = 0.05f;
  t_bgr = max(t_bgr, t0);
  t_bgr = min(t_bgr, 1.0f);

  return t_bgr;
}

Mat UnderWaterVideoEnhancementNode::imageRestoration(const Mat & src, const Mat & t_map, float A)
{
  Mat src_f;
  src.convertTo(src_f, CV_32F);

  Mat A_mat(src.size(), CV_32FC3, Scalar(A, A, A));

  Mat J = (src_f - A_mat) / t_map + A_mat;
  J = max(J, 0);
  J = min(J, 255);

  Mat J8U;
  J.convertTo(J8U, CV_8U);
  return J8U;
}

Mat UnderWaterVideoEnhancementNode::processFrame(
  const Mat & frame, float airlight, float beta_b, float beta_g, float beta_r)
{
  if (frame.empty()) return frame;
  Mat b, t, dst;

  b = calcBGRChannel(frame);
  t = calcTransmissionMap(b, airlight, beta_b, beta_g, beta_r);
  dst = imageRestoration(frame, t, airlight);

  return dst;
}

}  // namespace asr_sdm_underwater_video_enhancement

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<asr_sdm_underwater_video_enhancement::UnderWaterVideoEnhancementNode>(
      rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
