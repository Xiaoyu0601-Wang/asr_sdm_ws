#include "asr_sdm_video_enhancement/asr_sdm_video_enhancement.hpp"

namespace asr_sdm_video_enhancement
{

VideoEnhancementNode::VideoEnhancementNode(const rclcpp::NodeOptions & node_options)
: Node("asr_sdm_video_enhancement", node_options)
{
  // Parameter
  config_.manual_airlight = static_cast<int>(declare_parameter("airlight", 255));
  config_.scale = static_cast<double>(declare_parameter("scale", 0.5));
  config_.use_adaptive_airlight = declare_parameter("use_adaptive_airlight", true);
  config_.airlight_percentile =
    static_cast<double>(declare_parameter("airlight_percentile", 0.99));
  config_.transmission_floor =
    static_cast<double>(declare_parameter("transmission_floor", 0.25));
  config_.transmission_gain =
    static_cast<double>(declare_parameter("transmission_gain", 0.50));
  config_.gamma_blend = static_cast<double>(declare_parameter("gamma_blend", 0.30));
  config_.enable_white_balance = declare_parameter("enable_white_balance", true);
  config_.enable_clahe = declare_parameter("enable_clahe", true);
  config_.white_balance_blend =
    static_cast<double>(declare_parameter("white_balance_blend", 0.35));
  config_.clahe_blend =
    static_cast<double>(declare_parameter("clahe_blend", 0.30));
  config_.original_blend =
    static_cast<double>(declare_parameter("original_blend", 0.20));
  config_.clahe_clip_limit =
    static_cast<double>(declare_parameter("clahe_clip_limit", 1.4));
  config_.clahe_tile_size =
    static_cast<int>(declare_parameter("clahe_tile_size", 8));
  overlay_fps_text_ = declare_parameter("overlay_fps_text", true);

  // Subscriber
  auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  image_qos.best_effort();
  image_qos.durability_volatile();
  sub_image_ = this->create_subscription<CompressedImage>(
    "~/debug/image/compressed",
    image_qos,
    std::bind(&VideoEnhancementNode::onImageCallback, this, _1));

  // Publisher
  pub_image_ = image_transport::create_publisher(
    this, "~/output/image", rmw_qos_profile_sensor_data);
}

// Callback
void VideoEnhancementNode::onImageCallback(const CompressedImage::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "Got compressed image (%zu bytes)", msg->data.size());//节流
  try {
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    // Calculate FPS
    auto now = std::chrono::steady_clock::now();
    std::string fps_text = "FPS: --";
    if (frame_count_ > 0) {
      const double elapsed = std::chrono::duration<double>(now - last_frame_time_).count();
      const double fps_now = elapsed > 0.0 ? 1.0 / elapsed : 0.0;
      avg_fps_ = (avg_fps_ * (frame_count_ - 1) + fps_now) / frame_count_;
      fps_text = cv::format("FPS: %.1f", fps_now);
    }
    last_frame_time_ = now;
    frame_count_++;

    cv::Mat processed = processor_.process(cv_ptr->image, config_);
    if (overlay_fps_text_ && !fps_text.empty()) {
      cv::putText(
        processed, fps_text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX,
        1.0, cv::Scalar(0, 0, 0), 4);
      cv::putText(
        processed, fps_text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX,
        1.0, cv::Scalar(0, 255, 0), 2);
    }
    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", processed).toImageMsg();
    pub_image_.publish(*out_msg);

  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

} // namespace asr_sdm_video_enhancement

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<asr_sdm_video_enhancement::VideoEnhancementNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
