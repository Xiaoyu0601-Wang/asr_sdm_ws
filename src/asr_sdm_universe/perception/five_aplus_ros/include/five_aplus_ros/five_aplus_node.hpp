#ifndef FIVE_APLUS_ROS__FIVE_APLUS_NODE_HPP_
#define FIVE_APLUS_ROS__FIVE_APLUS_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace five_aplus_ros
{

class FiveAPlusNode : public rclcpp::Node
{
public:
  explicit FiveAPlusNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  std::vector<float> imageToTensor(const cv::Mat & rgb_image) const;
  cv::Mat tensorToBgrImage(float * output_data, const std::vector<int64_t> & shape) const;
  void logStats(std::chrono::duration<double, std::milli> callback_ms,
    std::chrono::duration<double, std::milli> inference_ms);

  Ort::Env env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  std::string model_path_;
  std::string input_topic_;
  std::string output_topic_;
  int64_t num_threads_;
  bool normalize_output_;

  size_t frame_count_;
  double accumulated_callback_ms_;
  double accumulated_inference_ms_;
};

}  // namespace five_aplus_ros

#endif  // FIVE_APLUS_ROS__FIVE_APLUS_NODE_HPP_
