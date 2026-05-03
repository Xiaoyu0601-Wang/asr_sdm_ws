#include "five_aplus_ros/five_aplus_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace enc = sensor_msgs::image_encodings;
using std::placeholders::_1;

namespace five_aplus_ros
{

FiveAPlusNode::FiveAPlusNode(const rclcpp::NodeOptions & options)
: Node("five_aplus_node", options),
  env_(ORT_LOGGING_LEVEL_WARNING, "five_aplus_ros"),
  session_(nullptr),
  num_threads_(0),
  normalize_output_(true),
  frame_count_(0),
  accumulated_callback_ms_(0.0),
  accumulated_inference_ms_(0.0)
{
  model_path_ = declare_parameter<std::string>("model_path", "models/five_aplus_epoch97.onnx");
  input_topic_ = declare_parameter<std::string>("input_topic", "/camera/camera/color/image_raw");
  output_topic_ = declare_parameter<std::string>("output_topic", "/five_aplus/image");
  num_threads_ = declare_parameter<int64_t>("num_threads", 0);
  normalize_output_ = declare_parameter<bool>("normalize_output", true);

  if (!std::filesystem::exists(model_path_)) {
    throw std::runtime_error("ONNX model does not exist: " + model_path_);
  }

  if (num_threads_ > 0) {
    session_options_.SetIntraOpNumThreads(static_cast<int>(num_threads_));
  }
  session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
  session_ = std::make_unique<Ort::Session>(env_, model_path_.c_str(), session_options_);

  Ort::AllocatorWithDefaultOptions allocator;
  auto input_name = session_->GetInputNameAllocated(0, allocator);
  auto output_name = session_->GetOutputNameAllocated(0, allocator);
  input_name_ = input_name.get();
  output_name_ = output_name.get();

  image_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, rclcpp::QoS(10));
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    input_topic_, rclcpp::SensorDataQoS(), std::bind(&FiveAPlusNode::imageCallback, this, _1));

  RCLCPP_INFO(get_logger(), "FiveA+ node loaded %s", model_path_.c_str());
  RCLCPP_INFO(get_logger(), "Subscribing %s, publishing %s", input_topic_.c_str(), output_topic_.c_str());
}

void FiveAPlusNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  const auto callback_start = std::chrono::steady_clock::now();
  if (msg->encoding != enc::BGR8 && msg->encoding != enc::RGB8) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Unsupported image encoding '%s'; expected bgr8 or rgb8.",
      msg->encoding.c_str());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
  } catch (const cv_bridge::Exception & error) {
    RCLCPP_WARN(get_logger(), "cv_bridge conversion failed: %s", error.what());
    return;
  }

  if (cv_ptr->image.rows < 128 || cv_ptr->image.cols < 128) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Input image is too small: %dx%d; minimum is 128x128.",
      cv_ptr->image.cols, cv_ptr->image.rows);
    return;
  }

  cv::Mat rgb_image;
  if (msg->encoding == enc::BGR8) {
    cv::cvtColor(cv_ptr->image, rgb_image, cv::COLOR_BGR2RGB);
  } else {
    rgb_image = cv_ptr->image;
  }
  if (!rgb_image.isContinuous()) {
    rgb_image = rgb_image.clone();
  }

  std::vector<float> input_tensor_values = imageToTensor(rgb_image);
  std::array<int64_t, 4> input_shape{
    1, 3, static_cast<int64_t>(rgb_image.rows), static_cast<int64_t>(rgb_image.cols)};
  Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    memory_info, input_tensor_values.data(), input_tensor_values.size(), input_shape.data(), input_shape.size());

  const char * input_names[] = {input_name_.c_str()};
  const char * output_names[] = {output_name_.c_str()};
  std::vector<Ort::Value> output_tensors;
  const auto inference_start = std::chrono::steady_clock::now();
  try {
    output_tensors = session_->Run(
      Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
  } catch (const Ort::Exception & error) {
    RCLCPP_ERROR(get_logger(), "ONNX Runtime inference failed: %s", error.what());
    return;
  }
  const auto inference_end = std::chrono::steady_clock::now();

  auto & output_tensor = output_tensors.front();
  auto output_info = output_tensor.GetTensorTypeAndShapeInfo();
  std::vector<int64_t> output_shape = output_info.GetShape();
  cv::Mat bgr_image = tensorToBgrImage(output_tensor.GetTensorMutableData<float>(), output_shape);

  auto output_msg = cv_bridge::CvImage(msg->header, enc::BGR8, bgr_image).toImageMsg();
  image_pub_->publish(*output_msg);

  const auto callback_end = std::chrono::steady_clock::now();
  logStats(
    std::chrono::duration<double, std::milli>(callback_end - callback_start),
    std::chrono::duration<double, std::milli>(inference_end - inference_start));
}

std::vector<float> FiveAPlusNode::imageToTensor(const cv::Mat & rgb_image) const
{
  cv::Mat rgb_float;
  rgb_image.convertTo(rgb_float, CV_32FC3, 1.0 / 255.0);
  const int height = rgb_float.rows;
  const int width = rgb_float.cols;
  std::vector<float> tensor(3 * height * width);
  for (int y = 0; y < height; ++y) {
    const auto * row = rgb_float.ptr<cv::Vec3f>(y);
    for (int x = 0; x < width; ++x) {
      const cv::Vec3f & pixel = row[x];
      const size_t offset = static_cast<size_t>(y * width + x);
      tensor[offset] = pixel[0];
      tensor[static_cast<size_t>(height * width) + offset] = pixel[1];
      tensor[static_cast<size_t>(2 * height * width) + offset] = pixel[2];
    }
  }
  return tensor;
}

cv::Mat FiveAPlusNode::tensorToBgrImage(float * output_data, const std::vector<int64_t> & shape) const
{
  if (shape.size() != 4 || shape[0] != 1 || shape[1] != 3 || shape[2] <= 0 || shape[3] <= 0) {
    throw std::runtime_error("Unexpected ONNX output shape.");
  }
  const int height = static_cast<int>(shape[2]);
  const int width = static_cast<int>(shape[3]);
  const size_t plane_size = static_cast<size_t>(height * width);
  const size_t tensor_size = 3 * plane_size;

  float min_value = std::numeric_limits<float>::infinity();
  float max_value = -std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < tensor_size; ++i) {
    min_value = std::min(min_value, output_data[i]);
    max_value = std::max(max_value, output_data[i]);
  }

  const bool needs_normalize = normalize_output_ && (min_value < 0.0F || max_value > 1.0F);
  const float denom = max_value - min_value + 1e-7F;
  cv::Mat bgr(height, width, CV_8UC3);
  for (int y = 0; y < height; ++y) {
    auto * row = bgr.ptr<cv::Vec3b>(y);
    for (int x = 0; x < width; ++x) {
      const size_t offset = static_cast<size_t>(y * width + x);
      float r = output_data[offset];
      float g = output_data[plane_size + offset];
      float b = output_data[2 * plane_size + offset];
      if (needs_normalize) {
        r = (r - min_value) / denom;
        g = (g - min_value) / denom;
        b = (b - min_value) / denom;
      }
      row[x] = cv::Vec3b{
        static_cast<unsigned char>(std::lround(std::clamp(b, 0.0F, 1.0F) * 255.0F)),
        static_cast<unsigned char>(std::lround(std::clamp(g, 0.0F, 1.0F) * 255.0F)),
        static_cast<unsigned char>(std::lround(std::clamp(r, 0.0F, 1.0F) * 255.0F))};
    }
  }
  return bgr;
}

void FiveAPlusNode::logStats(
  std::chrono::duration<double, std::milli> callback_ms,
  std::chrono::duration<double, std::milli> inference_ms)
{
  ++frame_count_;
  accumulated_callback_ms_ += callback_ms.count();
  accumulated_inference_ms_ += inference_ms.count();
  if (frame_count_ % 30 != 0) {
    return;
  }

  const double callback_fps = 1000.0 / (accumulated_callback_ms_ / static_cast<double>(frame_count_));
  const double inference_fps = 1000.0 / (accumulated_inference_ms_ / static_cast<double>(frame_count_));
  RCLCPP_INFO(
    get_logger(), "FiveA+ processed %zu frames: callback %.2f FPS, inference %.2f FPS",
    frame_count_, callback_fps, inference_fps);
}

}  // namespace five_aplus_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<five_aplus_ros::FiveAPlusNode>());
  rclcpp::shutdown();
  return 0;
}
