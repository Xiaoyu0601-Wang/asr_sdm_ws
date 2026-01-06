#pragma once
// #ifndef ASR_SDM_OBJECT_DETECTOR__IMG_DETECTOR_HPP_
// #define ASR_SDM_OBJECT_DETECTOR__IMG_DETECTOR_HPP_

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "asr_sdm_perception_msgs/msg/traffic_light_roi.hpp"
#include "asr_sdm_perception_msgs/msg/traffic_light_roi_array.hpp"
#include <sensor_msgs/msg/image.hpp>

// cv_bridge header differs between Ubuntu versions:
// Ubuntu 22.04 (ROS2 Humble): cv_bridge.h
// Ubuntu 24.04 (ROS2 Jazzy): cv_bridge.hpp
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

// #include <vision_msgs/vision_msgs/msg/bounding_box2_d.hpp>
// #include <vision_msgs/vision_msgs/msg/detection2_d.hpp>
// #include <vision_msgs/vision_msgs/msg/detection2_d_array.hpp>
// #include <vision_msgs/vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace img_detector
{

class ImgDetectorNode : public rclcpp::Node
{
public:
  explicit ImgDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  cv::dnn::Net net_;
  int input_w_;
  int input_h_;
  double score_thresh_;
  double nms_thresh_;
  int num_classes_;
  std::vector<std::string> class_names_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<asr_sdm_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr roi_pub_;

  bool test_mode_;
  std::string test_image_dir_;
  std::string output_image_dir_;

  cv::Mat makeBlob(const cv::Mat & img, float & scale, int & dw, int & dh);

  void decodeYolov8(
    const cv::Mat & out, const float scale, const int dw, const int dh, const int img_w,
    const int img_h, std::vector<cv::Rect> & boxes, std::vector<int> & class_ids,
    std::vector<float> & scores);

  bool readLabelFile(
    const std::string & label_path, int & num_classes, std::vector<std::string> & class_names);

  // ROS图像回调函数）
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  // 测试）
  void runTestMode();

  // 单张测试
  void processTestImage(const std::string & image_path);

  // 检测结果
  void drawRoiAndSave(
    const cv::Mat & image, const std::vector<cv::Rect> & boxes, const std::vector<int> & class_ids,
    const std::vector<float> & scores, const std::string & save_path);
};

}  // namespace img_detector
