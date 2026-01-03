#pragma once

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "asr_sdm_perception_msgs/msg/traffic_light_roi.hpp"
#include "asr_sdm_perception_msgs/msg/traffic_light_roi_array.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <filesystem>
#include <string>
#include <vector>

namespace img_classifier
{

class ImgClassifierNode : public rclcpp::Node
{
public:
  explicit ImgClassifierNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // void syncedCallback(
  //   const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  //   const asr_sdm_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & roi_msg
  // );

  // 工具函数
  bool readLabelFile(const std::string & label_path);
  cv::Mat makeBlob(const cv::Mat & img);
  bool classify(const cv::Mat & img, int & out_class_id, float & out_score);
  cv::Mat drawLabelOnImage(const cv::Mat & img, const std::string & label_text);

  // ROS 回调&逻辑
  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & img_sub_,
    const asr_sdm_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & roi_sub_);
  void runTestMode();
  void processTestImage(const std::string & image_path);
  void classifyAndAnnotate(const cv::Mat & input_img, cv::Mat & out_img, std::string & top_label);

  // 成员变量
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;

  int input_w_{224};
  int input_h_{224};
  double score_thresh_{0.5};

  bool test_mode_{false};
  std::string test_image_dir_;
  std::string output_image_dir_;
  bool publish_debug_image_{true};

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, asr_sdm_perception_msgs::msg::TrafficLightRoiArray>
    MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  message_filters::Subscriber<sensor_msgs::msg::Image> img_sub_;
  message_filters::Subscriber<asr_sdm_perception_msgs::msg::TrafficLightRoiArray> roi_sub_;
  // rclcpp::Subscription < asr_sdm_perception_msgs::msg::TrafficLightRoiArray::SharedPtr img_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr label_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};

}  // namespace img_classifier
