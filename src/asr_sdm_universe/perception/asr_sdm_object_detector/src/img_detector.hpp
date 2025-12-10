// #ifndef AUTOWARE_TRAFFIC_LIGHT_FINE_DETECTOR__TRAFFIC_LIGHT_FINE_DETECTOR_HPP_
// #define AUTOWARE_TRAFFIC_LIGHT_FINE_DETECTOR__TRAFFIC_LIGHT_FINE_DETECTOR_HPP_
// 待修改
#ifndef ASR_SDM_OBJECT_DETECTOR__IMG_DETECTOR_HPP_
#define ASR_SDM_OBJECT_DETECTOR__IMG_DETECTOR_HPP_

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include <vision_msgs/vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace img_detector
{

class ImgDetectorNode : public rclcpp::Node
{
public:
  explicit ImgDetectorNode(const rclcpp::NodeOptions & options);

private:
  cv::dnn::Net net_;
  int input_w_;
  int input_h_;
  double score_thresh_;
  double nms_thresh_;
  int num_classes_;
  std::vector<std::string> class_names_;

  // ROS2通信成员
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr output_detection_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 测试
  bool test_mode_;
  std::string test_image_dir_;
  std::string output_image_dir_;

  // 图像预处理
  cv::Mat makeBlob(const cv::Mat & img, float & scale, int & dw, int & dh);

  // 输出解码
  void decodeYolov8(
    const cv::Mat & out, const float scale, const int dw, const int dh, const int img_w,
    const int img_h, std::vector<cv::Rect> & boxes, std::vector<int> & class_ids,
    std::vector<float> & scores);

  // 读取标签文件
  bool readLabelFile(
    const std::string & label_path, int & num_classes, std::vector<std::string> & class_names);

  // 图像回调函数
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  // 节点连接回调
  void connectCb();

  // 测试模式入口
  void runTestMode();

  // 单张图像处理
  void processTestImage(const std::string & image_path);

  // 绘制检测结果并保存
  void drawRoiAndSave(
    const cv::Mat & image, const std::vector<cv::Rect> & boxes, const std::vector<int> & class_ids,
    const std::vector<float> & scores, const std::string & save_path);
};

}  // namespace img_detector

#endif  // ASR_SDM_OBJECT_DETECTOR__IMG_DETECTOR_HPP_
