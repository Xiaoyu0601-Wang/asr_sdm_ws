#include "asr_sdm_image_classifier/img_classifier.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <random>

namespace img_classifier
{

ImgClassifierNode::ImgClassifierNode(const rclcpp::NodeOptions & options)
: Node("img_classifier_node", options)
{
  // 订阅原始图像话题和上游发布的ROI话题
#if ASR_SDM_USE_HUMBLE_INTERFACE
  // humble
  RCLCPP_INFO(this->get_logger(), "ROS 2 Version: %s - Using Humble interface", ROS2_VERSION);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  rmw_qos_profile_t qos_profile = qos.get_rmw_qos_profile();

  img_sub_.subscribe(this, "/camera/image_raw", qos_profile);
  roi_sub_.subscribe(this, "/output/rois", qos_profile);
#else
  // jazzy
  RCLCPP_INFO(this->get_logger(), "ROS 2 Version: %s - Using Jazzy interface", ROS2_VERSION);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  img_sub_.subscribe(this, "/camera/image_raw", qos);
  roi_sub_.subscribe(this, "/output/rois", qos);
#endif

  // 参数
  const std::string model_path = this->declare_parameter<std::string>("model_path");
  const std::string label_path = this->declare_parameter<std::string>("label_path");

  input_w_ = this->declare_parameter<int>("input_width", 224);
  input_h_ = this->declare_parameter<int>("input_height", 224);
  score_thresh_ = this->declare_parameter<double>("score_thresh", 0.5);

  test_mode_ = this->declare_parameter<bool>("test_mode", false);
  test_image_dir_ = this->declare_parameter<std::string>("test_image_dir", "");
  output_image_dir_ = this->declare_parameter<std::string>("output_image_dir", "");
  publish_debug_image_ = this->declare_parameter<bool>("publish_debug_image", true);

  // 标签文件
  if (!readLabelFile(label_path)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load label file: %s", label_path.c_str());
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Loaded %zu classes from %s", class_names_.size(), label_path.c_str());
  }

  // 加载模型
  try {
    net_ = cv::dnn::readNetFromONNX(model_path);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    RCLCPP_INFO(this->get_logger(), "Successfully loaded ONNX model: %s", model_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load ONNX model: %s", e.what());
    throw;
  }

  // 发布
  sync_.reset(new Sync(MySyncPolicy(10), img_sub_, roi_sub_));
  sync_->registerCallback(
    std::bind(
      &ImgClassifierNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

  // label_pub_ = this->create_publisher<std_msgs::msg::String>("~/output/label", 10);
  // if (publish_debug_image_) {
  //   debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/output/image", 10);
  // }

  if (!test_mode_) {
    RCLCPP_INFO(this->get_logger(), "已启动图像与ROI同步订阅");

    // img_sub_ = this->create_subscription<asr_sdm_perception_msgs::msg::TrafficLightRoiArray>(
    //   "~/input/rois", rclcpp::QoS(10),
    //   std::bind(&ImgClassifierNode::imageCallback, this, std::placeholders::_1));

    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/output/image", 10);
    label_pub_ = this->create_publisher<std_msgs::msg::String>("~/output/label", 10);
  } else {
    RCLCPP_INFO(get_logger(), "Test mode: will read images from folder, no subscription.");
  }

  // 测试模式
  if (test_mode_) {
    runTestMode();
  }

  RCLCPP_INFO(this->get_logger(), "ImgClassifierNode initialized.");
}

// 读取label
bool ImgClassifierNode::readLabelFile(const std::string & label_path)
{
  class_names_.clear();
  std::ifstream ifs(label_path);
  if (!ifs.is_open()) {
    return false;
  }
  std::string line;
  while (std::getline(ifs, line)) {
    auto s = line.find_first_not_of(" \t\r\n");
    auto e = line.find_last_not_of(" \t\r\n");
    if (s == std::string::npos || e == std::string::npos) {
      continue;
    }
    class_names_.push_back(line.substr(s, e - s + 1));
  }
  return true;
}

// 图像预处理
cv::Mat ImgClassifierNode::makeBlob(const cv::Mat & img)
{
  cv::Mat resized;
  cv::resize(img, resized, cv::Size(input_w_, input_h_));

  cv::Mat blob = cv::dnn::blobFromImage(
    resized, 1.0 / 255.0, cv::Size(input_w_, input_h_), cv::Scalar(), true, false, CV_32F);

  return blob;
}

// 单张图像分类
bool ImgClassifierNode::classify(const cv::Mat & img, int & out_class_id, float & out_score)
{
  if (img.empty()) {
    return false;
  }

  cv::Mat blob = makeBlob(img);
  net_.setInput(blob);
  cv::Mat out = net_.forward();

  cv::Mat prob = out.reshape(1, 1);

  cv::Point classIdPoint;
  double confidence = 0.0;
  cv::minMaxLoc(prob, nullptr, &confidence, nullptr, &classIdPoint);
  int class_id = classIdPoint.x;

  out_class_id = class_id;
  out_score = static_cast<float>(confidence);
  return true;
}

cv::Mat ImgClassifierNode::drawLabelOnImage(const cv::Mat & img, const std::string & label_text)
{
  cv::Mat result = img.clone();

  int font_face = cv::FONT_HERSHEY_SIMPLEX;
  double font_scale = 0.8;
  int thickness = 2;
  int baseline = 0;
  cv::Size text_size = cv::getTextSize(label_text, font_face, font_scale, thickness, &baseline);

  int x = 10;
  int y = 10;

  cv::Rect bg_rect(x, y, text_size.width + 10, text_size.height + baseline + 10);

  bg_rect &= cv::Rect(0, 0, result.cols, result.rows);

  cv::rectangle(result, bg_rect, cv::Scalar(0, 0, 0), cv::FILLED);
  cv::putText(
    result, label_text, cv::Point(x + 5, y + 5 + text_size.height), font_face, font_scale,
    cv::Scalar(255, 255, 255), thickness);

  return result;
}

void ImgClassifierNode::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_sub_,
  const asr_sdm_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & roi_sub_)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_sub_, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  const cv::Mat & full_img = cv_ptr->image;

  if (full_img.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty image in TrafficLightRoiArray");
    return;
  }

  if (roi_sub_->rois.empty()) {
    RCLCPP_INFO(this->get_logger(), "No ROIs in TrafficLightRoiArray, nothing to classify");
    return;
  }

  cv::Mat vis = full_img.clone();
  cv::Rect img_rect(0, 0, full_img.cols, full_img.rows);

  std::vector<std::string> roi_labels;
  roi_labels.reserve(roi_sub_->rois.size());

  // 遍历ROI
  size_t i = 0;
  for (const auto & traffic_light_roi : roi_sub_->rois) {
    // 【关键】新的数据结构：坐标在 r.roi 中，且字段名是 x_offset, y_offset...
    const auto & r = traffic_light_roi.roi;
    int x = static_cast<int>(r.x_offset);
    int y = static_cast<int>(r.y_offset);
    int width = static_cast<int>(r.width);
    int height = static_cast<int>(r.height);

    cv::Rect roi_rect(x, y, width, height);

    roi_rect &= img_rect;  // 防止越界
    if (roi_rect.width <= 0 || roi_rect.height <= 0) {
      RCLCPP_WARN(this->get_logger(), "ROI %zu is out of image bounds, skip", i);
      continue;
    }

    // 裁剪
    cv::Mat roi_img = full_img(roi_rect).clone();

    int class_id = -1;
    float score = 0.0f;
    if (!classify(roi_img, class_id, score)) {
      RCLCPP_WARN(this->get_logger(), "Failed to classify ROI %zu", i);
      continue;
    }

    // 生成标签文本
    std::string cls_name;
    if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
      cls_name = class_names_[class_id];
    } else {
      cls_name = "cls_" + std::to_string(class_id);
    }

    char buf[16];
    std::snprintf(buf, sizeof(buf), "%.2f", score);

    std::string label_text;
    if (score >= score_thresh_) {
      label_text = cls_name + " (" + buf + ")";
    } else {
      label_text = "unknown (" + std::string(buf) + ")";
    }

    roi_labels.push_back(
      "roi" + std::to_string(i) + ": " + label_text + " [x=" + std::to_string(roi_rect.x) +
      ", y=" + std::to_string(roi_rect.y) + ", w=" + std::to_string(roi_rect.width) +
      ", h=" + std::to_string(roi_rect.height) + "]");

    // 画框 标签
    cv::Scalar color(0, 255, 0);  // 绿色
    cv::rectangle(vis, roi_rect, color, 2);

    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.6;
    int thickness = 2;
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(label_text, font_face, font_scale, thickness, &baseline);

    int text_x = roi_rect.x;
    int text_y = std::max(0, roi_rect.y - text_size.height - 4);

    cv::Rect text_bg(text_x, text_y, text_size.width + 4, text_size.height + baseline + 4);
    text_bg &= img_rect;  // 防止越界

    cv::rectangle(vis, text_bg, color, cv::FILLED);

    cv::putText(
      vis, label_text, cv::Point(text_bg.x + 2, text_bg.y + text_size.height), font_face,
      font_scale, cv::Scalar(255, 255, 255), thickness);

    i++;
  }

  // 文本标签
  if (!roi_labels.empty()) {
    std_msgs::msg::String label_msg;
    std::string all_labels;
    for (size_t i = 0; i < roi_labels.size(); ++i) {
      if (i > 0) {
        all_labels += "\n";
      }
      all_labels += roi_labels[i];
    }
    label_msg.data = all_labels;
    label_pub_->publish(label_msg);
  }

  // 发布整图
  if (publish_debug_image_) {
    cv_bridge::CvImage out_msg;
    out_msg.header = img_sub_->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = vis;
    debug_image_pub_->publish(*out_msg.toImageMsg());
  }
}

// 测试 遍历文件夹
void ImgClassifierNode::runTestMode()
{
  RCLCPP_INFO(this->get_logger(), "Running in test mode");
  RCLCPP_INFO(this->get_logger(), "Test image dir: %s", test_image_dir_.c_str());
  RCLCPP_INFO(this->get_logger(), "Output image dir: %s", output_image_dir_.c_str());

  if (!std::filesystem::exists(test_image_dir_)) {
    RCLCPP_ERROR(this->get_logger(), "Test image dir does not exist");
    return;
  }
  if (!std::filesystem::exists(output_image_dir_)) {
    std::filesystem::create_directories(output_image_dir_);
  }

  std::vector<std::string> exts = {".jpg", ".jpeg", ".png", ".bmp"};

  for (const auto & entry : std::filesystem::directory_iterator(test_image_dir_)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string ext = entry.path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (std::find(exts.begin(), exts.end(), ext) == exts.end()) {
      continue;
    }
    processTestImage(entry.path().string());
  }

  RCLCPP_INFO(
    this->get_logger(), "Test mode finished. Results saved to: %s", output_image_dir_.c_str());
}

void ImgClassifierNode::processTestImage(const std::string & image_path)
{
  RCLCPP_INFO(this->get_logger(), "Processing image: %s", image_path.c_str());

  cv::Mat img = cv::imread(image_path);
  if (img.empty()) {
    RCLCPP_WARN(this->get_logger(), "Failed to read image: %s", image_path.c_str());
    return;
  }

  int class_id = -1;
  float score = 0.0f;
  if (!classify(img, class_id, score)) {
    RCLCPP_WARN(this->get_logger(), "Failed to classify test image: %s", image_path.c_str());
    return;
  }

  std::string cls_name;
  if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
    cls_name = class_names_[class_id];
  } else {
    cls_name = "cls_" + std::to_string(class_id);
  }

  char buf[16];
  std::snprintf(buf, sizeof(buf), "%.2f", score);
  std::string label_text;
  if (score >= score_thresh_) {
    label_text = cls_name + " (" + buf + ")";
  } else {
    label_text = "unknown (" + std::string(buf) + ")";
  }

  cv::Mat vis = drawLabelOnImage(img, label_text);

  std::filesystem::path in_path(image_path);
  std::string out_path = output_image_dir_ + "/" + in_path.filename().string();

  if (!cv::imwrite(out_path, vis)) {
    RCLCPP_WARN(this->get_logger(), "Failed to save result image: %s", out_path.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Saved result image: %s", out_path.c_str());
  }
}

}  // namespace img_classifier

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<img_classifier::ImgClassifierNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}