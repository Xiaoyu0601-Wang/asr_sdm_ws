#include "img_detector.hpp"

namespace img_detector
{

ImgDetectorNode::ImgDetectorNode(const rclcpp::NodeOptions & options)
: Node("img_detector_node", options)
{
  // 声明并读取参数
  const std::string model_path = this->declare_parameter<std::string>("model_path");
  const std::string label_path = this->declare_parameter<std::string>("label_path");
  score_thresh_ = this->declare_parameter<double>("score_thresh", 0.3);
  nms_thresh_ = this->declare_parameter<double>("nms_thresh", 0.45);
  input_w_ = this->declare_parameter<int>("input_width", 640);
  input_h_ = this->declare_parameter<int>("input_height", 640);

  test_mode_ = this->declare_parameter<bool>("test_mode", false);
  test_image_dir_ = this->declare_parameter<std::string>("test_image_dir", "");
  output_image_dir_ = this->declare_parameter<std::string>("output_image_dir", "");

  // 读取标签文件
  if (!readLabelFile(label_path, num_classes_, class_names_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YOLOv8 label file!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Loaded YOLOv8 labels: %d classes", num_classes_);
  }

  // 加载模型
  try {
    net_ = cv::dnn::readNetFromONNX(model_path);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU); 
    RCLCPP_INFO(
      this->get_logger(), "Successfully loaded YOLOv8 ONNX model: %s", model_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load YOLOv8 ONNX model: %s", e.what());
    throw;
  }

  // 初始化ROS2定时器
  using namespace std::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&ImgDetectorNode::connectCb, this));
  
  // 创建发布者
  output_detection_pub_ =
    this->create_publisher<vision_msgs::msg::Detection2DArray>("~/output/detections", 1);

  // 测试模式
  if (test_mode_) {
    runTestMode();
  }

  RCLCPP_INFO(this->get_logger(), "Reached here: 4");
}

// 图像预处理：letterbox缩放+归一化+通道转换
cv::Mat ImgDetectorNode::makeBlob(const cv::Mat & img, float & scale, int & dw, int & dh)
{
  int img_h = img.rows;
  int img_w = img.cols;
  
  // 缩放比例
  scale = std::min((float)input_w_ / img_w, (float)input_h_ / img_h);
  int new_w = static_cast<int>(img_w * scale);
  int new_h = static_cast<int>(img_h * scale);

  // 填充量
  dw = (input_w_ - new_w) / 2;
  dh = (input_h_ - new_h) / 2;

  // 缩放+填充黑边
  cv::Mat resized_img, padded_img;
  cv::resize(img, resized_img, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
  cv::copyMakeBorder(
    resized_img, padded_img, dh, input_h_ - new_h - dh, dw, input_w_ - new_w - dw,
    cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

  // 预处理：BGR→RGB + 归一化/255 + 通道在前
  cv::Mat blob;
  cv::dnn::blobFromImage(
    padded_img, 
    1.0, 
    cv::Size(input_w_, input_h_), 
    cv::Scalar(0, 0, 0),
    true,  // swapRB=TRUE（BGR→RGB）
    false, 
    CV_32F);
  
  return blob;
}

// 读取标签文件
bool ImgDetectorNode::readLabelFile(
  const std::string & label_path, int & num_classes, std::vector<std::string> & class_names)
{
  class_names.clear();
  std::ifstream label_file(label_path);
  if (!label_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open label file: %s", label_path.c_str());
    return false;
  }

  std::string line;
  num_classes = 0;
  while (std::getline(label_file, line)) {
    line.erase(0, line.find_first_not_of(" \t\n\r"));
    line.erase(line.find_last_not_of(" \t\n\r") + 1);
    if (!line.empty()) {
      class_names.push_back(line);
      num_classes++;
    }
  }
  label_file.close();

  // 打印类别
  for (size_t i = 0; i < class_names.size(); ++i) {
    RCLCPP_DEBUG(this->get_logger(), "Class %zu: %s", i, class_names[i].c_str());
  }
  return true;
}

// YOLOv8解码：处理[1,84,8400]输出格式
void ImgDetectorNode::decodeYolov8(
  const cv::Mat & out, const float scale, const int dw, const int dh, const int img_w,
  const int img_h, std::vector<cv::Rect> & boxes, std::vector<int> & class_ids,
  std::vector<float> & scores)
{
  RCLCPP_INFO(this->get_logger(), "Reached here: 7");
  // 维度转置
  cv::Mat out_transposed;
  cv::transpose(out, out_transposed);
  out_transposed = out_transposed.reshape(0, 8400);

  // 遍历所有锚点
  for (int i = 0; i < out_transposed.rows; ++i) {
    const float * data = out_transposed.ptr<float>(i);

    // 解析坐标（归一化）、置信度
    float x = data[0];     // 中心x
    float y = data[1];     // 中心y
    float w = data[2];     // 宽度
    float h = data[3];     // 高度
    float conf = data[4];  // 目标置信度

    // 置信度筛选
    if (conf < score_thresh_) continue;

    // 找到置信度最高的类别
    float max_cls_conf = 0.0f;
    int cls_id = -1;
    for (int j = 0; j < num_classes_; ++j) {
      float cls_conf = data[5 + j];
      if (cls_conf > max_cls_conf) {
        max_cls_conf = cls_conf;
        cls_id = j;
      }
    }

    // 最终得分 = 目标置信度 × 类别置信度
    float final_score = conf * max_cls_conf;
    if (final_score < score_thresh_) continue;

    // 坐标还原：归一化→输入尺寸→原始图像尺寸
    float input_x = x * input_w_;
    float input_y = y * input_h_;
    float input_w = w * input_w_;
    float input_h = h * input_h_;

    // 减去填充量
    float padded_x = input_x - dw;
    float padded_y = input_y - dh;

    // 除以缩放比例，还原到原始图像
    float orig_x = padded_x / scale;
    float orig_y = padded_y / scale;
    float orig_w = input_w / scale;
    float orig_h = input_h / scale;

    // 转换为OpenCV Rect
    int left = static_cast<int>(orig_x - orig_w / 2);
    int top = static_cast<int>(orig_y - orig_h / 2);
    int width = static_cast<int>(orig_w);
    int height = static_cast<int>(orig_h);

    // 边界裁剪
    left = std::max(0, left);
    top = std::max(0, top);
    width = std::min(img_w - left, width);
    height = std::min(img_h - top, height);

    // 保存结果
    boxes.emplace_back(left, top, width, height);
    class_ids.push_back(cls_id);
    scores.push_back(final_score);
  }
}

// ROS图像回调函数
void ImgDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Reached here: 8");
  auto start = std::chrono::system_clock::now();

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat img = cv_ptr->image;

  // 预处理
  float scale = 1.f;
  int dw = 0, dh = 0;
  cv::Mat blob = makeBlob(img, scale, dw, dh);

  // 模型推理
  net_.setInput(blob);
  cv::Mat out = net_.forward();

  // 解码
  std::vector<cv::Rect> boxes;
  std::vector<int> class_ids;
  std::vector<float> scores;
  decodeYolov8(out, scale, dw, dh, img.cols, img.rows, boxes, class_ids, scores);

  // NMS非极大值抑制
  std::vector<int> keep;
  cv::dnn::NMSBoxes(boxes, scores, (float)score_thresh_, (float)nms_thresh_, keep);

  // 构造vision_msgs
  vision_msgs::msg::Detection2DArray detection_array;
  detection_array.header = msg->header;

  for (int idx : keep) {
    const auto & box = boxes[idx];
    const int cls_id = class_ids[idx];
    const float score = scores[idx];

    // 单个检测框
    vision_msgs::msg::Detection2D detection;
    detection.header = msg->header;

    detection.bbox.center.position.x = box.x + box.width / 2.0;   // x在position里
    detection.bbox.center.position.y = box.y + box.height / 2.0;  // y在position里
    detection.bbox.size_x = box.width;                            // size_x
    detection.bbox.size_y = box.height;                           // size_y

    // 类别置信度
    vision_msgs::msg::ObjectHypothesisWithPose hypo;
    if (cls_id >= 0 && cls_id < static_cast<int>(class_names_.size())) {
      hypo.hypothesis.class_id = class_names_[cls_id];
    } else {
      hypo.hypothesis.class_id = std::to_string(cls_id);
    }
    hypo.hypothesis.score = score;
    detection.results.push_back(hypo);

    // 添加到结果数组
    detection_array.detections.push_back(detection);
  }

  // 发布消息
  output_detection_pub_->publish(detection_array);
}

// 节点连接回调
void ImgDetectorNode::connectCb()
{
  RCLCPP_INFO(this->get_logger(), "Reached here: 9");
  if (
    output_detection_pub_->get_subscription_count() > 0 ||
    output_detection_pub_->get_intra_process_subscription_count() > 0) {
    if (!img_sub_) {
      img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "~/input/image", 1,
        std::bind(&ImgDetectorNode::imageCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Subscribed to input image topic");
    }
  } else {
    if (img_sub_) {
      img_sub_.reset();
      RCLCPP_INFO(this->get_logger(), "Unsubscribed from input image topic");
    }
  }
}

// 测试模式
void ImgDetectorNode::runTestMode()
{
  RCLCPP_INFO(this->get_logger(), "Running in test mode");
  RCLCPP_INFO(this->get_logger(), "Test image dir: %s", test_image_dir_.c_str());
  RCLCPP_INFO(this->get_logger(), "Output image dir: %s", output_image_dir_.c_str());

  // 测试文件夹
  if (!std::filesystem::exists(test_image_dir_)) {
    RCLCPP_ERROR(this->get_logger(), "Test image dir not exists: %s", test_image_dir_.c_str());
    return;
  }

  // 输出文件夹
  if (!std::filesystem::exists(output_image_dir_)) {
    std::filesystem::create_directories(output_image_dir_);
    RCLCPP_INFO(this->get_logger(), "Created output dir: %s", output_image_dir_.c_str());
  }

  // 遍历图像文件
  std::vector<std::string> image_extensions = {".jpg", ".jpeg", ".png", ".bmp"};
  for (const auto & entry : std::filesystem::directory_iterator(test_image_dir_)) {
    if (!entry.is_regular_file()) continue;

    // 文件后缀
    std::string ext = entry.path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (
      std::find(image_extensions.begin(), image_extensions.end(), ext) == image_extensions.end()) {
      continue;
    }

    // 处理单张图像
    processTestImage(entry.path().string());
  }

  RCLCPP_INFO(
    this->get_logger(), "Test mode finished! All results saved to: %s", output_image_dir_.c_str());
}

// 处理单张测试图像
void ImgDetectorNode::processTestImage(const std::string & image_path)
{
  RCLCPP_INFO(this->get_logger(), "Processing image: %s", image_path.c_str());

  // 读取图像
  cv::Mat img = cv::imread(image_path);
  if (img.empty()) {
    RCLCPP_WARN(this->get_logger(), "Failed to read image: %s", image_path.c_str());
    return;
  }

  // 预处理→推理→解码
  float scale = 1.f;
  int dw = 0, dh = 0;
  cv::Mat blob = makeBlob(img, scale, dw, dh);
  net_.setInput(blob);
  cv::Mat out = net_.forward();

  std::vector<cv::Rect> boxes;
  std::vector<int> class_ids;
  std::vector<float> scores;
  decodeYolov8(out, scale, dw, dh, img.cols, img.rows, boxes, class_ids, scores);

  // NMS
  std::vector<int> keep;
  cv::dnn::NMSBoxes(boxes, scores, (float)score_thresh_, (float)nms_thresh_, keep);

  // 筛选结果
  std::vector<cv::Rect> filtered_boxes;
  std::vector<int> filtered_ids;
  std::vector<float> filtered_scores;
  for (int idx : keep) {
    filtered_boxes.push_back(boxes[idx]);
    filtered_ids.push_back(class_ids[idx]);
    filtered_scores.push_back(scores[idx]);
  }

  // 保存路径
  std::filesystem::path input_path(image_path);
  std::string save_path = output_image_dir_ + "/" + input_path.filename().string();

  // 绘制并保存结果
  drawRoiAndSave(img, filtered_boxes, filtered_ids, filtered_scores, save_path);
}

// 绘制检测结果并保存
void ImgDetectorNode::drawRoiAndSave(
  const cv::Mat & image, const std::vector<cv::Rect> & boxes, const std::vector<int> & class_ids,
  const std::vector<float> & scores, const std::string & save_path)
{
  cv::Mat result = image.clone();

  // 生成随机颜色
  std::vector<cv::Scalar> colors;
  std::mt19937 rng(std::random_device{}());
  std::uniform_int_distribution<int> dist(0, 255);
  for (int i = 0; i < num_classes_; ++i) {
    colors.emplace_back(dist(rng), dist(rng), dist(rng));
  }

  // 绘制检测框
  for (size_t i = 0; i < boxes.size(); ++i) {
    const auto & box = boxes[i];
    const int cls_id = class_ids[i];
    const float score = scores[i];
    if (cls_id < 0 || cls_id >= static_cast<int>(class_names_.size())) {
      continue;
    }

    // 矩形框
    cv::Scalar color = colors[cls_id % colors.size()];
    cv::rectangle(result, box, color, 2);

    // 标签
    std::string label = class_names_[cls_id] + " (" + std::to_string(score).substr(0, 4) + ")";
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.5;
    int thickness = 1;
    int baseline = 0;
    cv::Size label_size = cv::getTextSize(label, font_face, font_scale, thickness, &baseline);
    cv::Rect label_rect(
      box.x, box.y - label_size.height - 5, label_size.width, label_size.height + baseline + 5);
    cv::rectangle(result, label_rect, color, -1);
    cv::putText(
      result, label, cv::Point(box.x, box.y - 5), font_face, font_scale, cv::Scalar(255, 255, 255),
      thickness);
  }

  // 保存图像
  if (!cv::imwrite(save_path, result)) {
    RCLCPP_WARN(this->get_logger(), "Failed to save result image: %s", save_path.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Result saved to: %s", save_path.c_str());
  }
}

}  // namespace img_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(img_detector::ImgDetectorNode)