#include "asr_sdm_object_detector/img_detector.hpp"

namespace img_detector
{

ImgDetectorNode::ImgDetectorNode(const rclcpp::NodeOptions & options)
: Node("img_detector_node", options), is_processing_(false)  // 初始化处理状态为false
{
  const std::string model_path = this->declare_parameter<std::string>("model_path");
  const std::string label_path = this->declare_parameter<std::string>("label_path");
  score_thresh_ = this->declare_parameter<double>("score_thresh", 0.3);
  nms_thresh_ = this->declare_parameter<double>("nms_thresh", 0.45);
  input_w_ = this->declare_parameter<int>("input_width", 640);
  input_h_ = this->declare_parameter<int>("input_height", 640);

  test_mode_ = this->declare_parameter<bool>("test_mode", false);
  test_image_dir_ = this->declare_parameter<std::string>("test_image_dir", "");
  output_image_dir_ = this->declare_parameter<std::string>("output_image_dir", "");

  if (!readLabelFile(label_path, num_classes_, class_names_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YOLOv8 label file!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Loaded YOLOv8 labels: %d classes", num_classes_);
  }

  // 加载模型
  try {
    net_ = cv::dnn::readNetFromONNX(model_path);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);  // cpu
    RCLCPP_INFO(
      this->get_logger(), "Successfully loaded YOLOv8 ONNX model: %s", model_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load YOLOv8 ONNX model: %s", e.what());
    throw;
  }

  roi_pub_ =
    this->create_publisher<asr_sdm_perception_msgs::msg::TrafficLightRoiArray>("/output/rois", 10);

  if (!test_mode_) {
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (is_processing_) {
          return;  // 如果在处理中，忽略此帧图像
        }

        is_processing_ = true;
        imageCallback(msg);
        is_processing_ = false;
      });
  } else {
    runTestMode();
  }

  RCLCPP_INFO(this->get_logger(), "Reached here: 4");
}

// 预处理
cv::Mat ImgDetectorNode::makeBlob(const cv::Mat & img, float & scale, int & dw, int & dh)
{
  int img_h = img.rows;
  int img_w = img.cols;
  scale = std::min((float)input_w_ / img_w, (float)input_h_ / img_h);
  int new_w = static_cast<int>(img_w * scale);
  int new_h = static_cast<int>(img_h * scale);
  dw = (input_w_ - new_w) / 2;
  dh = (input_h_ - new_h) / 2;
  cv::Mat resized_img, padded_img;
  cv::resize(img, resized_img, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
  cv::copyMakeBorder(
    resized_img, padded_img, dh, input_h_ - new_h - dh, dw, input_w_ - new_w - dw,
    cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

  cv::Mat blob;
  cv::Mat rgb_img;
  cv::cvtColor(padded_img, rgb_img, cv::COLOR_BGR2RGB);

  // 转换数据类型
  cv::Mat float_img;
  rgb_img.convertTo(float_img, CV_32F);
  float_img /= 255.0;

  int blob_sizes[] = {1, 3, input_h_, input_w_};
  blob.create(4, blob_sizes, CV_32F);

  std::vector<cv::Mat> channels;
  cv::split(float_img, channels);  // RGB

  for (int c = 0; c < 3; ++c) {
    cv::Mat channel_mat(input_h_, input_w_, CV_32F, blob.ptr<float>(0, c));
    channels[c].copyTo(channel_mat);
  }

  return blob;
}

// 读取标签
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

  for (size_t i = 0; i < class_names.size(); ++i) {
    RCLCPP_DEBUG(this->get_logger(), "Class %zu: %s", i, class_names[i].c_str());
  }
  return true;
}

static inline float sigmoid(float x)
{
  return 1.0f / (1.0f + std::exp(-x));
}

void ImgDetectorNode::decodeYolov8(
  const cv::Mat & out, const float scale, const int dw, const int dh, const int img_w,
  const int img_h, std::vector<cv::Rect> & boxes, std::vector<int> & class_ids,
  std::vector<float> & scores)
{
  RCLCPP_INFO(this->get_logger(), "Reached here: 7");

  boxes.clear();
  class_ids.clear();
  scores.clear();

  if (out.dims != 3 || out.size[0] != 1) {
    RCLCPP_ERROR(
      this->get_logger(), "Unsupported output shape, dims=%d, size[0]=%d", out.dims, out.size[0]);
    return;
  }

  const int feat_dim = out.size[1];
  const int num_proposals = out.size[2];
  const int net_num_classes = feat_dim - 4;

  float max_raw_cls = -1e9f;
  float max_score = -1e9f;

  for (int i = 0; i < num_proposals; ++i) {
    float cx = out.at<float>(0, 0, i);  // center x
    float cy = out.at<float>(0, 1, i);  // center y
    float w = out.at<float>(0, 2, i);   // width
    float h = out.at<float>(0, 3, i);   // height

    float best_raw_cls = -1e9f;
    int cls_id = -1;
    for (int c = 0; c < net_num_classes; ++c) {
      float raw_cls = out.at<float>(0, 4 + c, i);
      if (raw_cls > best_raw_cls) {
        best_raw_cls = raw_cls;
        cls_id = c;
      }
    }

    if (best_raw_cls > max_raw_cls) {
      max_raw_cls = best_raw_cls;
    }

    float score = sigmoid(best_raw_cls);
    if (score > max_score) max_score = score;

    float x0 = cx - w * 0.5f;
    float y0 = cy - h * 0.5f;
    float x1 = cx + w * 0.5f;
    float y1 = cy + h * 0.5f;

    x0 = (x0 - dw) / scale;
    y0 = (y0 - dh) / scale;
    x1 = (x1 - dw) / scale;
    y1 = (y1 - dh) / scale;

    x0 = std::max(0.0f, std::min((float)img_w - 1.0f, x0));
    y0 = std::max(0.0f, std::min((float)img_h - 1.0f, y0));
    x1 = std::max(0.0f, std::min((float)img_w - 1.0f, x1));
    y1 = std::max(0.0f, std::min((float)img_h - 1.0f, y1));

    float bw = x1 - x0;
    float bh = y1 - y0;
    if (bw <= 1.0f || bh <= 1.0f) {
      continue;
    }

    int left = static_cast<int>(x0);
    int top = static_cast<int>(y0);
    int width = static_cast<int>(bw);
    int height = static_cast<int>(bh);

    boxes.emplace_back(left, top, width, height);
    class_ids.push_back(cls_id);
    scores.push_back(score);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "decodeYolov8: feat_dim=%d, net_num_classes=%d, boxes=%zu, max_raw_cls=%.4f, max_score=%.4f",
    feat_dim, net_num_classes, boxes.size(), max_raw_cls, max_score);
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

  // 模型
  net_.setInput(blob);
  cv::Mat out = net_.forward();

  // 解码
  std::vector<cv::Rect> boxes;
  std::vector<int> class_ids;
  std::vector<float> scores;
  decodeYolov8(out, scale, dw, dh, img.cols, img.rows, boxes, class_ids, scores);

  // NMS
  RCLCPP_INFO(this->get_logger(), "After decode: %zu bostd::vector<int> keep;xes", boxes.size());

  std::vector<int> keep;
  cv::dnn::NMSBoxes(boxes, scores, (float)score_thresh_, (float)nms_thresh_, keep);
  RCLCPP_INFO(this->get_logger(), "After NMS: %zu boxes kept", keep.size());

  // build
  asr_sdm_perception_msgs::msg::TrafficLightRoiArray out_msg;
  out_msg.header = msg->header;
  // out_msg.image = *msg;

  for (int idx : keep) {
    const cv::Rect & box = boxes[idx];

    asr_sdm_perception_msgs::msg::TrafficLightRoi roi;
    roi.roi.x_offset = static_cast<uint32_t>(box.x);  // 类型转换
    roi.roi.y_offset = static_cast<uint32_t>(box.y);
    roi.roi.width = static_cast<uint32_t>(box.width);
    roi.roi.height = static_cast<uint32_t>(box.height);

    out_msg.rois.push_back(roi);
  }

  roi_pub_->publish(out_msg);
}

// 测试
void ImgDetectorNode::runTestMode()
{
  RCLCPP_INFO(this->get_logger(), "Running in test mode");
  RCLCPP_INFO(this->get_logger(), "Test image dir: %s", test_image_dir_.c_str());
  RCLCPP_INFO(this->get_logger(), "Output image dir: %s", output_image_dir_.c_str());

  // 测试文件
  if (!std::filesystem::exists(test_image_dir_)) {
    RCLCPP_ERROR(this->get_logger(), "Test image dir not exists: %s", test_image_dir_.c_str());
    return;
  }

  if (!std::filesystem::exists(output_image_dir_)) {
    std::filesystem::create_directories(output_image_dir_);
    RCLCPP_INFO(this->get_logger(), "Created output dir: %s", output_image_dir_.c_str());
  }

  // 遍历测试
  std::vector<std::string> image_extensions = {".jpg", ".jpeg", ".png", ".bmp"};
  for (const auto & entry : std::filesystem::directory_iterator(test_image_dir_)) {
    if (!entry.is_regular_file()) continue;
    std::string ext = entry.path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (
      std::find(image_extensions.begin(), image_extensions.end(), ext) == image_extensions.end()) {
      continue;
    }
    processTestImage(entry.path().string());
  }

  RCLCPP_INFO(
    this->get_logger(), "Test mode finished! All results saved to: %s", output_image_dir_.c_str());
}

void ImgDetectorNode::processTestImage(const std::string & image_path)
{
  RCLCPP_INFO(this->get_logger(), "Processing image: %s", image_path.c_str());
  cv::Mat img = cv::imread(image_path);
  if (img.empty()) {
    RCLCPP_WARN(this->get_logger(), "Failed to read image: %s", image_path.c_str());
    return;
  }

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

  std::vector<cv::Rect> filtered_boxes;
  std::vector<int> filtered_ids;
  std::vector<float> filtered_scores;

  filtered_boxes.reserve(keep.size());
  filtered_ids.reserve(keep.size());
  filtered_scores.reserve(keep.size());

  for (int idx : keep) {
    filtered_boxes.push_back(boxes[idx]);
    filtered_ids.push_back(class_ids[idx]);
    filtered_scores.push_back(scores[idx]);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "processTestImage: total=%zu, keep=%zu (score_thresh=%.3f, nms_thresh=%.3f)", boxes.size(),
    filtered_boxes.size(), score_thresh_, nms_thresh_);

  // 保存
  std::filesystem::path input_path(image_path);
  std::string save_path = output_image_dir_ + "/" + input_path.filename().string();

  if (!filtered_boxes.empty()) {
    drawRoiAndSave(img, filtered_boxes, filtered_ids, filtered_scores, save_path);
  } else {
    RCLCPP_INFO(
      this->get_logger(), "processTestImage: no detection kept after NMS and score_thresh");
  }
}

// 检测结果
void ImgDetectorNode::drawRoiAndSave(
  const cv::Mat & image, const std::vector<cv::Rect> & boxes, const std::vector<int> & class_ids,
  const std::vector<float> & scores, const std::string & save_path)
{
  cv::Mat result = image.clone();
  RCLCPP_INFO(
    this->get_logger(), "drawRoiAndSave: got %zu boxes, save_path=%s", boxes.size(),
    save_path.c_str());

  // id
  std::filesystem::path out_path(save_path);
  std::filesystem::path out_dir = out_path.parent_path();
  std::string base_name = out_path.stem().string();
  std::string ext = out_path.extension().string();
  cv::Rect img_rect(0, 0, image.cols, image.rows);

  std::vector<cv::Scalar> colors;
  colors.reserve(num_classes_);
  std::mt19937 rng(std::random_device{}());
  std::uniform_int_distribution<int> dist(0, 255);
  for (int i = 0; i < num_classes_; ++i) {
    colors.emplace_back(dist(rng), dist(rng), dist(rng));
  }

  std::size_t saved_crops = 0;

  for (size_t i = 0; i < boxes.size(); ++i) {
    const auto & box = boxes[i];
    const int cls_id = class_ids[i];
    const float score = scores[i];
    cv::Scalar color(0, 255, 0);  // 绿色
    if (!colors.empty() && cls_id >= 0 && cls_id < (int)colors.size()) {
      color = colors[cls_id];
    }
    // 避免out
    cv::Rect clipped_box = box & img_rect;
    if (clipped_box.width <= 0 || clipped_box.height <= 0) {
      continue;
    }

    cv::rectangle(result, clipped_box, color, 2);
    std::string cls_name;
    if (cls_id >= 0 && cls_id < (int)class_names_.size()) {
      cls_name = class_names_[cls_id];
    } else {
      cls_name = "cls_" + std::to_string(cls_id);
    }

    char score_buf[16];
    std::snprintf(score_buf, sizeof(score_buf), "%.2f", score);
    std::string label = cls_name + " " + score_buf;

    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.6;
    int thickness = 2;
    int baseline = 0;
    cv::Size label_size = cv::getTextSize(label, font_face, font_scale, thickness, &baseline);

    int label_x = clipped_box.x;
    int label_y = std::max(0, clipped_box.y - label_size.height - 4);

    cv::Rect label_bg(label_x, label_y, label_size.width + 4, label_size.height + baseline + 4);
    // 防止out
    label_bg &= img_rect;

    cv::rectangle(result, label_bg, color, cv::FILLED);
    cv::putText(
      result, label, cv::Point(label_bg.x + 2, label_bg.y + label_size.height), font_face,
      font_scale, cv::Scalar(255, 255, 255), thickness);

    cv::Mat crop = image(clipped_box).clone();
    if (crop.empty()) {
      continue;
    }

    std::string cls_name_safe = cls_name;
    std::replace(cls_name_safe.begin(), cls_name_safe.end(), ' ', '_');

    std::string crop_name =
      base_name + "_det" + std::to_string(i) + "_" + cls_name_safe + "_s" + score_buf + ext;

    std::filesystem::path crop_path = out_dir / crop_name;

    if (!cv::imwrite(crop_path.string(), crop)) {
      RCLCPP_WARN(this->get_logger(), "Failed to save crop image: %s", crop_path.c_str());
    } else {
      ++saved_crops;
      RCLCPP_DEBUG(
        this->get_logger(), "Saved crop image: %s (score=%.3f)", crop_path.c_str(), score);
    }
  }

  // 保存
  if (!cv::imwrite(save_path, result)) {
    RCLCPP_WARN(
      this->get_logger(), "Failed to save result image (with boxes): %s", save_path.c_str());
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Result image (with boxes) saved to: %s, crops=%zu", save_path.c_str(),
      saved_crops);
  }
}

}  // namespace img_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<img_detector::ImgDetectorNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}