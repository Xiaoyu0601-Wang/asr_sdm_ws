#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class ImageFolderPublisher : public rclcpp::Node
{
public:
  ImageFolderPublisher() : Node("image_folder_publisher"), current_index_(0)
  {
    // 声明参数
    this->declare_parameter<std::string>("image_folder", "images");
    this->declare_parameter<double>("publish_rate", 10.0);
    this->declare_parameter<std::string>("topic_name", "/camera/image_raw");
    this->declare_parameter<bool>("loop", true);
    this->declare_parameter<bool>("shuffle", false);
    this->declare_parameter<std::string>("image_encoding", "bgr8");
    this->declare_parameter<bool>("resize", false);
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 480);
    this->declare_parameter<bool>("verbose", true);

    // 获取参数值
    image_folder_ = this->get_parameter("image_folder").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    std::string topic_name = this->get_parameter("topic_name").as_string();
    loop_ = this->get_parameter("loop").as_bool();
    shuffle_ = this->get_parameter("shuffle").as_bool();
    image_encoding_ = this->get_parameter("image_encoding").as_string();
    resize_ = this->get_parameter("resize").as_bool();
    target_width_ = this->get_parameter("image_width").as_int();
    target_height_ = this->get_parameter("image_height").as_int();
    verbose_ = this->get_parameter("verbose").as_bool();

    // 初始化日志
    RCLCPP_INFO(this->get_logger(), "Starting Image Folder Publisher...");
    RCLCPP_INFO(this->get_logger(), "Image folder: %s", image_folder_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f Hz", publish_rate);
    RCLCPP_INFO(this->get_logger(), "Topic: %s", topic_name.c_str());

    // 加载图像文件列表
    if (!loadImageFiles()) {
      RCLCPP_FATAL(
        this->get_logger(), "Failed to load images from folder: %s", image_folder_.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu images", image_files_.size());

    // 如果启用随机播放，打乱顺序
    if (shuffle_) {
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(image_files_.begin(), image_files_.end(), g);
      RCLCPP_INFO(this->get_logger(), "Shuffled image order");
    }

    // 设置发布者
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      topic_name, rclcpp::SensorDataQoS().reliable());

    // 设置定时器
    timer_period_ = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ =
      this->create_wall_timer(timer_period_, std::bind(&ImageFolderPublisher::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Image folder publisher initialized successfully!");
  }

private:
  bool loadImageFiles()
  {
    // 支持的图像格式扩展名
    std::vector<std::string> extensions = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif",
                                           ".JPG", ".JPEG", ".PNG", ".BMP", ".TIFF", ".TIF"};

    fs::path folder_path(image_folder_);

    // 检查文件夹是否存在
    if (!fs::exists(folder_path)) {
      RCLCPP_ERROR(this->get_logger(), "Folder does not exist: %s", folder_path.string().c_str());
      return false;
    }

    if (!fs::is_directory(folder_path)) {
      RCLCPP_ERROR(this->get_logger(), "Path is not a directory: %s", folder_path.string().c_str());
      return false;
    }

    // 遍历文件夹中的所有文件
    for (const auto & entry : fs::directory_iterator(folder_path)) {
      if (entry.is_regular_file()) {
        std::string extension = entry.path().extension().string();

        // 检查文件扩展名是否在支持的列表中
        bool supported = false;
        for (const auto & ext : extensions) {
          if (extension == ext) {
            supported = true;
            break;
          }
        }

        if (supported) {
          image_files_.push_back(entry.path().string());
          if (verbose_) {
            RCLCPP_DEBUG(this->get_logger(), "Found: %s", entry.path().filename().c_str());
          }
        }
      }
    }

    if (image_files_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No image files found in %s", folder_path.string().c_str());
      return false;
    }

    // 按文件名排序
    std::sort(image_files_.begin(), image_files_.end());

    return true;
  }

  void timerCallback()
  {
    if (image_files_.empty() || current_index_ >= image_files_.size()) {
      return;
    }

    std::string image_path = image_files_[current_index_];

    try {
      // 读取图像
      cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);

      if (image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read image: %s", image_path.c_str());
        moveToNextImage();
        return;
      }

      // 调整大小（如果需要）
      if (resize_ && (image.cols != target_width_ || image.rows != target_height_)) {
        cv::resize(image, image, cv::Size(target_width_, target_height_));
      }

      // 转换图像消息
      std_msgs::msg::Header header;
      header.stamp = this->now();
      header.frame_id = "camera_frame";

      cv_bridge::CvImage cv_image(header, image_encoding_, image);
      auto msg = cv_image.toImageMsg();

      // 发布消息
      publisher_->publish(*msg);

      // 日志输出
      if (verbose_) {
        fs::path path_obj(image_path);
        RCLCPP_INFO(
          this->get_logger(), "Published: %s (%zu/%zu)", path_obj.filename().c_str(),
          current_index_ + 1, image_files_.size());
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "Error processing image %s: %s", image_path.c_str(), e.what());
    }

    moveToNextImage();
  }

  void moveToNextImage()
  {
    current_index_++;

    if (current_index_ >= image_files_.size()) {
      if (loop_) {
        current_index_ = 0;
        if (shuffle_) {
          std::random_device rd;
          std::mt19937 g(rd());
          std::shuffle(image_files_.begin(), image_files_.end(), g);
        }
        RCLCPP_INFO(this->get_logger(), "Restarting image sequence");
      } else {
        RCLCPP_INFO(this->get_logger(), "All images published. Stopping...");
        timer_->cancel();
        rclcpp::shutdown();
      }
    }
  }

  // ROS 2组件
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

  // 数据成员
  std::vector<std::string> image_files_;
  size_t current_index_;
  std::string image_folder_;
  bool loop_;
  bool shuffle_;
  bool resize_;
  bool verbose_;
  std::string image_encoding_;
  int target_width_;
  int target_height_;
  std::chrono::duration<double> timer_period_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<ImageFolderPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}