#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.hpp>
// #include <opencv2/imgproc.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <video_kit/cameras/ncamera.h>

using namespace std::chrono_literals;

class VioFrontendRos2 : public rclcpp::Node {
public:
  VioFrontendRos2() : rclcpp::Node("vio_frontend_ros2") {
    // Parameters
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    imu_topic_   = declare_parameter<std::string>("imu_topic", "/imu/data");
    cam_yaml_    = declare_parameter<std::string>(
      "calib_yaml",
      ament_index_cpp::get_package_share_directory("asr_sdm_vio_ros2") + "/param/calib/svo_test_pinhole.yaml");

    // Load camera
    try {
      ncam_ = vk::cameras::NCamera::loadFromYaml(cam_yaml_.c_str());
      if (!ncam_) {
        RCLCPP_ERROR(get_logger(), "Failed to load camera from %s", cam_yaml_.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Loaded camera rig from %s (num_cams=%zu)", cam_yaml_.c_str(), ncam_->numCameras());
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Exception while loading camera: %s", e.what());
    }

    // Subscriptions
    // Use RELIABLE QoS to be compatible with rosbag2_player default
    auto img_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();

    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, img_qos,
      std::bind(&VioFrontendRos2::imageCb, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos,
      std::bind(&VioFrontendRos2::imuCb, this, std::placeholders::_1));

    // Periodic status
    timer_ = create_wall_timer(1000ms, [this]() {
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                           "rx: image=%zu, imu=%zu", img_count_, imu_count_);
      img_count_ = 0; imu_count_ = 0;
    });

    RCLCPP_INFO(get_logger(), "VIO frontend node ready. image_topic=%s imu_topic=%s",
                image_topic_.c_str(), imu_topic_.c_str());
  }

private:
  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    img_count_++;
    try {
      // Convert to mono8 if needed
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
      cv::Mat gray;
      if (msg->encoding == "mono8") {
        gray = cv_ptr->image;
      } else {
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
      }
      // TODO: 后续将 gray 和时间戳传入 asr_sdm_vio 的 FrameHandler
      (void)gray;
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "imageCb convert exception: %s", e.what());
    }
  }

  void imuCb(const sensor_msgs::msg::Imu::ConstSharedPtr) {
    imu_count_++;
    // TODO: 后续将 IMU 数据传入 asr_sdm_vio 的 ImuHandler
  }

  // Params
  std::string image_topic_;
  std::string imu_topic_;
  std::string cam_yaml_;

  // Camera rig
  vk::cameras::NCamera::Ptr ncam_;

  // State
  size_t img_count_ = 0;
  size_t imu_count_ = 0;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VioFrontendRos2>());
  rclcpp::shutdown();
  return 0;
}

