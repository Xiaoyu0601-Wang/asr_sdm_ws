// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/config.h>
#include <svo_ros/visualizer.h>
#include <vikit/params_helper.h>
#include <vikit/camera_loader.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/user_input_thread.h>
#include <vikit/math_utils.h>

namespace svo {

/// SVO Interface as ROS2 Node
class VoNode : public rclcpp::Node
{
public:
  svo::FrameHandlerMono* vo_;
  std::unique_ptr<svo::Visualizer> visualizer_;
  bool publish_markers_;
  bool publish_dense_input_;
  std::shared_ptr<vk::UserInputThread> user_input_thread_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_remote_key_;
  std::string remote_input_;
  vk::AbstractCamera* cam_;
  bool quit_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber it_sub_;

  VoNode() : Node("svo"), vo_(nullptr), cam_(nullptr), quit_(false)
  {
    // Declare and get parameters
    publish_markers_ = vk::getParam<bool>(this, "publish_markers", true);
    publish_dense_input_ = vk::getParam<bool>(this, "publish_dense_input", false);
    remote_input_ = "";

    // Start user input thread in parallel thread that listens to console keys
    if(vk::getParam<bool>(this, "accept_console_user_input", true))
      user_input_thread_ = std::make_shared<vk::UserInputThread>();

    // Create Camera
    if(!vk::camera_loader::loadFromRosNode(this, "", cam_))
      throw std::runtime_error("Camera model not correctly specified.");

    // Init VO and start
    vo_ = new svo::FrameHandlerMono(cam_);
    vo_->start();

    // Subscribe to remote input
    sub_remote_key_ = this->create_subscription<std_msgs::msg::String>(
        "remote_key", 5, std::bind(&VoNode::remoteKeyCb, this, std::placeholders::_1));
  }

  // Initialize components that need shared_from_this() - must be called after construction
  void init()
  {
    // Create visualizer (needs shared_from_this)
    visualizer_ = std::make_unique<svo::Visualizer>(this->shared_from_this());

    // Get initial position and orientation
    visualizer_->T_world_from_vision_ = Sophus::SE3d(
        vk::rpy2dcm(Eigen::Vector3d(
            vk::getParam<double>(this, "init_rx", 0.0),
            vk::getParam<double>(this, "init_ry", 0.0),
            vk::getParam<double>(this, "init_rz", 0.0))),
        Eigen::Vector3d(
            vk::getParam<double>(this, "init_tx", 0.0),
            vk::getParam<double>(this, "init_ty", 0.0),
            vk::getParam<double>(this, "init_tz", 0.0)));

    // Subscribe to cam msgs (needs shared_from_this)
    std::string cam_topic = vk::getParam<std::string>(this, "cam_topic", "camera/image_raw");
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    it_sub_ = it_->subscribe(cam_topic, 5, std::bind(&VoNode::imgCb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SVO node initialized, subscribing to: %s", cam_topic.c_str());
  }

  ~VoNode()
  {
    delete vo_;
    delete cam_;
    if(user_input_thread_ != nullptr)
      user_input_thread_->stop();
  }

  void imgCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    cv::Mat img;
    try {
      img = cv_bridge::toCvShare(msg, "mono8")->image;
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    processUserActions();
    
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    vo_->addImage(img, timestamp);
    visualizer_->publishMinimal(img, vo_->lastFrame(), *vo_, timestamp);

    if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
      visualizer_->visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

    if(publish_dense_input_)
      visualizer_->exportToDense(vo_->lastFrame());

    if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
      usleep(100000);
  }

  void processUserActions()
  {
    char input = remote_input_.empty() ? 0 : remote_input_.c_str()[0];
    remote_input_ = "";

    if(user_input_thread_ != nullptr)
    {
      char console_input = user_input_thread_->getInput();
      if(console_input != 0)
        input = console_input;
    }

    switch(input)
    {
      case 'q':
        quit_ = true;
        RCLCPP_INFO(this->get_logger(), "SVO user input: QUIT");
        break;
      case 'r':
        vo_->reset();
        RCLCPP_INFO(this->get_logger(), "SVO user input: RESET");
        break;
      case 's':
        vo_->start();
        RCLCPP_INFO(this->get_logger(), "SVO user input: START");
        break;
      default: ;
    }
  }

  void remoteKeyCb(const std_msgs::msg::String::SharedPtr key_input)
  {
    remote_input_ = key_input->data;
  }

  bool shouldQuit() const { return quit_; }
};

} // namespace svo

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<svo::VoNode>();
  
  // Initialize components that need shared_from_this()
  node->init();
  
  RCLCPP_INFO(node->get_logger(), "SVO node created");
  
  rclcpp::Rate rate(100);
  while(rclcpp::ok() && !node->shouldQuit())
  {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "SVO terminated.");
  rclcpp::shutdown();
  return 0;
}
