#include "asr_sdm_ros2/svo_node_base.h"

#include <asr_sdm_vio/common/logging.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vikit/params_helper.h>

#include <rclcpp/utilities.hpp>

namespace svo_ros
{

void SvoNodeBase::initThirdParty(int argc, char ** argv)
{
  // glog initialization
  google::InitGoogleLogging(argv[0]);
  // We only pass program name to gflags to avoid parsing ROS arguments
  int gflags_argc = 1;
  char** gflags_argv = argv;
  google::ParseCommandLineFlags(&gflags_argc, &gflags_argv, true);
  google::InstallFailureSignalHandler();

  // rclcpp::init() is now called in main() in svo_node.cpp
  // and should not be called here.
}

SvoNodeBase::SvoNodeBase(const rclcpp::Node::SharedPtr & node, int argc, char** argv)
: node_(node)
{
  if (!node_) {
    throw std::invalid_argument("SvoNodeBase: node is null");
  }

  initThirdParty(argc, argv);

  const bool pipeline_is_stereo = node_->declare_parameter<bool>("pipeline_is_stereo", false);
  type_ = pipeline_is_stereo ? svo::PipelineType::kStereo : svo::PipelineType::kMono;

  // Delay construction until after the parameter is safely declared.
  svo_interface_ = std::make_unique<svo::SvoInterface>(type_, node_);

  if (svo_interface_->imu_handler_) {
    svo_interface_->subscribeImu();
  }
  svo_interface_->subscribeImage();
  svo_interface_->subscribeRemoteKey();
}

void SvoNodeBase::run()
{
  // ROS2 scheme: use a single executor to service all callback groups.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node_);
  exec.spin();

  SVO_INFO_STREAM("SVO quit");
  if (svo_interface_) svo_interface_->quit_ = true;
  SVO_INFO_STREAM("SVO terminated.\n");
}

}  // namespace svo_ros
