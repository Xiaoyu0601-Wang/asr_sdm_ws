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

#ifndef SVO_VISUALIZER_H_
#define SVO_VISUALIZER_H_

#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <svo/global.h>
#include <svo_msgs/msg/info.hpp>
#include <svo_msgs/msg/dense_input.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>

namespace svo {

class Frame;
class Point;
class Map;
class FrameHandlerMono;

typedef boost::shared_ptr<Frame> FramePtr;

/// This class bundles all functions to publish visualisation messages.
class Visualizer
{
public:
  rclcpp::Node::SharedPtr node_;
  size_t trace_id_;
  size_t img_pub_level_;
  size_t img_pub_nth_;
  size_t dense_pub_nth_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_frames_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_points_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<svo_msgs::msg::Info>::SharedPtr pub_info_;
  rclcpp::Publisher<svo_msgs::msg::DenseInput>::SharedPtr pub_dense_;
  image_transport::Publisher pub_images_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
  bool publish_world_in_cam_frame_;
  bool publish_map_every_frame_;
  rclcpp::Duration publish_points_display_time_;
  SE3d T_world_from_vision_;

  Visualizer(rclcpp::Node::SharedPtr node);

  ~Visualizer() {};

  void publishMinimal(
      const cv::Mat& img,
      const FramePtr& frame,
      const FrameHandlerMono& slam,
      const double timestamp);

  void visualizeMarkers(
      const FramePtr& frame,
      const set<FramePtr>& core_kfs,
      const Map& map);

  void publishMapRegion(set<FramePtr> frames);

  void removeDeletedPts(const Map& map);

  void displayKeyframeWithMps(const FramePtr& frame, int ts);

  void exportToDense(const FramePtr& frame);
};

} // end namespace svo

#endif /* SVO_VISUALIZER_H_ */
