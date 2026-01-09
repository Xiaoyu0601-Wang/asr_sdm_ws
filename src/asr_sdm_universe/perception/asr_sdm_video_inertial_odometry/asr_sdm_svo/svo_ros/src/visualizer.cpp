// =============================================================================
// VISUALIZER.CPP - ROS2 Visualization Interface for SVO
// =============================================================================
// This module provides real-time visualization of the SVO visual odometry
// pipeline. It publishes:
//   - Camera poses with uncertainty (covariance)
//   - Tracked feature images with annotations
//   - 3D markers for keyframes, trajectory, and map points
//   - TF transforms for coordinate frame relationships
//   - Dense reconstruction input data
// =============================================================================

#include <cv_bridge/cv_bridge.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <asr_sdm_perception_msgs/msg/dense_input.hpp>
#include <asr_sdm_perception_msgs/msg/info.hpp>

#include <svo/feature.h>
#include <svo/frame.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/point.h>
#include <svo_ros/visualizer.h>
#include <vikit/output_helper.h>
#include <vikit/params_helper.h>
#include <vikit/timer.h>

#include <algorithm>
#include <deque>
#include <fstream>
#include <iostream>

namespace svo
{

// =============================================================================
// CONSTRUCTOR
// =============================================================================
// Initializes the Visualizer with ROS2 publishers, TF broadcaster, and
// configuration parameters loaded from the ROS2 parameter server.
//
// Parameters loaded from ROS2:
//   - svo/publish_img_pyr_level: Which pyramid level to publish (0=full res)
//   - svo/publish_every_nth_img: Publish frequency for images
//   - svo/publish_every_nth_dense_input: Publish frequency for dense data
//   - svo/publish_world_in_cam_frame: Transform convention toggle
//   - svo/publish_map_every_frame: Whether to publish full map each frame
//   - svo/publish_point_display_time: How long points persist in RViz
// =============================================================================
Visualizer::Visualizer(rclcpp::Node::SharedPtr node)
: node_(node),
  trace_id_(0),  // Frame counter for nth-frame publishing logic
  img_pub_level_(vk::getParam<int>(node_.get(), "svo/publish_img_pyr_level", 0)),
  img_pub_nth_(vk::getParam<int>(node_.get(), "svo/publish_every_nth_img", 1)),
  dense_pub_nth_(vk::getParam<int>(node_.get(), "svo/publish_every_nth_dense_input", 1)),
  publish_world_in_cam_frame_(
    vk::getParam<bool>(node_.get(), "svo/publish_world_in_cam_frame", true)),
  publish_map_every_frame_(vk::getParam<bool>(node_.get(), "svo/publish_map_every_frame", false)),
  publish_points_display_time_(
    rclcpp::Duration::from_seconds(
      vk::getParam<double>(node_.get(), "svo/publish_point_display_time", 0.0))),
  // Identity transform: vision frame = world frame (can be overridden)
  T_world_from_vision_(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero())
{
  // ---------------------------------------------------------------------------
  // Initialize ROS2 Publishers
  // ---------------------------------------------------------------------------
  // Keyframe visualization markers (camera frustums)
  pub_frames_ = node_->create_publisher<visualization_msgs::msg::Marker>("svo/keyframes", 10);

  // 3D point cloud and trajectory markers (large queue for many points)
  pub_points_ = node_->create_publisher<visualization_msgs::msg::Marker>("svo/points", 1000);

  // Camera pose with 6x6 covariance matrix
  pub_pose_ =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("svo/pose", 10);

  // SLAM status information (stage, tracking quality, num matches, etc.)
  pub_info_ = node_->create_publisher<asr_sdm_perception_msgs::msg::Info>("svo/info", 10);

  // Dense reconstruction input (image + pose + depth range)
  pub_dense_ = node_->create_publisher<asr_sdm_perception_msgs::msg::DenseInput>("svo/dense_input", 10);

  // ---------------------------------------------------------------------------
  // Initialize TF2 Broadcaster
  // ---------------------------------------------------------------------------
  // Broadcasts camera pose transform to the TF tree
  br_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  // ---------------------------------------------------------------------------
  // Initialize Image Transport Publisher
  // ---------------------------------------------------------------------------
  // Uses image_transport for efficient image streaming (supports compressed)
  image_transport::ImageTransport it(node_);
  pub_images_ = it.advertise("svo/image", 10);
}

// =============================================================================
// PUBLISH MINIMAL
// =============================================================================
// Main publishing function called every frame. Publishes:
//   1. SLAM info message (status, keyframes, tracking quality)
//   2. Annotated debug image with tracked features
//   3. Camera pose with covariance
//
// The function handles different SLAM stages:
//   - STAGE_PAUSED: Only publish raw image
//   - STAGE_SECOND_FRAME: Draw feature tracking lines (initialization)
//   - STAGE_DEFAULT_FRAME: Normal operation, publish pose
// =============================================================================
void Visualizer::publishMinimal(
  const cv::Mat & img, const FramePtr & frame, const FrameHandlerMono & slam,
  const double timestamp)
{
  ++trace_id_;  // Increment frame counter

  // Create header with camera frame and timestamp
  std_msgs::msg::Header header_msg;
  header_msg.frame_id = "cam";
  header_msg.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));

  // ---------------------------------------------------------------------------
  // Publish SLAM Info Message
  // ---------------------------------------------------------------------------
  // Contains diagnostics: processing time, keyframe IDs, stage, quality
  if (pub_info_->get_subscription_count() > 0) {
    asr_sdm_perception_msgs::msg::Info msg_info;
    msg_info.header = header_msg;
    msg_info.processing_time = slam.lastProcessingTime();

    // Collect all keyframe IDs currently in the map
    msg_info.keyframes.reserve(slam.map().keyframes_.size());
    for (list<FramePtr>::const_iterator it = slam.map().keyframes_.begin();
         it != slam.map().keyframes_.end(); ++it)
      msg_info.keyframes.push_back((*it)->id_);

    msg_info.stage = static_cast<int>(slam.stage());
    msg_info.tracking_quality = static_cast<int>(slam.trackingQuality());

    // Number of feature matches in current frame
    if (frame != NULL)
      msg_info.num_matches = slam.lastNumObservations();
    else
      msg_info.num_matches = 0;

    pub_info_->publish(msg_info);
  }

  // ---------------------------------------------------------------------------
  // Handle NULL Frame Case (SLAM paused or lost)
  // ---------------------------------------------------------------------------
  if (frame == NULL) {
    // Still publish image when paused so user can see what camera sees
    if (pub_images_.getNumSubscribers() > 0 && slam.stage() == FrameHandlerBase::STAGE_PAUSED) {
      cv_bridge::CvImage img_msg;
      img_msg.header.stamp = node_->now();
      img_msg.header.frame_id = "image";
      img_msg.image = img;
      img_msg.encoding = sensor_msgs::image_encodings::MONO8;
      pub_images_.publish(img_msg.toImageMsg());
    }
    return;
  }

  // ---------------------------------------------------------------------------
  // Publish Annotated Debug Image
  // ---------------------------------------------------------------------------
  // Publishes every nth frame with tracked features drawn
  if (img_pub_nth_ > 0 && trace_id_ % img_pub_nth_ == 0 && pub_images_.getNumSubscribers() > 0) {
    // Scale factor for pyramid level (level 0 = 1x, level 1 = 2x, etc.)
    const int scale = (1 << img_pub_level_);

    // Convert grayscale pyramid image to RGB for colored annotations
    cv::Mat img_rgb(frame->img_pyr_[img_pub_level_].size(), CV_8UC3);
    cv::cvtColor(frame->img_pyr_[img_pub_level_], img_rgb, cv::COLOR_GRAY2RGB);

    // During initialization (STAGE_SECOND_FRAME): draw feature tracking lines
    // Shows correspondences between reference and current frame
    if (slam.stage() == FrameHandlerBase::STAGE_SECOND_FRAME) {
      const vector<cv::Point2f> & px_ref(slam.initFeatureTrackRefPx());
      const vector<cv::Point2f> & px_cur(slam.initFeatureTrackCurPx());
      for (vector<cv::Point2f>::const_iterator it_ref = px_ref.begin(), it_cur = px_cur.begin();
           it_ref != px_ref.end(); ++it_ref, ++it_cur)
        cv::line(
          img_rgb, cv::Point2f(it_cur->x / scale, it_cur->y / scale),
          cv::Point2f(it_ref->x / scale, it_ref->y / scale), cv::Scalar(0, 255, 0), 2);
    }

    // Draw tracked features on the image
    // Visualization style depends on pyramid level (resolution)
    if (img_pub_level_ == 0) {
      // Full resolution: draw detailed feature markers
      for (Features::iterator it = frame->fts_.begin(); it != frame->fts_.end(); ++it) {
        if ((*it)->type == Feature::EDGELET)
          // Edgelets: draw line segment perpendicular to gradient
          cv::line(
            img_rgb,
            cv::Point2f((*it)->px[0] + 3 * (*it)->grad[1], (*it)->px[1] - 3 * (*it)->grad[0]),
            cv::Point2f((*it)->px[0] - 3 * (*it)->grad[1], (*it)->px[1] + 3 * (*it)->grad[0]),
            cv::Scalar(255, 0, 255), 2);  // Magenta for edgelets
        else
          // Corner features: draw 5x5 green rectangle
          cv::rectangle(
            img_rgb, cv::Point2f((*it)->px[0] - 2, (*it)->px[1] - 2),
            cv::Point2f((*it)->px[0] + 2, (*it)->px[1] + 2), cv::Scalar(0, 255, 0), cv::FILLED);
      }
    } else if (img_pub_level_ == 1) {
      // Half resolution: draw smaller 3x3 rectangles
      for (Features::iterator it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
        cv::rectangle(
          img_rgb, cv::Point2f((*it)->px[0] / scale - 1, (*it)->px[1] / scale - 1),
          cv::Point2f((*it)->px[0] / scale + 1, (*it)->px[1] / scale + 1), cv::Scalar(0, 255, 0),
          cv::FILLED);
    } else {
      // Lower resolution: draw single pixel markers
      for (Features::iterator it = frame->fts_.begin(); it != frame->fts_.end(); ++it) {
        cv::Vec3b & p = img_rgb.at<cv::Vec3b>((*it)->px[1] / scale, (*it)->px[0] / scale);
        p[0] = 0;    // B
        p[1] = 255;  // G
        p[2] = 0;    // R
      }
    }

    // Publish the annotated image
    cv_bridge::CvImage img_msg;
    img_msg.header = header_msg;
    img_msg.image = img_rgb;
    img_msg.encoding = sensor_msgs::image_encodings::BGR8;
    pub_images_.publish(img_msg.toImageMsg());
  }

  // ---------------------------------------------------------------------------
  // Publish Camera Pose with Covariance
  // ---------------------------------------------------------------------------
  // Only publish during normal tracking (STAGE_DEFAULT_FRAME)
  if (
    pub_pose_->get_subscription_count() > 0 &&
    slam.stage() == FrameHandlerBase::STAGE_DEFAULT_FRAME) {
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    Eigen::Matrix<double, 6, 6> Cov;

    if (publish_world_in_cam_frame_) {
      // Convention 1: Publish world origin in camera frame (T_cam_world)
      // Useful for: robot-centric applications
      SE3d T_cam_from_world(frame->T_f_w_ * T_world_from_vision_);
      q = Eigen::Quaterniond(T_cam_from_world.rotationMatrix());
      p = T_cam_from_world.translation();
      Cov = frame->Cov_;
    } else {
      // Convention 2: Publish camera in world frame (T_world_cam)
      // Useful for: mapping, global localization
      SE3d T_world_from_cam(T_world_from_vision_ * frame->T_f_w_.inverse());
      q = Eigen::Quaterniond(
        T_world_from_cam.rotationMatrix() * T_world_from_vision_.rotationMatrix().transpose());
      p = T_world_from_cam.translation();
      // Transform covariance to world frame using adjoint
      Cov = T_world_from_cam.Adj() * frame->Cov_ * T_world_from_cam.inverse().Adj();
    }

    // Build and publish PoseWithCovarianceStamped message
    auto msg_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg_pose->header = header_msg;
    msg_pose->pose.pose.position.x = p[0];
    msg_pose->pose.pose.position.y = p[1];
    msg_pose->pose.pose.position.z = p[2];
    msg_pose->pose.pose.orientation.x = q.x();
    msg_pose->pose.pose.orientation.y = q.y();
    msg_pose->pose.pose.orientation.z = q.z();
    msg_pose->pose.pose.orientation.w = q.w();

    // Copy 6x6 covariance matrix (row-major to flat array)
    for (size_t i = 0; i < 36; ++i) msg_pose->pose.covariance[i] = Cov(i % 6, i / 6);
    pub_pose_->publish(*msg_pose);
  }
}

// =============================================================================
// VISUALIZE MARKERS
// =============================================================================
// Publishes 3D visualization markers for RViz:
//   - TF transform for camera position
//   - Camera marker at current position
//   - Trajectory point (breadcrumb trail)
//   - Keyframe markers and map points (if keyframe or publish_map_every_frame_)
// =============================================================================
void Visualizer::visualizeMarkers(
  const FramePtr & frame, const set<FramePtr> & core_kfs, const Map & map)
{
  if (frame == NULL) return;

  rclcpp::Time stamp = rclcpp::Time(static_cast<int64_t>(frame->timestamp_ * 1e9));

  // Broadcast TF transform: world -> cam_pos
  vk::output_helper::publishTfTransform(
    frame->T_f_w_ * T_world_from_vision_.inverse(), stamp, "cam_pos", "world", *br_);

  // Only publish markers if there are subscribers
  if (pub_frames_->get_subscription_count() > 0 || pub_points_->get_subscription_count() > 0) {
    // Current camera frustum marker (blue)
    vk::output_helper::publishCameraMarker(
      pub_frames_, "cam_pos", "cams", stamp, 1, 0.3, Eigen::Vector3d(0., 0., 1.));

    // Add point to trajectory trail (dark blue)
    vk::output_helper::publishPointMarker(
      pub_points_, T_world_from_vision_ * frame->pos(), "trajectory", node_->now(), trace_id_, 0,
      0.006, Eigen::Vector3d(0., 0., 0.5));

    // Publish full map region on keyframes (or every frame if configured)
    if (frame->isKeyframe() || publish_map_every_frame_) publishMapRegion(core_kfs);

    // Clean up markers for deleted map points
    removeDeletedPts(map);
  }
}

// =============================================================================
// PUBLISH MAP REGION
// =============================================================================
// Publishes visualization for a set of keyframes and their associated 3D points.
// Called when a new keyframe is created to update the map visualization.
// =============================================================================
void Visualizer::publishMapRegion(set<FramePtr> frames)
{
  if (pub_points_->get_subscription_count() > 0) {
    // Get current timestamp to avoid publishing same point multiple times
    int ts = vk::Timer::getCurrentTime();
    for (set<FramePtr>::iterator it = frames.begin(); it != frames.end(); ++it)
      displayKeyframeWithMps(*it, ts);
  }
}

// =============================================================================
// REMOVE DELETED POINTS
// =============================================================================
// Removes markers for map points that have been deleted from the map.
// Uses action=2 (DELETE) to remove markers from RViz.
// =============================================================================
void Visualizer::removeDeletedPts(const Map & map)
{
  if (pub_points_->get_subscription_count() > 0) {
    for (list<Point *>::const_iterator it = map.trash_points_.begin();
         it != map.trash_points_.end(); ++it)
      // Action 2 = DELETE marker with this ID
      vk::output_helper::publishPointMarker(
        pub_points_, Eigen::Vector3d(), "pts", node_->now(), (*it)->id_, 2, 0.006,
        Eigen::Vector3d());
  }
}

// =============================================================================
// DISPLAY KEYFRAME WITH MAP POINTS
// =============================================================================
// Visualizes a single keyframe and all its associated 3D map points:
//   - Keyframe: small frame marker showing camera orientation
//   - Map points: colored spheres at 3D positions (magenta)
//
// Uses timestamp to avoid publishing the same point multiple times per update.
// =============================================================================
void Visualizer::displayKeyframeWithMps(const FramePtr & frame, int ts)
{
  // Compute keyframe pose in world frame
  SE3d T_world_cam(T_world_from_vision_ * frame->T_f_w_.inverse());

  // Publish keyframe marker (small coordinate frame)
  // ID = frame->id_ * 10 to avoid collision with point IDs
  vk::output_helper::publishFrameMarker(
    pub_frames_, T_world_cam.rotationMatrix(), T_world_cam.translation(), "kfs", node_->now(),
    frame->id_ * 10, 0, 0.015);

  // Publish all map points observed in this keyframe
  for (Features::iterator it = frame->fts_.begin(); it != frame->fts_.end(); ++it) {
    // Skip features without associated 3D points
    if ((*it)->point == NULL) continue;

    // Skip if already published this update cycle (avoid duplicates)
    if ((*it)->point->last_published_ts_ == ts) continue;

    // Publish point marker (magenta sphere)
    vk::output_helper::publishPointMarker(
      pub_points_, T_world_from_vision_ * (*it)->point->pos_, "pts", node_->now(),
      (*it)->point->id_, 0, 0.005, Eigen::Vector3d(1.0, 0., 1.0), publish_points_display_time_);

    // Mark point as published this cycle
    (*it)->point->last_published_ts_ = ts;
  }
}

// =============================================================================
// EXPORT TO DENSE
// =============================================================================
// Publishes data required for dense reconstruction pipelines:
//   - Grayscale image
//   - Camera pose in world frame
//   - Depth range (min/max) from sparse features
//
// This allows external dense reconstruction nodes to compute dense depth maps.
// =============================================================================
void Visualizer::exportToDense(const FramePtr & frame)
{
  // Check publishing conditions
  if (
    frame != NULL && dense_pub_nth_ > 0 && trace_id_ % dense_pub_nth_ == 0 &&
    pub_dense_->get_subscription_count() > 0) {
    asr_sdm_perception_msgs::msg::DenseInput msg;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(frame->timestamp_ * 1e9));
    msg.header.frame_id = "world";
    msg.frame_id = frame->id_;

    // ---------------------------------------------------------------------------
    // Attach grayscale image
    // ---------------------------------------------------------------------------
    cv_bridge::CvImage img_msg;
    img_msg.header.stamp = msg.header.stamp;
    img_msg.header.frame_id = "camera";
    img_msg.image = frame->img();
    img_msg.encoding = sensor_msgs::image_encodings::MONO8;
    msg.image = *img_msg.toImageMsg();

    // ---------------------------------------------------------------------------
    // Compute depth range from sparse features
    // ---------------------------------------------------------------------------
    // Dense reconstruction uses this to bound the depth search range
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::min();
    for (Features::iterator it = frame->fts_.begin(); it != frame->fts_.end(); ++it) {
      if ((*it)->point == NULL) continue;
      // Transform point to camera frame to get depth (z-coordinate)
      Eigen::Vector3d pos = frame->T_f_w_ * (*it)->point->pos_;
      min_z = fmin(pos[2], min_z);
      max_z = fmax(pos[2], max_z);
    }
    msg.min_depth = (float)min_z;
    msg.max_depth = (float)max_z;

    // ---------------------------------------------------------------------------
    // Attach camera pose (camera in world frame)
    // ---------------------------------------------------------------------------
    SE3d T_world_from_cam(T_world_from_vision_ * frame->T_f_w_.inverse());
    Eigen::Quaterniond q(T_world_from_cam.rotationMatrix());
    Eigen::Vector3d p(T_world_from_cam.translation());

    msg.pose.position.x = p[0];
    msg.pose.position.y = p[1];
    msg.pose.position.z = p[2];
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();

    pub_dense_->publish(msg);
  }
}

}  // end namespace svo
