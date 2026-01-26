// =============================================================================
// OUTPUT_HELPER.CPP - ROS2 Visualization Marker Utilities
// =============================================================================
// This module provides helper functions for publishing ROS2 visualization
// markers to RViz. It simplifies the creation and publishing of common marker
// types used in visual odometry and SLAM visualization:
//
//   - TF transforms (coordinate frame broadcasting)
//   - Point markers (3D map points, trajectory breadcrumbs)
//   - Line markers (feature correspondences, constraints)
//   - Arrow markers (vectors, normals, directions)
//   - Camera markers (frustum visualization)
//   - Hexacopter markers (UAV visualization)
//   - Frame markers (coordinate axes at a pose)
//
// Created on: Dec 14, 2025
// Author: Michael Wang
// Ported to ROS2
// =============================================================================

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <vikit/output_helper.h>

namespace vk
{
namespace output_helper
{

// =============================================================================
// PUBLISH TF TRANSFORM
// =============================================================================
// Broadcasts a rigid body transformation to the TF2 tree.
//
// Parameters:
//   T             - SE3 transformation (Sophus library)
//   stamp         - Timestamp for the transform
//   frame_id      - Parent frame (e.g., "world")
//   child_frame_id - Child frame (e.g., "camera")
//   br            - TF2 broadcaster instance
//
// This allows other ROS2 nodes to query the transform between frames
// using tf2_ros::Buffer::lookupTransform().
// =============================================================================
void publishTfTransform(
  const Sophus::SE3d & T, const rclcpp::Time & stamp, const string & frame_id,
  const string & child_frame_id, tf2_ros::TransformBroadcaster & br)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = frame_id;
  transform_msg.child_frame_id = child_frame_id;

  // Extract rotation as quaternion from the SE3 transformation
  Eigen::Quaterniond q(T.rotationMatrix());

  // Set translation components
  transform_msg.transform.translation.x = T.translation().x();
  transform_msg.transform.translation.y = T.translation().y();
  transform_msg.transform.translation.z = T.translation().z();

  // Set rotation components (quaternion)
  transform_msg.transform.rotation.x = q.x();
  transform_msg.transform.rotation.y = q.y();
  transform_msg.transform.rotation.z = q.z();
  transform_msg.transform.rotation.w = q.w();

  br.sendTransform(transform_msg);
}


// =============================================================================
// PUBLISH POINT MARKER
// =============================================================================
// Publishes a cube marker at a 3D position. Used for visualizing:
//   - Map points in SLAM
//   - Trajectory waypoints
//   - Feature locations
//
// Parameters:
//   pub          - ROS2 marker publisher
//   pos          - 3D position in world frame
//   ns           - Namespace for marker grouping (e.g., "pts", "trajectory")
//   timestamp    - Marker timestamp
//   id           - Unique marker ID (same ID = update existing marker)
//   action       - 0=ADD/MODIFY, 1=DEPRECATED, 2=DELETE, 3=DELETEALL
//   marker_scale - Size of the cube (meters)
//   color        - RGB color [0-1] range
//   lifetime     - How long marker persists (0 = forever)
// =============================================================================
void publishPointMarker(
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, const Vector3d & pos,
  const string & ns, const rclcpp::Time & timestamp, int id, int action, double marker_scale,
  const Vector3d & color, rclcpp::Duration lifetime)
{
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = "world";
  msg.header.stamp = timestamp;
  msg.ns = ns;
  msg.id = id;
  msg.type = visualization_msgs::msg::Marker::CUBE;
  msg.action = action;  // 0 = add/modify, 2 = delete

  // Uniform scale for cube
  msg.scale.x = marker_scale;
  msg.scale.y = marker_scale;
  msg.scale.z = marker_scale;

  // Color with full opacity
  msg.color.a = 1.0;
  msg.color.r = color[0];
  msg.color.g = color[1];
  msg.color.b = color[2];

  msg.lifetime = lifetime;

  // Set position
  msg.pose.position.x = pos[0];
  msg.pose.position.y = pos[1];
  msg.pose.position.z = pos[2];

  pub->publish(msg);
}

// =============================================================================
// PUBLISH LINE MARKER
// =============================================================================
// Publishes a line segment between two 3D points. Used for visualizing:
//   - Feature correspondences between frames
//   - Graph edges in pose graphs
//   - Constraint connections
//
// Parameters:
//   pub          - ROS2 marker publisher
//   start        - Start point of line
//   end          - End point of line
//   ns           - Namespace for marker grouping
//   timestamp    - Marker timestamp
//   id           - Unique marker ID
//   action       - 0=ADD/MODIFY, 2=DELETE
//   marker_scale - Line width (only scale.x is used for LINE_STRIP)
//   color        - RGB color [0-1] range
//   lifetime     - How long marker persists
// =============================================================================
void publishLineMarker(
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, const Vector3d & start,
  const Vector3d & end, const string & ns, const rclcpp::Time & timestamp, int id, int action,
  double marker_scale, const Vector3d & color, rclcpp::Duration lifetime)
{
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = "world";
  msg.header.stamp = timestamp;
  msg.ns = ns;
  msg.id = id;
  msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
  msg.action = action;  // 0 = add/modify

  // For LINE_STRIP, only scale.x (line width) is used
  msg.scale.x = marker_scale;

  msg.color.a = 1.0;
  msg.color.r = color[0];
  msg.color.g = color[1];
  msg.color.b = color[2];

  // Define the two endpoints of the line
  msg.points.resize(2);
  msg.lifetime = lifetime;
  msg.points[0].x = start[0];
  msg.points[0].y = start[1];
  msg.points[0].z = start[2];
  msg.points[1].x = end[0];
  msg.points[1].y = end[1];
  msg.points[1].z = end[2];

  pub->publish(msg);
}

// =============================================================================
// PUBLISH ARROW MARKER
// =============================================================================
// Publishes an arrow from a position in a specified direction. Used for:
//   - Surface normals
//   - Velocity vectors
//   - Directional features
//
// Parameters:
//   pub          - ROS2 marker publisher
//   pos          - Arrow start position
//   dir          - Direction vector (will be scaled)
//   scale        - Length multiplier for the direction vector
//   ns           - Namespace for marker grouping
//   timestamp    - Marker timestamp
//   id           - Unique marker ID
//   action       - 0=ADD/MODIFY, 2=DELETE
//   marker_scale - Arrow shaft diameter
//   color        - RGB color [0-1] range
// =============================================================================
void publishArrowMarker(
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, const Vector3d & pos,
  const Vector3d & dir, double scale, const string & ns, const rclcpp::Time & timestamp, int id,
  int action, double marker_scale, const Vector3d & color)
{
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = "world";
  msg.header.stamp = timestamp;
  msg.ns = ns;
  msg.id = id;
  msg.type = visualization_msgs::msg::Marker::ARROW;
  msg.action = action;  // 0 = add/modify

  // Arrow dimensions:
  // scale.x = shaft diameter
  // scale.y = head diameter (35% wider than shaft)
  // scale.z = head length (0 = auto)
  msg.scale.x = marker_scale;
  msg.scale.y = marker_scale * 0.35;
  msg.scale.z = 0.0;

  msg.color.a = 1.0;
  msg.color.r = color[0];
  msg.color.g = color[1];
  msg.color.b = color[2];

  // Define arrow as two points: start and end
  msg.points.resize(2);
  msg.points[0].x = pos[0];
  msg.points[0].y = pos[1];
  msg.points[0].z = pos[2];
  // End point = start + scaled direction
  msg.points[1].x = pos[0] + scale * dir[0];
  msg.points[1].y = pos[1] + scale * dir[1];
  msg.points[1].z = pos[2] + scale * dir[2];

  pub->publish(msg);
}

// =============================================================================
// PUBLISH HEXACOPTER MARKER
// =============================================================================
// Publishes a 3D hexacopter (6-rotor UAV) visualization. Consists of:
//   - 6 rotor disks (cylinders)
//   - 3 arms connecting rotors (cubes)
//
// This function was originally written by Markus Achtelik from libsfly_viz.
//
// Parameters:
//   pub          - ROS2 marker publisher
//   frame_id     - Coordinate frame the hexacopter is attached to
//   ns           - Namespace for marker grouping
//   timestamp    - Marker timestamp
//   id           - Base marker ID (decrements for each component)
//   action       - Unused (always adds markers)
//   marker_scale - Overall scale factor for the hexacopter size
//   color        - RGB color for the arms [0-1] range
//
// Note: Uses multiple markers with decreasing IDs from the base ID.
// =============================================================================
void publishHexacopterMarker(
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, const string & frame_id,
  const string & ns, const rclcpp::Time & timestamp, int id, int action, double marker_scale,
  const Vector3d & color)
{
  /*
   * Function by Markus Achtelik from libsfly_viz.
   * Thank you.
   */
  (void)action;  // unused parameter

  const double sqrt2_2 = sqrt(2) / 2;

  visualization_msgs::msg::Marker marker;

  // The marker will be displayed relative to frame_id
  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.action = 0;  // ADD
  marker.id = id;

  // ---------------------------------------------------------------------------
  // Create 6 rotor disks (gray cylinders)
  // ---------------------------------------------------------------------------
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.scale.x = 0.2 * marker_scale;   // Diameter X
  marker.scale.y = 0.2 * marker_scale;   // Diameter Y
  marker.scale.z = 0.01 * marker_scale;  // Height (thin disk)
  marker.color.r = 0.4;
  marker.color.g = 0.4;
  marker.color.b = 0.4;
  marker.color.a = 0.8;
  marker.pose.position.z = 0;

  // Front left rotor
  marker.pose.position.x = 0.19 * marker_scale;
  marker.pose.position.y = 0.11 * marker_scale;
  marker.id--;
  pub->publish(marker);

  // Front right rotor
  marker.pose.position.x = 0.19 * marker_scale;
  marker.pose.position.y = -0.11 * marker_scale;
  marker.id--;
  pub->publish(marker);

  // Left rotor
  marker.pose.position.x = 0;
  marker.pose.position.y = 0.22 * marker_scale;
  marker.id--;
  pub->publish(marker);

  // Right rotor
  marker.pose.position.x = 0;
  marker.pose.position.y = -0.22 * marker_scale;
  marker.id--;
  pub->publish(marker);

  // Back left rotor
  marker.pose.position.x = -0.19 * marker_scale;
  marker.pose.position.y = 0.11 * marker_scale;
  marker.id--;
  pub->publish(marker);

  // Back right rotor
  marker.pose.position.x = -0.19 * marker_scale;
  marker.pose.position.y = -0.11 * marker_scale;
  marker.id--;
  pub->publish(marker);

  // ---------------------------------------------------------------------------
  // Create 3 arms connecting rotors (colored cubes)
  // ---------------------------------------------------------------------------
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = 0.44 * marker_scale;  // Arm length
  marker.scale.y = 0.02 * marker_scale;  // Arm width
  marker.scale.z = 0.01 * marker_scale;  // Arm thickness
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = -0.015 * marker_scale;  // Slightly below rotors
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;

  // Arm 1: 90 degree rotation (perpendicular)
  marker.pose.orientation.w = sqrt2_2;
  marker.pose.orientation.z = sqrt2_2;
  marker.id--;
  pub->publish(marker);

  // Arm 2: 30 degree rotation (quaternion: 0.9659, 0, 0, 0.2588)
  marker.pose.orientation.w = 0.9659;
  marker.pose.orientation.z = 0.2588;
  marker.id--;
  pub->publish(marker);

  // Arm 3: -30 degree rotation
  marker.pose.orientation.w = 0.9659;
  marker.pose.orientation.z = -0.2588;
  marker.id--;
  pub->publish(marker);
}

// =============================================================================
// PUBLISH CAMERA MARKER
// =============================================================================
// Publishes a camera frustum visualization as a pyramid shape. Consists of:
//   - 4 edges forming the frustum rectangle (far plane)
//   - 4 edges connecting to the camera origin (pyramid edges)
//
// This is commonly used to visualize camera poses and field of view.
//
// Parameters:
//   pub          - ROS2 marker publisher
//   frame_id     - Coordinate frame the camera is attached to
//   ns           - Namespace for marker grouping
//   timestamp    - Marker timestamp
//   id           - Base marker ID (decrements for each edge)
//   marker_scale - Overall scale factor for frustum size
//   color        - RGB color [0-1] range
//
// Note: Uses 8 cube markers with specific orientations to form edges.
// =============================================================================
void publishCameraMarker(
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, const string & frame_id,
  const string & ns, const rclcpp::Time & timestamp, int id, double marker_scale,
  const Vector3d & color)
{
  /*
   * Draw a pyramid as the camera marker representing the viewing frustum
   */
  const double sqrt2_2 = sqrt(2) / 2;

  visualization_msgs::msg::Marker marker;

  // The marker will be displayed relative to frame_id
  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.action = 0;  // ADD
  marker.id = id;

  // ---------------------------------------------------------------------------
  // Create frustum rectangle (4 edges at the far plane)
  // ---------------------------------------------------------------------------
  double r_w = 1.0;                             // Aspect ratio width
  double z_plane = (r_w / 2.0) * marker_scale;  // Distance to far plane

  marker.type = visualization_msgs::msg::Marker::CUBE;
  // Thin elongated cubes as edges
  marker.scale.x = r_w * marker_scale;
  marker.scale.y = 0.04 * marker_scale;
  marker.scale.z = 0.04 * marker_scale;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1;

  // Top horizontal edge
  marker.pose.position.x = 0;
  marker.pose.position.y = (r_w / 4.0) * marker_scale;
  marker.pose.position.z = z_plane;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.id--;
  pub->publish(marker);

  // Bottom horizontal edge
  marker.pose.position.y = -(r_w / 4.0) * marker_scale;
  marker.id--;
  pub->publish(marker);

  // Left vertical edge (rotated 90 degrees)
  marker.scale.x = (r_w / 2.0) * marker_scale;
  marker.pose.position.x = (r_w / 2.0) * marker_scale;
  marker.pose.position.y = 0;
  marker.pose.orientation.w = sqrt2_2;
  marker.pose.orientation.z = sqrt2_2;
  marker.id--;
  pub->publish(marker);

  // Right vertical edge
  marker.pose.position.x = -(r_w / 2.0) * marker_scale;
  marker.id--;
  pub->publish(marker);

  // ---------------------------------------------------------------------------
  // Create pyramid edges (4 edges from origin to corners)
  // ---------------------------------------------------------------------------
  // These connect the camera origin to the frustum corners
  marker.scale.x = (3.0 * r_w / 4.0) * marker_scale;
  marker.pose.position.z = 0.5 * z_plane;  // Midpoint between origin and far plane

  // Top-right edge (quaternion pre-computed for this orientation)
  marker.pose.position.x = (r_w / 4.0) * marker_scale;
  marker.pose.position.y = (r_w / 8.0) * marker_scale;
  marker.pose.orientation.x = 0.08198092;
  marker.pose.orientation.y = -0.34727674;
  marker.pose.orientation.z = 0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  pub->publish(marker);

  // Top-left edge
  marker.pose.position.x = -(r_w / 4.0) * marker_scale;
  marker.pose.position.y = (r_w / 8.0) * marker_scale;
  marker.pose.orientation.x = 0.08198092;
  marker.pose.orientation.y = 0.34727674;
  marker.pose.orientation.z = -0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  pub->publish(marker);

  // Bottom-left edge
  marker.pose.position.x = -(r_w / 4.0) * marker_scale;
  marker.pose.position.y = -(r_w / 8.0) * marker_scale;
  marker.pose.orientation.x = -0.08198092;
  marker.pose.orientation.y = 0.34727674;
  marker.pose.orientation.z = 0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  pub->publish(marker);

  // Bottom-right edge
  marker.pose.position.x = (r_w / 4.0) * marker_scale;
  marker.pose.position.y = -(r_w / 8.0) * marker_scale;
  marker.pose.orientation.x = -0.08198092;
  marker.pose.orientation.y = -0.34727674;
  marker.pose.orientation.z = -0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  pub->publish(marker);
}

// =============================================================================
// PUBLISH FRAME MARKER
// =============================================================================
// Publishes a coordinate frame (3 orthogonal arrows) at a given pose.
// Used to visualize:
//   - Keyframe poses
//   - Sensor coordinate frames
//   - Reference frames
//
// Creates 3 arrows following the RGB = XYZ convention:
//   - Red arrow   = X-axis
//   - Green arrow = Y-axis
//   - Blue arrow  = Z-axis
//
// Parameters:
//   pub          - ROS2 marker publisher
//   rot          - 3x3 rotation matrix defining frame orientation
//   pos          - 3D position of frame origin
//   ns           - Namespace for marker grouping
//   timestamp    - Marker timestamp
//   id           - Base marker ID (uses id, id+1, id+2 for 3 arrows)
//   action       - 0=ADD/MODIFY, 2=DELETE
//   marker_scale - Length of the arrows
//   lifetime     - How long markers persist
// =============================================================================
void publishFrameMarker(
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, const Matrix3d & rot,
  const Vector3d & pos, const string & ns, const rclcpp::Time & timestamp, int id, int action,
  double marker_scale, rclcpp::Duration lifetime)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.id = id++;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = action;  // 0 = add/modify
  marker.points.reserve(2);

  geometry_msgs::msg::Point point;

  // ---------------------------------------------------------------------------
  // Z-axis arrow (Blue)
  // ---------------------------------------------------------------------------
  // Start point (frame origin)
  point.x = static_cast<float>(pos.x());
  point.y = static_cast<float>(pos.y());
  point.z = static_cast<float>(pos.z());
  marker.points.push_back(point);

  // End point (along Z-axis of the rotation matrix)
  point.x = static_cast<float>(pos.x() + marker_scale * rot(0, 2));
  point.y = static_cast<float>(pos.y() + marker_scale * rot(1, 2));
  point.z = static_cast<float>(pos.z() + marker_scale * rot(2, 2));
  marker.points.push_back(point);

  marker.scale.x = 0.5 * marker_scale;  // Shaft diameter
  marker.scale.y = 0.5 * marker_scale;  // Head diameter
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;  // Blue for Z
  marker.lifetime = lifetime;
  pub->publish(marker);

  // ---------------------------------------------------------------------------
  // X-axis arrow (Red)
  // ---------------------------------------------------------------------------
  marker.id = id++;
  marker.points.clear();

  // Start point (frame origin)
  point.x = static_cast<float>(pos.x());
  point.y = static_cast<float>(pos.y());
  point.z = static_cast<float>(pos.z());
  marker.points.push_back(point);

  // End point (along X-axis of the rotation matrix)
  point.x = static_cast<float>(pos.x() + marker_scale * rot(0, 0));
  point.y = static_cast<float>(pos.y() + marker_scale * rot(1, 0));
  point.z = static_cast<float>(pos.z() + marker_scale * rot(2, 0));
  marker.points.push_back(point);

  marker.color.r = 1.0;  // Red for X
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.lifetime = lifetime;
  pub->publish(marker);

  // ---------------------------------------------------------------------------
  // Y-axis arrow (Green)
  // ---------------------------------------------------------------------------
  marker.id = id++;
  marker.points.clear();

  // Start point (frame origin)
  point.x = static_cast<float>(pos.x());
  point.y = static_cast<float>(pos.y());
  point.z = static_cast<float>(pos.z());
  marker.points.push_back(point);

  // End point (along Y-axis of the rotation matrix)
  point.x = static_cast<float>(pos.x() + marker_scale * rot(0, 1));
  point.y = static_cast<float>(pos.y() + marker_scale * rot(1, 1));
  point.z = static_cast<float>(pos.z() + marker_scale * rot(2, 1));
  marker.points.push_back(point);

  marker.color.r = 0.0;
  marker.color.g = 1.0;  // Green for Y
  marker.color.b = 0.0;
  marker.lifetime = lifetime;
  pub->publish(marker);
}

}  // namespace output_helper
}  // namespace vk
