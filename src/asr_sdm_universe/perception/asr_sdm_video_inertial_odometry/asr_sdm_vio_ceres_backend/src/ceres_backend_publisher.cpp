#include "asr_sdm_vio/ceres_backend_publisher.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


namespace
{
template <typename T>
void normalizeVector(const std::vector<T>& in, std::vector<float>* out)
{
  auto res = std::minmax_element(in.begin(), in.end());
  const T min = *res.first;
  const T max = *res.second;
  const float dist = static_cast<float>(max - min);

  out->resize(in.size());
  for (size_t i = 0; i < in.size(); i++)
  {
    (*out)[i] = (in[i] - min) / dist;
  }
}
}

namespace svo
{
CeresBackendPublisher::CeresBackendPublisher(
    rclcpp::Node* node,
    const std::shared_ptr<ceres_backend::Map>& map_ptr)
  : node_(node)
  , map_ptr_(map_ptr)
{
  pub_imu_pose_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "backend_pose_imu", 10);
  pub_imu_pose_viz_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>("backend_pose_imu_viz", 10);
  pub_points_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("backend_points", 10);

}

void CeresBackendPublisher::publish(const ViNodeState& state,
                                    const int64_t timestamp,
                                    const int32_t seq)
{
  publishImuPose(state, timestamp, seq);
  publishBackendLandmarks(timestamp);
}

void CeresBackendPublisher::publishImuPose(const ViNodeState& state,
                                           const int64_t timestamp,
                                           const int32_t seq)
{
  // Trace state
  state_ = state;

  {
    std::lock_guard<std::mutex> lock(mutex_frame_id_);
    state_frame_id_ = BundleId(seq);
  }

  size_t n_pose_sub = pub_imu_pose_->get_subscription_count();
  size_t n_pose_viz_sub = pub_imu_pose_viz_->get_subscription_count();
  if (n_pose_sub == 0 && n_pose_viz_sub == 0)
  {
    return;
  }
  VLOG(100) << "Publish IMU Pose";

  Eigen::Quaterniond q = state.get_T_W_B().getRotation().toImplementation();
  Eigen::Vector3d p = state.get_T_W_B().getPosition();
  rclcpp::Time time(timestamp);

  if (n_pose_sub > 0)
  {
    auto msg_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg_pose->header.stamp = time;
    msg_pose->header.frame_id = kWorldFrame;
    msg_pose->pose.pose.position.x = p[0];
    msg_pose->pose.pose.position.y = p[1];
    msg_pose->pose.pose.position.z = p[2];
    msg_pose->pose.pose.orientation.x = q.x();
    msg_pose->pose.pose.orientation.y = q.y();
    msg_pose->pose.pose.orientation.z = q.z();
    msg_pose->pose.pose.orientation.w = q.w();
    for (size_t i = 0; i < 36; ++i)
      msg_pose->pose.covariance[i] = 0;
    pub_imu_pose_->publish(*msg_pose);
  }

  if (n_pose_viz_sub > 0)
  {
    auto msg_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    msg_pose->header.stamp = time;
    msg_pose->header.frame_id = kWorldFrame;
    msg_pose->pose.position.x = p[0];
    msg_pose->pose.position.y = p[1];
    msg_pose->pose.position.z = p[2];
    msg_pose->pose.orientation.x = q.x();
    msg_pose->pose.orientation.y = q.y();
    msg_pose->pose.orientation.z = q.z();
    msg_pose->pose.orientation.w = q.w();
    pub_imu_pose_viz_->publish(*msg_pose);
  }
}

void CeresBackendPublisher::publishBackendLandmarks(
    const int64_t timestamp) const
{
  if (pub_points_->get_subscription_count() == 0)
  {
    return;
  }

  // get all landmarks
  const std::unordered_map<
      uint64_t, std::shared_ptr<ceres_backend::ParameterBlock> >& idmap =
      map_ptr_->idToParameterBlockMap();
  std::vector<const double*> landmark_pointers;
  std::vector<uint64_t> point_ids;
  landmark_pointers.reserve(idmap.size());
  point_ids.reserve(idmap.size());

  for (const auto& it : idmap)
  {
    if (it.second->typeInfo() == "HomogeneousPointParameterBlock" &&
        !it.second->fixed())
    {
      landmark_pointers.push_back(it.second->parameters());
      point_ids.push_back(it.second->id());
    }
  }

  const size_t n_pts = landmark_pointers.size();
  if (n_pts < 5)
  {
    return;
  }

  std::vector<float> intensities;
  normalizeVector(point_ids, &intensities);

  // Build PCL point cloud and convert to ROS2 message
  pcl::PointCloud<pcl::PointXYZI> pc;
  pc.reserve(n_pts);
  for (size_t i = 0; i < n_pts; ++i)
  {
    const double* p = landmark_pointers[i];
    pcl::PointXYZI pt;
    pt.x = static_cast<float>(p[0]);
    pt.y = static_cast<float>(p[1]);
    pt.z = static_cast<float>(p[2]);
    pt.intensity = static_cast<float>(intensities[i]);
    pc.push_back(pt);
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(pc, cloud_msg);
  cloud_msg.header.stamp = rclcpp::Time(timestamp);
  cloud_msg.header.frame_id = kWorldFrame;
  pub_points_->publish(cloud_msg);
}

}  // namespace svo
