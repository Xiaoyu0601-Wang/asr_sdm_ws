#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class RigPoseToTf : public rclcpp::Node {
public:
  RigPoseToTf() : rclcpp::Node("rig_pose_to_tf") {
    pose_topic_ = declare_parameter<std::string>("pose_topic", "Rig");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "map");
    child_frame_  = declare_parameter<std::string>("child_frame", "rig");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic_, 10,
      std::bind(&RigPoseToTf::poseCb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "rig_pose_to_tf: subscribing %s, frames %s->%s",
                pose_topic_.c_str(), parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void poseCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = parent_frame_;
    t.child_frame_id = child_frame_;
    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.y;
    t.transform.translation.z = msg->pose.position.z;
    t.transform.rotation = msg->pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }

  std::string pose_topic_;
  std::string parent_frame_;
  std::string child_frame_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RigPoseToTf>());
  rclcpp::shutdown();
  return 0;
}

