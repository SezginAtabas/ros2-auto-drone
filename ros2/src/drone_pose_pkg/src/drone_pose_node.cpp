#include "drone_pose_node.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

DronePoseNode::DronePoseNode() : rclcpp::Node("drone_pose_node")
{
  reentrant_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  sub_options_ = rclcpp::SubscriptionOptions();
  sub_options_.callback_group = reentrant_callback_group_;

  subscriber_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/zed/zed_node/pose_with_covariance", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped & msg) {
      this->PoseDataCallback(msg);
    },
    sub_options_);

  publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/mavros/vision_pose/pose_cov", rclcpp::SensorDataQoS());
}

void DronePoseNode::PoseDataCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg) const
{
  publisher_->publish(msg);
}
