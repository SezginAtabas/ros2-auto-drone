#pragma once
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

/**
 * @class DronePoseNode
 * @brief A ROS2 node for handling the pose of a drone using
 * PoseWithCovarianceStamped messages.
 *
 * The DronePoseNode class subscribes to a topic to receive
 * PoseWithCovarianceStamped messages from the camera, processes them, and
 * republishes the received pose data to a mavros topic.
 */

class DronePoseNode : public rclcpp::Node
{
private:
  rclcpp::CallbackGroup::SharedPtr reentrant_callback_group_;
  rclcpp::SubscriptionOptions sub_options_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_;

  void PoseDataCallback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) const;

public:
  DronePoseNode();
};