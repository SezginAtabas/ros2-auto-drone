#pragma once
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>


/**
 * @class DronePoseNode
 * @brief A ROS2 node for handling the pose of a drone using PoseWithCovarianceStamped messages.
 *
 * The DronePoseNode class subscribes to a topic to receive PoseWithCovarianceStamped messages from the camera,
 * processes them, and republishes the received pose data to a mavros topic.
 */

class DronePoseNode : public rclcpp::Node {
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      publisher_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      subscriber_;

  void _pose_data_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) const;

public:
  DronePoseNode();
};