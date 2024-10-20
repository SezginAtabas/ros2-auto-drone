#include "drone_pose_node.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

DronePoseNode::DronePoseNode() : rclcpp::Node("drone_pose_node") {

  subscriber_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/zed/zed_node/pose_with_covariance", 10,
          [this](const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
          {this->_pose_data_callback(msg);});

  publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/mavros/vision_pose/pose_cov", 10);
}

void DronePoseNode::_pose_data_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) const {
    publisher_->publish(msg);
}
