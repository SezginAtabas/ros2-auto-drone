#pragma once
#include <mavros_msgs/srv/detail/set_mode__struct.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/stream_rate.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

class DroneControllerNode : public rclcpp::Node {
 private:
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::StreamRate>::SharedPtr stream_rate_client_;

 public:
  DroneControllerNode();
  void change_mode(std::string mode);
  void change_mode_response_callback(
      rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
};