
#include "drone_controller.hpp"

#include <mavros_msgs/srv/message_interval.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "mavros_msgs/srv/detail/message_interval__struct.hpp"
#include "mavros_msgs/srv/detail/set_mode__struct.hpp"

DroneControllerNode::DroneControllerNode()
    : rclcpp::Node("drone_controller_node") {
  mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  stream_rate_client_ = create_client<mavros_msgs::srv::MessageInterval>(
      "/mavros/set_message_interval");
}

void DroneControllerNode::change_mode(std::string mode) {
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = mode;
  auto result = mode_client_->async_send_request(
      request,
      [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        this->change_mode_response_callback(future);
      });
}

void DroneControllerNode::request_message_interval(uint32_t mavlink_message_id,
                                                   float nanosec_send_rate) {
  auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
  std::cout << "request" << std::endl;
  request->message_id = mavlink_message_id;
  request->message_rate = nanosec_send_rate;

  auto result = stream_rate_client_->async_send_request(
      request,
      [this](rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture
                 future) { this->message_interval_response_callback(future); });
}

void DroneControllerNode::change_mode_response_callback(
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
  std::cout << "callback" << std::endl;
  auto response = future.get();
  if (response->mode_sent) {
    RCLCPP_INFO(this->get_logger(), "SUCCESS");
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}

void DroneControllerNode::message_interval_response_callback(
    rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture future) {
  auto response = future.get();
  if (response->success) {
    RCLCPP_INFO(this->get_logger(), "SUCCESS");
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}