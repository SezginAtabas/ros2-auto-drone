
#include "drone_controller.hpp"

#include <chrono>
#include <functional>
#include <future>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/stream_rate.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

DroneControllerNode::DroneControllerNode()
    : rclcpp::Node("drone_controller_node") {
  mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  stream_rate_client_ = create_client<mavros_msgs::srv::StreamRate>(
      "/mavros/set_message_interval");
}

void DroneControllerNode::change_mode(std::string mode) {
  auto request = std::make_shared<mavros_msgs::srv::SetMode_Request>();
  request->custom_mode = mode;
  auto result = mode_client_->async_send_request(
      request, std::bind(&DroneControllerNode::change_mode_response_callback,
                         this, std::placeholders::_1));
}

void DroneControllerNode::change_mode_response_callback(
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
  auto status = future.wait_for(std::chrono::milliseconds(1000));
  if (status == std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "SUCCESS");
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}