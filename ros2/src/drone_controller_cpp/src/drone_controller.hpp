#pragma once
#include <cstdint>
#include <mavros_msgs/srv/detail/message_interval__struct.hpp>
#include <mavros_msgs/srv/message_interval.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include "mavros_msgs/srv/detail/set_mode__struct.hpp"

class DroneControllerNode : public rclcpp::Node {
 private:
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedPtr
      stream_rate_client_;

 public:
  DroneControllerNode();

  void change_mode(std::string mode);
  void change_mode_response_callback(
      rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);

  void request_message_interval(uint32_t mavlink_message_id,
                                float nanosec_send_rate);
  void message_interval_response_callback(
      rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture future);
};