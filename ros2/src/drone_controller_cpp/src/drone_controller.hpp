#pragma once
#include <cstdint>
#include <mavros_msgs/srv/detail/message_interval__struct.hpp>
#include <mavros_msgs/srv/message_interval.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mavros_msgs/srv/detail/set_mode__struct.hpp"

class DroneControllerNode : public rclcpp::Node {
 private:
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedPtr
      stream_rate_client_;

 public:
  DroneControllerNode();

  void change_mode(const std::string& mode) const;
  void change_mode_response_callback(
      const rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture& future)
      const;

  void request_message_interval(uint32_t mavlink_message_id,
                                float message_rate) const;
  void message_interval_response_callback(
      const rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture&
          future) const;
};