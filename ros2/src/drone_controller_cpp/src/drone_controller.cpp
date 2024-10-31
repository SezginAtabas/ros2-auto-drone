
#include "drone_controller.hpp"

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/message_interval.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include "mavros_msgs/srv/detail/command_bool__struct.hpp"
#include "mavros_msgs/srv/detail/message_interval__struct.hpp"
#include "mavros_msgs/srv/detail/set_mode__struct.hpp"

/**
 * @brief Constructor for the DroneControllerNode class.
 *
 * This constructor initializes a ROS 2 node named "drone_controller_node"
 * and creates clients for the services SetMode and MessageInterval provided by
 * MAVROS.
 */
DroneControllerNode::DroneControllerNode() : Node("drone_controller_node")
{
  mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  stream_rate_client_ =
    create_client<mavros_msgs::srv::MessageInterval>("/mavros/set_message_interval");
  arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

  service_timer_ =
    this->create_wall_timer(std::chrono::milliseconds(1000), [this] { Arm(); });
}

void DroneControllerNode::Arm() const
{
  const auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = true;
  auto result = arm_client_->async_send_request(
    request, [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      this->ArmCallback(future);
    });
}

void DroneControllerNode::ArmCallback(
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) const
{
  if (const auto & response = future.get(); response->success) {
    RCLCPP_INFO(this->get_logger(), "SUCCESS");
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}

void DroneControllerNode::ServiceTimerCallback() const
{
  RCLCPP_INFO(this->get_logger(), "DroneController ServiceTimerCallback.");
  std::uint32_t message_id = 32;
  float message_rate = 100000;
  SetMessageInterval(message_id, message_rate);
}

/**
 * Changes the mode of the drone by sending a request to the relevant service.
 *
 * @param mode The new mode to be set for the drone as a string.
 *
 * @note This function uses the async_send_request method to send a set mode
 *       request to the MAVROS service, and it binds a callback function
 *       to handle the service response.
 */
void DroneControllerNode::SetMode(const std::string & mode) const
{
  const auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = mode;
  auto result = mode_client_->async_send_request(
    request, [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      this->SetModeCallback(future);
    });
}

/**
 * @brief Sends a request to set the message interval for a specified MAVLink
 * message ID.
 *
 * This function creates a request to adjust the interval at which a specific
 * MAVLink message is sent. The desired MAVLink message ID and the interval rate
 * (in nanoseconds) are provided as input parameters. The request is then sent
 * asynchronously.
 *
 * @param mavlink_message_id The MAVLink message identifier for which the
 * interval is to be set.
 * @param message_rate The desired send rate interval in nanoseconds.
 */
void DroneControllerNode::SetMessageInterval(
  const uint32_t mavlink_message_id, const float message_rate) const
{
  const auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
  request->message_id = mavlink_message_id;
  request->message_rate = message_rate;

  auto result = stream_rate_client_->async_send_request(
    request, [this](rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture future) {
      this->MessageIntervalCallback(future);
    });
}

/**
 * @brief Callback function that handles the response from the asynchronous
 * service request to change the drone's mode.
 *
 * This function is called when the future associated with the mode change
 * request is ready. It checks if the mode change was successful and logs
 * the result accordingly.
 *
 * @param future A shared future object representing the asynchronous
 *               result of the SetMode service request.
 */
void DroneControllerNode::SetModeCallback(
  const rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture & future) const
{
  if (const auto & response = future.get(); response->mode_sent) {
    RCLCPP_INFO(this->get_logger(), "SUCCESS");
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}

/**
 * @brief Handles the response from the message interval service request.
 *
 * This callback function is triggered when the asynchronous request to the
 * message interval service is completed. It checks the success status of the
 * response and logs the appropriate message.
 *
 * @param future The future object containing the result from the message
 *               interval service request.
 */
void DroneControllerNode::MessageIntervalCallback(
  const rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture & future) const
{
  if (const auto & response = future.get(); response->success) {
    RCLCPP_INFO(this->get_logger(), "SUCCESS");
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}