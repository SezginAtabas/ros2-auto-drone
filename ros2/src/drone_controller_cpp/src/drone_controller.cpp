#include "drone_controller.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
DroneControllerNode::DroneControllerNode() : Node("drone_controller_node") {
  // Service Clients
  mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  stream_rate_client_ =
      create_client<mavros_msgs::srv::MessageInterval>("/mavros/set_message_interval");
  arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

  // Publishers
  local_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/setpoint_position/local", rclcpp::SensorDataQoS());
  // Subscribers
  local_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PoseStamped &msg) { LocalPoseCallback(msg); });

  follow_position_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/my_drone/target_position", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PointStamped &msg) { FollowPositionCallback(msg); });

  update_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100), [this] { UpdateTimerCallback(); });

  // prevent initialization warning.
  drone_state_ = DroneOffState;
  // Start off the state machine.
  UpdateDroneState(DroneGuidedState);
}

// <--------- Getter Methods --------->
float DroneControllerNode::GetFollowDistance() { return 1.5; }

float DroneControllerNode::GetTakeoffAltitude() { return 3.0; }

geometry_msgs::msg::PoseStamped DroneControllerNode::DroneLocalPose() { return drone_local_pose_; }

DroneState DroneControllerNode::GetDroneState() const { return this->drone_state_; }

const std::array<Eigen::Vector3d, 4> *DroneControllerNode::GetSearchWaypoints() const {
  return &search_waypoints_;
}

/**
 * @brief Retrieves the current follow position of the drone.
 *
 * This function returns the stored follow position encapsulated in a PointStamped message,
 * which contains the position coordinates and associated timestamp.
 *
 * @return The current follow position as a PointStamped.
 */
geometry_msgs::msg::PointStamped DroneControllerNode::GetFollowPosition() {
  return follow_position_;
}

// <--------- Setter Methods --------->
void DroneControllerNode::SetDroneState(const DroneState state) { this->drone_state_ = state; }

void DroneControllerNode::SetFollowPosition(
  const geometry_msgs::msg::PointStamped &follow_position) {
  this->follow_position_ = follow_position;
}

void DroneControllerNode::SetDroneLocalPose(const geometry_msgs::msg::PoseStamped &pose_stamped) {
  drone_local_pose_ = pose_stamped;
}

// <--------- Callback Methods --------->
void DroneControllerNode::FollowPositionCallback(const geometry_msgs::msg::PointStamped &msg) {
  SetFollowPosition(msg);
  RCLCPP_INFO(
    this->get_logger(), "Follow point: x:%f, y:%f, z:%f", msg.point.x, msg.point.y, msg.point.z);
}

/**
 * @brief Callback function for receiving local pose information.
 *
 * This method is triggered each time a new PoseStamped message is received
 * on the local pose subscriber topic. It logs the position (x, y, z) of the drone.
 *
 * @param msg The PoseStamped message containing the current local pose of the drone.
 */
void DroneControllerNode::LocalPoseCallback(const geometry_msgs::msg::PoseStamped &msg) {
  SetDroneLocalPose(msg);
  RCLCPP_INFO(
    this->get_logger(), "Local pose received x:%f y:%f z:%f", msg.pose.position.x,
    msg.pose.position.y, msg.pose.position.z);
}

/**
 * @brief Callback function for the takeoff command.
 *
 * This function is called when the takeoff request is completed. It checks
 * the response from the service and logs whether the takeoff command was
 * successful or if the service is still in progress.
 *
 * @param future The future object containing the response from the CommandTOL service.
 */
void DroneControllerNode::TakeoffCallback(
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
  if (const auto &response = future.get(); response->success) {
    RCLCPP_INFO(this->get_logger(), "Takeoff Success.");
    UpdateDroneState(DroneSearchState);
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}

/**
 * @brief Callback function for the arming command.
 *
 * This callback is called when the arming command sent to the MAVROS service
 * receives a response. It checks the success status of the response and logs
 * the appropriate message.
 *
 * @param future The shared future object containing the response from the
 *               arming service.
 */
void DroneControllerNode::ArmCallback(
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
  if (const auto &response = future.get(); response->success) {
    RCLCPP_INFO(this->get_logger(), "DRONE ARMED, Starting Takeoff ...");
    UpdateDroneState(DroneTakeoffState);
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
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
 * @param mode Requested mode of the drone.
 */
void DroneControllerNode::ChangeModeCallback(
  const rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture &future, const std::string &mode) {
  if (const auto &response = future.get(); response->mode_sent) {
    RCLCPP_INFO(this->get_logger(), "Mode changed successfully.");
    if (mode == "GUIDED") {
      RCLCPP_INFO(this->get_logger(), "Arming drone ...");
      UpdateDroneState(DroneArmedState);
    }
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
  const rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture &future) const {
  if (const auto &response = future.get(); response->success) {
    RCLCPP_INFO(this->get_logger(), "SUCCESS");
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}

void DroneControllerNode::UpdateTimerCallback() {
  if (GetDroneState() == DroneSearchState) {
    if (!CheckForValidTarget()) {
      // When a target is found, make sure the drone keeps its current position
      // and any command that may be still active is overwritten.
      const auto current_pose = DroneLocalPose();
      auto target_pose = geometry_msgs::msg::PoseStamped();

      target_pose.pose = current_pose.pose;
      target_pose.header.stamp = now();

      SetDroneLocalPose(target_pose);
      UpdateDroneState(DroneFollowState);
    } else {
      static int i = 0;
      static constexpr double tolerance{0.2};
      const auto waypoints = GetSearchWaypoints();

      if (const auto current_pose = DroneLocalPose();
        current_pose.pose.position.x - waypoints->at(i)[0] <= tolerance &&
        current_pose.pose.position.y - waypoints->at(i)[1] <= tolerance) {
        auto target_pose = geometry_msgs::msg::PoseStamped();

        target_pose.pose.orientation = current_pose.pose.orientation;
        target_pose.header.frame_id = current_pose.header.frame_id;
        target_pose.header.stamp = now();

        target_pose.pose.position.x = waypoints->at(i)[0];
        target_pose.pose.position.y = waypoints->at(i)[1];
        target_pose.pose.position.z = waypoints->at(i)[2];

        SetDroneLocalPose(target_pose);
        ++i;
      }
    }
  }
}

// <-------- Future Request Methods --------->
/**
 * Changes the mode of the drone by sending a request to the relevant service.
 *
 * @param mode The new mode to be set for the drone as a string.
 *
 * @note This function uses the async_send_request method to send a set mode
 *       request to the MAVROS service, and it binds a callback function
 *       to handle the service response.
 */
void DroneControllerNode::ChangeMode(const std::string &mode) {
  const auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = mode;
  auto result = mode_client_->async_send_request(
    request, [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      this->ChangeModeCallback(future, mode);
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
  const uint32_t mavlink_message_id, const float message_rate) const {
  const auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
  request->message_id = mavlink_message_id;
  request->message_rate = message_rate;

  auto result = stream_rate_client_->async_send_request(
    request, [this](rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture future) {
      this->MessageIntervalCallback(future);
    });
}

/**
 * @brief Initiates the takeoff procedure to a specified altitude.
 *
 * This method sends a request to the MAVROS CommandTOL service to command the drone
 * to take off and reach the specified altitude.
 *
 * @param altitude The target altitude to reach upon takeoff, in meters.
 */
void DroneControllerNode::Takeoff(const float altitude) {
  const auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  request->altitude = altitude;
  auto result = takeoff_client_->async_send_request(
    request, [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
      this->TakeoffCallback(future);
    });
}

/**
 * @brief Sends a request to arm the drone.
 *
 * This method creates a request to arm the drone and sends it asynchronously
 * using the arm_client_ service. When the request completes, the ArmCallback
 * method is called to handle the result.
 */
void DroneControllerNode::Arm() {
  const auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = true;
  auto result = arm_client_->async_send_request(
    request, [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      this->ArmCallback(future);
    });
}

// <--------- Other Methods --------->
void DroneControllerNode::GenerateSearchWaypoints(const double distance) {
  const auto initial_pos = DroneLocalPose().pose.position;
  search_waypoints_ = std::array<Eigen::Vector3d, 4>{
    Eigen::Vector3d{initial_pos.x + distance, initial_pos.y + distance, initial_pos.z},
    Eigen::Vector3d{initial_pos.x - distance, initial_pos.y + distance, initial_pos.z},
    Eigen::Vector3d{initial_pos.x - distance, initial_pos.y - distance, initial_pos.z},
    Eigen::Vector3d{initial_pos.x + distance, initial_pos.y - distance, initial_pos.z}
  };
}

/**
 * @brief Checks if the current follow position is within valid constraints.
 *
 * This method retrieves the current follow position and verifies if the x, y, and z coordinates
 * are all below the predefined constraints of 15, 15, and 100 respectively. If so, it returns true,
 * indicating that the target position is valid, otherwise returns false.
 *
 * @return true if the current follow position is within valid constraints, false otherwise.
 */
bool DroneControllerNode::CheckForValidTarget() {
  static constexpr double constraint_x = 10;
  static constexpr double constraint_y = 10;
  static constexpr double constraint_z = 100;

  if (const auto current_point = GetFollowPosition(); current_point.point.x < constraint_x &&
                                                      current_point.point.y < constraint_y &&
                                                      current_point.point.z < constraint_z) {
    return true;
  }
  return false;
}

/**
 * @brief Updates the drone's state based on the provided target state.
 *
 * The drone's behavior and state are updated according to the specified target state,
 * invoking state-specific actions such as arming the drone, taking off, or setting the drone's mode.
 *
 * @param target_state The desired state to transition the drone to, represented by an enum value of DroneState.
 */
void DroneControllerNode::UpdateDroneState(const DroneState target_state) {
  switch (target_state) {
    case DroneOffState:
      RCLCPP_INFO(this->get_logger(), "Drone Off");
      break;
    case DroneGuidedState:
      RCLCPP_INFO(this->get_logger(), "DroneGuided");
      SetDroneState(DroneGuidedState);
      ChangeMode("GUIDED");
      break;
    case DroneArmedState:
      RCLCPP_INFO(this->get_logger(), "DroneArmed");
      SetDroneState(DroneArmedState);
      Arm();
      break;
    case DroneTakeoffState:
      RCLCPP_INFO(this->get_logger(), "DroneTakeoff");
      SetDroneState(DroneTakeoffState);
      Takeoff(GetTakeoffAltitude());
      break;
    case DroneSearchState:
      RCLCPP_INFO(this->get_logger(), "DroneSearch");
      SetDroneState(DroneSearchState);
      break;
    case DroneFollowState:
      RCLCPP_INFO(this->get_logger(), "DroneFollow");
      break;
    case DroneLandingState:
      RCLCPP_INFO(this->get_logger(), "DroneLanding");
      break;
  }
}
