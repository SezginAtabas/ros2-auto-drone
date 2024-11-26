#pragma once
#include <cstdint>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/detail/command_tol__struct.hpp>
#include <mavros_msgs/srv/detail/message_interval__struct.hpp>
#include <mavros_msgs/srv/message_interval.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mavros_msgs/srv/detail/command_bool__struct.hpp"
#include "mavros_msgs/srv/detail/set_mode__struct.hpp"

enum DroneState
{
  DroneOffState,
  DroneGuidedState,
  DroneArmedState,
  DroneTakeoffState,
  DroneSearchState,
  DroneFollowState,
  DroneLandingState,
};

using namespace mavros_msgs::srv;
using namespace geometry_msgs::msg;

class DroneControllerNode : public rclcpp::Node
{
private:
  // Clients
  rclcpp::Client<SetMode>::SharedPtr mode_client_;
  rclcpp::Client<MessageInterval>::SharedPtr stream_rate_client_;
  rclcpp::Client<CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<CommandTOL>::SharedPtr takeoff_client_;

  rclcpp::TimerBase::SharedPtr update_timer_;
  void UpdateTimerCallback() const;

  DroneState drone_state_;
  // position drone will follow, distance relative to the drone
  PointStamped follow_position_;
  PoseStamped drone_local_pose_;

public:
  DroneControllerNode();

  static float GetTakeoffAltitude();
  static float GetFollowDistance();

  DroneState GetDroneState() const;
  void SetDroneState(DroneState state);
  void UpdateDroneState(DroneState target_state);

  PointStamped GetFollowPosition();
  void SetFollowPosition(const PointStamped & follow_position);
  bool CheckForValidTarget();

  void SetDroneLocalPose(const PoseStamped & pose_stamped);
  PoseStamped DroneLocalPose();

  // Publishers
  rclcpp::Publisher<PoseStamped>::SharedPtr local_pose_pub_;
  // Subscribers
  rclcpp::Subscription<PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<PointStamped>::SharedPtr follow_position_sub_;

  // Topic callbacks
  void LocalPoseCallback(const PoseStamped & msg);
  void FollowPositionCallback(const PointStamped & msg);

  // Drone States
  void SetMode(const std::string & mode);
  void SetMessageInterval(uint32_t mavlink_message_id, float message_rate) const;
  void Arm();
  void Takeoff(float altitude);
  void Search();

  // Service Callbacks
  void TakeoffCallback(rclcpp::Client<CommandTOL>::SharedFuture future);
  void ArmCallback(rclcpp::Client<CommandBool>::SharedFuture future);
  void SetModeCallback(
    const rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture & future,
    const std::string & mode);
  void MessageIntervalCallback(const rclcpp::Client<MessageInterval>::SharedFuture & future) const;
};