#pragma once

#include <cstdint>
#include <eigen3/Eigen/Core>
// rclcpp
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
// ROS2 messages
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
// Mavros services and messages
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/message_interval.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include "mavros_msgs/srv/detail/command_bool__struct.hpp"
#include <mavros_msgs/srv/detail/command_tol__struct.hpp>
#include <mavros_msgs/srv/detail/message_interval__struct.hpp>
#include "mavros_msgs/srv/detail/set_mode__struct.hpp"

enum DroneState {
  DroneOffState,
  DroneGuidedState,
  DroneArmedState,
  DroneTakeoffState,
  DroneSearchState,
  DroneFollowState,
  DroneLandingState,
  DroneAutoLandingState,
};

class DroneControllerNode : public rclcpp::Node {
private:
  // Clients
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedPtr stream_rate_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

  rclcpp::TimerBase::SharedPtr update_timer_;

  // This callback methods is periodically called by timer to control
  // the behaviour of the drone. This is where searching and following logic
  // is executed.
  void UpdateTimerCallback();

  void DroneSearchBehaviour();

  void DroneFollowBehaviour();

  void DroneLandBehaviour();

  DroneState drone_state_;
  // position drone will follow, distance relative to the drone
  geometry_msgs::msg::PointStamped follow_position_;
  geometry_msgs::msg::PoseStamped drone_local_pose_;
  std::array<Eigen::Vector3d, 4> search_waypoints_;

  rclcpp::Time target_detect_time_{};
  mavros_msgs::msg::State drone_mode_;

public:
  DroneControllerNode();

  // <--------- Publishers and Subscribers --------->
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_pub_;
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr follow_position_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr drone_mode_sub_;

  // <--------- Getter Methods --------->
  static float GetFollowDistance();

  static float GetTakeoffAltitude();

  static float GetFollowTimeout();

  static float GetLandAltitude();

  geometry_msgs::msg::PoseStamped DroneLocalPose();

  DroneState GetDroneState() const;

  const std::array<Eigen::Vector3d, 4> *GetSearchWaypoints() const;

  geometry_msgs::msg::PointStamped GetFollowPosition();

  rclcpp::Time GetTargetDetectTime();

  // <--------- Setter Methods --------->
  void SetDroneState(DroneState state);

  void SetFollowPosition(const geometry_msgs::msg::PointStamped &follow_position);

  void SetDroneLocalPose(const geometry_msgs::msg::PoseStamped &pose_stamped);

  void SetTargetDetectTime(const rclcpp::Time &other);

  // <--------- Callback Methods --------->
  void FollowPositionCallback(const geometry_msgs::msg::PointStamped &msg);

  void LocalPoseCallback(const geometry_msgs::msg::PoseStamped &msg);

  void TakeoffCallback(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future);

  void ArmCallback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future);

  void ChangeModeCallback(
    const rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture &future,
    const std::string &mode);

  void MessageIntervalCallback(
    const rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture &future) const;

  void DroneModeCallback(const mavros_msgs::msg::State &msg);

  // <-------- Future Request Methods --------->
  void ChangeMode(const std::string &mode);

  void SetMessageInterval(uint32_t mavlink_message_id, float message_rate) const;

  void Takeoff(float altitude);

  void Arm();

  // <--------- Other Methods --------->
  void GenerateSearchWaypoints(double distance);

  bool CheckForValidTarget();

  void UpdateDroneState(DroneState target_state);

  void MoveToTarget(const float &z_to_target);
};
