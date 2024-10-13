import rclpy
from rclpy.node import Node

# import message definitions for receiving status and position
from mavros_msgs.msg import State, AttitudeTarget
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion
from std_msgs.msg import String

# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import (
    SetMode,
    CommandBool,
    CommandTOL,
    CommandLong,
    MessageInterval,
)

from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation as R
from ros2_poselib.poselib import Pose3D


# gz sim -v4 -r iris_runway.sdf
# sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
# ros2 launch mavros px4.launch fcu_url:=tcp://127.0.0.1:5762@
# ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=<camera model>

# in ardupilot sitl use: param set SIM_GPS_TYPE 0 and param set SIM_GPS2_TYPE 0 to disable gps


class DroneControllerNode(Node):
    def __init__(self):
        super().__init__("drone_controller_node")

        # <------ Variables ------>
        # keep track of the drone status.
        self.drone_state_queue = deque(maxlen=1)
        self.drone_local_pos_queue = deque(maxlen=100)

        # <------ Clients ------>
        # for long command (datastream requests)...
        self.cmd_cli = self.create_client(CommandLong, "/mavros/cmd/command")
        # for mode changes ...
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        # for arming ...
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        # for takeoff
        self.takeoff_cli = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        # to set interval between received MavLink messages
        self.message_interval_cli = self.create_client(
            MessageInterval, "/mavros/set_message_interval"
        )

        # <------ Publishers and Subscribers ------>

        # Quality of service used for state topics, like ~/state, ~/mission/waypoints etc.
        state_qos = rclpy.qos.QoSProfile(
            depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        # Quality of service used for the target publisher/
        target_qos = rclpy.qos.QoSProfile(
            depth=20,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        # Quality of service used for most sensor streams
        sensor_qos = rclpy.qos.qos_profile_sensor_data
        # PARAMETERS_QOS used for parameter streams
        parameter_qos = rclpy.qos.qos_profile_parameters

        # Subscriber for the state of the drone.
        self.state_sub = self.create_subscription(
            State, "/mavros/state", self.state_callback, state_qos
        )
        # publisher for setpoint commands, this used to control the drone
        # in its local coordinate system. Received and sent commands are in
        # meters.
        self.target_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        # subscriber for drone`s local position.
        self.pos_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.local_position_callback,
            sensor_qos,
        )

        # MavLink messages to request from the drone flight controller.
        # These are drone position, attitude etc. And are requested using
        # set_all_message_interval function which makes async calls to
        # /mavros/set_message_interval service.
        # Message id, Interval in microseconds
        self.messages_to_request = (
            (32, 100000),  # local position
            (33, 100000),  # global position
        )

        # Flight plan of the drone.
        # Determines what actions drone will take.
        self.flight_plan = {
            "takeoff": {"altitude": 3.0},
            "hover": {"duration": 5.0},
            "land": {},
        }
        # Flight plan tracking variables
        self.flight_plan_actions = [
            action for action in self.flight_plan.keys() if action != "takeoff"
        ]
        self.current_action_index = 0
        self.action_start_time = None
        self.state = "INACTIVE"

        self.timer = self.create_timer(1.0, self.main_loop)

    def local_position_callback(self, msg: PoseStamped) -> None:
        self.drone_local_pos_queue.append(Pose3D.from_msg(msg))

    def state_callback(self, msg: State) -> None:
        self.drone_state_queue.append(msg)

    def set_all_message_interval(self) -> None:
        """
        Requests data from the drone flight controller in the form of MavLink messages.
        Requested messages will be sent at periodic intervals, which are then processed by
        mavros and published to topics. This function makes async calls to "/mavros/set_message_interval service"
        to request these messages. Which then sends a MAV_CMD_SET_MESSAGE_INTERVAL commands to the drone.

        MavLink message ids can be found here: https://mavlink.io/en/messages/common.html

        Returns:
            None
        """
        for msg_id, msg_interval in self.messages_to_request:
            cmd = MessageInterval.Request()
            cmd.message_id = msg_id
            cmd.message_rate = float(msg_interval)
            future = self.message_interval_cli.call_async(cmd)
            future.add_done_callback(
                lambda f, message_id=msg_id: self.message_interval_callback(f, msg_id)
            )

    def message_interval_callback(self, future, message_id):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Set message interval for msg with id:{message_id} successfully."
                )
            else:
                self.get_logger().error(
                    f"Failed to set message interval for msg with id:{message_id}."
                )
        except Exception as e:
            self.get_logger().error(
                f"Service call failed for msg with id:{message_id}: {e}"
            )

    def takeoff(self, target_alt: float) -> None:
        """Makes drone takeoff until it reaches target altitude.
        Drone needs the armed for this to work.

        Args:
            target_alt (float): The target altitude for the takeoff operation.

        Returns:
            None
        """
        self.state = "TAKEOFF"
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = target_alt
        future = self.takeoff_cli.call_async(takeoff_req)
        future.add_done_callback(self.takeoff_callback)

    def takeoff_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Takeoff initiated.")
            else:
                self.get_logger().error("Takeoff failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def change_mode(self, new_mode: str) -> None:
        """Sends an asynchronous request to changes the mode of the drone using the SetMode service.

        Args:
            new_mode (str): The new mode to set for the system.
            Set to `LAND` to land the drone to its current position.

        Returns:
            None
        """

        mode_req = SetMode.Request()
        mode_req.custom_mode = new_mode
        future = self.mode_cli.call_async(mode_req)
        future.add_done_callback(self.mode_change_callback)

    def mode_change_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f"Mode changed successfully.")
            else:
                self.get_logger().error(f"Failed to change mode.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def arm(self) -> None:
        self.state = "ARMING"
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arm_cli.call_async(arm_req)
        future.add_done_callback(self.arm_callback)

    def arm_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Drone armed.")
                self.state = "ARMED"
            else:
                self.get_logger().error("Arming failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def to_local_pose(self, target_pose: Pose3D) -> None:
        self.target_pub.publish(target_pose.to_msg())

    def main_loop(self):
        if self.state == "INACTIVE":
            self.set_all_message_interval()
            self.arm()
        elif self.state == "ARMED":
            self.takeoff(self.flight_plan["takeoff"]["altitude"])
        elif self.state == "TAKEOFF":
            # Wait until drone reaches target altitude
            if self.has_reached_altitude(self.flight_plan["takeoff"]["altitude"]):
                self.state = "FLYING"
                self.action_start_time = self.get_clock().now()
        elif self.state == "FLYING":
            self.execute_current_action()
        elif self.state == "LANDING":
            # Monitor landing process
            if self.is_landed():
                self.state = "LANDED"
                self.get_logger().info("Drone has landed.")

    def has_reached_altitude(self, target_altitude):
        if not self.drone_local_pos_queue:
            return False
        current_altitude = self.drone_local_pos_queue[-1].position[-1]
        return abs(current_altitude - target_altitude) < 0.1  # 10 cm tolerance

    def is_landed(self):
        # Implement logic to check if the drone has landed
        # For simplicity, check if altitude is near zero
        if not self.drone_local_pos_queue:
            return False
        current_altitude = self.drone_local_pos_queue[-1].position[-1]
        return current_altitude < 0.1

    def execute_current_action(self):
        if self.current_action_index >= len(self.flight_plan_actions):
            self.get_logger().info("Flight plan completed.")
            return

        current_action = self.flight_plan_actions[self.current_action_index]
        action_params = self.flight_plan[current_action]

        if current_action == "hover":
            self.hover(action_params["duration"])
        elif current_action == "land":
            self.land()

    def hover(self, duration):
        if self.action_start_time is None:
            self.action_start_time = self.get_clock().now()
            self.get_logger().info("Starting hover.")

        elapsed_time = (
            self.get_clock().now() - self.action_start_time
        ).nanoseconds / 1e9
        if elapsed_time >= duration:
            self.get_logger().info("Hover duration completed.")
            self.current_action_index += 1
            self.action_start_time = None
        else:
            # Keep publishing the current position to maintain hover
            if self.drone_local_pos_queue:
                current_pose = self.drone_local_pos_queue[-1]
                self.to_local_pose(current_pose)

    def land(self):
        self.get_logger().info("Initiating landing.")
        self.change_mode("LAND")
        self.state = "LANDING"
        self.current_action_index += 1


def main(args=None):
    rclpy.init(args=args)
    drone_controller_node = DroneControllerNode()

    try:
        rclpy.spin(drone_controller_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        drone_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()