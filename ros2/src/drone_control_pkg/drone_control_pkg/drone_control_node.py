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
        """
        The `DroneControllerNode` class is responsible for controlling the behavior of a drone using ROS2.
        It handles communication via clients, publishers, and subscribers for tasks such as state monitoring,
        mode changes, takeoff, arming, and interval settings for MavLink messages.
        """
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
        """Creates a `Pose3D` object from the local position message received from the drone then
        appends it to `drone_local_pos_queue`.

        Args:
            msg (PoseStamped): The pose message received from the drone.
            This contains the position and orientation information of the
            drone in a ROS standard PoseStamped message format.
        """
        self.drone_local_pos_queue.append(Pose3D.from_msg(msg))

    def state_callback(self, msg: State) -> None:
        """Appends the received state message to `drone_state_queue`.
        This state data is used to check if the drone is armed, landed, etc.

        Args:
            msg (State): The current state of the drone.
        """
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
        """Handles the callback for the set_message_interval service call. Logs whether the message interval
        request was successful.

        Args:
            future: The future object representing asynchronous result of a service call.
            message_id: The identifier of the message for which the interval is set.

        """
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
        """Makes drone takeoff until it reaches target altitude. Only works if the drone is armed.
        `takeoff` callback is added using `future.add_done_callback` to handle the response.

        Args:
            target_alt (float): The target altitude for the takeoff operation in meters..

        Returns:
            None
        """
        self.state = "TAKEOFF"
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = target_alt
        future = self.takeoff_cli.call_async(takeoff_req)
        future.add_done_callback(self.takeoff_callback)

    def takeoff_callback(self, future):
        """Handles the callback for the takeoff service call. Logs whether the takeoff was initiated successfully,
        failed, or if the service call itself failed due to an exception.

        Args:
            future: The Future object that contains the result of the takeoff service call.

        """
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
        `mode_change_callback` callback is added using `future.add_done_callback` to handle the response.

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
        """Handles the result of an asynchronous service call to change the mode of the drone.
        If the service call is successful, the drone mode is changed successfully. If the service call fails,
         an error message is logged.

        Args:
            future: A future object representing the asynchronous execution of the mode change service call.
        """
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f"Mode changed successfully.")
            else:
                self.get_logger().error(f"Failed to change mode.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def arm(self) -> None:
        """
        Initiates the arming process for the system.

        Changes the system's state to "ARMING" and sends a request to arm the system.
        A callback is added to handle the response of the arming request.
        """
        self.state = "ARMING"
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arm_cli.call_async(arm_req)
        future.add_done_callback(self.arm_callback)

    def arm_callback(self, future):
        """Handles the result of an asynchronous service call to arm the drone. If the service call is successful,
        the drone is armed and the state is updated to "ARMED". If the service call fails, an error message is logged.

        Args:
            future: A Future object representing the result of an asynchronous service call.
        """
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

    def is_guided(self):
        """
        Checks if the drone is currently in guided mode.

        Returns:
            bool: True if the drone is in guided mode, False otherwise.
        """
        if not self.drone_state_queue:
            return
        return self.drone_state_queue[-1].guided

    def main_loop(self):
        """
        Controls the main loop of the drone state machine. Depending on the current state, the function
        coordinates various actions such as setting operational mode, arming, taking off, flying, and landing.

        State transitions:
        - INACTIVE: Sets message intervals, changes mode to GUIDED, and transitions to SETTING_MODE
        - SETTING_MODE: Checks if the mode is GUIDED and transitions to GUIDED
        - GUIDED: Arms the drone
        - ARMED: Initiates the takeoff sequence
        - TAKEOFF: Waits for the drone to reach the target altitude and transitions to FLYING
        - FLYING: Executes the current action in the flight plan
        - LANDING: Monitors the landing process and confirms landing

        Each state represents a critical step in the drone's operation, ensuring smooth transitions
        and appropriate action execution.
        """
        if self.state == "INACTIVE":
            self.set_all_message_interval()
            self.change_mode("GUIDED")
            self.state = "SETTING_MODE"
        elif self.state == "SETTING_MODE":
            if self.is_guided():
                self.state = "GUIDED"
        elif self.state == "GUIDED":
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

    def has_reached_altitude(self, target_altitude: float, tolerance: float = 0.1):
        """Determines if the drone has reached `target_altitude` within set `tolerance`.

        Checks the last recorded position of the drone from the
        drone_local_pos_queue. If the altitude (z-coordinate) is
        within the acceptable range, it determines that the drone has reached the `target altitude`.

        Args:
            target_altitude (float): The desired altitude the drone needs to reach.
            tolerance (float): Acceptable range within the target altitude in meters. (default is 0.1).

        Returns:
            bool: True if the drone's current altitude is within the tolerance of the target altitude, False otherwise.
        """
        if not self.drone_local_pos_queue:
            return False
        current_altitude = self.drone_local_pos_queue[-1].position[-1]
        return abs(current_altitude - target_altitude) < tolerance  # 10 cm tolerance

    def is_landed(self):
        """
        Determines if the drone has landed by checking its altitude.

        Checks the last recorded position of the drone from the
        drone_local_pos_queue. If the altitude (z-coordinate) is
        near zero, it determines that the drone has landed.

        Returns:
            bool: True if the drone's altitude is near zero, False otherwise.
        """
        # Implement logic to check if the drone has landed
        # For simplicity, check if altitude is near zero
        if not self.drone_local_pos_queue:
            return False
        current_altitude = self.drone_local_pos_queue[-1].position[-1]
        return current_altitude < 0.1

    def execute_current_action(self):
        """
        Executes the current action in the flight plan.

        If the current action index exceeds the length of the flight plan actions,
        logs that the flight plan is completed. Otherwise, retrieves the current
        action and its associated parameters and performs the action based on its type.
        """
        if self.current_action_index >= len(self.flight_plan_actions):
            self.get_logger().info("Flight plan completed.")
            return

        current_action = self.flight_plan_actions[self.current_action_index]
        action_params = self.flight_plan[current_action]

        if current_action == "hover":
            self.hover(action_params["duration"])
        elif current_action == "land":
            self.land()

    def hover(self, duration: float):
        """Makes the drone keep its pose `hover` for a set duration.
        When the set duration elapsed the current action index is increased by one.

        Args:
            duration (float): The amount of time in seconds that the hover action should be maintained.
        """
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
        """
        Initiates the landing process for the drone.

        Logs the initiation of the landing process, changes the mode
        of the drone to "LAND", updates the state to "LANDING", and
        increments the current action index.

        Returns:
            None
        """
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


if __name__ == "__main__":
    main()