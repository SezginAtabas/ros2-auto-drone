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

import math
from ros2_poselib.poselib import Pose


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

    def local_position_callback(self, msg: PoseStamped) -> None:
        self.drone_local_pos_queue.append(Pose.from_msg(msg))

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
            cmd.message_rate = msg_interval
            future = self.message_interval_cli.call_async(cmd)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                self.get_logger().info(
                    f"Set message interval result for msg with id:{msg_id} - {future.result()}"
                )
            else:
                self.get_logger().error(
                    f"Failed to call set_message_interval service for msg with id:{msg_id}"
                )

    def takeoff(self, target_alt: float) -> None:
        """Makes drone takeoff until it reaches target altitude.
        Drone needs the armed for this to work.

        Args:
            target_alt (float): The target altitude for the takeoff operation.

        Returns:
            None
        """
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = target_alt
        future = self.takeoff_cli.call_async(takeoff_req)
        return rclpy.spin_until_future_complete(self, future)

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
        return rclpy.spin_until_future_complete(self, future)

    def arm(self) -> None:
        """
        Sends an asynchronous request to arm the drone using the CommandBool service.

        Returns:
            None
        """
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arm_cli.call_async(arm_req)
        return rclpy.spin_until_future_complete(self, future)


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