import numpy as np
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass

from rclpy.time import Time
from geometry_msgs.msg import PoseStamped


@dataclass
class Pose3D:
    """
    Pose class represents a 3D pose with timestamp, frame ID, position, and rotation.

    Attributes:
        timestamp (Time): The timestamp of the pose.
        frame_id (str): The frame ID to which the pose is associated.
        position (np.ndarray): The 3D position of the pose.
        rotation (R): The rotation of the pose represented as a quaternion.

    Methods:
        from_msg(cls, msg: PoseStamped)
            Creates a Pose instance from a PoseStamped message.

        to_msg(self) -> PoseStamped
            Converts the Pose instance to a PoseStamped message.
    """

    timestamp: Time
    frame_id: str
    position: np.ndarray
    rotation: R

    @classmethod
    def from_msg(cls, msg: PoseStamped):
        return cls(
            timestamp=Time.from_msg(msg.header.stamp),
            frame_id=msg.header.frame_id,
            position=np.array(
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            ),
            rotation=R.from_quat(
                [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ]
            ),
        )

    def to_msg(self) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.timestamp.to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.position
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = self.rotation.as_quat()
        return msg


def generate_rand_pose_msg() -> PoseStamped:
    """
    Generates a random PoseStamped message with randomized position and orientation values.

    Returns:
        PoseStamped: A message containing randomized pose information, including timestamp,
        frame ID, position coordinates (x, y, z), and orientation quaternion (x, y, z, w).
    """
    msg = PoseStamped()
    msg.header.stamp = Time(seconds=np.random.randint(low=0, high=10000)).to_msg()
    msg.header.frame_id = "map"
    (
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
    ) = np.random.random_sample(size=3)
    (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    ) = R.random(random_state=np.random.randint(low=0, high=10000)).as_quat()

    return msg