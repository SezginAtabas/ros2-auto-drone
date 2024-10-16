from typing import Any

import numpy as np
from numpy import floating
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass

from rclpy.time import Time
from geometry_msgs.msg import PoseStamped


@dataclass
class Pose3D:
    """
    Represents a 3D pose with a timestamp, frame identifier, position, and orientation.

    Attributes:
        timestamp (Time): The timestamp of the pose.
        frame_id (str): The reference frame of the pose.
        position (np.ndarray): The position vector of the pose.
        orientation (R): The orientation of the pose represented as a rotation.

    Methods:
        from_msg(cls, msg: PoseStamped): Creates a Pose3D instance from a ROS PoseStamped message.
        to_msg(self) -> PoseStamped: Converts the Pose3D instance to a ROS PoseStamped message.
        is_near(self, pose: Pose3D, pos_threshold: float = 1e-6, orientation_threshold: float = 0.3) -> bool:
            Compares two poses to check if they are close enough to each other.
    """

    timestamp: Time
    frame_id: str
    position: np.ndarray
    orientation: R

    @classmethod
    def from_msg(cls, msg: PoseStamped):
        return cls(
            timestamp=Time.from_msg(msg.header.stamp),
            frame_id=msg.header.frame_id,
            position=np.array(
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            ),
            orientation=R.from_quat(
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
        ) = self.orientation.as_quat()
        return msg

    def pos_diff(self, pose: "Pose3D") -> floating[Any]:
        """Returns the Euclidean distance between the current position and the provided pose's position.'

        Args:
            pose (Pose3D): The pose to which the difference is calculated.

        Returns:
            floating[Any]: The Euclidean distance between the current position and the provided pose's position.
        """
        return np.linalg.norm(self.position - pose.position)

    def orient_diff(self, pose: "Pose3D") -> floating[Any]:
        """Returns the difference in orientation between the current pose and the provided pose.

        Args:
            pose (Pose3D): The 3D pose to compare against.

        Returns:
            floating[Any]: The difference in orientation between the current pose and the provided pose.
        """
        return self.orientation.inv() * pose.orientation

    def is_near(
        self,
        pose: "Pose3D",
        pos_threshold: float = 1e-6,
        orientation_threshold: float = 0.3,
    ) -> bool:
        """Compares two poses to check if they are close enough to each other.

        Args:
            pose (Pose3D): The pose to compare against.
            pos_threshold (float): The maximum allowed distance between the two positions.
            orientation_threshold (float): The maximum allowed angular difference between the two orientations. Must be in the range [0, pi].

        Raises:
            ValueError: If the orientation_threshold is not in the range [0, pi].

        Returns:
            bool: True if the position difference is less than pos_threshold and the orientation difference is less than orientation_threshold, False otherwise.
        """
        if not (0 <= orientation_threshold <= np.pi):
            raise ValueError(f"expected ")

        position_diff = np.linalg.norm(self.position - pose.position)
        orientation_diff = self.orientation.inv() * pose.orientation
        angle_diff = orientation_diff.magnitude()

        return position_diff < pos_threshold and angle_diff < orientation_threshold


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