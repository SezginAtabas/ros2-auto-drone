import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch.events.process.process_exited import ProcessExited
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    drone_bringup_pkg_share = get_package_share_directory("drone_bringup")
    # Parameters for Mavros.
    mavros_params = os.path.join(drone_bringup_pkg_share, "/config/apm_params.yaml")
    mavros_config = os.path.join(drone_bringup_pkg_share, "/config/apm_config.yaml")
    mavros_plugins = os.path.join(drone_bringup_pkg_share, "/config/apm_pluginlists.yaml")

    # Launch arguments
    log_level = LaunchConfiguration("log_level")

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[mavros_params, mavros_config, mavros_plugins],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return [mavros_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            OpaqueFunction(function=launch_setup),
        ]
    )
