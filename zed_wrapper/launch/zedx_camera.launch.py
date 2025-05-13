import os.path as osp
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    zed_wrapper_dir = get_package_share_directory("zed_wrapper")
    config_common_path = osp.join(zed_wrapper_dir, "config", "common.yaml")
    config_camera_path = osp.join(zed_wrapper_dir, "config", "zedx.yaml")

    zed_node = Node(
        package="zed_wrapper",
        executable="zed_wrapper",
        name="zed_node",
        output="screen",
        parameters=[
            config_common_path,
            config_camera_path,
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
        zed_node,
    ])

