import yaml
from argparse import Namespace
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import os 

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package='stereo_cam_usb', executable='stereo_cam_usb_node_exe', output="log",
        name="stereo_cam_usb",
        respawn=True
        ))
    ld.add_action(Node(
        package='stereo_dso', executable='stereo_dso_node', output="log",
        name="stereo_dso",
        respawn=True
        ))
    return ld
