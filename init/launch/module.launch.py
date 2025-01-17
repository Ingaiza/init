import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    module_client = Node(
        package="yrm100",
        executable="module_client",
        output="screen",
        respawn=True,  
        respawn_delay=1.0  
    )

    command_client = Node(
    package="yrm100",
    executable="command_client",
    output="screen"
    )

    return LaunchDescription([
        module_client,
        command_client
    ])