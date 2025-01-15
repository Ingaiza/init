import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    imu_node = Node(
        package="wit_imu_driver",
        executable='wit_imu_node',
        output='screen',
        parameters=[{
            'orientation_covariance': [0.01, 0, 0,
                                        0, 0.01, 0,
                                        0, 0, 0.01],
            'angular_velocity_covariance': [0.01, 0, 0,
                                            0, 0.01, 0,
                                            0, 0, 0.01],
            'linear_acceleration_covariance': [0.1, 0, 0,
                                                0, 0.1, 0,
                                                0, 0, 0.1]
        }]
    )

    diff_motion = Node(
        package="diff_wheel_drive",
        executable="diff_motion",
        output="screen"
    )

    module_server = Node(
        package="yrm100",
        executable="module_server",
        output="screen"
    )

   

    return LaunchDescription([
        imu_node,
        diff_motion,
        module_server
    ])