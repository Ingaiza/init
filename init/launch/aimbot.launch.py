import os
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    module_server = Node(
        package="yrm100",
        executable="module_server",
        output="screen",
        respawn=True,
        respawn_delay=1.0,
        arguments=['--ros-args', '--log-level', 'debug'],
        prefix="bash -c 'sleep 2; $0 $@'"  
    )
    
    module_server_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=module_server,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Module server exited'))
            ]
        )
    )

    diff_motion = Node(
        package="diff_wheel_drive",
        executable="diff_motion",
        output="screen"
    )

    return LaunchDescription([
        diff_motion,
        module_server,
        module_server_event_handler
    ])