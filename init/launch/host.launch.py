import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    urdf_path = os.path.join(get_package_share_path('bringup'), 'urdf', 'aimbot.xacro')
    rviz_config_path = os.path.join(get_package_share_path('bringup'), 'rviz', 'urdf_config.rviz')

    robot_description = ParameterValue(Command(['xacro ', str(urdf_path)]), value_type=str)

   # add diff calc since it publishes odom which is filtered , but first modify diff calc not to publish duty
   
    # Wheel Odometry Node
    wheel_odom = Node(
        package="bringup",
        executable="odometry",
        name="odometry_node",
        output="screen",
        parameters=[
            {'publish_tf': True}
        ]         
    )

    # EKF Node with more verbose diagnostics
    ekf_fuse_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_se',
        output='screen',
        parameters=[{
            # Basic configuration
            'frequency': 30.0,
            'sensor_timeout': 0.5, # changed from 0.1
            'two_d_mode': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_footprint',
            'world_frame': 'odom',

            # IMU sensor configuration
            'imu0': '/imu/data',
            'imu0_config': [
                False, False, False,  # Disable position
                True, True, True,     # Enable orientation
                False, False, False,  # Disable velocity
                True, True, True,     # Enable angular velocity
                False, False, False   # Disable acceleration bias
            ],
            'imu0_differential': False,
            'imu0_relative': False,
            'imu0_remove_gravitational_acceleration': True,

            # Wheel odometry configuration
            'odom0': '/odom/wheel',
            'odom0_config': [
                True,  True,  False,  # Enable X,Y position, disable Z
                False, False, True,   # Disable roll/pitch, enable yaw
                True,  True,  False,  # Enable X,Y velocity, disable Z
                False, False, True,   # Disable roll/pitch rates, enable yaw rate
                False, False, False   # Disable all accelerations
            ],
            'odom0_differential': False,
            'odom0_relative': False,

            # Process noise parameters - how much we trust our motion model
            # 'process_noise_covariance_diagonal': [
            #     0.005, 0.005, 0.005,    # position noise (x, y, z)
            #     0.0001, 0.0001, 0.0001, # orientation noise (roll, pitch, yaw)
            #     0.01, 0.01, 0.01,       # velocity noise (x, y, z)
            #     0.001, 0.001, 0.001,    # angular velocity noise (roll, pitch, yaw)
            #     0.01, 0.01, 0.01        # acceleration noise (x, y, z)
            # ],
            'process_noise_covariance_diagonal': [
                0.05, 0.05, 0.05,     # Slightly increase position uncertainty
                0.03, 0.03, 0.03,     # Increase orientation uncertainty
                0.05, 0.05, 0.05,     # Increase velocity uncertainty
                0.03, 0.03, 0.03,     # Increase angular velocity uncertainty
                0.01, 0.01, 0.01      # Keep acceleration uncertainty low
            ],

            # Initial uncertainty parameters - our starting confidence
            'initial_estimate_covariance_diagonal': [
                1.0, 1.0, 1.0,    # initial position uncertainty
                1.0, 1.0, 1.0,    # initial orientation uncertainty
                1.0, 1.0, 1.0,    # initial velocity uncertainty
                1.0, 1.0, 1.0,    # initial angular velocity uncertainty
                1.0, 1.0, 1.0     # initial acceleration uncertainty
            ],

            # Debugging options
            'print_diagnostics': True,
            'publish_tf': True,
            'publish_acceleration': False
        }]
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     'publish_tf': True
                     }]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        wheel_odom,
        ekf_fuse_node,
        robot_state_publisher_node,
        rviz2_node
    ])