# nova_nav/launch/agv_bringup.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')

    pkg_path = get_package_share_directory('nova_nav')
    urdf_path = os.path.join(pkg_path, 'urdf', 'nova_agv.urdf')

    # Load URDF as string
    with open(urdf_path, 'r') as f:
        urdf_xml = f.read()

    robot_description = {'robot_description': urdf_xml}

    rviz_config = os.path.join(pkg_path, 'rviz', 'rviz_nova_agv.rviz')

    return LaunchDescription([
        # --- Launch arguments ---
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the Mega'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baudrate for the Mega'
        ),

        # --- URDF / TF publisher ---
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[robot_description],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),
        
        # --- AGV bridge: /cmd_vel <-> serial, /odom + TF ---
        Node(
            package='nova_nav',
            executable='agv_bridge_node',   # match whatever you installed in setup.py
            name='agv_bridge_node',
            output='screen',
            parameters=[{
                'port': port,
                'baudrate': baudrate,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
            }],
        ),

        # --- RViz ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        ),
    ])
