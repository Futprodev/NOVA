import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = get_package_share_directory('nova_arm')
    # Point to whichever you actually have:
    model_path = os.path.join(pkg_path, 'urdf', 'nova_arm.urdf')  # or robot.urdf.xacro

    ext = Path(model_path).suffix.lower()
    if ext == '.xacro':
        import xacro
        urdf_xml = xacro.process_file(model_path).toxml()
    else:
        with open(model_path, 'r') as f:
            urdf_xml = f.read()

    params = {'robot_description': urdf_xml, 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
        node_robot_state_publisher
    ])
