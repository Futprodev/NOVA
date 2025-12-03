from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = get_package_share_directory('nova_nav')
    model_path = os.path.join(pkg_path, 'urdf', 'nova_agv.urdf')

    ext = Path(model_path).suffix.lower()
    if ext == '.xacro':
        import xacro
        urdf_xml = xacro.process_file(model_path).toxml()
    else:
        with open(model_path, 'r') as f:
            urdf_xml = f.read()

    robot_description = {'robot_description': urdf_xml}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Publishes /joint_states from the URDF (default angles)
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

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_path,'rviz','rviz_nova_agv.rviz')],
        ),
    ])
