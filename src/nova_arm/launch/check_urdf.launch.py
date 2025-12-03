from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = get_package_share_directory('nova_arm')
    model_path = os.path.join(pkg_path, 'urdf', 'nova_arm.urdf')

    ext = Path(model_path).suffix.lower()
    if ext == '.xacro':
        import xacro
        urdf_xml = xacro.process_file(model_path).toxml()
    else:
        with open(model_path, 'r') as f:
            urdf_xml = f.read()

    params = {'robot_description': urdf_xml, 'use_sim_time': use_sim_time}

    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nova_arm_kinematics',
            executable='ik_rviz_node',
            name='nova_arm_planar_ik_rviz',
            parameters=[{
                'd1': 0.151,
                'L1': 0.200,
                'L2': 0.175,
                'L3': 0.0,
                'elbow_sign': -1,
                'joint_names': ['joint_1','joint_2','joint_3','joint_4','joint_5'],
                'zero_offsets': [0.0, 1.57079632679, 3.14159265359],  # 0, +90°, +180°
                'signs':        [1.0, -1.0, 1.0],                    # yaw ok, J2/J3 flipped
            }],
            output='screen'),

        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(pkg_path,'rviz','rviz_nova_arm.rviz')])
    ])
