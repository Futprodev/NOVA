from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('nova'),
        'urdf',
        'nova.urdf'
    )

    with open(urdf_file, 'r') as inf:
        robot_description = inf.read()

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['/arm_joint_states'],  # uses IK output
            'delta': 0.0,
        }]
    )


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
    ])
