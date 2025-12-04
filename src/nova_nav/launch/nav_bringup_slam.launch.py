import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ---------- Launch args ----------
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz         = LaunchConfiguration('rviz')
    rviz_config  = LaunchConfiguration('rviz_config')

    # Our package
    pkg_nova_nav = get_package_share_directory('nova_nav')

    # Hardware bringup (AGV + LiDAR + RViz basic)
    hw_launch_path = os.path.join(
        pkg_nova_nav,
        'launch',
        'nav_bringup.launch.py'   # the one we wrote earlier
    )

    # Nav2 navigation launch from nav2_bringup
    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    # Slam Toolbox online async launch
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    # Our config files (you create these in nova_nav/config/)
    navigation_params_path = os.path.join(
        pkg_nova_nav,
        'config',
        'nav2_params.yaml'
    )

    slam_toolbox_params_path = os.path.join(
        pkg_nova_nav,
        'config',
        'mapper_params_online_async.yaml'
    )

    # RViz config for nav2 (can reuse your existing or create a new one)
    rviz_config_path = PathJoinSubstitution(
        [pkg_nova_nav, 'rviz', rviz_config]
    )

    # ---------- Declare arguments ----------
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',     # real robot => false
        description='Use simulation (Gazebo) clock if true'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value='navigation.rviz',
        description='RViz config file name (in nova_nav/rviz)'
    )

    # ---------- Include AGV hardware bringup ----------
    # This should start:
    #   - robot_state_publisher (URDF + TF)
    #   - agv_bridge_node (cmd_vel <-> serial, odom + TF)
    #   - rplidar_ros
    #   - optionally RViz
    hw_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hw_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # you can override ports here if needed:
            # 'port': '/dev/ttyUSB0',
            # 'lidar_port': '/dev/ttyUSB1',
        }.items()
    )

    # ---------- Slam Toolbox ----------
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    # ---------- Nav2 navigation stack ----------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': navigation_params_path,
        }.items()
    )

    # ---------- RViz (Nav2 view) ----------
    # Optional: a dedicated Nav2 RViz config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_nav2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz)
    ld.add_action(declare_rviz_config)

    # Start hardware + LiDAR
    ld.add_action(hw_bringup)

    # Start SLAM (map->odom)
    ld.add_action(slam_toolbox_launch)

    # Start Nav2 (planners, controllers, BT navigator)
    ld.add_action(navigation_launch)

    # Optional RViz for Nav2
    ld.add_action(rviz_node)

    return ld
