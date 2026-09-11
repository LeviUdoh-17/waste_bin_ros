"""Laptop-side navigation against a pre-recorded map.

Pairs with bin_real.launch.py running on the robot. Use laptop_slam_nav2.launch.py
instead when you are still building the map — that one runs slam_toolbox in mapping
mode; this one localises against a saved map with AMCL and brings up Nav2.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bin_bringup_dir = get_package_share_directory('bin_bringup')
    bin_description_dir = get_package_share_directory('bin_description')
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    nav2_params_file = os.path.join(bin_bringup_dir, 'config', 'nav2_params.yaml')
    waypoints_file = os.path.join(bin_bringup_dir, 'config', 'waypoints.yaml')
    rviz_config_file = os.path.join(bin_description_dir, 'rviz', 'slam.rviz')
    default_map = os.path.join(bin_bringup_dir, 'maps', 'bin_floor.yaml')

    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map yaml to localise against')
    declare_run_waypoints = DeclareLaunchArgument(
        'run_waypoints', default_value='false',
        description='Start waypoint_runner automatically instead of driving from RViz')

    # map_server + amcl. Their params come from nav2_params.yaml; the map path is
    # passed here because localization_launch.py overrides yaml_filename from it.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'localization_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
        }.items(),
    )

    # Planner, controller, behaviours, waypoint_follower, velocity_smoother,
    # collision_monitor. This launch file is also what remaps cmd_vel -> cmd_vel_nav,
    # giving controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed ->
    # collision_monitor -> cmd_vel -> twist_stamper on the robot.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    waypoint_runner = Node(
        package='bin_behavior',
        executable='waypoint_runner',
        name='waypoint_runner',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_waypoints')),
        parameters=[{
            'use_sim_time': False,
            'waypoints_file': waypoints_file,
        }],
    )

    return LaunchDescription([
        declare_map,
        declare_run_waypoints,
        localization,
        navigation,
        rviz_node,
        waypoint_runner,
    ])
