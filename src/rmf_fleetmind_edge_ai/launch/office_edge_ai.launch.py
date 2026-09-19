#!/usr/bin/env python3
"""
Integrated Launch file running Open-RMF Office Demo + FleetMind Edge-AI Stack.
Launches 3 tinyRobot AMRs with Edge-AI coordination and optional dashboard.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    pkg_demos = get_package_share_directory('rmf_demos')
    pkg_edge_ai = get_package_share_directory('rmf_fleetmind_edge_ai')

    use_sim_time = LaunchConfiguration('use_sim_time')
    dashboard = LaunchConfiguration('dashboard')

    # Demo XML launch (supports both mock and full gazebo office demo)
    office_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(pkg_demos, 'office_mock.launch.xml')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Edge-AI fleet launch for tinyRobot1, tinyRobot2, tinyRobot3
    edge_ai_fleet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_edge_ai, 'launch', 'edge_ai_fleet.launch.py')
        ),
        launch_arguments={
            'fleet_name': 'tinyRobot',
            'robot_names': "['tinyRobot1', 'tinyRobot2', 'tinyRobot3']",
            'use_sim_time': use_sim_time
        }.items()
    )

    # Fleet Dashboard (optional)
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('fleetmind_dashboard'),
                'launch', 'dashboard.launch.py'
            )
        ),
        condition=IfCondition(dashboard)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock /clock'
        ),
        DeclareLaunchArgument(
            'dashboard',
            default_value='true',
            description='Launch the FleetMind web dashboard'
        ),
        office_launch,
        # Boot Edge-AI nodes after RMF schedule and map server initialize
        TimerAction(
            period=5.0,
            actions=[edge_ai_fleet_launch]
        ),
        # Boot Dashboard after Edge-AI nodes are up
        TimerAction(
            period=8.0,
            actions=[dashboard_launch]
        )
    ])
