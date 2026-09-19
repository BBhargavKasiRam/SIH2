#!/usr/bin/env python3
"""
Integrated Launch file running Open-RMF Hotel Demo + FleetMind Edge-AI Stack.
Demonstrates Edge AI multi-fleet coordination on the Hotel multi-floor environment.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    pkg_demos = get_package_share_directory('rmf_demos')
    pkg_edge_ai = get_package_share_directory('rmf_fleetmind_edge_ai')

    use_sim_time = LaunchConfiguration('use_sim_time')

    hotel_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(pkg_demos, 'hotel_mock.launch.xml')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    edge_ai_fleet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_edge_ai, 'launch', 'edge_ai_fleet.launch.py')
        ),
        launch_arguments={
            'fleet_name': 'cleanerBotA',
            'robot_names': "['cleanerBotA_1', 'cleanerBotA_2']",
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock /clock'
        ),
        hotel_launch,
        TimerAction(
            period=5.0,
            actions=[edge_ai_fleet_launch]
        )
    ])
