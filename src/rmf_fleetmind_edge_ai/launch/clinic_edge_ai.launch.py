#!/usr/bin/env python3
"""
Integrated Launch file running Open-RMF Clinic Demo + FleetMind Edge-AI Stack.
Demonstrates multi-fleet Edge-AI coordination with ≥3 AMRs across two fleets
(tinyRobot + deliveryRobot) as required by SIH-26123.
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

    clinic_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(pkg_demos, 'clinic_mock.launch.xml')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Edge-AI for tinyRobot fleet (2 robots)
    edge_ai_tiny_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_edge_ai, 'launch', 'edge_ai_fleet.launch.py')
        ),
        launch_arguments={
            'fleet_name': 'tinyRobot',
            'robot_names': "['tinyRobot1', 'tinyRobot2']",
            'use_sim_time': use_sim_time
        }.items()
    )

    # Edge-AI for deliveryRobot fleet (1+ robots) — ensures ≥3 AMRs total
    edge_ai_delivery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_edge_ai, 'launch', 'edge_ai_fleet.launch.py')
        ),
        launch_arguments={
            'fleet_name': 'deliveryRobot',
            'robot_names': "['deliveryBot_1']",
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
        clinic_launch,
        TimerAction(
            period=5.0,
            actions=[edge_ai_tiny_launch, edge_ai_delivery_launch]
        ),
        TimerAction(
            period=8.0,
            actions=[dashboard_launch]
        )
    ])
