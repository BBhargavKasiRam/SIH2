#!/usr/bin/env python3
"""
Integrated Launch file running Open-RMF Warehouse Demo + FleetMind Edge-AI Stack.
Primary demonstration entry point for SIH-26123: Edge-AI Based Distributed
Fleet Coordination for Autonomous Mobile Robots in Smart Warehouses.

Uses the campus map as a warehouse environment proxy with 3 warehouseBot AMRs.
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

    # Warehouse base launch (map, schedule, fleet adapter, mock sim)
    warehouse_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(pkg_demos, 'warehouse_mock.launch.xml')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Edge-AI fleet launch for 3 warehouse AMRs
    edge_ai_fleet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_edge_ai, 'launch', 'edge_ai_fleet.launch.py')
        ),
        launch_arguments={
            'fleet_name': 'warehouseBot',
            'robot_names': "['warehouseBot_1', 'warehouseBot_2', 'warehouseBot_3']",
            'use_sim_time': use_sim_time
        }.items()
    )

    # Fleet Dashboard (optional, default enabled)
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
        warehouse_launch,
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
