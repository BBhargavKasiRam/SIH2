#!/usr/bin/env python3
"""
Launch file for FleetMind Dashboard.
Starts the dashboard web server node that provides real-time fleet monitoring.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'http_port',
            default_value='7860',
            description='HTTP port for the dashboard web server'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        Node(
            package='fleetmind_dashboard',
            executable='dashboard_node',
            name='fleetmind_dashboard',
            parameters=[{
                'http_port': LaunchConfiguration('http_port'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'update_rate_hz': 2.0,
            }],
            output='screen'
        ),
    ])
