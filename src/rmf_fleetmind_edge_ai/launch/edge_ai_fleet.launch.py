#!/usr/bin/env python3
"""
Universal Launch file for FleetMind Edge-AI distributed fleet coordination.
Hardware and demo agnostic: can be configured for any fleet and set of AMRs.
"""
import ast
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_edge_nodes(context, *args, **kwargs):
    fleet_name = LaunchConfiguration('fleet_name').perform(context)
    robot_names_raw = LaunchConfiguration('robot_names').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    try:
        robot_names = ast.literal_eval(robot_names_raw)
        if isinstance(robot_names, str):
            robot_names = [robot_names]
    except Exception:
        robot_names = [r.strip() for r in robot_names_raw.split(',') if r.strip()]

    pkg_share = get_package_share_directory('rmf_fleetmind_edge_ai')
    edge_params_file = LaunchConfiguration('edge_params_file').perform(context)
    if not edge_params_file or not os.path.isfile(edge_params_file):
        edge_params_file = os.path.join(pkg_share, 'config', 'edge_ai_params.yaml')

    model_path = os.path.join(pkg_share, 'models', 'best_model.joblib')

    nodes = []

    # 1. Fleet Coordinator
    nodes.append(
        Node(
            package='rmf_fleetmind_edge_ai',
            executable='fleet_coordinator',
            name=f'{fleet_name}_coordinator',
            parameters=[
                edge_params_file,
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        )
    )

    # 1.5 Benchmark Collector (Enabled optionally via YAML config)
    nodes.append(
        Node(
            package='rmf_fleetmind_edge_ai',
            executable='benchmark_collector',
            name=f'benchmark_{fleet_name}',
            parameters=[
                edge_params_file,
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        )
    )

    # 2. Per-AMR Edge AI Stack
    for rname in robot_names:
        # Edge AI Decision Node
        nodes.append(
            Node(
                package='rmf_fleetmind_edge_ai',
                executable='edge_ai_node',
                name=f'edge_ai_{rname}',
                parameters=[
                    edge_params_file,
                    {
                        'robot_id': rname,
                        'fleet_name': fleet_name,
                        'use_sim_time': use_sim_time,
                        'ml_model_path': model_path
                    }
                ],
                output='screen'
            )
        )

        # RMF Safety & Override Adapter
        nodes.append(
            Node(
                package='rmf_fleetmind_edge_ai',
                executable='rmf_adapter',
                name=f'rmf_adapter_{rname}',
                parameters=[
                    edge_params_file,
                    {
                        'robot_id': rname,
                        'fleet_name': fleet_name,
                        'use_sim_time': use_sim_time
                    }
                ],
                output='screen'
            )
        )

        # Distributed Task Allocator
        nodes.append(
            Node(
                package='rmf_fleetmind_edge_ai',
                executable='distributed_task_allocator',
                name=f'task_allocator_{rname}',
                parameters=[
                    edge_params_file,
                    {
                        'robot_id': rname,
                        'fleet_name': fleet_name,
                        'use_sim_time': use_sim_time
                    }
                ],
                output='screen'
            )
        )

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('rmf_fleetmind_edge_ai')
    default_config = os.path.join(pkg_share, 'config', 'edge_ai_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fleet_name',
            default_value='tinyRobot',
            description='Name of the robot fleet'
        ),
        DeclareLaunchArgument(
            'robot_names',
            default_value="['tinyRobot1', 'tinyRobot2', 'tinyRobot3']",
            description='List of robot IDs in the fleet'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether to synchronize with /clock'
        ),
        DeclareLaunchArgument(
            'edge_params_file',
            default_value=default_config,
            description='Path to edge_ai_params.yaml'
        ),
        OpaqueFunction(function=launch_edge_nodes)
    ])
