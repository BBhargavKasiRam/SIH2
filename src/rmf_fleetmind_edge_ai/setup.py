from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rmf_fleetmind_edge_ai'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FleetMind Team',
    maintainer_email='fleetmind@example.com',
    description='Edge-AI distributed fleet coordination and collision avoidance framework for Open-RMF',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'edge_ai_node = rmf_fleetmind_edge_ai.edge_ai_node:main',
            'rmf_adapter = rmf_fleetmind_edge_ai.rmf_adapter:main',
            'fleet_coordinator = rmf_fleetmind_edge_ai.fleet_coordinator:main',
            'distributed_task_allocator = rmf_fleetmind_edge_ai.distributed_task_allocator:main',
            'mock_robot_sim = rmf_fleetmind_edge_ai.mock_robot_sim:main',
            'benchmark_collector = rmf_fleetmind_edge_ai.benchmark_collector:main',
        ],
    },
)
