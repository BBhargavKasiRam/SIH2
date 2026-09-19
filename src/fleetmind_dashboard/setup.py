from setuptools import setup
import os
from glob import glob

package_name = 'fleetmind_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FleetMind Team',
    maintainer_email='fleetmind@example.com',
    description='Lightweight web-based fleet monitoring dashboard for FleetMind Edge-AI AMR coordination.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = fleetmind_dashboard.dashboard_node:main',
        ],
    },
)
