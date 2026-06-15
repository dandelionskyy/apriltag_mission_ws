from setuptools import setup
import os
from glob import glob

package_name = 'mission_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dandelion',
    maintainer_email='dandelion@example.com',
    description='Mission state machine for Apriltag-triggered tasks',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_node = mission_manager.mission_node:main',
        ],
    },
)
