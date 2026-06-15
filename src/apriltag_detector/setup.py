from setuptools import setup
import os
from glob import glob

package_name = 'apriltag_detector'

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
    description='AprilTag detection node using pupil-apriltags',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detector_node = apriltag_detector.detector_node:main',
        ],
    },
)
