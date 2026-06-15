from setuptools import setup
import os
from glob import glob

package_name = 'visual_servo_controller'

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
    description='Visual servoing controller for Apriltag approach',
    license='MIT',
    entry_points={
        'console_scripts': [
            'servo_node = visual_servo_controller.servo_node:main',
        ],
    },
)
