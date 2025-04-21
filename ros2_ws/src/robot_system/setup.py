from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/assays.yaml']),  # Install config file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aqc',
    maintainer_email='alberto.quintana.criado@proton.me',
    description='ROS2 package for robot control and logging',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                'control_node = robot_system.robot_system:control_main',
                'logger_node = robot_system.robot_system:logger_main',
            ],
    },
)
