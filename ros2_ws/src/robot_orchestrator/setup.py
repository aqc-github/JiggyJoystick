from setuptools import setup
import os
from glob import glob

package_name = 'robot_orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='albertoquintana',
    maintainer_email='alberto.quintana.criado@proton.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'experiment_manager_node = robot_orchestrator.main:experiment_manager_main',
            'control_node = robot_orchestrator.main:control_main',
            'logger_node = robot_orchestrator.main:logger_main',
        ],
    },
)
