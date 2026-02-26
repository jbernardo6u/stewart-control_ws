#!/home/rem/rtimulib-env/bin/python3

from setuptools import setup
import os
from glob import glob

package_name = 'stewart_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'RTIMU',
        'rclpy',
        'PySide6',
        'sensor_msgs',
    ],
    zip_safe=True,
    maintainer='rem',
    maintainer_email='rem@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'imu_node=stewart_control.imu_node:main',
            'stewart_node=stewart_control.stewart_node:main',
            'interface_node=stewart_control.interface_node:main',
            'aruco_node=stewart_control.aruco_node:main',
            'posHome_node=stewart_control.posHome_node:main',
            'manual_stewart_node=stewart_control.manual_stewart_node:main',
            'fusion_node = stewart_control.fusion_node:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        ('share/' + package_name, ['share/stewart_control/calib_int.npz']),
        ('share/' + package_name, ['share/stewart_control/calib_ext3.npz']),
    ],
)
