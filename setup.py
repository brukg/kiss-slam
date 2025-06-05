import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'kiss_slam'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('kiss_slam/config/*.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'geometry_msgs',
        'nav_msgs',
        'tf2_ros',
        'tf2_geometry_msgs',
        'visualization_msgs',
        'std_msgs',
        'numpy',
        'scipy',
    ],
    zip_safe=True,
    maintainer='Tiziano Guadagnino',
    maintainer_email='frevo93@gmail.com',
    description='KISS-SLAM: A Simple, Robust, and Accurate 3D LiDAR SLAM System With Enhanced Generalization Capabilities for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
) 