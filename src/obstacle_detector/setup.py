import os
from glob import glob

from setuptools import setup

package_name = 'obstacle_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Obstacle detection by clustering 2D lidar scan points.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detector = obstacle_detector.detector_node:main',
            'odom_tf_broadcaster = obstacle_detector.odom_tf_broadcaster:main',
        ],
    },
)
