from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include semua launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include konfigurasi RViz
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Include konfigurasi SLAM (YAML) - Wajib agar tidak error path
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kanal-raspi5',
    maintainer_email='kanal-raspi5@todo.todo',
    description='Paket Bringup untuk Robot AMR',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_bridge = robot_bringup.stm32_bridge:main',
            'odometry_node = robot_bringup.odometry_node:main',
            'laser_restamper = robot_bringup.laser_restamper:main',
        ],
    },
)
