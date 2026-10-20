import os
from glob import glob

from setuptools import setup

package_name = 'tb4_nav_yolo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tran Luu Hung',
    maintainer_email='your_email@example.com',
    description='TurtleBot 4 waypoint navigation (Nav2) + YOLO object detection',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_navigator = tb4_nav_yolo.waypoint_navigator:main',
            'yolo_detector = tb4_nav_yolo.yolo_detector:main',
        ],
    },
)
