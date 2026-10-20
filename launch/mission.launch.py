"""
mission.launch.py

Khởi chạy đồng thời:
  - yolo_detector      : nhận diện vật thể từ camera
  - waypoint_navigator : điều hướng tuần tự qua các goal (nếu navigate:=true)

Yêu cầu trước: Gazebo + localization (AMCL) + Nav2 đã chạy (xem README).

Ví dụ:
  ros2 launch tb4_nav_yolo mission.launch.py
  ros2 launch tb4_nav_yolo mission.launch.py navigate:=false          # chỉ chạy YOLO
  ros2 launch tb4_nav_yolo mission.launch.py params_file:=/path/goals.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tb4_nav_yolo')
    default_params = os.path.join(pkg_share, 'config', 'goals.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    navigate = LaunchConfiguration('navigate')
    detect = LaunchConfiguration('detect')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='File YAML chứa toạ độ goal và tham số YOLO'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('navigate', default_value='true',
                              description='Chạy node điều hướng'),
        DeclareLaunchArgument('detect', default_value='true',
                              description='Chạy node YOLO'),

        Node(
            package='tb4_nav_yolo',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            condition=IfCondition(detect),
        ),
        Node(
            package='tb4_nav_yolo',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            condition=IfCondition(navigate),
        ),
    ])
