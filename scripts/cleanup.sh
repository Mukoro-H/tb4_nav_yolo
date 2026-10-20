#!/usr/bin/env bash
# Tắt sạch các tiến trình Gazebo / ROS 2 còn chạy ngầm trước mỗi lần launch.

for p in "ign gazebo" ruby parameter_bridge robot_state_publisher spawner \
         slam_toolbox amcl map_server lifecycle_manager \
         planner_server controller_server bt_navigator behavior_server \
         rviz2 teleop_twist_keyboard; do
  pkill -9 -f "$p" 2>/dev/null
done

ros2 daemon stop >/dev/null 2>&1

if ps aux | grep -E "ign gazebo|amcl|slam_toolbox|rviz2" | grep -v grep >/dev/null; then
  echo "[cleanup] Van con tien trinh dang chay:"
  ps aux | grep -E "ign gazebo|amcl|slam_toolbox|rviz2" | grep -v grep
else
  echo "[cleanup] Da tat sach."
fi
