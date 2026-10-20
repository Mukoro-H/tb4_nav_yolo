#!/usr/bin/env python3
"""
waypoint_navigator.py

Node điều hướng TurtleBot 4 tuần tự qua các goal (Goal 1 -> Goal 2 -> Goal 3 ...)
bằng nav2_simple_commander (BasicNavigator), gọi action /navigate_to_pose của Nav2.

Luồng hoạt động:
  1. (tuỳ chọn) Đặt initial pose cho AMCL qua topic /initialpose (robot còn trên dock).
  2. Chờ AMCL và bt_navigator active.
  3. (tuỳ chọn) Undock robot qua action /undock.
  4. Với mỗi goal: gửi goToPose(), theo dõi feedback, chờ kết quả.
  5. In bảng tổng kết kết quả từng goal.

Tham số (xem config/goals.yaml):
  goals              : [x1, y1, yaw1_deg, x2, y2, yaw2_deg, ...] trong frame map
  undock_first       : true/false
  set_initial_pose   : true/false
  initial_pose       : [x, y, yaw_deg]
  wait_at_goal_sec   : thời gian dừng tại mỗi goal (để YOLO quan sát)
  goal_timeout_sec   : quá thời gian thì huỷ goal
  loop               : lặp lại danh sách goal
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from irobot_create_msgs.action import Undock


def yaw_to_quaternion(yaw_rad):
    """Chuyển góc yaw (rad) quanh trục z thành quaternion (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


class WaypointNavigator(BasicNavigator):
    def __init__(self):
        super().__init__(node_name='waypoint_navigator')

        self.declare_parameter('goals', [0.0, 0.0, 0.0])
        self.declare_parameter('undock_first', True)
        self.declare_parameter('set_initial_pose', False)
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('wait_at_goal_sec', 3.0)
        self.declare_parameter('goal_timeout_sec', 180.0)
        self.declare_parameter('loop', False)

        flat = list(self.get_parameter('goals').value)
        if len(flat) % 3 != 0 or len(flat) == 0:
            raise ValueError('Tham số "goals" phải có dạng [x, y, yaw_deg, ...]')
        self.goals = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]

        self.undock_first = self.get_parameter('undock_first').value
        self.set_init = self.get_parameter('set_initial_pose').value
        self.init_pose = list(self.get_parameter('initial_pose').value)
        self.wait_at_goal = float(self.get_parameter('wait_at_goal_sec').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        self.loop = self.get_parameter('loop').value

    # ------------------------------------------------------------------ utils
    def make_pose(self, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(math.radians(yaw_deg))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def undock(self, timeout_sec=30.0):
        """Gửi action /undock. Nếu robot không nằm trên dock thì action kết thúc ngay."""
        client = ActionClient(self, Undock, '/undock')
        if not client.wait_for_server(timeout_sec=5.0):
            self.warn('Không tìm thấy action /undock, bỏ qua bước undock.')
            return
        self.info('Đang undock robot...')
        send_future = client.send_goal_async(Undock.Goal())
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.warn('Goal undock bị từ chối (có thể robot đã ở ngoài dock).')
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        self.info('Undock xong.')

    # ------------------------------------------------------------------- main
    def run(self):
        # 1. Đặt initial pose TRƯỚC khi undock, lúc robot còn nằm trên dock
        #    (dock = gốc map, yaw = 0). Lệnh undock của Create 3 lùi robot ra
        #    rồi quay ~180 độ, nên nếu đặt pose sau khi undock sẽ sai hướng.
        if self.set_init:
            x, y, yaw = self.init_pose
            self.info(f'Đặt initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.1f} deg')
            self.setInitialPose(self.make_pose(x, y, yaw))
        else:
            # Chờ AMCL publish /amcl_pose (sau khi đặt 2D Pose Estimate trên RViz2).
            # Phải chờ ở đây, nếu không waitUntilNav2Active() sẽ tự publish pose (0, 0).
            self.info('Chờ initial pose (đặt bằng 2D Pose Estimate trên RViz2)...')
            while rclpy.ok() and not self.initial_pose_received:
                rclpy.spin_once(self, timeout_sec=0.5)

        # 2. Chờ AMCL nhận initial pose và bt_navigator chuyển sang active
        self.waitUntilNav2Active(localizer='amcl')

        # 3. Undock. AMCL theo dõi chuyển động lúc undock qua odometry.
        if self.undock_first:
            self.undock()
        self.info(f'Nav2 đã sẵn sàng. Số goal: {len(self.goals)}')

        while True:
            summary = []
            for idx, (x, y, yaw) in enumerate(self.goals, start=1):
                summary.append(self.go_to(idx, x, y, yaw))
            self.print_summary(summary)
            if not self.loop:
                break

    def go_to(self, idx, x, y, yaw):
        self.info(f'==> Goal {idx}: x={x:.2f}, y={y:.2f}, yaw={yaw:.1f} deg')
        start = time.time()
        self.goToPose(self.make_pose(x, y, yaw))

        last_log = 0.0
        while not self.isTaskComplete():
            fb = self.getFeedback()
            now = time.time()
            if fb is not None and now - last_log > 1.0:
                last_log = now
                self.info(
                    f'   Goal {idx}: còn {fb.distance_remaining:.2f} m, '
                    f'recoveries={fb.number_of_recoveries}')
            if fb is not None and Duration.from_msg(fb.navigation_time) > \
                    Duration(seconds=self.goal_timeout):
                self.warn(f'   Goal {idx}: quá {self.goal_timeout:.0f}s, huỷ goal.')
                self.cancelTask()

        result = self.getResult()
        elapsed = time.time() - start
        if result == TaskResult.SUCCEEDED:
            status = 'SUCCEEDED'
            self.info(f'<== Goal {idx} thành công sau {elapsed:.1f}s')
            if self.wait_at_goal > 0:
                time.sleep(self.wait_at_goal)
        elif result == TaskResult.CANCELED:
            status = 'CANCELED'
            self.warn(f'<== Goal {idx} bị huỷ')
        else:
            status = 'FAILED'
            self.error(f'<== Goal {idx} thất bại')
        return idx, x, y, yaw, status, elapsed

    def print_summary(self, summary):
        self.info('================ KẾT QUẢ ================')
        for idx, x, y, yaw, status, elapsed in summary:
            self.info(f'Goal {idx} ({x:.2f}, {y:.2f}, {yaw:.0f} deg): '
                      f'{status} - {elapsed:.1f}s')
        ok = sum(1 for s in summary if s[4] == 'SUCCEEDED')
        self.info(f'Hoàn thành {ok}/{len(summary)} goal')
        self.info('=========================================')


def main(args=None):
    rclpy.init(args=args)
    nav = WaypointNavigator()
    try:
        nav.run()
    except KeyboardInterrupt:
        nav.cancelTask()
    finally:
        nav.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
