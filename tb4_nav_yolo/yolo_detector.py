#!/usr/bin/env python3
"""
yolo_detector.py

Node nhận diện vật thể từ camera RGB của TurtleBot 4 bằng YOLO (Ultralytics).

Luồng hoạt động:
  1. Subscribe ảnh camera (sensor_msgs/Image), mặc định /oakd/rgb/preview/image_raw.
  2. Chuyển ROS Image -> OpenCV (bgr8) bằng cv_bridge.
  3. Chạy YOLO theo tần số cố định (rate_hz) trên khung hình mới nhất.
  4. Vẽ bounding box + tên class + độ tự tin:
       - hiển thị trên cửa sổ OpenCV (show_window: true)
       - publish lên /yolo/image_annotated để xem trên RViz2
  5. In danh sách vật thể ra terminal, kèm vị trí robot trên map (/amcl_pose),
     và publish dạng JSON lên /yolo/detections.

Tham số:
  image_topic : topic ảnh đầu vào
  model       : yolov8n.pt, yolo11n.pt, yolov8s-world.pt, ...
  classes     : (chỉ dùng với YOLO-World) danh sách class, ngăn cách bởi dấu phẩy
  conf        : ngưỡng độ tự tin
  rate_hz     : số lần chạy YOLO mỗi giây
  imgsz       : kích thước ảnh đầu vào cho YOLO
  device      : '' (tự chọn), 'cpu', '0' (GPU 0)
  show_window : bật/tắt cửa sổ OpenCV
  log_period  : chu kỳ in kết quả ra terminal (giây)
"""

import json
import math
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('classes', '')
        self.declare_parameter('conf', 0.4)
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', '')
        self.declare_parameter('show_window', True)
        self.declare_parameter('log_period', 1.0)

        p = lambda n: self.get_parameter(n).value
        self.image_topic = p('image_topic')
        self.conf = float(p('conf'))
        self.imgsz = int(p('imgsz'))
        self.device = p('device') or None
        self.show_window = bool(p('show_window')) and bool(os.environ.get('DISPLAY'))
        self.log_period = float(p('log_period'))

        # --- Load model YOLO
        model_name = p('model')
        self.get_logger().info(f'Loading YOLO model: {model_name}')
        self.model = YOLO(model_name)
        classes = [c.strip() for c in p('classes').split(',') if c.strip()]
        if classes:
            if 'world' in model_name.lower():
                self.model.set_classes(classes)
                self.get_logger().info(f'YOLO-World classes: {classes}')
            else:
                self.get_logger().warn('Tham số "classes" chỉ dùng cho YOLO-World, bỏ qua.')

        self.bridge = CvBridge()
        self.latest_msg = None
        self.robot_xy = None
        self.last_log = 0.0
        self.frame_count = 0
        self.fps_t0 = time.time()
        self.fps = 0.0

        # --- ROS interfaces
        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        self.pub_image = self.create_publisher(Image, '/yolo/image_annotated', 10)
        self.pub_det = self.create_publisher(String, '/yolo/detections', 10)
        self.create_timer(1.0 / float(p('rate_hz')), self.process)

        self.get_logger().info(f'Subscribed: {self.image_topic}')
        self.get_logger().info('Publishing: /yolo/image_annotated, /yolo/detections')

    # ---------------------------------------------------------------- callbacks
    def image_cb(self, msg):
        self.latest_msg = msg            # chỉ giữ khung hình mới nhất

    def pose_cb(self, msg):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # ---------------------------------------------------------------- inference
    def process(self):
        if self.latest_msg is None:
            return
        msg, self.latest_msg = self.latest_msg, None

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        results = self.model.predict(frame, conf=self.conf, imgsz=self.imgsz,
                                     device=self.device, verbose=False)
        r = results[0]

        detections = []
        for box in r.boxes:
            cls_id = int(box.cls[0])
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            detections.append({
                'class': r.names[cls_id],
                'conf': round(float(box.conf[0]), 3),
                'bbox': [x1, y1, x2, y2],
            })

        # --- FPS
        self.frame_count += 1
        dt = time.time() - self.fps_t0
        if dt >= 1.0:
            self.fps = self.frame_count / dt
            self.frame_count, self.fps_t0 = 0, time.time()

        # --- Vẽ kết quả
        annotated = r.plot()
        cv2.putText(annotated, f'FPS: {self.fps:.1f}  objects: {len(detections)}',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = msg.header
        self.pub_image.publish(out)

        self.pub_det.publish(String(data=json.dumps({
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'robot_xy': self.robot_xy,
            'detections': detections,
        })))

        if self.show_window:
            cv2.imshow('TurtleBot 4 - YOLO', annotated)
            cv2.waitKey(1)

        self.log(detections)

    def log(self, detections):
        now = time.time()
        if now - self.last_log < self.log_period:
            return
        self.last_log = now
        where = ''
        if self.robot_xy is not None and not any(math.isnan(v) for v in self.robot_xy):
            where = f' | robot @ map({self.robot_xy[0]:.2f}, {self.robot_xy[1]:.2f})'
        if not detections:
            self.get_logger().info(f'[YOLO] không phát hiện vật thể{where}')
            return
        names = ', '.join(f"{d['class']}({d['conf']:.2f})" for d in detections)
        self.get_logger().info(f'[YOLO] {len(detections)} vật thể: {names}{where}')
        for d in detections:
            self.get_logger().info(f"    - {d['class']:<15} conf={d['conf']:.2f} bbox={d['bbox']}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
