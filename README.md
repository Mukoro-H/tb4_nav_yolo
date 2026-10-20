# TurtleBot 4 – Autonomous Navigation (Nav2) + Object Detection (YOLO)

Repo gồm các thành phần sau:

- Package ROS 2 `tb4_nav_yolo`, chạy cho robot TurtleBot 4 trong Gazebo.
- Bản đồ đã quét ở bài trước (`map.yaml`, `map.pgm`), dùng để **định vị bằng AMCL**.
- Node **`waypoint_navigator`**: dùng `nav2_simple_commander` để robot tự đi tuần tự qua các goal (Goal 1 → Goal 2 → Goal 3).
- Node **`yolo_detector`**: nhận diện vật thể từ camera RGB bằng **YOLO** trong lúc robot di chuyển.

| | |
|---|---|
| **Sinh viên** | Trần Lưu Hưng |
| **MSSV** | 23020747 |
| **Video demo** | _[Google Drive link]_ |
| **Báo cáo PDF** | [`docs/report.pdf`](docs/report.pdf) |

---

## 1. Cấu trúc repository

```
tb4_nav_yolo/
├── tb4_nav_yolo/
│   ├── waypoint_navigator.py   # Node điều hướng qua các goal (Nav2 Simple Commander)
│   └── yolo_detector.py        # Node nhận diện vật thể từ camera (YOLO + cv_bridge)
├── launch/
│   └── mission.launch.py       # Chạy đồng thời 2 node trên
├── config/
│   └── goals.yaml              # Toạ độ goal + tham số YOLO
├── maps/
│   ├── map.yaml                # Bản đồ từ bài thực hành trước
│   └── map.pgm
├── scripts/
│   └── cleanup.sh              # Tắt sạch Gazebo/ROS chạy ngầm
├── package.xml
├── setup.py
└── README.md
```

## 2. Cài đặt

Môi trường: **Ubuntu 22.04**, **ROS 2 Humble**, **Gazebo Fortress**, TurtleBot 4 Simulator. Các gói này đã cài ở bài trước.

```bash
sudo apt update
sudo apt install -y ros-humble-turtlebot4-navigation ros-humble-nav2-simple-commander \
                    ros-humble-cv-bridge python3-colcon-common-extensions
pip install ultralytics opencv-python "numpy<2"
```

> **Phải giữ `numpy<2`.** `cv_bridge` của Humble được build với NumPy 1.x, nên khi `pip install ultralytics` tự nâng NumPy lên 2.x thì node sẽ lỗi `_ARRAY_API not found` hoặc `numpy.core.multiarray failed to import`.

### Build package

```bash
mkdir -p ~/tb4_ws/src && cd ~/tb4_ws/src
git clone https://github.com/<username>/tb4_nav_yolo.git
cd ~/tb4_ws
colcon build --symlink-install
source install/setup.bash
echo "source ~/tb4_ws/install/setup.bash" >> ~/.bashrc

mkdir -p ~/maps && cp src/tb4_nav_yolo/maps/map.* ~/maps/
chmod +x src/tb4_nav_yolo/scripts/cleanup.sh
```

## 3. Chạy chương trình

> Mỗi `T1`, `T2`, ... là **một terminal riêng**. Trước khi bắt đầu, chạy `~/tb4_ws/src/tb4_nav_yolo/scripts/cleanup.sh` để tắt các tiến trình cũ.
> **Không dùng `model:=lite`**, vì bản TurtleBot 4 Lite không có camera OAK-D.

### Phần 1 – Simulation + Localization + Nav2

```bash
# T1 – Gazebo, đúng world đã dùng để quét bản đồ
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=warehouse

# T2 – Map server + AMCL
ros2 launch turtlebot4_navigation localization.launch.py map:=$HOME/maps/map.yaml use_sim_time:=true

# T3 – Nav2
ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true

# T4 – RViz2
ros2 launch nav2_bringup rviz_launch.py
```

Cần đợi T2 và T3 báo `Managed nodes are active`.

**Thêm vật thể vào Gazebo** để YOLO có cái nhận diện: trong cửa sổ Gazebo, bấm menu ⋮ (góc trên bên phải), mở **Resource Spawner**, chọn **Fuel resources**, tìm `person`, `chair`, `fire extinguisher`, ... rồi kéo vào các hành lang gần goal.

**Kiểm tra initial pose:** dùng **2D Pose Estimate** tại vị trí dock. Dock là gốc `(0, 0)` của bản đồ. Kiểm tra `/scan` trùng khớp với tường trên map. Node điều hướng có thể tự đặt initial pose bằng `set_initial_pose: true` trong `config/goals.yaml`.

Trong RViz2, chọn **Add → By topic** và thêm `/yolo/image_annotated` (Image) để xem kết quả YOLO ngay trên RViz.

### Phần 2 – Lấy toạ độ goal trên bản đồ

```bash
# T5
ros2 topic echo /clicked_point
```

Trên RViz2, bấm **Publish Point** rồi click vào vị trí muốn đặt goal. Ghi lại `x` và `y` từ T5. Hướng `yaw` (độ) tự chọn, ví dụ `0` là hướng theo trục x của map, `90` là hướng theo trục y. Cũng có thể lái robot tới một vị trí rồi đọc `ros2 topic echo /amcl_pose`.

Điền toạ độ vào `config/goals.yaml`:

```yaml
waypoint_navigator:
  ros__parameters:
    goals: [
      x1, y1, yaw1,   # Goal 1
      x2, y2, yaw2,   # Goal 2
      x3, y3, yaw3,   # Goal 3
    ]
```

Vì đã build với `--symlink-install`, sửa file YAML xong không cần build lại.

### Phần 3 + 4 – Điều hướng tự động + YOLO

```bash
# T5 – chạy cả 2 node
ros2 launch tb4_nav_yolo mission.launch.py
```

Có thể chạy riêng từng node:

```bash
ros2 launch tb4_nav_yolo mission.launch.py navigate:=false   # chỉ YOLO
ros2 launch tb4_nav_yolo mission.launch.py detect:=false     # chỉ điều hướng
ros2 run tb4_nav_yolo yolo_detector --ros-args -p model:=yolo11n.pt
```

Kết quả khi chạy:

- **Node điều hướng:**
  - Tự undock và đặt initial pose.
  - Chờ Nav2 active, rồi lần lượt gửi Goal 1 → 2 → 3.
  - In quãng đường còn lại mỗi giây.
  - Cuối cùng in bảng `SUCCEEDED / FAILED` cho từng goal.
  - Sau khi chạy node, **không điều khiển robot thủ công và không đặt goal bằng chuột**.
- **Node YOLO:**
  - Mở cửa sổ OpenCV có bounding box, tên class, độ tự tin và FPS.
  - In danh sách vật thể ra terminal, kèm vị trí robot trên map.
  - Publish ảnh đã vẽ lên `/yolo/image_annotated`.
  - Publish kết quả dạng JSON lên `/yolo/detections`.

### Dùng YOLO-World (nhận diện class tuỳ chọn)

Model YOLO thường (COCO) chỉ nhận diện được 80 class như `person`, `chair`, `car`, `bottle`, ... Nếu muốn nhận diện kệ hàng, thùng carton hay bình chữa cháy trong warehouse, dùng YOLO-World:

```bash
ros2 launch tb4_nav_yolo mission.launch.py   # sau khi sửa trong goals.yaml:
#   model: yolov8s-world.pt
#   classes: "person, box, shelf, fire extinguisher, pallet"
```

## 4. Luồng hoạt động

```
                ┌───────────────── Gazebo (warehouse + TurtleBot 4) ─────────────────┐
                │   LiDAR /scan      Odometry /odom      Camera /oakd/rgb/preview/... │
                └─────┬─────────────────────┬──────────────────────┬──────────────────┘
                      │                     │                      │
       map.yaml ─▶ map_server ─▶ /map ─▶  AMCL ─▶ TF map→odom      │
                      │                     │ /amcl_pose           │
                      ▼                     ▼                      ▼
   waypoint_navigator ──(action /navigate_to_pose)──▶ Nav2     yolo_detector
     goals.yaml          bt_navigator → planner → controller    cv_bridge → YOLO
                                                   │            → /yolo/image_annotated
                                                   ▼            → /yolo/detections
                                              /cmd_vel → robot
```

| Thành phần | Loại | Vai trò |
|---|---|---|
| `/scan` | topic `sensor_msgs/LaserScan` | LiDAR cho AMCL và costmap |
| `/odom`, `/tf` | topic | Odometry và cây toạ độ `map → odom → base_link` |
| `/map` | topic `nav_msgs/OccupancyGrid` | Bản đồ do `map_server` phát |
| `/initialpose` | topic | Vị trí ban đầu cho AMCL |
| `/amcl_pose` | topic | Vị trí robot trên map |
| `/clicked_point` | topic | Toạ độ click bằng Publish Point |
| `/undock` | action `irobot_create_msgs/Undock` | Đưa robot ra khỏi dock |
| `/navigate_to_pose` | action `nav2_msgs/NavigateToPose` | Gửi goal cho Nav2 (qua `BasicNavigator.goToPose`) |
| `/cmd_vel` | topic `geometry_msgs/Twist` | Lệnh vận tốc do Nav2 controller xuất |
| `/oakd/rgb/preview/image_raw` | topic `sensor_msgs/Image` | Ảnh camera RGB đầu vào cho YOLO |
| `/yolo/image_annotated` | topic `sensor_msgs/Image` | Ảnh đã vẽ bounding box |
| `/yolo/detections` | topic `std_msgs/String` (JSON) | Danh sách vật thể + vị trí robot |

## 5. Xử lý lỗi thường gặp

| Hiện tượng | Cách xử lý |
|---|---|
| `_ARRAY_API not found` / lỗi numpy khi chạy `yolo_detector` | `pip install "numpy<2"` |
| YOLO không nhận được ảnh | Kiểm tra bằng `ros2 topic list \| grep oakd`, sửa `image_topic` cho đúng; không dùng `model:=lite` |
| Node điều hướng đứng ở `Chờ initial pose...` | Đặt 2D Pose Estimate trên RViz2, hoặc đặt `set_initial_pose: true` |
| Goal báo `FAILED` | Goal nằm trong vùng unknown hoặc sát vật cản; lấy lại toạ độ ở giữa hành lang |
| YOLO không phát hiện vật thể nào | Warehouse ít vật thể thuộc COCO, cần thêm `person` / `chair` từ Fuel, hoặc dùng YOLO-World |
| Gazebo rất lag, camera giật | Kiểm tra `glxinfo -B \| grep renderer`, cần GPU NVIDIA thay vì `llvmpipe` |
| Treo ở `waiting for service /controller_manager` | Chạy `scripts/cleanup.sh` rồi launch lại Gazebo |
| Không có cửa sổ OpenCV | Xem trên RViz2 qua topic `/yolo/image_annotated` |

## 6. Kết quả

| Bản đồ + các goal | Robot lập đường đi | Nhận diện YOLO |
|---|---|---|
| ![goals](docs/images/goals.png) | ![path](docs/images/path.png) | ![yolo](docs/images/yolo.png) |

Video demo: _[Google Drive link]_

## 7. Tài liệu tham khảo

- TurtleBot 4 Navigator: <https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html>
- Nav2 Simple Commander API: <https://docs.nav2.org/commander_api/index.html>
- Ultralytics YOLO: <https://docs.ultralytics.com/>
