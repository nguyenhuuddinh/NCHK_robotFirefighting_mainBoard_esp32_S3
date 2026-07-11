# TỔNG HỢP LỆNH VÀ KIẾN TRÚC ROS 2 (PHASE 1)

Tài liệu này tổng hợp lại toàn bộ các lệnh terminal đã sử dụng, cấu trúc thư mục và ý nghĩa của từng file đã tạo ra trong quá trình làm Phase 1 (Khởi tạo Workspace & Robot Description). Rất hữu ích để tra cứu lại sau này.

---

## 1. CÁC LỆNH ROS 2 VÀ LINUX ĐÃ SỬ DỤNG

### 1.1. Khởi tạo và Build Workspace
```bash
# 1. Tạo thư mục workspace và thư mục con src chứa source code
mkdir -p ros2_ws/src

# 2. Di chuyển vào thư mục workspace
cd ros2_ws/

# 3. Tạo một package ROS 2 mới tên là fire_robot_description (loại C++)
ros2 pkg create fire_robot_description --build-type ament_cmake

# 4. Cài đặt các package ROS 2 cần thiết (nếu thiếu)
sudo apt install -y ros-humble-xacro ros-humble-joint-state-publisher

# 5. Nạp biến môi trường của ROS 2 gốc (phải chạy mỗi khi mở terminal mới)
source /opt/ros/humble/setup.bash

# 6. Build riêng package fire_robot_description
colcon build --packages-select fire_robot_description

# 7. Nạp biến môi trường của workspace vừa build (để ROS 2 nhận diện package mới)
source install/setup.bash
```

### 1.2. Chạy và Kiểm tra (Debug)
```bash
# 1. Chạy file launch để bật robot_state_publisher và RViz2
ros2 launch fire_robot_description description.launch.py rviz:=true

# 2. Kiểm tra danh sách các Topic đang hoạt động
ros2 topic list

# 3. Xem nội dung dữ liệu của một Topic cụ thể
ros2 topic echo /robot_description

# 4. Xuất cây tọa độ (TF Tree) ra file PDF (frames.pdf) để kiểm tra
ros2 run tf2_tools view_frames

# 5. In ra terminal khoảng cách/góc xoay thực tế giữa 2 frame (VD: base_link và laser_frame)
ros2 run tf2_ros tf2_echo base_link laser_frame
```

---

## 2. CẤU TRÚC THƯ MỤC VÀ Ý NGHĨA CÁC FILE ĐÃ TẠO

Dưới đây là cây thư mục hiện tại của `ros2_ws` sau Phase 1, kèm giải thích chức năng từng file:

```text
ros2_ws/
├── .gitignore
│   # Chặn Git theo dõi các thư mục sinh ra khi build (build/, install/, log/). Giúp repo nhẹ và sạch.
│
└── src/
    └── fire_robot_description/
        ├── package.xml
        │   # Khai báo tên package, version, tác giả và quan trọng nhất là các thư viện phụ thuộc (dependencies) như: xacro, rviz2, robot_state_publisher.
        │
        ├── CMakeLists.txt
        │   # File cấu hình cho trình biên dịch Cmake. Ở đây chủ yếu ra lệnh copy các thư mục (urdf, launch, config) vào thư mục `install/` khi chạy colcon build.
        │
        ├── urdf/
        │   └── fire_robot.urdf.xacro
        │       # QUAN TRỌNG NHẤT: Khai báo mô hình vật lý của xe (chiều dài, rộng, cao, bánh xe). 
        │       # Đồng thời định nghĩa Cây Tọa Độ (TF Tree) xác định vị trí chính xác của Lidar, Camera, IMU so với tâm xe (base_link).
        │
        ├── launch/
        │   └── description.launch.py
        │       # Kịch bản khởi chạy (Launch file viết bằng Python). 
        │       # File này tự động biên dịch URDF bằng `xacro`, truyền vào `robot_state_publisher` để phát TF lên ROS 2, và bật phần mềm RViz2.
        │
        └── config/
            └── fire_robot.rviz
                # Lưu cấu hình giao diện của phần mềm RViz2 (Màu sắc lưới, góc nhìn camera mặc định, bật sẵn plugin hiển thị RobotModel và TF).
```

---

## 3. LƯU Ý KHI CODE ROS 2
1. **Luôn `source`**: Mở terminal mới ra là phải `source /opt/ros/humble/setup.bash` và `source install/setup.bash` trước khi chạy bất kỳ lệnh `ros2` nào.
2. **Build sau khi sửa code**: 
   - Nếu sửa file trong `urdf/`, `launch/` hoặc `config/` (với ament_cmake), bạn BẮT BUỘC phải chạy `colcon build` lại thì ROS 2 mới cập nhật.
   - Nếu là code Python (ament_python) dùng tùy chọn `--symlink-install` thì sửa xong chạy luôn không cần build lại.
3. **Phân biệt `ros2 run` và `ros2 launch`**:
   - `ros2 run`: Chạy duy nhất 1 node đơn lẻ.
   - `ros2 launch`: Chạy nhiều node cùng lúc với các tham số cấu hình phức tạp.
