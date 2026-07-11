# NHẬT KÝ PHÁT TRIỂN ROS 2 - PHASE 1 (URDF & TF TREE)

> **Mục đích tài liệu:** Ghi chép lại toàn bộ quá trình thực hiện Phase 1 (Tạo Workspace, Code URDF, Khởi chạy RViz) cực kỳ chi tiết. Dành để đọc lại, ôn tập và hiểu sâu lý do đằng sau mỗi câu lệnh, mỗi dòng code và các lỗi (bug) đã gặp phải cùng cách khắc phục.

---

## 1. TƯ DUY KHỞI ĐẦU: TẠI SAO LÀM THẾ NÀY?

### Câu hỏi từ User:
*Tại sao `cau_truc_ros2_workspace.md` lại gộp chung hết vào 1 workspace? Trên Pi và Laptop dùng chung luôn à?*

### Giải thích:
Đúng vậy! Trong hệ thống ROS 2 phân tán (Laptop + Pi), **chỉ cần 1 repository duy nhất**. Ta sẽ clone source code này về cả Pi và Laptop rồi chạy `colcon build` giống hệt nhau.
- **Lý do:** Cả 2 máy đều cần "hiểu" những thông tin chung như kích thước xe (`fire_robot_description`), kiểu bản tin nhắn, và cấu hình.
- **Cách hoạt động:** Dù code nằm chung 1 chỗ, nhưng khi chạy thực tế:
  - **Trạm Laptop** sẽ gọi: `ros2 launch fire_robot_bringup station.launch.py` (chỉ bật AI, Nav2, SLAM).
  - **Trạm Pi** sẽ gọi: `ros2 launch fire_robot_bringup robot.launch.py` (chỉ bật Camera, Lidar, micro-ROS).
-> Cách này giúp quản lý Git cực dễ, không bao giờ lo 2 máy bị "lệch phiên bản code" với nhau.

---

## 2. LẤY THÔNG SỐ VÀ QUY TẮC TỌA ĐỘ (REP-103)

Trước khi code URDF (Mô hình 3D), ta phải thống nhất kích thước thật bằng thước đo.

### Thông số xe:
- **Khung (Chassis):** Dài 25.5cm, Rộng 15cm, Cao 5.2cm, Khoảng sáng gầm (Ground Clearance) 6.5cm.
- **Bánh (Wheels):** Bán kính 3.4cm, Dày 2.6cm, Trục trái-phải (Track Width) 18.0cm, Trục trước-sau (Wheelbase) 13.4cm.
- **Vị trí Cảm biến (Tính từ tâm đất của xe):**
  - **IMU:** X=1.6cm, Y=-0.5cm, Z từ đất=7.5cm (cao hơn đáy 1cm)
  - **Lidar:** X=9.4cm, Y=0.0cm, Z từ đất=18.3cm
  - **Camera:** X=12.75cm, Y=0.0cm, Z từ đất=12.8cm, chúc xuống ~4 độ.

### Quy tắc tọa độ ROS 2 (Bàn tay phải):
- **Trục X (Đỏ):** Mũi xe (Tiến về trước là số Dương).
- **Trục Y (Xanh lá):** Trái/Phải (Sang TRÁI là số Dương, sang PHẢI là số Âm).
- **Trục Z (Xanh dương):** Lên/Xuống (Lên trên là số Dương).
- **Góc Pitch:** Là xoay quanh trục Y. Do quy tắc bàn tay phải, góc Pitch **Dương (+) sẽ khiến mũi xe chúc xuống đất**, và Âm (-) sẽ khiến mũi xe ngóc lên trời.

---

## 3. CÁC BƯỚC CODE & Ý NGHĨA

### 3.1. Tạo Workspace và Package
**Lệnh:**
```bash
mkdir -p ros2_ws/src
cd ros2_ws/
ros2 pkg create fire_robot_description --build-type ament_cmake
```
**Ý nghĩa:** Tạo "không gian làm việc" (workspace) và tạo một "gói" (package) tên là `fire_robot_description`. Loại `ament_cmake` là chuẩn cho C++ và các package chứa file dữ liệu (.urdf, .launch, .yaml).

### 3.2. Code File URDF (`urdf/fire_robot.urdf.xacro`)
- Đây là file XML đặc biệt (xacro) định nghĩa xe có những bộ phận nào (`link`) và cách nối chúng với nhau (`joint`).
- **Lý do dùng `fixed` joint:** Vì hệ thống của ta Lidar, Camera, IMU và cả 4 bánh xe hiện tại đều gắn chặt vào khung (skid-steer không bẻ lái vật lý). Chỉnh tất cả thành `fixed` giúp hệ thống nhẹ hơn, tự động phát tọa độ tĩnh (TF_static).

### 3.3. Code File Launch (`launch/description.launch.py`)
- Kịch bản tự động hóa viết bằng Python. Thay vì phải gõ 3,4 lệnh để bật RViz, dịch file URDF... ta chỉ cần gọi 1 file Launch.
- **Node `robot_state_publisher`:** Nhận file URDF, tính toán tọa độ và phát liên tục vào mạng lưới ROS 2.

---

## 4. CÁC BUG ĐÃ GẶP VÀ CÁCH KHẮC PHỤC THỰC TẾ

### 🐞 Bug 1: Lỗi 404 Not Found khi chạy `sudo apt install`
- **Tình trạng:** Chạy lệnh cài `ros-humble-xacro` thì báo lỗi đỏ lòm `404 Not Found`.
- **Nguyên nhân:** Danh sách phần mềm (cache) lưu trên máy tính bị cũ so với máy chủ Ubuntu. Máy tính đòi tải bản 2.4.0 cũ nhưng server ROS đã xóa nó và đưa bản mới lên.
- **Cách fix:** 
  ```bash
  sudo apt clean    # Xóa cache cũ
  sudo apt update   # Tải danh sách mới nhất
  sudo apt install -y ros-humble-xacro --fix-missing  # Cài lại
  ```
- **Hỏi đáp User:** *Tôi update thấy dòng chữ `N: Skipping acquire... i386` có sao không?*
  - **Giải thích:** Hoàn toàn vô hại. Chữ N là Notice (chú ý). Nó báo kho Google Chrome không có bản 32-bit (i386). Máy mình 64-bit nên cứ mặc kệ nó.

### 🐞 Bug 2: Lỗi xacro báo `expected exactly one input file`
- **Tình trạng:** Khi chạy `ros2 launch`, báo lỗi lệnh `xacro` bị sai tham số.
- **Nguyên nhân:** Đường dẫn thư mục chứa workspace của dự án có dấu khoảng trắng (`/media/huudinh/New Volume/...`). Lệnh bash truyền thống sẽ tách khoảng trắng đó ra làm 2 tham số.
- **Cách fix:** Mở file Python launch, bọc cái biến đường dẫn vào trong **dấu ngoặc kép** thành chuỗi literal string:
  ```python
  robot_description = Command(['xacro "' + urdf_file + '"'])
  ```

### 🐞 Bug 3: Lỗi YAML parser - `Unable to parse the value... as yaml`
- **Tình trạng:** Sửa xong đường dẫn thì ROS 2 báo lỗi không parse được YAML.
- **Nguyên nhân:** Bản chất file URDF là định dạng XML. Khi nạp vào ROS 2 Humble qua Command, nó hiểu lầm đây là 1 chuỗi thông số dạng YAML và cố biên dịch, dẫn đến nổ tung.
- **Cách fix:** Dùng hàm `ParameterValue` để ép ROS 2 hiểu rằng "Ê, đây là một chuỗi văn bản (String) thuần túy, đừng có cố hiểu nó".
  ```python
  from launch_ros.parameter_descriptions import ParameterValue
  robot_description = ParameterValue(Command(['xacro "' + urdf_file + '"']), value_type=str)
  ```

### 🐞 Bug 4: Trục Camera bị ngửa lên trời thay vì chúc xuống
- **Tình trạng:** Trong RViz, mũi tên màu Đỏ (trục X) của camera bị ngóc lên trên.
- **Nguyên nhân:** Ban đầu AI truyền góc pitch là `-0.07`. Theo quy tắc bàn tay phải (quanh trục Y), giá trị ÂM sẽ ngửa mũi xe lên, giá trị DƯƠNG sẽ chúi xuống.
- **Cách fix:** Sửa `camera_pitch` trong file `.urdf.xacro` thành `+0.07`.

### 🐞 Bug 5: Giao diện RViz hiện tên các trục tọa độ bự chà bá che mất xe
- **Tình trạng:** User thấy 1 đống chữ trắng to lấn át mô hình.
- **Cách fix:** Cấu hình trên giao diện RViz: Cột bên trái > Sổ menu `TF` xuống > **Bỏ tick ô `Show Names`**. Sau đó nhấn `Ctrl + S` để lưu config lại (`fire_robot.rviz`).

---

## 5. DEBUG BẰNG LỆNH (GIẢI THÍCH CHUYÊN SÂU)

User đã tự gõ các lệnh để kiểm chứng và nhận được kết quả tốt. Đây là giải thích:

### Lệnh 1: `ros2 topic list`
- **Kết quả:** Thấy các topic `/tf`, `/tf_static`, `/robot_description`.
- **Tư duy:** Bất kỳ lúc nào bật ROS lên, lệnh đầu tiên phải gõ là `ros2 topic list` để xem "mạch máu" có đang đập không. Có mấy kênh này chứng tỏ node Launch thành công.

### Lệnh 2: `ros2 topic echo /robot_description`
- **Kết quả:** Ra 1 đống code XML trôi liên tục.
- **Tư duy:** Lệnh `echo` dùng để "bắt trộm" gói tin và in ra màn hình. Thấy code XML nghĩa là bộ sinh mô hình (robot_state_publisher) đang hoạt động và truyền file 3D đi khắp mạng.

### Lệnh 3: `ros2 run tf2_tools view_frames`
- **Kết quả:** Sinh ra file `frames.pdf`.
- **Tư duy:** Debug tối thượng khi Nav2 lỗi! Lệnh này tạo cái biểu đồ cây (Tree). Nếu bánh xe rớt khỏi khung, hoặc Lidar rớt khỏi xe, nhìn biểu đồ này sẽ thấy nó không có đường nối `parent -> child`.

### Lệnh 4: `ros2 run tf2_ros tf2_echo base_link laser_frame`
- **Kết quả:** `Translation: [0.094, 0.000, 0.183]`, `Rotation: ...`
- **Tư duy:** Để xác minh lại máy tính tính toán khoảng cách có đúng ý mình không. 
  - Đọc tọa độ: Từ tâm xe (base_link), đi thẳng tới trước 9.4cm (X=0.094), đi lên trời 18.3cm (Z=0.183) là sẽ tới Lidar. => Khớp 100% thước đo ngoài đời thực!

---

## 6. KẾT LUẬN PHASE 1
- **Trạng thái:** Hoàn thành 100%.
- **Sản phẩm:** Một gói `fire_robot_description` hoàn chỉnh.
- **Sẵn sàng:** ROS 2 bây giờ đã hiểu rõ thể xác của con robot. Nó biết Lidar nằm đâu, Camera nằm đâu để giai đoạn sau tính toán né vật cản và bắn súng nước chính xác.
- **Tiếp theo:** Bước sang Phase 2 — Kết nối Lidar và Camera thực tế trên Pi.
