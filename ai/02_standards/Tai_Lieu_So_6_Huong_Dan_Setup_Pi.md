# HƯỚNG DẪN CẤU HÌNH RASPBERRY PI — WIFI ACCESS POINT + ROS 2

## 1. CÀI ĐẶT HỆ ĐIỀU HÀNH

### 1.1. Flash Ubuntu Server
- **OS:** Ubuntu Server 22.04 LTS (64-bit) cho Raspberry Pi
- **Tool:** Raspberry Pi Imager hoặc Balena Etcher
- **Lưu ý:** Dùng Server (không GUI) để tiết kiệm tài nguyên cho ROS 2

### 1.2. Cấu hình ban đầu (headless)
```bash
# Trước khi boot, tạo file trong boot partition:
# 1. SSH: tạo file rỗng tên "ssh"
# 2. WiFi tạm (để SSH vào lần đầu): tạo file "network-config"
```

---

## 2. CÀI ĐẶT ROS 2 HUMBLE

```bash
# Thêm ROS 2 repo
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Cài ROS 2 Humble (ros-base, không GUI)
sudo apt update
sudo apt install ros-humble-ros-base python3-colcon-common-extensions

# Thêm vào ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Cài các package cần thiết
sudo apt install ros-humble-rplidar-ros \
                 ros-humble-usb-cam \
                 ros-humble-micro-ros-agent \
                 ros-humble-rosbridge-server \
                 ros-humble-image-transport-plugins
```

---

## 3. CẤU HÌNH WIFI ACCESS POINT

### 3.1. Cài hostapd + dnsmasq
```bash
sudo apt install hostapd dnsmasq
sudo systemctl stop hostapd
sudo systemctl stop dnsmasq
```

### 3.2. Cấu hình IP tĩnh cho wlan0
```bash
# /etc/dhcpcd.conf (thêm cuối file)
interface wlan0
    static ip_address=10.0.0.1/24
    nohook wpa_supplicant
```

### 3.3. Cấu hình DHCP Server (dnsmasq)
```bash
# Backup file gốc
sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.bak

# Tạo file mới: /etc/dnsmasq.conf
interface=wlan0
dhcp-range=10.0.0.10,10.0.0.50,255.255.255.0,24h
```

### 3.4. Cấu hình Access Point (hostapd)
```bash
# /etc/hostapd/hostapd.conf
interface=wlan0
driver=nl80211
ssid=FireRobot_AP
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=firerobot2024
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
```

```bash
# Khai báo config file cho hostapd
# /etc/default/hostapd
DAEMON_CONF="/etc/hostapd/hostapd.conf"
```

### 3.5. Bật dịch vụ
```bash
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo systemctl enable dnsmasq
sudo reboot
```

### 3.6. Verify
```
✅ Laptop thấy WiFi "FireRobot_AP" trong danh sách
✅ Kết nối → Laptop nhận IP 10.0.0.x
✅ ping 10.0.0.1 → reply < 10ms
✅ ssh pi@10.0.0.1 → đăng nhập thành công
```

---

## 4. CẤU HÌNH ROS 2 NETWORK

### 4.1. Thiết lập trên Pi
```bash
# Thêm vào ~/.bashrc
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### 4.2. Thiết lập trên Laptop
```bash
# Thêm vào ~/.bashrc (SAU KHI kết nối WiFi FireRobot_AP)
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### 4.3. Verify ROS 2 Discovery
```bash
# Trên Pi:
ros2 run demo_nodes_cpp talker

# Trên Laptop:
ros2 run demo_nodes_cpp listener
# → Laptop nhận được message từ Pi

# Kiểm tra:
ros2 node list  # → thấy node từ cả Pi và Laptop
ros2 topic list # → thấy topic từ cả Pi và Laptop
```

---

## 5. CẤU HÌNH USB DEVICES

### 5.1. Phân biệt USB devices
```bash
# Liệt kê USB devices
ls /dev/ttyUSB*  # → Lidar (thường ttyUSB0)
ls /dev/ttyACM*  # → ESP32-S3 (thường ttyACM0)
ls /dev/video*   # → Webcam (thường video0)
```

### 5.2. Tạo udev rules (cố định tên device)
```bash
# /etc/udev/rules.d/99-firerobot.rules
# Lidar
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar"
# ESP32-S3
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", SYMLINK+="esp32"
# Webcam
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", SYMLINK+="webcam"
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
# → ls /dev/lidar /dev/esp32 /dev/webcam
```

### 5.3. Quyền truy cập
```bash
sudo usermod -a -G dialout $USER  # Serial (Lidar + ESP32)
sudo usermod -a -G video $USER    # Camera
# Reboot để áp dụng
```

---

## 6. QUẢN LÝ TÀI NGUYÊN (Cgroups)

```bash
# Giới hạn CPU cho usb_cam_node (tránh chiếm hết tài nguyên)
# Tạo cgroup:
sudo cgcreate -g cpu:/camera_limit
sudo cgset -r cpu.cfs_quota_us=50000 camera_limit  # Giới hạn 50% 1 core

# Chạy node trong cgroup:
sudo cgexec -g cpu:camera_limit ros2 run usb_cam usb_cam_node_exe
```

---

## 7. AUTO-START KHI BOOT (systemd)

```bash
# /etc/systemd/system/firerobot.service
[Unit]
Description=Fire Robot ROS 2 Bringup
After=network.target

[Service]
Type=simple
User=pi
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch fire_robot_bringup robot.launch.py"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable firerobot.service
sudo systemctl start firerobot.service
```

---

## 8. TIÊU CHÍ NGHIỆM THU SETUP PI

| Hạng mục | Chỉ tiêu | Lệnh kiểm tra |
|:---|:---|:---|
| WiFi AP | Laptop kết nối, IP 10.0.0.x | `ping 10.0.0.1` |
| ROS 2 Discovery | Pi ↔ Laptop thấy nhau | `ros2 node list` từ Laptop |
| Lidar | `/scan` có data | `ros2 topic hz /scan` → ~10Hz |
| Camera | `/image_raw/compressed` có ảnh | `rqt_image_view` |
| micro-ROS | ESP32 node connected | `ros2 node list` → `/motion_slave` |
| CPU | < 70% full load | `htop` |
| Ping | < 10ms | `ping 10.0.0.1` |
| Boot tự động | ROS 2 chạy sau reboot | `sudo reboot` → chờ → `ros2 node list` |
