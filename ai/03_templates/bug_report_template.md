# 🐞 BÁO CÁO LỖI KỸ THUẬT: [MÃ LỖI]

## 1. Triệu chứng (Symptom)
- **Hiện tượng:** [Mô tả robot bị làm sao]
- **Tần suất:** [Luôn xảy ra / Ngẫu nhiên / Chỉ khi ...]
- **Môi trường:** [Pin 12V / USB debug / micro-ROS connected / ...]
- **State Machine:** [IDLE / AUTO / MANUAL / EMERGENCY]

## 2. Phân tích nguyên nhân gốc (Root Cause)
- **Tên file / Dòng code:** [Ví dụ: `lib/service/PIDController.cpp` dòng 45]
- **Module liên quan:** [Driver / Service / App]
- **Lý do kỹ thuật:** [Ví dụ: Tràn số khi tính toán tích phân vì thiếu Anti-windup]

## 3. Giải pháp Khắc phục
- **Thay đổi:** [Mô tả đoạn code đã sửa]
- **File bị ảnh hưởng:** [Liệt kê tất cả file đã sửa]
- **Side-effect:** [Có ảnh hưởng module nào khác không?]

## 4. Kiểm chứng (Verification)
- **Cách test:** [Ví dụ: Chạy xe đi thẳng 1m, đọc log PID]
- **Kết quả mong đợi:** [Ví dụ: PWM không vượt 255, không âm]
- **Kết quả thực tế:** [Điền sau khi test]

## 5. Bài học rút ra (Learning)
- [Ví dụ: Luôn clamp giá trị PWM trước khi ghi ra phần cứng]