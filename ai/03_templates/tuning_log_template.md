# 📈 NHẬT KÝ TINH CHỈNH: [TÊN THAM SỐ]

- **Ngày thực hiện:** [DD/MM/YYYY]
- **Module:** [Ví dụ: PID tốc độ bánh xe / Odometry / Complementary Filter]
- **Firmware version:** [Ví dụ: commit hash hoặc ngày build]
- **Điều kiện test:** [Mặt sàn / Tốc độ target / Tải nặng hay không]

## Bảng kết quả

| Lần | Tham số | Kết quả | Overshoot | Settle time | Ghi chú |
|:---|:---|:---|:---|:---|:---|
| #01 | Kp=1.0, Ki=0.1, FF=25 | Robot giật | ~30% | >2s | P quá cao |
| #02 | Kp=0.8, Ki=0.1, FF=25 | Mượt hơn | ~15% | 1.5s | Tốt hơn |
| #03 | Kp=0.8, Ki=0.2, FF=25 | Ổn định | ~10% | 0.8s | ✅ Tốt nhất |
| ... | ... | ... | ... | ... | ... |

## Kết luận
- **Tham số chọn:** Lần thử #[...]
- **Lý do:** [...]
- **Đã cập nhật vào:** `lib/common/RobotConfig.h` dòng [...]

## Log Serial (đính kèm nếu có)
```
[PID] FL: sp=5.0 act=4.8 err=0.2 pwm=128
[PID] FR: sp=5.0 act=5.1 err=-0.1 pwm=122
```