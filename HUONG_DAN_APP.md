# AeroScope — xem dữ liệu IMU và BMP388

Ứng dụng PC đọc dữ liệu CSV từ ESP32 qua cổng Serial. Ứng dụng tương thích với
các dòng `IMU,...` và `BMP,...` mà firmware trong `src/` đang gửi.

## Chạy trên Windows

1. Nạp firmware vào ESP32 và nối cảm biến như cấu hình trong project.
2. Cài Python 3.10 trở lên nếu máy chưa có.
3. Mở PowerShell tại thư mục project và chạy:

   ```powershell
   py -m pip install -r requirements-app.txt
   py app.py
   ```

4. Chọn cổng COM của ESP32, sau đó nhấn **KẾT NỐI**. Đóng Serial Monitor
   trong PlatformIO trước khi kết nối vì mỗi cổng COM chỉ mở được bởi một app.

## Dữ liệu hiển thị

- Roll và pitch được ước lượng bằng bộ lọc kết hợp gia tốc kế và con quay hồi
  chuyển qua bộ lọc AHRS Fusion; bộ lọc tự theo dõi bias gyro và loại bớt mẫu
  gia tốc/từ trường bị nhiễu.
- **ĐẶT YAW = 0°** lấy hướng hiện tại làm mốc yaw mới.
- Khi vừa kết nối, giữ cảm biến đứng yên khoảng 3 giây để bộ lọc lấy bias gyro.
- Nhấn **HIỆU CHUẨN TỪ KẾ**, xoay cảm biến chậm đủ mọi hướng trong 15–30 giây,
  rồi nhấn **LƯU HIỆU CHUẨN**. Hiệu chuẩn QMC6309 được lưu trong
  `aeroscope_mag_calibration.json`.
- Mô hình drone 3D và biểu đồ chuyển động cập nhật theo dữ liệu IMU.
- BMP388 hiển thị áp suất, nhiệt độ và độ cao ước tính theo mốc áp suất biển
  `1013.25 hPa` được cấu hình trong firmware.
- Hiệu chuẩn giúp giảm trôi yaw; không thể đảm bảo hết trôi khi thiếu dữ liệu từ
  kế hoặc khi QMC6309 bị nhiễu bởi nam châm, kim loại hay dòng điện lớn.

Thuật toán AHRS và bias gyro dựa trên thư viện [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion)
và ví dụ Python [advanced_example.py](https://github.com/xioTechnologies/Fusion/blob/main/Examples/Python/advanced_example.py).
