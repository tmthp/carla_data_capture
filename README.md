# 🚗 CARLA Data Collector & Replay

Bộ công cụ thu thập dữ liệu tự động và thủ công cho [CARLA Simulator](https://carla.org/), đi kèm với tiện ích tạo video replay trực quan.

Bộ công cụ gồm 2 module chính:
1. `carla_capture.py`: Điều khiển xe (thủ công hoặc Autopilot) và ghi lại dữ liệu cảm biến (Hình ảnh camera, Vận tốc, IMU, Góc lái, Chân ga, Phanh,...).
2. `carla_replay.py`: Đọc dữ liệu đã lưu và render ra video `.mp4` có chèn bảng thông số (HUD) hiển thị trạng thái xe.

---

## 📋 Yêu cầu hệ thống

- Đã cài đặt và đang chạy **CARLA Simulator** (thường ở port `2000`).
- **Python:** 3.7 trở lên.

## 🛠 Cài đặt

Mở terminal hoặc command prompt và chạy lệnh sau để cài đặt các thư viện Python cần thiết:

    pip install carla pygame opencv-python numpy pandas

> **Lưu ý:** Phiên bản thư viện `carla` cài đặt qua pip cần tương thích với phiên bản CARLA Simulator bạn đang sử dụng.

---

## 🚀 Hướng dẫn sử dụng

### 1. Chạy Carla Server 

    cd Carla_folder && ./CarlaUE4.sh -prefernvidia

### 2. Thu thập dữ liệu (`carla_capture.py`)

Chắc chắn rằng CARLA server đang chạy, sau đó mở một terminal mới khởi chạy script thu thập dữ liệu:

    python3 carla_capture.py

#### 📌 Các tham số tùy chọn (Arguments)

| Tham số | Ví dụ | Mô tả |
| :--- | :--- | :--- |
| `--map` | `--map Town04` | Chọn bản đồ để tải (VD: `Town01`, `Town02`). Bỏ qua nếu muốn dùng map hiện tại. |
| `--filter` | `--filter vehicle.tesla.model3` | Lọc và chọn loại xe muốn spawn. |
| `--weather` | `--weather ClearNoon` | Chọn thời tiết (VD: `ClearNoon`, `HardRainSunset`, `WetCloudyNoon`). |
| `--res` | `--res 1920x1080` | Độ phân giải cửa sổ hiển thị và ảnh thu thập (Mặc định: `1280x720`). |
| `-a`, `--autopilot` | `-a` | Bật chế độ lái tự động (Autopilot) ngay khi khởi chạy. |
| `--sync` | `--sync` | Bật chế độ Synchronous mode, giúp ghi data ổn định, không bị rớt frame. |

**Ví dụ chạy với đầy đủ tham số:**

    python3 carla_capture.py --map Town04 --filter vehicle.audi.etron --weather HardRainSunset --res 1920x1080 --sync

#### ⌨️ Phím tắt điều khiển

| Phím | Chức năng | Phím | Chức năng |
| :--- | :--- | :--- | :--- |
| `W / A / S / D` | Ga, Phanh, Lái Trái, Lái Phải | `F5` | **▶ BẮT ĐẦU** ghi dữ liệu |
| `Q` | Chuyển số Tiến / Lùi | `F6` | **⏸ TẠM DỪNG** ghi |
| `Space` | Phanh tay | `F7` | **▶ TIẾP TỤC** ghi |
| `P` | Bật/Tắt Autopilot | `F8` | **⏹ KẾT THÚC** & Lưu dữ liệu |
| `M` | Đổi hộp số Tay / Tự động | `TAB` | Đổi góc nhìn Camera |
| `, / .` | Giảm / Tăng số (hộp số tay) | `ESC` | Thoát chương trình |

---

### 3. Tạo video Replay (`carla_replay.py`)

Sau khi thu thập dữ liệu, sử dụng script này để render ra một video chứa hình ảnh từ camera và bảng HUD thông số xe. Quá trình render chạy ngầm và hiển thị thanh tiến trình trên terminal.

**Cú pháp cơ bản:**

    python3 carla_replay.py --data_dir <đường_dẫn_tới_thư_mục_session>

#### 📌 Các tham số tùy chọn (Arguments)

| Tham số | Bắt buộc | Mặc định | Mô tả |
| :--- | :---: | :--- | :--- |
| `--data_dir` | **Có** | - | Đường dẫn tới thư mục chứa file `data.csv` và thư mục `images/`. |
| `--output` | Không | `replay.mp4` | Tên/Đường dẫn file video đầu ra. |
| `--fps` | Không | `30.0` | Tốc độ khung hình (Frame rate) của video. |

**Ví dụ:**

    python3 carla_replay.py --data_dir data/session_20260331_143137 --output my_replay.mp4 --fps 30

---

## 📂 Cấu trúc thư mục dữ liệu đầu ra

Sau khi bấm `F8` để lưu dữ liệu trong `carla_capture.py`, hệ thống sẽ tự động tạo thư mục theo cấu trúc thời gian như sau:

    data/
    └── session_YYYYMMDD_HHMMSS/
        ├── images/                 # Thư mục chứa các frame ảnh (.png)
        │   ├── frame_000001.png
        │   ├── frame_000002.png
        │   └── ...
        └── data.csv                # File chứa log thông số (vận tốc, góc lái, ga, phanh,...)