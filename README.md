# CARLA Data Collector & Replay

Bộ công cụ này dùng để:

1. Thu thập dữ liệu chạy xe trong [CARLA Simulator](https://carla.org/) bằng tay, Autopilot hoặc kịch bản CSV.
2. Replay lại session đã lưu thành video `.mp4` với HUD hiển thị trạng thái xe.

Hai script chính:

1. `carla_capture.py`: spawn xe, hiển thị camera live, ghi ảnh + dữ liệu pose/IMU/control.
2. `carla_replay.py`: đọc `data.csv` và `images/`, dựng video replay có HUD và timestamp.

---

## Yêu cầu hệ thống

- CARLA Simulator đang chạy, mặc định tại `localhost:2000`.
- Python 3.7+.
- Gói Python cần thiết:

```bash
pip install carla pygame opencv-python numpy pandas
```

Lưu ý: phiên bản `carla` cài qua `pip` phải tương thích với bản CARLA server đang dùng.

---

## Chạy CARLA server

```bash
cd Carla_folder
./CarlaUE4.sh -prefernvidia
```

---

## Thu thập dữ liệu với `carla_capture.py`

Script hỗ trợ 3 chế độ:

1. `MANUAL`: lái bằng bàn phím.
2. `AUTOPILOT`: bật autopilot ngay từ đầu bằng `-a`.
3. `SCRIPT`: chạy tự động theo file kịch bản CSV bằng `--script`; chế độ này tự bắt đầu ghi ngay khi khởi chạy.

Ngoài việc spawn xe theo `--filter`, script hiện sẽ ưu tiên tìm một đoạn đường thẳng dài, căn xe vào giữa làn rồi mới bắt đầu ghi.

### Cú pháp cơ bản

```bash
python3 carla_capture.py
```

### Tham số dòng lệnh

| Tham số | Mặc định | Mô tả |
| :--- | :--- | :--- |
| `--host` | `localhost` | Địa chỉ CARLA server. |
| `-p`, `--port` | `2000` | Port CARLA server. |
| `--res` | `1280x720` | Độ phân giải cửa sổ và ảnh camera. |
| `--filter` | `vehicle.tesla.model3` | Filter blueprint xe để spawn. |
| `--map` | `Town04` | Map sẽ được load khi khởi chạy. |
| `--weather` | `ClearNoon` | Preset thời tiết của CARLA. |
| `-a`, `--autopilot` | `False` | Bật autopilot từ đầu. |
| `--sync` | `False` | Bật synchronous mode. |
| `--hz` | `100` | Tần số tick/ghi khi dùng sync mode hoặc game loop. |
| `--script` | `None` | Đường dẫn tới file kịch bản CSV để chạy chế độ script. |

### Ví dụ

Chạy manual/autopilot:

```bash
python3 carla_capture.py --map Town04 --filter vehicle.tesla.model3 --weather ClearNoon --res 1920x1080 --sync --hz 100
```

Chạy theo kịch bản:

```bash
python3 carla_capture.py --script scenario.csv --sync --hz 100
```

### Phím điều khiển

#### Manual / Autopilot

| Phím | Chức năng |
| :--- | :--- |
| `W / Up` | Ga |
| `S / Down` | Phanh |
| `A / Left` | Lái trái |
| `D / Right` | Lái phải |
| `Q` | Đảo chiều tiến/lùi |
| `Space` | Phanh tay |
| `P` | Bật/tắt Autopilot |
| `M` | Chuyển hộp số tay/tự động |
| `, / .` | Giảm / tăng số khi ở hộp số tay |
| `F5` | Bắt đầu ghi |
| `F6` | Tạm dừng ghi |
| `F7` | Tiếp tục ghi |
| `F8` | Dừng và lưu |
| `TAB` | Đổi góc camera |
| `ESC` hoặc `Ctrl+Q` | Thoát |

#### Script mode

- Khi dùng `--script`, script sẽ tự tạo session và tự bắt đầu ghi.
- Có thể dùng `F6` để tạm dừng, `F7` để tiếp tục, `F8` để dừng sớm.
- Khi kịch bản chạy xong, chương trình tự lưu dữ liệu và tự thoát.

### Định dạng file kịch bản `scenario.csv`

`carla_capture.py` đọc file CSV với các cột:

| Cột | Ý nghĩa |
| :--- | :--- |
| `t_start` | Thời điểm bắt đầu phase (giây) |
| `t_end` | Thời điểm kết thúc phase (giây) |
| `target_speed_kmh` | Tốc độ mục tiêu |
| `steer_deg` | Góc lái mục tiêu theo độ |
| `zigzag_period` | Chu kỳ đổi dấu góc lái; `0` nếu không zigzag |

Ví dụ:

```csv
t_start,t_end,target_speed_kmh,steer_deg,zigzag_period
0,8,20,0,0
8,10,20,-10,0
10,12,30,10,0
```

Script mode hiện dùng bộ điều khiển PID để bám tốc độ mục tiêu, và chuẩn hóa góc lái theo giới hạn `45` độ.

---

## Tạo video replay với `carla_replay.py`

Script replay đọc session đã lưu, chèn HUD vào từng frame rồi ghi ra video `.mp4`.

HUD hiện hiển thị:

- Position: `x`, `y`, `z`
- Rotation: `roll`, `pitch`, `yaw` (hiển thị trên HUD dưới đơn vị `rad`)
- Angular velocity: `rollRate`, `pitchRate`, `yawRate` (hiển thị trên HUD dưới đơn vị `rad/s`)
- IMU acceleration: `ax`, `ay`, `az`
- Control: `throttle`, `brake`, `steer`, `gear`

### Cú pháp

```bash
python3 carla_replay.py --data_dir data/session_YYYYMMDD_HHMMSS
```

### Tham số dòng lệnh

| Tham số | Bắt buộc | Mặc định | Mô tả |
| :--- | :---: | :--- | :--- |
| `--data_dir` | Có | - | Thư mục session chứa `data.csv` và `images/`. |
| `--output` | Không | `replay.mp4` | File video đầu ra. |
| `--fps` | Không | `30.0` | FPS của video replay. |

### Ví dụ

```bash
python3 carla_replay.py --data_dir data/session_20260401_101048 --output replay.mp4 --fps 30
```

---

## Cấu trúc dữ liệu đầu ra

Mỗi lần bắt đầu ghi, script tạo một thư mục session dạng:

```text
data/
└── session_YYYYMMDD_HHMMSS/
    ├── images/
    │   ├── frame_000001.png
    │   ├── frame_000002.png
    │   └── ...
    └── data.csv
```

### Schema `data.csv`

File CSV hiện có các cột:

```text
frame_id,time,image_file,
x,y,z,
roll,pitch,yaw,
rollRate,pitchRate,yawRate,
ax,ay,az,
throttle,brake,steer,gear
```

Ý nghĩa:

- `frame_id`: số thứ tự frame đã ghi.
- `time`: thời gian mô phỏng tính từ lúc bắt đầu ghi.
- `image_file`: tên file ảnh tương ứng trong `images/`.
- `x, y, z`: vị trí xe.
- `roll, pitch, yaw`: góc quay xe.
- `rollRate, pitchRate, yawRate`: vận tốc góc lấy từ `vehicle.get_angular_velocity()`.
- `ax, ay, az`: gia tốc từ IMU gắn gần vị trí đầu người lái.
- `throttle, brake, steer, gear`: lệnh điều khiển xe tại thời điểm ghi.
