"""
carla_replay.py
───────────────
Tạo video replay từ dữ liệu thu được bởi carla_capture.py (Bản cập nhật).

Yêu cầu:
    pip install opencv-python numpy pandas

Sử dụng:
    python carla_replay.py --data_dir data/session_YYYYMMDD_HHMMSS
                           [--output replay.mp4]
                           [--fps 30]

Đầu ra:
    Video .mp4 gồm ảnh gốc CARLA + overlay HUD thông số xe.
"""

import argparse
import os
import sys
import math
import cv2
import numpy as np
import pandas as pd


# ─────────────────────────── Cấu hình HUD ────────────────────────────────────
HUD_W          = 360          # Nới rộng bảng HUD để chứa text dài hơn
HUD_H          = 440          # Chiều cao HUD sau khi bỏ nhóm vận tốc thành phần
HUD_MARGIN     = 16           
HUD_ALPHA      = 0.72         
HUD_BG_COLOR   = (10, 10, 10) 
HUD_BORDER_CLR = (0, 200, 80) 
HUD_TITLE_CLR  = (0, 220, 110)
HUD_VALUE_CLR  = (220, 220, 220)
HUD_UNIT_CLR   = (130, 180, 130)
HUD_SEP_CLR    = (60, 60, 60)

FONT            = cv2.FONT_HERSHEY_DUPLEX
FONT_SM         = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE_LBL  = 0.42
FONT_SCALE_VAL  = 0.52
FONT_SCALE_TTL  = 0.50
FONT_THICK      = 1

PROGRESS_CLR    = (0, 180, 80)
PROGRESS_BG     = (50, 50, 50)
# ─────────────────────────────────────────────────────────────────────────────


def load_session(data_dir: str) -> pd.DataFrame:
    csv_path = os.path.join(data_dir, "data.csv")
    if not os.path.isfile(csv_path):
        sys.exit(f"[ERROR] Không tìm thấy file CSV: {csv_path}")
    
    df = pd.read_csv(csv_path)
    
    # Nội suy vận tốc để tính tốc độ tổng hiển thị ở tiêu đề HUD
    dt = df['time'].diff()
    df['vx'] = (df['x'].diff() / dt).fillna(0.0)
    df['vy'] = (df['y'].diff() / dt).fillna(0.0)
    df['vz'] = (df['z'].diff() / dt).fillna(0.0)

    print(f"[INFO] Đã tải {len(df)} frames từ {csv_path}")
    return df


def compute_speed(row) -> float:
    """Tốc độ km/h nội suy từ vị trí theo thời gian."""
    return math.sqrt(row.vx**2 + row.vy**2 + row.vz**2) * 3.6


# ─────────────────────────── Vẽ HUD ──────────────────────────────────────────
def draw_hud(frame: np.ndarray, row: pd.Series,
             frame_idx: int, total: int) -> np.ndarray:
    """Vẽ bảng thông số xe lên góc phải dưới của frame."""
    h, w = frame.shape[:2]
    roll_rad = row.roll
    pitch_rad = row.pitch
    yaw_rad = row.yaw
    roll_rate_rad = row.rollRate
    pitch_rate_rad = row.pitchRate
    yaw_rate_rad = row.yawRate

    # Vị trí góc trên-trái của HUD
    hud_x = w - HUD_W - HUD_MARGIN
    hud_y = h - HUD_H - HUD_MARGIN

    # ── Nền bán trong suốt ────────────────────────────────────────────────
    overlay = frame.copy()
    cv2.rectangle(overlay,
                  (hud_x, hud_y),
                  (hud_x + HUD_W, hud_y + HUD_H),
                  HUD_BG_COLOR, -1)
    cv2.addWeighted(overlay, HUD_ALPHA, frame, 1 - HUD_ALPHA, 0, frame)

    # ── Viền ──────────────────────────────────────────────────────────────
    cv2.rectangle(frame,
                  (hud_x, hud_y),
                  (hud_x + HUD_W, hud_y + HUD_H),
                  HUD_BORDER_CLR, 1)

    # ── Tiêu đề ───────────────────────────────────────────────────────────
    speed_kmh = compute_speed(row)
    title = f"VEHICLE DATA   {speed_kmh:5.1f} km/h"
    cv2.putText(frame, title,
                (hud_x + 10, hud_y + 20),
                FONT, FONT_SCALE_TTL, HUD_TITLE_CLR, FONT_THICK, cv2.LINE_AA)

    # Đường kẻ dưới tiêu đề
    cv2.line(frame,
             (hud_x + 8,       hud_y + 28),
             (hud_x + HUD_W - 8, hud_y + 28),
             HUD_SEP_CLR, 1)

    # ── Nhóm thông số (Đã tích hợp dữ liệu mới) ───────────────────────────
    groups = [
        ("POSITION",    [("X", f"{row.x:+10.4f}", "m"),
                         ("Y", f"{row.y:+10.4f}", "m"),
                         ("Z", f"{row.z:+10.4f}", "m")]),
        ("ROTATION",    [("Roll",  f"{roll_rad:+10.4f}",  "rad"),
                         ("Pitch", f"{pitch_rad:+10.4f}", "rad"),
                         ("Yaw",   f"{yaw_rad:+10.4f}",   "rad")]),
        ("ANGULAR VEL", [("Roll R.",  f"{roll_rate_rad:+10.4f}", "rad/s"),
                         ("Pitch R.", f"{pitch_rate_rad:+10.4f}", "rad/s"),
                         ("Yaw R.",   f"{yaw_rate_rad:+10.4f}",   "rad/s")]),
        ("ACCEL IMU",   [("Ax", f"{row.ax:+10.4f}", "m/s²"),
                         ("Ay", f"{row.ay:+10.4f}", "m/s²"),
                         ("Az", f"{row.az:+10.4f}", "m/s²")]),
        ("CONTROL",     [("Throttle", f"{row.throttle:10.4f}", ""),
                         ("Brake",    f"{row.brake:10.4f}", ""),
                         ("Steer",    f"{row.steer:+10.4f}", ""),
                         ("Gear",     f"{int(row.gear):>10}", "")])
    ]

    LINE_H    = 17          # pixel mỗi dòng
    GRP_PAD   = 6           # padding trên mỗi nhóm
    COL_LBL   = hud_x + 12
    COL_VAL   = hud_x + 110 # Dịch nhẹ sang phải để chứa nhãn dài
    COL_UNIT  = hud_x + 232 # Dịch nhẹ sang phải

    cur_y = hud_y + 45

    for grp_title, items in groups:
        # Header nhóm
        cv2.putText(frame, grp_title,
                    (COL_LBL, cur_y),
                    FONT_SM, 0.34, HUD_TITLE_CLR, 1, cv2.LINE_AA)
        cur_y += LINE_H - 2

        for label, val_str, unit in items:
            cv2.putText(frame, f"{label}",
                        (COL_LBL, cur_y),
                        FONT_SM, FONT_SCALE_LBL, (160, 160, 160), 1, cv2.LINE_AA)
            cv2.putText(frame, val_str,
                        (COL_VAL, cur_y),
                        FONT, FONT_SCALE_VAL, HUD_VALUE_CLR, 1, cv2.LINE_AA)
            cv2.putText(frame, unit,
                        (COL_UNIT, cur_y),
                        FONT_SM, 0.36, HUD_UNIT_CLR, 1, cv2.LINE_AA)
            cur_y += LINE_H

        cur_y += GRP_PAD

    # ── Thanh tiến trình ──────────────────────────────────────────────────
    prog_x1 = hud_x + 8
    prog_x2 = hud_x + HUD_W - 8
    prog_y  = hud_y + HUD_H - 12

    cv2.rectangle(frame, (prog_x1, prog_y - 5),
                  (prog_x2, prog_y + 3), PROGRESS_BG, -1)

    progress = (frame_idx + 1) / max(total, 1)
    fill_x   = int(prog_x1 + (prog_x2 - prog_x1) * progress)
    cv2.rectangle(frame, (prog_x1, prog_y - 5),
                  (fill_x, prog_y + 3), PROGRESS_CLR, -1)

    # Frame counter
    cv2.putText(frame, f"{frame_idx + 1}/{total}",
                (hud_x + HUD_W//2 - 28, prog_y - 9),
                FONT_SM, 0.34, (200, 200, 200), 1, cv2.LINE_AA)

    return frame


# ─────────────────────────── Watermark timestamp ──────────────────────────────
def draw_timestamp(frame: np.ndarray, row: pd.Series) -> np.ndarray:
    # [CẬP NHẬT] Đổi row.timestamp thành row.time
    txt = f"t = {row.time:.3f} s   Frame {int(row.frame_id):06d}"
    cv2.putText(frame, txt, (14, 28),
                FONT_SM, 0.52, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, txt, (14, 28),
                FONT_SM, 0.52, (230, 230, 230), 1, cv2.LINE_AA)
    return frame


# ─────────────────────────── Thanh tiến trình CLI ────────────────────────────
def cli_progress(cur, total, width=40):
    ratio = cur / max(total, 1)
    filled = int(width * ratio)
    bar = "█" * filled + "░" * (width - filled)
    print(f"\r  [{bar}] {cur:>5}/{total}  ({ratio*100:.1f}%)", end="", flush=True)


# ─────────────────────────── Main ─────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Replay CARLA session data thành video (Background rendering)."
    )
    parser.add_argument("--data_dir",   required=True,
                        help="Thư mục session (chứa images/ và data.csv)")
    parser.add_argument("--output",     default="replay.mp4",
                        help="Tên file video đầu ra (mặc định: replay.mp4)")
    parser.add_argument("--fps",        type=float, default=30.0,
                        help="Frame rate video (mặc định: 30)")
    
    args = parser.parse_args()

    data_dir   = args.data_dir
    images_dir = os.path.join(data_dir, "images")

    if not os.path.isdir(images_dir):
        sys.exit(f"[ERROR] Không tìm thấy thư mục ảnh: {images_dir}")

    df = load_session(data_dir)
    total = len(df)

    if total == 0:
        sys.exit("[ERROR] CSV không có dữ liệu.")

    # Tìm frame hợp lệ đầu tiên để đo kích thước ảnh
    vid_h, vid_w = 720, 1280
    for _, r in df.iterrows():
        if pd.notna(r.image_file):
            sample_img = os.path.join(images_dir, str(r.image_file))
            if os.path.isfile(sample_img):
                sample = cv2.imread(sample_img)
                vid_h, vid_w = sample.shape[:2]
                break
                
    print(f"[INFO] Kích thước video: {vid_w}x{vid_h}  FPS: {args.fps}")

    # ── Khởi tạo VideoWriter ──────────────────────────────────────────────
    output_path = args.output
    fourcc      = cv2.VideoWriter_fourcc(*"mp4v")
    writer      = cv2.VideoWriter(output_path, fourcc, args.fps,
                                  (vid_w, vid_h))
    if not writer.isOpened():
        sys.exit(f"[ERROR] Không thể tạo file video: {output_path}")

    print(f"[INFO] Đang dựng video ngầm, vui lòng đợi → {output_path}")
    print()

    # ── Duyệt từng frame ──────────────────────────────────────────────────
    missing = 0
    for idx, row in df.iterrows():
        # Bỏ qua nếu dòng này thiếu tên file ảnh (NaN)
        if pd.isna(row.image_file):
            missing += 1
            continue

        img_path = os.path.join(images_dir, str(row.image_file))

        if not os.path.isfile(img_path):
            missing += 1
            continue

        frame = cv2.imread(img_path)

        # Vẽ HUD + timestamp
        frame = draw_hud(frame, row, idx, total)
        frame = draw_timestamp(frame, row)

        # Ghi frame vào video
        writer.write(frame)

        # Cập nhật thanh tiến trình
        cli_progress(idx + 1, total)

    print()
    writer.release()

    print(f"\n[DONE] Video đã lưu thành công: {os.path.abspath(output_path)}")
    if missing > 0:
        print(f"[WARN] Đã bỏ qua {missing} frames do thiếu dữ liệu hoặc không tìm thấy file ảnh.")


if __name__ == "__main__":
    main()
