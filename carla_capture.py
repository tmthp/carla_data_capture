#!/usr/bin/env python
"""
carla_capture.py  ─  Thu thập dữ liệu CARLA + Manual / Autopilot / Script control
═══════════════════════════════════════════════════════════════════════════════════

Điều khiển xe (giống manual_control.py gốc):
──────────────────────────────────────────
  W / ↑          : ga (throttle)
  S / ↓          : phanh (brake)
  A / ←          : lái trái
  D / →          : lái phải
  Q              : đảo chiều (reverse)
  Space          : phanh tay (hand-brake)
  P              : bật/tắt Autopilot
  M              : chuyển hộp số tay/tự động
  , / .          : xuống/lên số (hộp số tay)

Thu thập dữ liệu:
──────────────────────────────────────────
  F5             : Bắt đầu thu thập (Start)
  F6             : Tạm dừng (Pause)
  F7             : Tiếp tục (Resume)
  F8             : Dừng & Lưu (Stop + Save)

Khác:
──────────────────────────────────────────
  TAB            : Chuyển góc camera
  ESC / Ctrl+Q   : Thoát

Sử dụng:
    python carla_capture.py --script scenario.csv --hz 100 --sync
"""

from __future__ import print_function

import carla
import argparse
import csv
import math
import os
import random
import re
import sys
import threading
import time as _time
import weakref
from datetime import datetime as dt

try:
    import pygame
    from pygame.locals import (
        KMOD_CTRL, KMOD_SHIFT,
        K_COMMA, K_PERIOD, K_ESCAPE, K_SPACE, K_TAB,
        K_F5, K_F6, K_F7, K_F8,
        K_LEFT, K_RIGHT, K_UP, K_DOWN,
        K_a, K_c, K_d, K_f, K_m, K_p, K_q, K_s, K_w,
    )
except ImportError:
    raise RuntimeError("Cài đặt pygame trước:  pip install pygame")

try:
    import numpy as np
except ImportError:
    raise RuntimeError("Cài đặt numpy trước:  pip install numpy")

try:
    import cv2
except ImportError:
    raise RuntimeError("Cài đặt opencv trước:  pip install opencv-python")


# ═══════════════════════════════════════════════════════════════════════════════
# Cấu hình
# ═══════════════════════════════════════════════════════════════════════════════
CARLA_HOST    = "localhost"
CARLA_PORT    = 2000
CARLA_TIMEOUT = 10.0

WIN_W, WIN_H  = 1280, 720
CAM_FOV       = 90

OUTPUT_ROOT   = "data"
IMAGES_DIR    = "images"
CSV_FILENAME  = "data.csv"

# roll, pitch, yaw lấy từ transform của actor IMU.
# rollRate, pitchRate, yawRate lấy từ gyroscope của IMU.
CSV_HEADER = [
    "frame_id", "time", "image_file",
    "x", "y", "z",
    "roll", "pitch", "yaw",
    "rollRate", "pitchRate", "yawRate",
    "ax", "ay", "az",
    "throttle", "brake", "steer", "gear"
]

# ═══════════════════════════════════════════════════════════════════════════════
# Tiện ích
# ═══════════════════════════════════════════════════════════════════════════════
def find_weather_presets():
    rgx   = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name  = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    names = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in names]

# ═══════════════════════════════════════════════════════════════════════════════
# DataWriter
# ═══════════════════════════════════════════════════════════════════════════════
class DataWriter:
    def __init__(self, session_dir: str):
        self.images_path = os.path.join(session_dir, IMAGES_DIR)
        os.makedirs(self.images_path, exist_ok=True)
        self._csv_file   = open(
            os.path.join(session_dir, CSV_FILENAME), "w", newline="", encoding="utf-8"
        )
        self._writer     = csv.DictWriter(self._csv_file, fieldnames=CSV_HEADER)
        self._writer.writeheader()
        self.saved_count = 0

    def write(self, frame_id, time_elapsed, img_bgr, imu, transform, control):
        img_name = f"frame_{frame_id:06d}.png"
        cv2.imwrite(os.path.join(self.images_path, img_name), img_bgr)
        loc = transform.location
        self._writer.writerow({
            "frame_id":   frame_id,
            "time":       f"{time_elapsed:.6f}",
            "image_file": img_name,
            "x":        f"{loc.x:.6f}", "y": f"{loc.y:.6f}", "z": f"{loc.z:.6f}",
            "roll":     f"{imu['roll']:.6f}", "pitch": f"{imu['pitch']:.6f}",
            "yaw":      f"{imu['yaw']:.6f}",
            "rollRate":  f"{imu['rollRate']:.6f}",
            "pitchRate": f"{imu['pitchRate']:.6f}",
            "yawRate":   f"{imu['yawRate']:.6f}",
            "ax":    f"{imu['ax']:.6f}", "ay": f"{imu['ay']:.6f}",
            "az":    f"{imu['az']:.6f}",
            "throttle": f"{control.throttle:.6f}",
            "brake":    f"{control.brake:.6f}",
            "steer":    f"{control.steer:.6f}",
            "gear":     f"{control.gear}"
        })
        self._csv_file.flush()
        self.saved_count += 1

    def close(self):
        if not self._csv_file.closed:
            self._csv_file.close()

# ═══════════════════════════════════════════════════════════════════════════════
# IMUSensor
# ═══════════════════════════════════════════════════════════════════════════════
class IMUSensor:
    def __init__(self, parent_actor):
        self._lock  = threading.Lock()
        self.roll = self.pitch = self.yaw = 0.0
        self.roll_rate = self.pitch_rate = self.yaw_rate = 0.0
        self.ax = self.ay = self.az = 0.0
        world      = parent_actor.get_world()
        bp         = world.get_blueprint_library().find("sensor.other.imu")
        
        # Vị trí IMU ở đầu người lái
        driver_head_transform = carla.Transform(carla.Location(x=0.4, y=-0.4, z=1.2))
        
        self.sensor = world.spawn_actor(bp, driver_head_transform, attach_to=parent_actor)
        weak_self   = weakref.ref(self)
        self.sensor.listen(lambda d: IMUSensor._cb(weak_self, d))

    @staticmethod
    def _cb(weak_self, data):
        self = weak_self()
        if not self: return
        with self._lock:
            rot = data.transform.rotation
            self.roll, self.pitch, self.yaw = rot.roll, rot.pitch, rot.yaw
            self.roll_rate, self.pitch_rate, self.yaw_rate = (
                data.gyroscope.x,
                data.gyroscope.y,
                data.gyroscope.z,
            )
            self.ax, self.ay, self.az = (
                data.accelerometer.x,
                data.accelerometer.y,
                data.accelerometer.z,
            )

    def get(self):
        with self._lock:
            return {
                "roll": self.roll,
                "pitch": self.pitch,
                "yaw": self.yaw,
                "rollRate": self.roll_rate,
                "pitchRate": self.pitch_rate,
                "yawRate": self.yaw_rate,
                "ax": self.ax,
                "ay": self.ay,
                "az": self.az,
            }

    def destroy(self):
        if self.sensor:
            self.sensor.stop()
            self.sensor.destroy()

# ═══════════════════════════════════════════════════════════════════════════════
# CameraCapture
# ═══════════════════════════════════════════════════════════════════════════════
class CameraCapture:
    VIEWS = [
        (carla.Transform(carla.Location(x=1.6, z=2.4),  carla.Rotation(pitch=-12)), "Driver"),
        (carla.Transform(carla.Location(x=-6.0, z=3.5), carla.Rotation(pitch=-12)), "Third-person"),
        (carla.Transform(carla.Location(z=40.0),         carla.Rotation(pitch=-90)), "Top-down"),
    ]

    def __init__(self, parent_actor, win_w, win_h):
        self._parent  = parent_actor
        self._win_w   = win_w
        self._win_h   = win_h
        self._lock    = threading.Lock()
        self._view_i  = 0
        self.surface  = None
        self._bgr     = None
        self.sensor   = None
        self._spawn()

    def _spawn(self):
        if self.sensor:
            self.sensor.stop()
            self.sensor.destroy()
        world = self._parent.get_world()
        bp    = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(self._win_w))
        bp.set_attribute("image_size_y", str(self._win_h))
        bp.set_attribute("fov",          str(CAM_FOV))
        tf = self.VIEWS[self._view_i][0]
        self.sensor = world.spawn_actor(bp, tf, attach_to=self._parent)
        ws = weakref.ref(self)
        self.sensor.listen(lambda img: CameraCapture._cb(ws, img))

    @staticmethod
    def _cb(ws, image):
        self = ws()
        if not self: return
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))[:, :, :3]
        bgr = arr[:, :, ::-1].copy()
        rgb = arr.copy()
        with self._lock:
            self._bgr    = (bgr, image.timestamp)
            self.surface = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))

    def toggle_view(self):
        self._view_i = (self._view_i + 1) % len(self.VIEWS)
        self._spawn()
        return self.VIEWS[self._view_i][1]

    def current_view_name(self):
        return self.VIEWS[self._view_i][1]

    def get_bgr(self):
        with self._lock:
            return self._bgr

    def render(self, display):
        with self._lock:
            if self.surface:
                display.blit(self.surface, (0, 0))

    def destroy(self):
        if self.sensor:
            self.sensor.stop()
            self.sensor.destroy()

# ═══════════════════════════════════════════════════════════════════════════════
# ScriptPlayer – Điều khiển xe theo kịch bản CSV (Đã cập nhật PID)
# ═══════════════════════════════════════════════════════════════════════════════
class ScriptPlayer:
    MAX_STEER_DEG = 45.0

    def __init__(self, script_path: str, vehicle):
        self.vehicle  = vehicle
        self.phases   = []
        self.total_duration = 0.0
        self._control = carla.VehicleControl()

        if not os.path.isfile(script_path):
            sys.exit(f"[ERROR] Không tìm thấy file script: {script_path}")

        with open(script_path, newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.phases.append({
                    't_start':          float(row['t_start']),
                    't_end':            float(row['t_end']),
                    'target_speed_kmh': float(row['target_speed_kmh']),
                    'steer_deg':        float(row['steer_deg']),
                    'zigzag_period':    float(row.get('zigzag_period', 0)),
                })

        if self.phases:
            self.total_duration = max(p['t_end'] for p in self.phases)
        print(f"[SCRIPT] Đã tải {len(self.phases)} giai đoạn, tổng {self.total_duration:.0f}s → {script_path}")

    def is_done(self, elapsed: float) -> bool:
        return elapsed >= self.total_duration

    def apply(self, elapsed: float):
        phase = None
        for p in self.phases:
            if p['t_start'] <= elapsed < p['t_end']:
                phase = p
                break

        if phase is None:
            self._control.throttle = 0.0
            self._control.brake    = 1.0
            self._control.steer    = 0.0
            self.vehicle.apply_control(self._control)
            return

        # ── [UPDATE] Điều khiển tốc độ bằng bộ điều khiển PID ────────────────
        vel = self.vehicle.get_velocity()
        current_kmh = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) * 3.6
        err = phase['target_speed_kmh'] - current_kmh

        # Khởi tạo các biến PID nếu chưa có
        if not hasattr(self, '_err_integral'):
            self._err_integral = 0.0
            self._last_err = 0.0
            self._last_time = elapsed
            self._last_target = phase['target_speed_kmh']

        # Reset bộ nhớ PID nếu kịch bản chuyển sang tốc độ mới
        if self._last_target != phase['target_speed_kmh']:
            self._err_integral = 0.0
            self._last_err = 0.0
            self._last_target = phase['target_speed_kmh']

        # Tính dt
        dt_sim = elapsed - self._last_time
        self._last_time = elapsed
        if dt_sim <= 0: 
            dt_sim = 0.01

        # Cập nhật I (Tích phân) và giới hạn chống wind-up
        self._err_integral += err * dt_sim
        self._err_integral = max(-150.0, min(self._err_integral, 150.0))

        # Cập nhật D (Đạo hàm)
        derivative = (err - self._last_err) / dt_sim
        self._last_err = err

        # Hệ số PID
        Kp = 0.15  # Bám sát nhanh
        Ki = 0.03  # Giữ tốc độ ổn định lâu dài
        Kd = 0.05  # Làm mượt, tránh vọt lố

        control_val = (Kp * err) + (Ki * self._err_integral) + (Kd * derivative)

        if control_val > 0.0:
            self._control.throttle = min(control_val, 1.0)
            self._control.brake    = 0.0
        else:
            self._control.throttle = 0.0
            self._control.brake    = min(-control_val, 1.0)

        # ── Điều khiển lái ──────────────────────────────────────────────────
        steer_deg      = phase['steer_deg']
        zigzag_period  = phase['zigzag_period']

        if zigzag_period > 0:
            phase_ratio = (elapsed % zigzag_period) / zigzag_period
            steer_deg   = steer_deg if phase_ratio < 0.5 else -steer_deg

        steer_norm = max(-1.0, min(1.0, steer_deg / self.MAX_STEER_DEG))
        self._control.steer   = steer_norm
        self._control.reverse = False

        self.vehicle.apply_control(self._control)

    @property
    def control(self):
        return self._control

    @property
    def total_time(self):
        return self.total_duration

# ═══════════════════════════════════════════════════════════════════════════════
# KeyboardControl
# ═══════════════════════════════════════════════════════════════════════════════
class KeyboardControl:
    def __init__(self, vehicle, start_autopilot=False):
        self.vehicle            = vehicle
        self._autopilot         = start_autopilot
        self._control           = carla.VehicleControl()
        self._steer_cache       = 0.0
        vehicle.set_autopilot(start_autopilot)

    def parse_events(self, events: list, clock_ms: float) -> dict:
        actions = {}
        for event in events:
            if event.type == pygame.QUIT:
                actions["quit"] = True
            elif event.type == pygame.KEYUP:
                k   = event.key
                mod = pygame.key.get_mods()
                if k == K_ESCAPE or (k == K_q and mod & KMOD_CTRL):
                    actions["quit"] = True
                elif k == K_TAB:
                    actions["toggle_camera"] = True
                elif isinstance(self._control, carla.VehicleControl):
                    if k == K_p and not (mod & KMOD_CTRL):
                        self._autopilot = not self._autopilot
                        self.vehicle.set_autopilot(self._autopilot)
                        actions["notification"] = f"Autopilot {'ON ▶' if self._autopilot else 'OFF ■'}"
                    elif k == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif k == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = self.vehicle.get_control().gear
                        actions["notification"] = f"{'Manual' if self._control.manual_gear_shift else 'Auto'} Transmission"
                    elif self._control.manual_gear_shift and k == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and k == K_PERIOD:
                        self._control.gear += 1

        if not self._autopilot:
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock_ms)
            self._control.reverse = self._control.gear < 0
            self.vehicle.apply_control(self._control)

        return actions

    def _parse_vehicle_keys(self, keys, ms):
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.05, 1.0)
        else:
            self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1.0)
        else:
            self._control.brake = 0.0

        steer_increment = 5e-4 * ms
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0: self._steer_cache = 0.0
            else: self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0: self._steer_cache = 0.0
            else: self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0

        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer      = round(self._steer_cache, 1)
        self._control.hand_brake = bool(keys[K_SPACE])

    @property
    def is_autopilot(self): return self._autopilot
    @property
    def control(self):      return self._control

# ═══════════════════════════════════════════════════════════════════════════════
# HUDOverlay
# ═══════════════════════════════════════════════════════════════════════════════
class HUDOverlay:
    C_GREEN, C_YELLOW, C_RED = (0, 220, 100), (240, 200, 40), (240, 60, 60)
    C_WHITE, C_GRAY, C_CYAN, C_TITLE = (230, 230, 230), (150, 150, 150), (80, 210, 255), (0, 210, 130)

    def __init__(self, w, h):
        self.w, self.h = w, h
        pygame.font.init()
        self._fnt    = pygame.font.SysFont("ubuntumono,courier,monospace", 14)
        self._fnt_b  = pygame.font.SysFont("ubuntumono,courier,monospace", 15, bold=True)
        self._notif  = ""
        self._notif_t = 0.0

    def notify(self, msg, seconds=2.5):
        self._notif, self._notif_t = msg, seconds

    def tick(self, dt_s):
        if self._notif_t > 0: self._notif_t -= dt_s

    def render(self, display, vehicle, imu_sensor, kb_ctrl, rec_state, frame_count, session_dir, weather_name, cam_name, elapsed_time=0.0, script_player=None):
        tf, ctrl, imu, vel = vehicle.get_transform(), kb_ctrl.control if script_player is None else script_player.control, imu_sensor.get(), vehicle.get_velocity()
        spd = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) * 3.6
        self._draw_ctrl_panel(display, ctrl, kb_ctrl, spd, rec_state, frame_count, session_dir, weather_name, cam_name, elapsed_time, script_player)
        self._draw_data_panel(display, tf.location, imu)
        self._draw_keybind_bar(display, script_player is not None)
        if self._notif_t > 0:
            alpha = min(255, int(255 * min(1.0, self._notif_t)))
            surf  = self._fnt_b.render(self._notif, True, self.C_WHITE)
            bg    = pygame.Surface((surf.get_width() + 22, surf.get_height() + 10), pygame.SRCALPHA)
            bg.fill((20, 20, 20, alpha))
            surf.set_alpha(alpha)
            bx = self.w // 2 - bg.get_width() // 2
            display.blit(bg,   (bx, self.h - 76))
            display.blit(surf, (bx + 11, self.h - 71))

    def _draw_ctrl_panel(self, display, ctrl, kb_ctrl, spd, rec_state, frame_count, session_dir, weather_name, cam_name, elapsed_time, script_player):
        PAD, LH, rows = 10, 20, []
        if script_player is not None:
            rows.extend([("MODE ", self.C_GRAY, "SCRIPT", self.C_CYAN), ("TIME ", self.C_GRAY, f"{elapsed_time:6.1f}s  ({min(100.0, elapsed_time / max(script_player.total_time, 1) * 100):.0f}%)", self.C_WHITE)])
        else:
            rows.extend([("MODE ", self.C_GRAY, "AUTOPILOT" if kb_ctrl.is_autopilot else "MANUAL", self.C_YELLOW if kb_ctrl.is_autopilot else self.C_GREEN), ("TIME ", self.C_GRAY, f"{elapsed_time:6.1f}s", self.C_WHITE)])
        rows.extend([
            ("SPD  ", self.C_GRAY, f"{spd:6.1f} km/h  G:{'R' if ctrl.gear < 0 else str(max(ctrl.gear, 0))}", self.C_WHITE),
            ("THR  ", self.C_GRAY, self._bar(ctrl.throttle, 10, "▮"), self.C_GREEN),
            ("BRK  ", self.C_GRAY, self._bar(ctrl.brake,    10, "▮"), self.C_RED),
            ("STR  ", self.C_GRAY, self._steer_bar(ctrl.steer, 11), self.C_CYAN), None
        ])
        rc, rt = {"idle": (self.C_GRAY, "● IDLE"), "recording": (self.C_GREEN, "● RECORDING"), "paused": (self.C_YELLOW, "⏸ PAUSED"), "stopped": (self.C_RED, "■ STOPPED")}.get(rec_state, (self.C_GRAY, rec_state))
        rows.extend([("REC  ", self.C_GRAY, rt, rc), ("FRM  ", self.C_GRAY, str(frame_count), self.C_WHITE)])
        if session_dir: rows.append(("DIR  ", self.C_GRAY, ("…" + session_dir[-24:]) if len(session_dir) > 26 else session_dir, self.C_GRAY))
        rows.extend([None, ("CAM  ", self.C_GRAY, cam_name, self.C_CYAN), ("WTH  ", self.C_GRAY, weather_name, self.C_CYAN)])
        bg = pygame.Surface((330, sum(1 if r else 0 for r in rows) * LH + sum(1 if not r else 0 for r in rows) * (LH // 2) + PAD * 2), pygame.SRCALPHA)
        bg.fill((10, 10, 10, 190))
        display.blit(bg, (PAD, PAD))
        y = PAD + 6
        for row in rows:
            if row is None: y += LH // 2; continue
            display.blit(self._fnt.render(row[0], True, row[1]), (PAD + 8, y))
            display.blit(self._fnt.render(row[2], True, row[3]), (PAD + 63, y))
            y += LH

    def _draw_data_panel(self, display, loc, imu):
        PAD, LH = 10, 17
        groups = [
            ("POSITION", [("X", f"{loc.x:+9.3f} m"), ("Y", f"{loc.y:+9.3f} m"), ("Z", f"{loc.z:+9.3f} m")]),
            ("ROTATION", [("Roll", f"{imu['roll']:+8.2f} °"), ("Pitch", f"{imu['pitch']:+8.2f} °"), ("Yaw", f"{imu['yaw']:+8.2f} °")]),
            ("ANGULAR RATE", [("rollRate", f"{imu['rollRate']:+7.2f} °/s"), ("pitchRate", f"{imu['pitchRate']:+7.2f} °/s"), ("yawRate", f"{imu['yawRate']:+7.2f} °/s")]),
            ("ACCEL IMU", [("Ax", f"{imu['ax']:+7.3f} m/s²"), ("Ay", f"{imu['ay']:+7.3f} m/s²"), ("Az", f"{imu['az']:+7.3f} m/s²")])
        ]
        pw, ph = 260, (sum(1 + len(g[1]) for g in groups) + len(groups) - 1) * LH + PAD * 2 + 4
        bg = pygame.Surface((pw, ph), pygame.SRCALPHA)
        bg.fill((10, 10, 10, 190))
        display.blit(bg, (self.w - pw - PAD, self.h - ph - 34))
        y = self.h - ph - 34 + PAD
        for i, (title, items) in enumerate(groups):
            display.blit(self._fnt_b.render(f"─ {title} ─", True, self.C_TITLE), (self.w - pw - PAD + 6, y)); y += LH
            for lbl, val in items:
                display.blit(self._fnt.render(f"{lbl:<10}", True, self.C_GRAY), (self.w - pw - PAD + 8, y))
                display.blit(self._fnt.render(val, True, self.C_WHITE), (self.w - pw - PAD + 88, y))
                y += LH
            if i < len(groups) - 1: y += 3

    def _draw_keybind_bar(self, display, script_mode=False):
        H = 26
        bg = pygame.Surface((self.w, H), pygame.SRCALPHA)
        bg.fill((15, 15, 15, 215))
        display.blit(bg, (0, self.h - H))
        hints = [("SCRIPT", "Auto"), ("F6", "Pause"), ("F7", "Resume"), ("F8", "Stop"), ("TAB", "Cam"), ("ESC", "Quit")] if script_mode else [("W/A/S/D", "Drive"), ("Q", "Rev"), ("Space", "H-brake"), ("P", "Autopilot"), ("F5", "Start"), ("F6", "Pause"), ("F7", "Resume"), ("F8", "Save"), ("TAB", "Cam"), ("ESC", "Quit")]
        x = 6
        for key, desc in hints:
            sk, sd = self._fnt.render(f"[{key}]", True, self.C_CYAN), self._fnt.render(f" {desc}  ", True, self.C_GRAY)
            display.blit(sk, (x, self.h - H + 6)); x += sk.get_width()
            display.blit(sd, (x, self.h - H + 6)); x += sd.get_width()

    @staticmethod
    def _bar(v, n, ch): f = int(round(v * n)); return ch * f + "·" * (n - f)

    @staticmethod
    def _steer_bar(steer, n):
        mid = n // 2; fill = int(round(abs(steer) * mid)); bar = ["·"] * n; bar[mid] = "|"
        for i in (range(mid - fill, mid) if steer < 0 else range(mid + 1, mid + 1 + fill)):
            if 0 <= i < n: bar[i] = "◄" if steer < 0 else "►"
        return "".join(bar)

# ═══════════════════════════════════════════════════════════════════════════════
# Main game loop
# ═══════════════════════════════════════════════════════════════════════════════
def game_loop(args):
    pygame.init()
    pygame.font.init()
    clock = pygame.time.Clock()

    display = pygame.display.set_mode(
        (args.width, args.height),
        pygame.HWSURFACE | pygame.DOUBLEBUF,
    )
    pygame.display.set_caption("CARLA Collector")
    display.fill((0, 0, 0))
    pygame.display.flip()

    print(f"[INFO] Kết nối {args.host}:{args.port} …")
    client = carla.Client(args.host, args.port)
    client.set_timeout(CARLA_TIMEOUT)

    if args.map is not None:
        print(f"[INFO] Đang tải map: {args.map} ...")
        world = client.load_world(args.map)
    else:
        world = client.get_world()

    # ── Thời tiết ──────────────────────────────────────────────────────────
    if hasattr(carla.WeatherParameters, args.weather):
        weather_preset = getattr(carla.WeatherParameters, args.weather)
        world.set_weather(weather_preset)
        w_name = args.weather
        print(f"[INFO] Đã thiết lập thời tiết: {w_name}")
    else:
        world.set_weather(carla.WeatherParameters.ClearNoon)
        w_name = "ClearNoon"

    # ── Synchronous mode ──────────────────────────────────────────────────
    original_settings = None
    if args.sync:
        original_settings = world.get_settings()
        s = world.get_settings()
        s.synchronous_mode    = True
        s.fixed_delta_seconds = 1.0 / args.hz
        world.apply_settings(s)
        client.get_trafficmanager().set_synchronous_mode(True)
        print(f"[INFO] Sync mode ON – {args.hz} Hz (fixed_delta={1/args.hz:.4f}s)")

    bp_lib   = world.get_blueprint_library()
    veh_bps  = bp_lib.filter(args.filter)

    if not veh_bps:
        sys.exit(f"[ERROR] Không tìm thấy xe nào phù hợp với filter '{args.filter}'.")

    veh_bp   = random.choice(veh_bps)
    veh_bp.set_attribute("role_name", "hero")

    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()
    if not spawn_points:
        sys.exit("[ERROR] Không có điểm spawn point nào trong map này.")

    # ── [UPDATE] Tìm đường thẳng dài và căn xe giữa làn ────────────────────
    print("[INFO] Đang tìm vị trí đường thẳng dài và căn giữa làn...")
    best_sp_transform = None
    
    for sp in spawn_points:
        wp = carla_map.get_waypoint(sp.location)
        # Lấy waypoint cách đó 100 mét để kiểm tra độ thẳng
        wp_next_list = wp.next(100.0)
        
        if wp_next_list:
            wp_next = wp_next_list[0]
            # Tính độ lệch góc (yaw). Gần 0 nghĩa là đường thẳng
            yaw_diff = abs(wp.transform.rotation.yaw - wp_next.transform.rotation.yaw)
            yaw_diff = yaw_diff % 360
            if yaw_diff > 180:
                yaw_diff = 360 - yaw_diff
                
            if yaw_diff < 0.5:
                # Đặt vị trí spawn chính xác vào giữa làn (tọa độ của waypoint)
                best_sp_transform = wp.transform
                # Nâng nhẹ trục Z để xe rơi nhẹ xuống, không bị kẹt dưới mặt đất
                best_sp_transform.location.z += 0.5  
                break

    if best_sp_transform is None:
        print("[WARN] Không tìm thấy đoạn đường hoàn toàn thẳng, chọn và căn giữa ngẫu nhiên.")
        random_sp = random.choice(spawn_points)
        best_sp_transform = carla_map.get_waypoint(random_sp.location).transform
        best_sp_transform.location.z += 0.5

    vehicle = world.try_spawn_actor(veh_bp, best_sp_transform)
    # ───────────────────────────────────────────────────────────────────────

    if vehicle is None:
        sys.exit("[ERROR] Không thể spawn xe. Điểm spawn bị kẹt.")
    print(f"[INFO] Xe đã spawn: {vehicle.type_id}")

    imu    = IMUSensor(vehicle)
    camera = CameraCapture(vehicle, args.width, args.height)
    kb     = KeyboardControl(vehicle, start_autopilot=args.autopilot)
    hud    = HUDOverlay(args.width, args.height)

    script_player = None
    if args.script:
        script_player = ScriptPlayer(args.script, vehicle)

    rec_state        = "idle"
    frame_count      = 0
    frame_id         = 0
    session_dir      = ""
    writer           = None
    rec_start_sim_t  = None

    if args.sync: world.tick()
    else: world.wait_for_tick()

    if script_player is not None:
        ts          = dt.now().strftime("%Y%m%d_%H%M%S")
        session_dir = os.path.join(OUTPUT_ROOT, f"session_{ts}")
        os.makedirs(session_dir, exist_ok=True)
        writer      = DataWriter(session_dir)
        frame_id    = frame_count = 0
        rec_start_sim_t = None
        rec_state   = "recording"
        hud.notify(f"▶ Script auto-start  ({script_player.total_time:.0f}s)", seconds=4)
    else:
        hud.notify("Sẵn sàng!  WASD=lái · F5=Start · F8=Save", seconds=5)

    auto_quit = False

    try:
        while True:
            if args.sync: world.tick()

            dt_s   = clock.tick_busy_loop(args.hz) / 1000.0
            events = pygame.event.get()

            for ev in events:
                if ev.type == pygame.KEYUP:
                    if ev.key == K_F5 and script_player is None and rec_state in ("idle", "stopped"):
                        ts          = dt.now().strftime("%Y%m%d_%H%M%S")
                        session_dir = os.path.join(OUTPUT_ROOT, f"session_{ts}")
                        os.makedirs(session_dir, exist_ok=True)
                        writer      = DataWriter(session_dir)
                        frame_id    = frame_count = 0
                        rec_start_sim_t = None
                        rec_state   = "recording"
                        hud.notify("▶ Bắt đầu thu thập")

                    elif ev.key == K_F6 and rec_state == "recording":
                        rec_state = "paused"
                        hud.notify("⏸ Tạm dừng")

                    elif ev.key == K_F7 and rec_state == "paused":
                        rec_state = "recording"
                        hud.notify("▶ Tiếp tục")

                    elif ev.key == K_F8 and rec_state in ("recording", "paused"):
                        rec_state = "stopped"
                        if writer: writer.close(); writer = None
                        hud.notify(f"💾 Đã lưu {frame_count} frames", seconds=4)
                        if script_player is not None: auto_quit = True

            actions = kb.parse_events(events, clock.get_time())
            if actions.get("quit"): break
            if actions.get("toggle_camera"): hud.notify(f"Camera: {camera.toggle_view()}")
            if actions.get("notification"): hud.notify(actions["notification"])

            elapsed_time = 0.0
            if rec_start_sim_t is not None:
                snap = camera.get_bgr()
                if snap is not None:
                    _, sim_ts   = snap
                    elapsed_time = sim_ts - rec_start_sim_t

            if script_player is not None and rec_state == "recording":
                script_player.apply(elapsed_time)
                if script_player.is_done(elapsed_time) and rec_start_sim_t is not None:
                    rec_state = "stopped"
                    if writer: writer.close(); writer = None
                    hud.notify(f"✅ Script hoàn thành  {frame_count} frames", seconds=3)
                    auto_quit = True

            if rec_state == "recording":
                snap = camera.get_bgr()
                if snap is not None:
                    bgr, sim_ts = snap
                    if rec_start_sim_t is None: rec_start_sim_t = sim_ts
                    elapsed_time = sim_ts - rec_start_sim_t
                    frame_id    += 1
                    frame_count += 1
                    if writer:
                        writer.write(frame_id, elapsed_time, bgr, imu.get(), vehicle.get_transform(), vehicle.get_control())

            camera.render(display)
            hud.tick(dt_s)
            hud.render(display, vehicle, imu, kb, rec_state, frame_count, session_dir, w_name, camera.current_view_name(), elapsed_time=elapsed_time, script_player=script_player)
            pygame.display.flip()

            if auto_quit:
                _time.sleep(1.5)
                break

    finally:
        print("\n[INFO] Đang dọn dẹp …")
        if writer: writer.close()
        camera.destroy()
        imu.destroy()
        if vehicle: vehicle.destroy()
        if original_settings: world.apply_settings(original_settings)
        pygame.quit()
        if frame_count > 0: print(f"[DONE] Đã lưu {frame_count} frames → {session_dir}")

def main():
    parser = argparse.ArgumentParser(description="CARLA Data Collector + Script Control")
    parser.add_argument("--host",    default=CARLA_HOST)
    parser.add_argument("-p", "--port", default=CARLA_PORT, type=int)
    parser.add_argument("--res",     default=f"{WIN_W}x{WIN_H}", metavar="WxH")
    parser.add_argument("--filter",  default="vehicle.tesla.model3")
    # [UPDATE] Đổi mặc định sang Town04
    parser.add_argument("--map",     default="Town04",
                        help="Tên map để load (Mặc định: Town04)")
    parser.add_argument("--weather", default="ClearNoon")
    parser.add_argument("-a", "--autopilot", action="store_true")
    parser.add_argument("--sync",    action="store_true")
    parser.add_argument("--hz",      default=100, type=int)
    parser.add_argument("--script",  default=None, metavar="SCENARIO.CSV")

    args = parser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split("x")]

    try:
        game_loop(args)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
