#!/usr/bin/env python
"""
carla_capture.py  ─  Thu thập dữ liệu CARLA + Manual / Autopilot control
═══════════════════════════════════════════════════════════════════════════

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
    python carla_capture.py --weather ClearNoon --filter vehicle.tesla.model3
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

CSV_HEADER = [
    "frame_id", "timestamp", "image_file",
    "x", "y", "z",
    "roll", "pitch", "yaw",
    "vx", "vy", "vz",
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

    def write(self, frame_id, timestamp, img_bgr, imu, transform, velocity, control):
        img_name = f"frame_{frame_id:06d}.png"
        cv2.imwrite(os.path.join(self.images_path, img_name), img_bgr)
        loc, rot = transform.location, transform.rotation
        self._writer.writerow({
            "frame_id":   frame_id,
            "timestamp":  f"{timestamp:.6f}",
            "image_file": img_name,
            "x":     f"{loc.x:.6f}", "y": f"{loc.y:.6f}", "z": f"{loc.z:.6f}",
            "roll":  f"{rot.roll:.6f}", "pitch": f"{rot.pitch:.6f}",
            "yaw":   f"{rot.yaw:.6f}",
            "vx":    f"{velocity.x:.6f}", "vy": f"{velocity.y:.6f}",
            "vz":    f"{velocity.z:.6f}",
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
        self.ax = self.ay = self.az = 0.0
        world      = parent_actor.get_world()
        bp         = world.get_blueprint_library().find("sensor.other.imu")
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=parent_actor)
        weak_self   = weakref.ref(self)
        self.sensor.listen(lambda d: IMUSensor._cb(weak_self, d))

    @staticmethod
    def _cb(weak_self, data):
        self = weak_self()
        if not self: return
        with self._lock:
            self.ax, self.ay, self.az = (data.accelerometer.x, data.accelerometer.y, data.accelerometer.z)

    def get(self):
        with self._lock:
            return {"ax": self.ax, "ay": self.ay, "az": self.az}

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
# KeyboardControl (Đã cập nhật logic giống manual_control.py)
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
                    # Autopilot
                    if k == K_p and not (mod & KMOD_CTRL):
                        self._autopilot = not self._autopilot
                        self.vehicle.set_autopilot(self._autopilot)
                        actions["notification"] = f"Autopilot {'ON ▶' if self._autopilot else 'OFF ■'}"

                    # Reverse (Số lùi)
                    elif k == K_q:
                        self._control.gear = 1 if self._control.reverse else -1

                    # Manual gear shift toggle
                    elif k == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = self.vehicle.get_control().gear
                        actions["notification"] = f"{'Manual' if self._control.manual_gear_shift else 'Auto'} Transmission"

                    # Gear up / down
                    elif self._control.manual_gear_shift and k == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and k == K_PERIOD:
                        self._control.gear += 1

        # ── Phím giữ (Cập nhật liên tục) ──
        if not self._autopilot:
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock_ms)
            self._control.reverse = self._control.gear < 0
            self.vehicle.apply_control(self._control)

        return actions

    def _parse_vehicle_keys(self, keys, ms):
        # Throttle
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.05, 1.0)
        else:
            self._control.throttle = 0.0

        # Brake
        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1.0)
        else:
            self._control.brake = 0.0

        # Steer
        steer_increment = 5e-4 * ms
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0.0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0.0
            else:
                self._steer_cache += steer_increment
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
    C_GREEN  = (0,   220, 100)
    C_YELLOW = (240, 200,  40)
    C_RED    = (240,  60,  60)
    C_WHITE  = (230, 230, 230)
    C_GRAY   = (150, 150, 150)
    C_CYAN   = ( 80, 210, 255)
    C_TITLE  = (  0, 210, 130)

    def __init__(self, w, h):
        self.w, self.h = w, h
        pygame.font.init()
        self._fnt    = pygame.font.SysFont("ubuntumono,courier,monospace", 14)
        self._fnt_b  = pygame.font.SysFont("ubuntumono,courier,monospace", 15, bold=True)
        self._notif  = ""
        self._notif_t = 0.0

    def notify(self, msg, seconds=2.5):
        self._notif   = msg
        self._notif_t = seconds

    def tick(self, dt_s):
        if self._notif_t > 0:
            self._notif_t -= dt_s

    def render(self, display, vehicle, imu_sensor, kb_ctrl,
               rec_state, frame_count, session_dir, weather_name, cam_name):
        tf    = vehicle.get_transform()
        vel   = vehicle.get_velocity()
        ctrl  = kb_ctrl.control
        imu   = imu_sensor.get()
        loc   = tf.location
        rot   = tf.rotation
        spd   = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) * 3.6

        self._draw_ctrl_panel(display, ctrl, kb_ctrl, spd, rec_state, frame_count, session_dir, weather_name, cam_name)
        self._draw_data_panel(display, loc, rot, vel, imu)
        self._draw_keybind_bar(display)

        if self._notif_t > 0:
            alpha = min(255, int(255 * min(1.0, self._notif_t)))
            surf  = self._fnt_b.render(self._notif, True, self.C_WHITE)
            bg    = pygame.Surface((surf.get_width() + 22, surf.get_height() + 10), pygame.SRCALPHA)
            bg.fill((20, 20, 20, alpha))
            surf.set_alpha(alpha)
            bx = self.w // 2 - bg.get_width() // 2
            display.blit(bg,   (bx, self.h - 76))
            display.blit(surf, (bx + 11, self.h - 71))

    def _draw_ctrl_panel(self, display, ctrl, kb_ctrl, spd, rec_state, frame_count, session_dir, weather_name, cam_name):
        PAD, LH = 10, 20
        rows = []
        mode_txt = "AUTOPILOT" if kb_ctrl.is_autopilot else "MANUAL"
        mode_col = self.C_YELLOW if kb_ctrl.is_autopilot else self.C_GREEN
        rows.append(("MODE ", self.C_GRAY, mode_txt, mode_col))

        gear_str = "R" if ctrl.gear < 0 else str(max(ctrl.gear, 0))
        rows.append(("SPD  ", self.C_GRAY, f"{spd:6.1f} km/h  G:{gear_str}", self.C_WHITE))

        rows.append(("THR  ", self.C_GRAY, self._bar(ctrl.throttle, 10, "▮"), self.C_GREEN))
        rows.append(("BRK  ", self.C_GRAY, self._bar(ctrl.brake,    10, "▮"), self.C_RED))
        rows.append(("STR  ", self.C_GRAY, self._steer_bar(ctrl.steer, 11), self.C_CYAN))

        rows.append(None)
        
        _rc = {
            "idle":      (self.C_GRAY,   "● IDLE"),
            "recording": (self.C_GREEN,  "● RECORDING"),
            "paused":    (self.C_YELLOW, "⏸ PAUSED"),
            "stopped":   (self.C_RED,    "■ STOPPED"),
        }
        rc, rt = _rc.get(rec_state, (self.C_GRAY, rec_state))
        rows.append(("REC  ", self.C_GRAY, rt, rc))
        rows.append(("FRM  ", self.C_GRAY, str(frame_count), self.C_WHITE))
        if session_dir:
            short = ("…" + session_dir[-24:]) if len(session_dir) > 26 else session_dir
            rows.append(("DIR  ", self.C_GRAY, short, self.C_GRAY))

        rows.append(None)
        rows.append(("CAM  ", self.C_GRAY, cam_name,     self.C_CYAN))
        rows.append(("WTH  ", self.C_GRAY, weather_name, self.C_CYAN))

        pw = 310
        ph = (sum(1 if r else 0 for r in rows) * LH + sum(1 if not r else 0 for r in rows) * (LH // 2) + PAD * 2)

        bg = pygame.Surface((pw, ph), pygame.SRCALPHA)
        bg.fill((10, 10, 10, 190))
        display.blit(bg, (PAD, PAD))

        x0, y = PAD + 8, PAD + 6
        for row in rows:
            if row is None:
                y += LH // 2
                continue
            lbl, lc, val, vc = row
            display.blit(self._fnt.render(lbl, True, lc), (x0,       y))
            display.blit(self._fnt.render(val, True, vc), (x0 + 55,  y))
            y += LH

    def _draw_data_panel(self, display, loc, rot, vel, imu):
        PAD, LH = 10, 17
        groups = [
            ("POSITION", [("X", f"{loc.x:+9.3f} m"), ("Y", f"{loc.y:+9.3f} m"), ("Z", f"{loc.z:+9.3f} m")]),
            ("ROTATION", [("Roll", f"{rot.roll:+8.2f} °"), ("Pitch", f"{rot.pitch:+8.2f} °"), ("Yaw", f"{rot.yaw:+8.2f} °")]),
            ("VELOCITY", [("Vx", f"{vel.x:+8.3f} m/s"), ("Vy", f"{vel.y:+8.3f} m/s"), ("Vz", f"{vel.z:+8.3f} m/s")]),
            ("ACCEL IMU", [("Ax", f"{imu['ax']:+7.3f} m/s²"), ("Ay", f"{imu['ay']:+7.3f} m/s²"), ("Az", f"{imu['az']:+7.3f} m/s²")]),
        ]

        pw  = 250
        n_rows = sum(1 + len(g[1]) for g in groups) + len(groups) - 1
        ph  = n_rows * LH + PAD * 2 + 4
        x0  = self.w - pw - PAD
        y0  = self.h - ph - 34

        bg  = pygame.Surface((pw, ph), pygame.SRCALPHA)
        bg.fill((10, 10, 10, 190))
        display.blit(bg, (x0, y0))

        y = y0 + PAD
        for i, (title, items) in enumerate(groups):
            display.blit(self._fnt_b.render(f"─ {title} ─", True, self.C_TITLE), (x0 + 6, y))
            y += LH
            for lbl, val in items:
                display.blit(self._fnt.render(f"{lbl:<6}", True, self.C_GRAY),  (x0 + 8,  y))
                display.blit(self._fnt.render(val,         True, self.C_WHITE), (x0 + 55, y))
                y += LH
            if i < len(groups) - 1: y += 3

    def _draw_keybind_bar(self, display):
        H  = 26
        bg = pygame.Surface((self.w, H), pygame.SRCALPHA)
        bg.fill((15, 15, 15, 215))
        display.blit(bg, (0, self.h - H))

        hints = [
            ("W/A/S/D", "Drive"),  ("Q", "Rev"),  ("Space", "H-brake"),
            ("P", "Autopilot"),
            ("F5", "Start"),    ("F6", "Pause"), ("F7", "Resume"),   ("F8", "Save&Quit"),
            ("TAB", "Cam"),     ("ESC", "Quit"),
        ]
        x = 6
        for key, desc in hints:
            sk = self._fnt.render(f"[{key}]", True, self.C_CYAN)
            sd = self._fnt.render(f" {desc}  ", True, self.C_GRAY)
            display.blit(sk, (x, self.h - H + 6))
            x += sk.get_width()
            display.blit(sd, (x, self.h - H + 6))
            x += sd.get_width()

    @staticmethod
    def _bar(v, n, ch):
        f = int(round(v * n))
        return ch * f + "·" * (n - f)

    @staticmethod
    def _steer_bar(steer, n):
        mid  = n // 2
        fill = int(round(abs(steer) * mid))
        bar  = ["·"] * n
        bar[mid] = "|"
        rng = range(mid - fill, mid) if steer < 0 else range(mid + 1, mid + 1 + fill)
        ch  = "◄" if steer < 0 else "►"
        for i in rng:
            if 0 <= i < n: bar[i] = ch
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

    # ── Thiết lập Thời tiết theo Argument ──────────────────────────────────
    if hasattr(carla.WeatherParameters, args.weather):
        weather_preset = getattr(carla.WeatherParameters, args.weather)
        world.set_weather(weather_preset)
        w_name = args.weather
        print(f"[INFO] Đã thiết lập thời tiết: {w_name}")
    else:
        print(f"[WARN] Không tìm thấy preset '{args.weather}'. Đang dùng ClearNoon.")
        world.set_weather(carla.WeatherParameters.ClearNoon)
        w_name = "ClearNoon"

    original_settings = None
    if args.sync:
        original_settings = world.get_settings()
        s = world.get_settings()
        if not s.synchronous_mode:
            s.synchronous_mode      = True
            s.fixed_delta_seconds   = 0.05
        world.apply_settings(s)
        client.get_trafficmanager().set_synchronous_mode(True)

    bp_lib   = world.get_blueprint_library()
    veh_bps  = bp_lib.filter(args.filter)
    
    if not veh_bps:
        sys.exit(f"[ERROR] Không tìm thấy xe nào phù hợp với filter '{args.filter}'.")
        
    veh_bp   = random.choice(veh_bps)
    veh_bp.set_attribute("role_name", "hero")
    
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        sys.exit("[ERROR] Không có điểm spawn point nào trong map này.")
        
    sp       = random.choice(spawn_points)
    vehicle  = world.try_spawn_actor(veh_bp, sp)
    
    if vehicle is None:
        sys.exit("[ERROR] Không thể spawn xe. Điểm spawn bị kẹt.")
    print(f"[INFO] Xe đã spawn: {vehicle.type_id}")

    imu    = IMUSensor(vehicle)
    camera = CameraCapture(vehicle, args.width, args.height)

    kb     = KeyboardControl(vehicle, start_autopilot=args.autopilot)
    hud    = HUDOverlay(args.width, args.height)

    rec_state   = "idle"  
    frame_count = 0
    frame_id    = 0
    session_dir = ""
    writer: DataWriter | None = None

    if args.sync:
        world.tick()
    else:
        world.wait_for_tick()

    hud.notify("Sẵn sàng!  WASD=lái · F5=Start · F8=Save", seconds=5)
    print("[INFO] Sẵn sàng.  WASD=lái  P=Autopilot  F5=Start  F8=Save&Quit")

    try:
        while True:
            if args.sync:
                world.tick()

            dt_s   = clock.tick_busy_loop(60) / 1000.0
            events = pygame.event.get()

            for ev in events:
                if ev.type == pygame.KEYUP:
                    if ev.key == K_F5 and rec_state in ("idle", "stopped"):
                        ts          = dt.now().strftime("%Y%m%d_%H%M%S")
                        session_dir = os.path.join(OUTPUT_ROOT, f"session_{ts}")
                        os.makedirs(session_dir, exist_ok=True)
                        writer      = DataWriter(session_dir)
                        frame_id    = frame_count = 0
                        rec_state   = "recording"
                        hud.notify("▶ Bắt đầu thu thập")
                        print(f"[REC] Start → {session_dir}")

                    elif ev.key == K_F6 and rec_state == "recording":
                        rec_state = "paused"
                        hud.notify("⏸ Tạm dừng")
                        print("[REC] Paused")

                    elif ev.key == K_F7 and rec_state == "paused":
                        rec_state = "recording"
                        hud.notify("▶ Tiếp tục")
                        print("[REC] Resumed")

                    elif ev.key == K_F8 and rec_state in ("recording", "paused"):
                        rec_state = "stopped"
                        if writer:
                            writer.close()
                            writer = None
                        hud.notify(f"💾 Đã lưu {frame_count} frames", seconds=4)
                        print(f"[REC] Saved {frame_count} frames → {session_dir}")

            actions = kb.parse_events(events, clock.get_time())

            if actions.get("quit"):
                break
            if actions.get("toggle_camera"):
                cam_n = camera.toggle_view()
                hud.notify(f"Camera: {cam_n}")
            if actions.get("notification"):
                hud.notify(actions["notification"])

            if rec_state == "recording":
                snap = camera.get_bgr()
                if snap is not None:
                    bgr, timestamp = snap
                    frame_id    += 1
                    frame_count += 1
                    if writer:
                        writer.write(
                            frame_id, timestamp, bgr, imu.get(),
                            vehicle.get_transform(), vehicle.get_velocity(),
                            vehicle.get_control() 
                        )

            camera.render(display)
            hud.tick(dt_s)
            hud.render(display, vehicle, imu, kb, rec_state, frame_count, session_dir, w_name, camera.current_view_name())
            pygame.display.flip()

    finally:
        print("\n[INFO] Đang dọn dẹp …")
        if writer:
            writer.close()
        camera.destroy()
        imu.destroy()
        if vehicle:
            vehicle.destroy()
        if original_settings:
            world.apply_settings(original_settings)
        pygame.quit()
        if frame_count > 0:
            print(f"[DONE] Đã lưu {frame_count} frames → {session_dir}")
        else:
            print("[DONE] Không có dữ liệu nào được lưu.")

# ═══════════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="CARLA Data Collector + Manual/Autopilot Control")
    parser.add_argument("--host",    default=CARLA_HOST)
    parser.add_argument("-p", "--port", default=CARLA_PORT, type=int)
    parser.add_argument("--res",     default=f"{WIN_W}x{WIN_H}", metavar="WxH")
    parser.add_argument("--filter",  default="vehicle.tesla.model3", help="Bộ lọc tên xe (VD: vehicle.audi.a2, vehicle.*)")
    parser.add_argument("--map",     default=None, help="Tên map để load (VD: Town01, Town04)")
    
    # ── Argument mới: Weather ──────────────────────────────────────────────
    parser.add_argument("--weather", default="ClearNoon", help="Thời tiết (VD: ClearNoon, HardRainSunset, WetCloudyNoon...)")
    # ───────────────────────────────────────────────────────────────────────
    
    parser.add_argument("-a", "--autopilot", action="store_true", help="Bật autopilot ngay khi khởi động")
    parser.add_argument("--sync",    action="store_true", help="Synchronous mode")
    
    args = parser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split("x")]

    try:
        game_loop(args)
    except KeyboardInterrupt:
        print("\n[INFO] Đã thoát.")

if __name__ == "__main__":
    main()