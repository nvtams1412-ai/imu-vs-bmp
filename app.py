"""Desktop telemetry viewer for the ESP32 IMU + BMP388 firmware."""

from __future__ import annotations

import math
import csv
import json
import queue
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    import imufusion
    import numpy as np
except ImportError:
    imufusion = None
    np = None


BG = "#0b1018"
PANEL = "#111a26"
PANEL_2 = "#172231"
LINE = "#253448"
TEXT = "#e7edf5"
MUTED = "#8293a8"
CYAN = "#4dd6e5"
BLUE = "#5b8cff"
GREEN = "#4bd69a"
AMBER = "#f2b95f"
RED = "#ff6877"
FONT = "Segoe UI"
CSV_FIELDS = (
    "host_time", "packet_type", "firmware_ms",
    "ax_ms2", "ay_ms2", "az_ms2", "gx_dps", "gy_dps", "gz_dps",
    "imu_temp_c", "mx_raw", "my_raw", "mz_raw",
    "bmp_temp_c", "pressure_hpa", "altitude_m",
    "roll_deg", "pitch_deg", "yaw_deg", "quat_w", "quat_x", "quat_y", "quat_z",
    "raw_serial",
)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def quaternion_multiply(a: tuple[float, float, float, float],
                        b: tuple[float, float, float, float]
                        ) -> tuple[float, float, float, float]:
    """Multiply scalar-first quaternions (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def rotate_by_quaternion(quaternion: tuple[float, float, float, float],
                         point: tuple[float, float, float]
                         ) -> tuple[float, float, float]:
    w, x, y, z = quaternion
    vector = (0.0, *point)
    conjugate = (w, -x, -y, -z)
    rotated = quaternion_multiply(quaternion_multiply(quaternion, vector), conjugate)
    return rotated[1], rotated[2], rotated[3]


def wrap_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def quaternion_conjugate(q: tuple[float, float, float, float]
                        ) -> tuple[float, float, float, float]:
    return q[0], -q[1], -q[2], -q[3]


def normalize_quaternion(q: tuple[float, float, float, float]
                         ) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in q))
    return tuple(value / norm for value in q) if norm > 0 else (1.0, 0.0, 0.0, 0.0)


def with_yaw(q: tuple[float, float, float, float], yaw: float
             ) -> tuple[float, float, float, float]:
    """Replace the quaternion's Z twist with the displayed yaw angle."""
    w, _x, _y, z = q
    norm = math.hypot(w, z)
    twist = (w / norm, 0.0, 0.0, z / norm) if norm > 1e-8 else (1.0, 0.0, 0.0, 0.0)
    swing = quaternion_multiply(q, quaternion_conjugate(twist))
    half_yaw = math.radians(yaw) / 2.0
    yaw_q = (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw))
    return normalize_quaternion(quaternion_multiply(swing, yaw_q))


class MagnetometerCalibration:
    """Hard-iron offsets and per-axis soft-iron scales for QMC6309 readings."""

    def __init__(self) -> None:
        self.path = Path(__file__).with_name("aeroscope_mag_calibration.json")
        self.offset = [0.0, 0.0, 0.0]
        self.scale = [1.0, 1.0, 1.0]
        self.calibrated = False
        self.collecting = False
        self.minimum = [math.inf, math.inf, math.inf]
        self.maximum = [-math.inf, -math.inf, -math.inf]
        self.count = 0
        self._load()

    def _load(self) -> None:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            offset = [float(value) for value in data["offset"]]
            scale = [float(value) for value in data["scale"]]
            if len(offset) == len(scale) == 3 and all(0.1 <= value <= 10 for value in scale):
                self.offset, self.scale = offset, scale
                self.calibrated = True
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
            pass

    def start(self) -> None:
        self.minimum = [math.inf, math.inf, math.inf]
        self.maximum = [-math.inf, -math.inf, -math.inf]
        self.count = 0
        self.collecting = True

    def add(self, mx: float | None, my: float | None, mz: float | None) -> None:
        if not self.collecting or mx is None or my is None or mz is None:
            return
        values = (mx, my, mz)
        if math.sqrt(sum(value * value for value in values)) < 10:
            return
        for axis, value in enumerate(values):
            self.minimum[axis] = min(self.minimum[axis], value)
            self.maximum[axis] = max(self.maximum[axis], value)
        self.count += 1

    def finish(self) -> tuple[bool, str]:
        self.collecting = False
        spans = [high - low for low, high in zip(self.minimum, self.maximum)]
        widest = max(spans, default=0.0)
        if self.count < 200 or widest <= 0 or min(spans) < max(20.0, widest * 0.25):
            return False, "Chưa đủ dữ liệu phủ cả 3 trục; hãy xoay cảm biến nhiều hướng hơn rồi thử lại."

        radii = [span / 2 for span in spans]
        reference_radius = sum(radii) / 3
        offset = [(low + high) / 2 for low, high in zip(self.minimum, self.maximum)]
        scale = [reference_radius / radius for radius in radii]
        try:
            self.path.write_text(json.dumps({"offset": offset, "scale": scale}, indent=2),
                                 encoding="utf-8")
        except OSError as error:
            return False, f"Không lưu được hiệu chuẩn: {error}"
        self.offset, self.scale = offset, scale
        self.calibrated = True
        return True, "Đã lưu hiệu chuẩn từ kế QMC6309."

    def apply(self, mx: float, my: float, mz: float) -> tuple[float, float, float]:
        values = (mx, my, mz)
        return tuple((value - self.offset[axis]) * self.scale[axis]
                     for axis, value in enumerate(values))


class Attitude:
    """x-io Fusion AHRS with gyro-bias tracking and calibrated magnetic heading."""

    def __init__(self, magnetometer: MagnetometerCalibration) -> None:
        self.magnetometer = magnetometer
        self.roll = self.pitch = self.yaw = 0.0
        self.quaternion = (1.0, 0.0, 0.0, 0.0)
        self.absolute_quaternion = (1.0, 0.0, 0.0, 0.0)
        self.level_reference = (1.0, 0.0, 0.0, 0.0)
        self._last_quaternion: tuple[float, float, float, float] | None = None
        self.last_ms: int | None = None
        self.yaw_offset: float | None = None
        self.yaw_bias = 0.0
        self.raw_yaw: float | None = None
        self.reset()

    def _new_ahrs(self) -> None:
        if imufusion is None:
            self.ahrs = None
            return
        self.ahrs = imufusion.Ahrs()
        self.ahrs.set_settings(imufusion.AhrsSettings(
            sample_rate=100,
            convention=imufusion.CONVENTION_NWU,
            gain=0.5,
            gyroscope_range=2000,
            acceleration_rejection=10,
            magnetic_rejection=10,
            rejection_timeout=5,
        ))

    def reset(self) -> None:
        self.roll = self.pitch = self.yaw = 0.0
        self.quaternion = (1.0, 0.0, 0.0, 0.0)
        self.absolute_quaternion = (1.0, 0.0, 0.0, 0.0)
        self.level_reference = (1.0, 0.0, 0.0, 0.0)
        self._last_quaternion = None
        self.last_ms = None
        self.yaw_offset = None
        self.yaw_bias = 0.0
        self.raw_yaw = None
        if imufusion is None:
            self.bias = None
            self.ahrs = None
            return
        self.bias = imufusion.Bias()
        self.bias.set_settings(imufusion.BiasSettings(sample_rate=100))
        self._new_ahrs()

    def restart_ahrs(self) -> None:
        self.yaw_offset = None
        self.yaw_bias = 0.0
        self.raw_yaw = None
        self.quaternion = (1.0, 0.0, 0.0, 0.0)
        self.absolute_quaternion = (1.0, 0.0, 0.0, 0.0)
        self.level_reference = (1.0, 0.0, 0.0, 0.0)
        self._last_quaternion = None
        self._new_ahrs()

    def zero_yaw(self) -> None:
        self.yaw_offset = self.raw_yaw
        self.yaw_bias = 0.0
        self.yaw = 0.0

    def set_level(self) -> bool:
        if self.last_ms is None:
            return False
        current_yaw = self.yaw
        self.level_reference = self.absolute_quaternion
        self.yaw_bias = current_yaw
        self.yaw_offset = 0.0
        self.roll = self.pitch = 0.0
        self.raw_yaw = 0.0
        self.yaw = wrap_degrees(current_yaw)
        self.quaternion = with_yaw((1.0, 0.0, 0.0, 0.0), self.yaw)
        return True

    def update(self, sample: dict) -> None:
        if imufusion is None or np is None or self.ahrs is None:
            return
        now_ms = sample["ms"]
        dt = 0.01 if self.last_ms is None else ((now_ms - self.last_ms) & 0xFFFFFFFF) / 1000.0
        dt = clamp(dt, 0.001, 0.1)
        self.last_ms = now_ms

        # The earlier UI mapping swaps displayed roll/pitch after Fusion outputs Euler angles.
        gyro = np.asarray((sample["gx"], sample["gy"], sample["gz"]), dtype=np.float32)
        gyro = self.bias.update(gyro)
        accel = np.asarray((sample["ax"], sample["ay"], sample["az"]), dtype=np.float32) / 9.80665
        self.ahrs.set_sample_period(dt)

        mx, my, mz = sample["mx"], sample["my"], sample["mz"]
        if mx is not None and my is not None and mz is not None and math.sqrt(mx * mx + my * my + mz * mz) > 10:
            mag = np.asarray(self.magnetometer.apply(mx, my, mz), dtype=np.float32)
            self.ahrs.update(gyro, accel, mag)
        else:
            self.ahrs.update_no_magnetometer(gyro, accel)

        sensor_q = tuple(float(value) for value in self.ahrs.get_quaternion())
        # Remap the sensor axes to the model axes with a fixed -90 degree Z rotation.
        # Remapping Euler components directly causes the model to flip near 90 degrees.
        half = math.sqrt(0.5)
        axis_remap = (half, 0.0, 0.0, -half)
        inverse_remap = (half, 0.0, 0.0, half)
        model_q = quaternion_multiply(
            quaternion_multiply(axis_remap, sensor_q), inverse_remap)
        norm = math.sqrt(sum(value * value for value in model_q))
        if norm > 0:
            model_q = tuple(value / norm for value in model_q)
        if (self._last_quaternion is not None and
                sum(a * b for a, b in zip(model_q, self._last_quaternion)) < 0):
            model_q = tuple(-value for value in model_q)
        self._last_quaternion = model_q
        self.absolute_quaternion = model_q
        relative_q = normalize_quaternion(quaternion_multiply(
            quaternion_conjugate(self.level_reference), model_q))
        # Reverse roll and pitch directions while keeping yaw unchanged.
        display_q = (relative_q[0], -relative_q[1], -relative_q[2], relative_q[3])
        
        # Read each axis twist from the quaternion to avoid Euler singularities near 90 degrees.
        self.roll = wrap_degrees(math.degrees(2.0 * math.atan2(display_q[1], display_q[0])))
        self.pitch = wrap_degrees(math.degrees(2.0 * math.atan2(display_q[2], display_q[0])))
        self.raw_yaw = wrap_degrees(math.degrees(2.0 * math.atan2(display_q[3], display_q[0])))
        if self.yaw_offset is None:
            self.yaw_offset = self.raw_yaw
        self.yaw = wrap_degrees(self.raw_yaw + self.yaw_bias - self.yaw_offset)
        self.quaternion = with_yaw(display_q, self.yaw)


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, output: queue.Queue) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.output = output
        self.stop_event = threading.Event()
        self.connection = None

    def stop(self) -> None:
        self.stop_event.set()
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass

    def run(self) -> None:
        try:
            self.connection = serial.Serial(self.port, self.baud, timeout=0.2)
            self.output.put(("connected", self.port))
            while not self.stop_event.is_set():
                try:
                    line = self.connection.readline().decode("utf-8", errors="ignore").strip()
                except Exception as error:
                    if not self.stop_event.is_set():
                        self.output.put(("error", str(error)))
                    break
                if line:
                    received_ms = int(time.monotonic() * 1000)
                    self.output.put(("line", (line, received_ms)))
        except Exception as error:
            self.output.put(("error", str(error)))
        finally:
            if self.connection is not None:
                try:
                    self.connection.close()
                except Exception:
                    pass
            self.output.put(("closed", None))


class DroneCanvas(tk.Canvas):
    """Small orthographic renderer for an X-frame quadcopter."""

    def __init__(self, parent, **kwargs) -> None:
        super().__init__(parent, background="#080d14", highlightthickness=0, **kwargs)
        self.quaternion = (1.0, 0.0, 0.0, 0.0)
        self.bind("<Configure>", lambda _event: self.draw())

    def set_quaternion(self, quaternion: tuple[float, float, float, float]) -> None:
        self.quaternion = quaternion
        self.draw()

    def project(self, point: tuple[float, float, float], scale: float,
                cx: float, cy: float) -> tuple[float, float, float]:
        # The model nose points opposite to the sensor's forward axis.
        x, y, z = rotate_by_quaternion(self.quaternion,
                                       (-point[0], -point[1], point[2]))
        # A fixed camera above the front edge gives the model a readable 3D tilt.
        elev = math.radians(25)
        sx = x
        sy = y * math.sin(elev) + z * math.cos(elev)
        depth = -y * math.cos(elev) + z * math.sin(elev)
        return cx + sx * scale, cy - sy * scale, depth

    def draw(self) -> None:
        self.delete("all")
        width, height = max(1, self.winfo_width()), max(1, self.winfo_height())
        cx, cy = width * 0.5, height * 0.52
        scale = min(width, height) * 0.30
        # Subtle floor grid and center marks.
        for index in range(-4, 5):
            x = cx + index * scale * 0.42
            self.create_line(x, cy + scale * 0.82, cx + index * scale * 0.25,
                             cy + scale * 1.13, fill="#131e2b")
        self.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="#2d435c", outline="")

        # Cross arms and motors in an X layout.
        rotors = [(-0.86, -0.86), (0.86, -0.86), (0.86, 0.86), (-0.86, 0.86)]
        for x, y in rotors:
            start = self.project((0, 0, 0.05), scale, cx, cy)
            end = self.project((x, y, 0.05), scale, cx, cy)
            self.create_line(start[0], start[1], end[0], end[1], fill="#52657b", width=12,
                             capstyle=tk.ROUND)
            self.create_line(start[0], start[1], end[0], end[1], fill="#a7b9cb", width=5,
                             capstyle=tk.ROUND)

        # Rotor disks and two small propeller blades per motor.
        for index, (x, y) in enumerate(rotors):
            radius = scale * 0.25
            disk_color = "#173746" if index % 2 == 0 else "#1d3045"
            ring = [self.project((x + 0.25 * math.cos(step * math.tau / 16),
                                  y + 0.25 * math.sin(step * math.tau / 16), 0.12),
                                 scale, cx, cy) for step in range(16)]
            self.create_polygon(*[coord for point in ring for coord in point[:2]],
                                fill=disk_color, outline="#4d718a", width=2)
            for dx, dy in ((0.19, 0), (0, 0.19)):
                blade_start = self.project((x - dx, y - dy, 0.135), scale, cx, cy)
                blade_end = self.project((x + dx, y + dy, 0.135), scale, cx, cy)
                self.create_line(blade_start[0], blade_start[1], blade_end[0], blade_end[1],
                                 fill="#92a9be", width=3, capstyle=tk.ROUND)
            rotor = self.project((x, y, 0.12), scale, cx, cy)
            self.create_oval(rotor[0] - 4, rotor[1] - 4, rotor[0] + 4, rotor[1] + 4,
                             fill=CYAN, outline="")

        # Faceted central frame, projected from a shallow 3D body.
        body_top = [(-0.31, -0.27, 0.19), (0.0, -0.39, 0.19),
                    (0.31, -0.27, 0.19), (0.39, 0.0, 0.19),
                    (0.31, 0.27, 0.19), (0.0, 0.39, 0.19),
                    (-0.31, 0.27, 0.19), (-0.39, 0.0, 0.19)]
        pts = [self.project(point, scale, cx, cy) for point in body_top]
        flat = [coord for point in pts for coord in point[:2]]
        self.create_polygon(*flat, fill="#234e9b", outline="#79a5ff", width=2)
        nose = [self.project((-0.14, -0.23, 0.205), scale, cx, cy),
                self.project((0, -0.37, 0.205), scale, cx, cy),
                self.project((0.14, -0.23, 0.205), scale, cx, cy),
                self.project((0, -0.08, 0.205), scale, cx, cy)]
        self.create_polygon(*[coord for point in nose for coord in point[:2]],
                             fill="#63a1ff", outline="#9fc6ff", width=1)
        self.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill="#d8e7ff", outline="")
        self.create_text(18, height - 18, anchor="w", text="3D ATTITUDE  ·  X FRAME",
                         fill="#70849a", font=(FONT, 8, "bold"))


class TelemetryApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("AeroScope · LSM6DSV & BMP388")
        self.geometry("1370x840")
        self.minsize(1050, 700)
        self.configure(background=BG)
        self.reader: SerialReader | None = None
        self.messages: queue.Queue = queue.Queue()
        self.mag_calibration = MagnetometerCalibration()
        self.attitude = Attitude(self.mag_calibration)
        self.last_rx = 0.0
        self.link_up = False
        self.sample_count = 0
        self.last_chart: list[tuple[float, float, float]] = []
        self.latest_imu_data: dict[str, object] = {}
        self.latest_bmp_data: dict[str, object] = {
            "bmp_temp_c": "", "pressure_hpa": "", "altitude_m": ""}
        self.record_file = None
        self.record_writer = None
        self.record_path: Path | None = None
        self.record_rows = 0
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Chưa kết nối")
        self._build_styles()
        self._build_ui()
        self.refresh_ports()
        self.after(50, self.process_messages)
        self.after(500, self.refresh_link_status)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        if imufusion is None or np is None:
            self._log("Thiếu bộ lọc Fusion. Chạy: py -m pip install -r requirements-app.txt", RED)
        elif self.mag_calibration.calibrated:
            self._log("Đã nạp hiệu chuẩn từ kế QMC6309 đã lưu.", GREEN)
        else:
            self._log("QMC6309 chưa hiệu chuẩn; hãy dùng nút HIỆU CHUẨN TỪ KẾ để giảm sai số yaw.", AMBER)

    def _build_styles(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TCombobox", fieldbackground=PANEL_2, background=PANEL_2,
                        foreground=TEXT, arrowcolor=CYAN, bordercolor=LINE,
                        lightcolor=LINE, darkcolor=LINE, padding=7)
        style.map("TCombobox", fieldbackground=[("readonly", PANEL_2)],
                  foreground=[("readonly", TEXT)])

    def _label(self, parent, text: str, *, size=10, color=TEXT, bold=False, **kwargs):
        return tk.Label(parent, text=text, bg=kwargs.pop("bg", parent["bg"]), fg=color,
                        font=(FONT, size, "bold" if bold else "normal"), **kwargs)

    def _panel(self, parent, *, padx=16, pady=14):
        frame = tk.Frame(parent, bg=PANEL, highlightbackground=LINE, highlightthickness=1)
        frame.pack_propagate(True)
        return frame

    def _build_ui(self) -> None:
        header = tk.Frame(self, bg=BG, height=75)
        header.pack(fill="x", padx=22, pady=(18, 13))
        brand = tk.Frame(header, bg=BG)
        brand.pack(side="left", fill="y")
        self._label(brand, "AEROSCOPE", size=19, color=TEXT, bold=True).pack(anchor="w")
        self._label(brand, "ESP32  /  FLIGHT SENSOR MONITOR", size=8,
                    color=MUTED, bold=True).pack(anchor="w", pady=(3, 0))

        connect = tk.Frame(header, bg=BG)
        connect.pack(side="right", fill="y")
        self.port_box = ttk.Combobox(connect, textvariable=self.port_var, width=15,
                                     state="readonly", values=[])
        self.port_box.pack(side="left", padx=(0, 8), pady=9)
        self._button(connect, "↻", self.refresh_ports, subdued=True, width=3).pack(side="left", padx=(0, 8))
        self.connect_button = self._button(connect, "KẾT NỐI", self.toggle_connection,
                                           accent=True, width=11)
        self.connect_button.pack(side="left", padx=(0, 14))
        status = tk.Frame(connect, bg=BG)
        status.pack(side="left", pady=12)
        self.status_dot = tk.Label(status, text="●", bg=BG, fg=AMBER, font=(FONT, 10))
        self.status_dot.pack(side="left", padx=(0, 6))
        self._label(status, "LINK", size=8, color=MUTED, bold=True).pack(side="left")

        separator = tk.Frame(self, bg=LINE, height=1)
        separator.pack(fill="x", padx=22)

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=22, pady=18)
        left = tk.Frame(body, bg=BG)
        left.pack(side="left", fill="both", expand=True)
        right = tk.Frame(body, bg=BG, width=385)
        right.pack(side="right", fill="y", padx=(16, 0))
        right.pack_propagate(False)
        self.metrics = {}

        attitude_panel = self._panel(left)
        attitude_panel.pack(fill="both", expand=True)
        attitude_head = tk.Frame(attitude_panel, bg=PANEL)
        attitude_head.pack(fill="x", padx=18, pady=(15, 10))
        title_box = tk.Frame(attitude_head, bg=PANEL)
        title_box.pack(side="left")
        self._label(title_box, "TƯ THẾ MÁY BAY", size=11, color=TEXT, bold=True).pack(anchor="w")
        self._label(title_box, "Góc ước lượng từ LSM6DSV và từ kế QMC6309",
                    size=9, color=MUTED).pack(anchor="w", pady=(4, 0))
        mag_state = "QMC6309 ĐÃ HIỆU CHUẨN" if self.mag_calibration.calibrated else "QMC6309 CẦN HIỆU CHUẨN"
        mag_color = GREEN if self.mag_calibration.calibrated else AMBER
        self.mag_status_label = self._label(title_box, mag_state, size=8,
                                            color=mag_color, bold=True)
        self.mag_status_label.pack(anchor="w", pady=(4, 0))
        attitude_actions = tk.Frame(attitude_head, bg=PANEL)
        attitude_actions.pack(side="right", pady=2)
        self.mag_cal_button = self._button(attitude_actions, "HIỆU CHUẨN TỪ KẾ",
                                           self.toggle_mag_calibration,
                                           subdued=True, width=18)
        self.mag_cal_button.pack(side="left", padx=(0, 8))
        self.level_button = self._button(attitude_actions, "ĐẶT CÂN BẰNG", self.set_level,
                                         subdued=True, width=16)
        self.level_button.pack(side="left", padx=(0, 8))
        self.zero_button = self._button(attitude_actions, "ĐẶT YAW = 0°", self.zero_yaw,
                                        subdued=True, width=15)
        self.zero_button.pack(side="left")

        view_frame = tk.Frame(attitude_panel, bg="#080d14", highlightbackground="#1d2b3c",
                              highlightthickness=1)
        view_frame.pack(fill="both", expand=True, padx=16, pady=(0, 12))
        self.drone = DroneCanvas(view_frame)
        self.drone.pack(fill="both", expand=True)

        angle_row = tk.Frame(attitude_panel, bg=PANEL)
        angle_row.pack(fill="x", padx=16, pady=(0, 15))
        self.angle_labels = {}
        for name, color, desc in (("ROLL", CYAN, "LĂN"), ("PITCH", BLUE, "CHÚC NGẨNG"),
                                  ("YAW", GREEN, "HƯỚNG")):
            tile = tk.Frame(angle_row, bg=PANEL_2, highlightbackground=LINE, highlightthickness=1)
            tile.pack(side="left", fill="x", expand=True, padx=(0, 8))
            self._label(tile, desc, size=8, color=MUTED, bold=True).pack(anchor="w", padx=12, pady=(9, 2))
            value = self._label(tile, "0.0°", size=18, color=color, bold=True)
            value.pack(anchor="w", padx=12, pady=(0, 9))
            self.angle_labels[name] = value

        charts = self._panel(left)
        charts.pack(fill="x", pady=(14, 0))
        chart_head = tk.Frame(charts, bg=PANEL)
        chart_head.pack(fill="x", padx=16, pady=(11, 4))
        self._label(chart_head, "GÓC THEO THỜI GIAN", size=9, color=TEXT, bold=True).pack(side="left")
        self._label(chart_head, "ROLL  /  PITCH  /  YAW", size=8, color=MUTED).pack(side="right")
        self.chart = tk.Canvas(charts, height=88, bg=PANEL, highlightthickness=0)
        self.chart.pack(fill="x", padx=14, pady=(0, 9))
        self.chart.bind("<Configure>", lambda _event: self.draw_chart())

        self._make_metric_section(right, "BMP388 · KHÍ ÁP", [
            ("pressure", "Áp suất", "—", "hPa"),
            ("altitude", "Độ cao ước tính", "—", "m"),
            ("bmp_temp", "Nhiệt độ", "—", "°C"),
        ])
        self._make_metric_section(right, "LSM6DSV · CHUYỂN ĐỘNG", [
            ("accel", "Gia tốc XYZ", "— / — / —", "m/s²"),
            ("gyro", "Con quay XYZ", "— / — / —", "°/s"),
            ("mag", "QMC6309 XYZ", "— / — / —", "raw"),
            ("imu_temp", "Nhiệt độ IMU", "—", "°C"),
        ])
        log_panel = self._panel(right)
        log_panel.pack(fill="both", expand=True, pady=(14, 0))
        log_head = tk.Frame(log_panel, bg=PANEL)
        log_head.pack(fill="x", padx=14, pady=(12, 8))
        self._label(log_head, "SERIAL CONSOLE", size=9, color=TEXT, bold=True).pack(side="left")
        self._label(log_head, "115200 BAUD", size=8, color=MUTED, bold=True).pack(side="right")
        self.record_button = self._button(log_head, "GHI CSV", self.toggle_recording,
                                          subdued=True, width=9)
        self.record_button.pack(side="right", padx=(0, 8))
        self.console = tk.Text(log_panel, bg="#0a1018", fg="#78d9a8", insertbackground=TEXT,
                                relief="flat", wrap="word", height=8,
                                font=("Consolas", 8), padx=10, pady=8,
                                state="disabled")
        self.console.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self._log("Sẵn sàng. Chọn cổng COM để bắt đầu.", MUTED)

        footer = tk.Frame(self, bg=BG)
        footer.pack(fill="x", padx=22, pady=(0, 13))
        self._label(footer, "DỮ LIỆU TỪ FIRMWARE CSV  ·  IMU ~100 Hz  ·  BMP388 ~2 Hz",
                    size=8, color=MUTED).pack(side="left")
        self.sample_label = self._label(footer, "0 mẫu", size=8, color=MUTED)
        self.sample_label.pack(side="right")

    def _make_metric_section(self, parent, title: str, rows: list[tuple[str, str, str, str]]) -> None:
        panel = self._panel(parent)
        panel.pack(fill="x", pady=(0, 13))
        self._label(panel, title, size=9, color=CYAN, bold=True).pack(anchor="w", padx=15, pady=(12, 8))
        for key, label, initial, unit in rows:
            row = tk.Frame(panel, bg=PANEL)
            row.pack(fill="x", padx=15, pady=(0, 9))
            self._label(row, label, size=9, color=MUTED).pack(side="left")
            right = tk.Frame(row, bg=PANEL)
            right.pack(side="right")
            value = self._label(right, initial, size=10, color=TEXT, bold=True)
            value.pack(side="left")
            self._label(right, unit, size=8, color=MUTED).pack(side="left", padx=(4, 0))
            self.metrics[key] = value

    def _button(self, parent, text: str, command, *, accent=False, subdued=False, width=None):
        bg = "#257a91" if accent else (PANEL_2 if subdued else PANEL)
        fg = "#f5fdff" if accent else TEXT
        button = tk.Button(parent, text=text, command=command, bg=bg, fg=fg,
                           activebackground="#3195ad" if accent else "#22344a",
                           activeforeground=TEXT, relief="flat", bd=0,
                           font=(FONT, 9, "bold"), padx=10, pady=8,
                           cursor="hand2", width=width)
        return button

    def refresh_ports(self) -> None:
        if list_ports is None:
            self._log("Thiếu pyserial. Chạy: pip install -r requirements-app.txt", RED)
            return
        ports = list(list_ports.comports())
        values = [port.device for port in ports]
        self.port_box["values"] = values
        if self.port_var.get() not in values:
            self.port_var.set(values[0] if values else "")
        if not values:
            self._log("Không tìm thấy cổng Serial.", AMBER)

    def toggle_connection(self) -> None:
        if self.reader and self.reader.is_alive():
            self.reader.stop()
            self.connect_button.configure(text="KẾT NỐI")
            self._set_status("Đang ngắt", AMBER)
            return
        if serial is None:
            self._log("Thiếu pyserial. Chạy: pip install -r requirements-app.txt", RED)
            return
        if imufusion is None or np is None:
            self._log("Thiếu Fusion/Numpy. Chạy: py -m pip install -r requirements-app.txt", RED)
            return
        port = self.port_var.get()
        if not port:
            self.refresh_ports()
            return
        self.attitude.reset()
        self.sample_count = 0
        self.last_chart.clear()
        self.connect_button.configure(text="NGẮT KẾT NỐI")
        self._set_status("Đang kết nối", AMBER)
        self._log(f"Đang mở {port} ở 115200 baud…", MUTED)
        self.reader = SerialReader(port, 115200, self.messages)
        self.reader.start()

    def _set_status(self, text: str, color: str) -> None:
        self.status_var.set(text)
        self.status_dot.configure(fg=color)

    def process_messages(self) -> None:
        while True:
            try:
                kind, value = self.messages.get_nowait()
            except queue.Empty:
                break
            if kind == "connected":
                self.link_up = True
                self.last_rx = time.monotonic()
                self._set_status(f"Đã kết nối {value}", GREEN)
                self._log(f"Đã kết nối {value} @ 115200 baud", GREEN)
                self._log("Giữ cảm biến đứng yên 3 giây đầu để bộ lọc ước lượng bias gyro.", AMBER)
            elif kind == "closed":
                self.link_up = False
                self._stop_recording()
                self._set_status("Chưa kết nối", AMBER)
                self.connect_button.configure(text="KẾT NỐI")
            elif kind == "error":
                self._set_status("Lỗi Serial", RED)
                self._log(value, RED)
            elif kind == "line":
                self.last_rx = time.monotonic()
                line, received_ms = value
                if not self.parse_line(line, received_ms):
                    self._record_event("RAW", line)
                    self._log(line)
        self.after(30, self.process_messages)

    def refresh_link_status(self) -> None:
        if self.link_up and time.monotonic() - self.last_rx > 2.0:
            self._set_status("Chờ dữ liệu", AMBER)
        self.after(500, self.refresh_link_status)

    def toggle_recording(self) -> None:
        if self.record_file is not None:
            self._stop_recording()
            return
        if not self.link_up:
            self._log("Kết nối ESP32 trước khi bắt đầu ghi CSV.", AMBER)
            return
        default_name = f"aeroscope_{datetime.now():%Y%m%d_%H%M%S}.csv"
        filename = filedialog.asksaveasfilename(
            parent=self,
            title="Lưu dữ liệu IMU và BMP",
            initialdir=str(Path(__file__).parent),
            initialfile=default_name,
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("Tất cả tệp", "*.*")],
        )
        if not filename:
            return
        output = None
        try:
            output = open(filename, "w", newline="", encoding="utf-8-sig")
            writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
            writer.writeheader()
            output.flush()
        except OSError as error:
            if output is not None:
                output.close()
            self._log(f"Không mở được file CSV: {error}", RED)
            return
        self.record_file = output
        self.record_writer = writer
        self.record_path = Path(filename)
        self.record_rows = 0
        self.record_button.configure(text="DỪNG GHI", bg="#8b3948",
                                     activebackground="#a74859")
        self._log(f"Đang ghi dữ liệu vào: {self.record_path}", GREEN)

    def _record_event(self, packet_type: str, raw_line: str,
                      *, firmware_ms: str = "") -> None:
        if self.record_writer is None or self.record_file is None:
            return
        row = {field: "" for field in CSV_FIELDS}
        row.update(self.latest_imu_data)
        row.update(self.latest_bmp_data)
        row.update({
            "host_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "packet_type": packet_type,
            "firmware_ms": firmware_ms,
            "roll_deg": self.attitude.roll,
            "pitch_deg": self.attitude.pitch,
            "yaw_deg": self.attitude.yaw,
            "raw_serial": raw_line,
        })
        try:
            self.record_writer.writerow(row)
            self.record_rows += 1
            if self.record_rows % 100 == 0:
                self.record_file.flush()
        except (OSError, csv.Error) as error:
            self._stop_recording(report=False)
            self._log(f"Lỗi ghi CSV, đã dừng ghi: {error}", RED)

    def _stop_recording(self, *, report: bool = True) -> None:
        output = self.record_file
        if output is None:
            return
        path = self.record_path
        rows = self.record_rows
        self.record_file = None
        self.record_writer = None
        self.record_path = None
        self.record_rows = 0
        try:
            output.flush()
            output.close()
        except OSError as error:
            self._log(f"Không hoàn tất được file CSV: {error}", RED)
            report = False
        self.record_button.configure(text="GHI CSV", bg=PANEL_2,
                                     activebackground="#22344a")
        if report:
            self._log(f"Đã lưu {rows:,} dòng: {path}", GREEN)

    def parse_line(self, line: str, received_ms: int | None = None) -> bool:
        fields = [part.strip() for part in line.split(",")]
        if not fields:
            return False
        try:
            if fields[0] == "IMU" and len(fields) >= 8:
                number = lambda idx: float(fields[idx])
                mx = my = mz = None
                if (len(fields) >= 11 and
                        all(fields[idx].upper() != "NA" for idx in (8, 9, 10))):
                    mx, my, mz = number(8), number(9), number(10)
                sample_ms = received_ms if received_ms is not None else int(time.monotonic() * 1000)
                firmware_ms = ""
                if len(fields) >= 12 and fields[11]:
                    firmware_ms = fields[11]
                    try:
                        sample_ms = int(number(11))
                    except ValueError:
                        pass
                sample = {"ax": number(1), "ay": number(2), "az": number(3),
                          "gx": number(4), "gy": number(5), "gz": number(6),
                          "temp": number(7), "mx": mx, "my": my, "mz": mz,
                          "ms": sample_ms}
                self.update_imu(sample, raw_line=line, firmware_ms=firmware_ms)
                return True
            if fields[0] == "BMP" and len(fields) >= 4:
                temperature, pressure, altitude = map(float, fields[1:4])
                self._metric("bmp_temp", f"{temperature:.2f}")
                self._metric("pressure", f"{pressure:.2f}")
                self._metric("altitude", f"{altitude:.2f}")
                self.latest_bmp_data = {
                    "bmp_temp_c": temperature,
                    "pressure_hpa": pressure,
                    "altitude_m": altitude,
                }
                self._record_event("BMP", line)
                return True
        except (ValueError, IndexError):
            return False
        return False

    def update_imu(self, sample: dict, *, raw_line: str = "",
                   firmware_ms: str = "") -> None:
        self.mag_calibration.add(sample["mx"], sample["my"], sample["mz"])
        self.attitude.update(sample)
        roll, pitch, yaw = self.attitude.roll, self.attitude.pitch, self.attitude.yaw
        self.drone.set_quaternion(self.attitude.quaternion)
        self.latest_imu_data = {
            "firmware_ms": firmware_ms,
            "ax_ms2": sample["ax"], "ay_ms2": sample["ay"], "az_ms2": sample["az"],
            "gx_dps": sample["gx"], "gy_dps": sample["gy"], "gz_dps": sample["gz"],
            "imu_temp_c": sample["temp"],
            "mx_raw": sample["mx"], "my_raw": sample["my"], "mz_raw": sample["mz"],
            "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw,
            "quat_w": self.attitude.quaternion[0], "quat_x": self.attitude.quaternion[1],
            "quat_y": self.attitude.quaternion[2], "quat_z": self.attitude.quaternion[3],
        }
        self._record_event("IMU", raw_line, firmware_ms=firmware_ms)
        self.angle_labels["ROLL"].configure(text=f"{roll:+.1f}°")
        self.angle_labels["PITCH"].configure(text=f"{pitch:+.1f}°")
        self.angle_labels["YAW"].configure(text=f"{yaw:+.1f}°")
        self._metric("accel", f"{sample['ax']:+.2f} / {sample['ay']:+.2f} / {sample['az']:+.2f}")
        self._metric("gyro", f"{sample['gx']:+.1f} / {sample['gy']:+.1f} / {sample['gz']:+.1f}")
        if sample["mx"] is None:
            self._metric("mag", "NA / NA / NA")
        else:
            self._metric("mag", f"{sample['mx']:.0f} / {sample['my']:.0f} / {sample['mz']:.0f}")
        self._metric("imu_temp", f"{sample['temp']:.2f}")
        self.sample_count += 1
        self.sample_label.configure(text=f"{self.sample_count:,} mẫu")
        self.last_chart.append((roll, pitch, yaw))
        if len(self.last_chart) > 130:
            del self.last_chart[:-130]
        if self.sample_count % 3 == 0:
            self.draw_chart()

    def _metric(self, key: str, value: str) -> None:
        label = self.metrics.get(key)
        if label:
            label.configure(text=value)

    def draw_chart(self) -> None:
        if not hasattr(self, "chart"):
            return
        canvas = self.chart
        canvas.delete("all")
        width, height = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        top, bottom, left, right = 10, height - 8, 30, width - 10
        for angle in (-180, 0, 180):
            y = top + (180 - angle) / 360 * (bottom - top)
            canvas.create_line(left, y, right, y, fill="#223144", dash=(2, 4))
            canvas.create_text(2, y, text=str(angle), anchor="w", fill="#62758b",
                               font=(FONT, 7))
        if len(self.last_chart) < 2:
            canvas.create_text(width / 2, height / 2, text="Chờ dữ liệu IMU…",
                               fill=MUTED, font=(FONT, 8))
            return
        colors = (CYAN, BLUE, GREEN)
        for column, color in enumerate(colors):
            points = []
            values = self.last_chart
            for index, row in enumerate(values):
                x = left + index / max(1, len(values) - 1) * (right - left)
                angle = clamp(row[column], -180, 180)
                y = top + (180 - angle) / 360 * (bottom - top)
                points.extend((x, y))
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=2, smooth=True)

    def zero_yaw(self) -> None:
        self.attitude.zero_yaw()
        self._log("Yaw đã đặt về 0° tại hướng hiện tại.", CYAN)

    def set_level(self) -> None:
        if not self.link_up or not self.attitude.set_level():
            self._log("Kết nối IMU trước khi đặt vị trí cân bằng.", AMBER)
            return
        self.last_chart.clear()
        self.draw_chart()
        self.drone.set_quaternion(self.attitude.quaternion)
        self.angle_labels["ROLL"].configure(text="+0.0°")
        self.angle_labels["PITCH"].configure(text="+0.0°")
        self.angle_labels["YAW"].configure(text=f"{self.attitude.yaw:+.1f}°")
        self._log("Đã đặt tư thế hiện tại làm cân bằng: roll/pitch về 0°, giữ nguyên yaw.", CYAN)

    def toggle_mag_calibration(self) -> None:
        if not self.link_up:
            self._log("Kết nối ESP32 trước khi hiệu chuẩn từ kế.", AMBER)
            return
        if not self.mag_calibration.collecting:
            self.mag_calibration.start()
            self.mag_cal_button.configure(text="LƯU HIỆU CHUẨN")
            self.mag_status_label.configure(text="ĐANG THU MẪU TỪ KẾ…", fg=AMBER)
            self._log("Xoay cảm biến chậm đủ mọi hướng trong 15–30 giây; để xa kim loại và dây có dòng lớn.", AMBER)
            return

        success, message = self.mag_calibration.finish()
        self.mag_cal_button.configure(text="HIỆU CHUẨN TỪ KẾ")
        if success:
            self.attitude.restart_ahrs()
            self.mag_status_label.configure(text="QMC6309 ĐÃ HIỆU CHUẨN", fg=GREEN)
            self._log(message, GREEN)
            self._log(f"Offset QMC6309: {self.mag_calibration.offset}; scale: {self.mag_calibration.scale}", MUTED)
        else:
            self.mag_status_label.configure(text="QMC6309 CẦN THU THÊM MẪU", fg=RED)
            self._log(message, RED)

    def _log(self, message: str, color: str | None = None) -> None:
        if not hasattr(self, "console"):
            return
        self.console.configure(state="normal")
        self.console.insert("end", message + "\n")
        if color:
            line = self.console.index("end-2l")
            tag = f"line_{time.monotonic_ns()}"
            self.console.tag_add(tag, line, "end-1c")
            self.console.tag_configure(tag, foreground=color)
        self.console.see("end")
        # Keep the live console bounded while preserving recent diagnostics.
        lines = int(self.console.index("end-1c").split(".")[0])
        if lines > 120:
            self.console.delete("1.0", "20.0")
        self.console.configure(state="disabled")

    def on_close(self) -> None:
        self._stop_recording()
        if self.reader:
            self.reader.stop()
        self.destroy()


if __name__ == "__main__":
    app = TelemetryApp()
    app.mainloop()
