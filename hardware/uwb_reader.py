import glob
import json
import math
import os
import re
import threading
import time
from datetime import datetime

from config import APP_SETTINGS, COMM_SETTINGS, DRIVE_SETTINGS, UWB_ANCHORS_M


class UWBReceiver:
    def __init__(self, ports, baudrate=115200, excluded_ports=None):
        self.ports = ports
        self.excluded_ports = set(excluded_ports or [])
        self.baudrates = baudrate if isinstance(baudrate, (list, tuple)) else [baudrate]
        self.serial = None
        self.running = False
        self.lock = threading.Lock()
        self.last_connect_attempt = 0.0
        self.esp32_wifi_aps = []
        self.wifi_collection_enabled = False
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.origin_calibrated = False
        self.last_valid_x = None
        self.last_valid_y = None
        self.lowpass_x = None
        self.lowpass_y = None
        self.drive_x = None
        self.drive_y = None
        self.display_x = None
        self.display_y = None
        self.invalid_position_count = 0
        self.position_outlier_count = 0
        self.position_outlier_reason = ""
        self.last_position_clamped = False
        self.last_position_holding = False
        self._last_distance_log = 0.0
        self._last_imu_command = 0.0
        self._imu_command_index = 0
        self._imu_command_first_sent = None
        self.anchor_line_distances = {
            "d1": None,
            "d2": None,
            "d3": None,
            "d4": None,
        }
        self.anchor_line_seen_monotonic = {
            "d1": None,
            "d2": None,
            "d3": None,
            "d4": None,
        }

        self.data = {
            "x": None,
            "y": None,
            "display_x": None,
            "display_y": None,
            "drive_x": None,
            "drive_y": None,
            "sample_x": None,
            "sample_y": None,
            "map_x": None,
            "map_y": None,
            "unbounded_map_x": None,
            "unbounded_map_y": None,
            "raw_x": None,
            "raw_y": None,
            "raw_distances": {},
            "filtered_distances": {},
            "x_offset": 0.0,
            "y_offset": 0.0,
            "uwb_map_scale_x": DRIVE_SETTINGS.get("UWB_MAP_SCALE_X", 1.0),
            "uwb_map_scale_y": DRIVE_SETTINGS.get("UWB_MAP_SCALE_Y", 1.0),
            "uwb_origin_calibrated": False,
            "uwb_position_geofence_valid": False,
            "uwb_position_clamped": False,
            "uwb_position_holding": False,
            "uwb_invalid_position_count": 0,
            "uwb_position_outlier_count": 0,
            "uwb_position_outlier_reason": "",
            "uwb_geofence_margin_m": DRIVE_SETTINGS.get("UWB_GEOFENCE_MARGIN_M", 0.0),
            "d1": None,
            "d2": None,
            "d3": None,
            "d4": None,
            "rssi1": None,
            "rssi2": None,
            "rssi3": None,
            "rssi4": None,
            "uwb_rssi": None,
            "uwb_valid": False,
            "uwb_anchor_count": 0,
            "uwb_rssi_count": 0,
            "uwb_distance_error_m": None,
            "uwb_distance_error_max_m": None,
            "uwb_last_seen": None,
            "uwb_connected": False,
            "uwb_port": None,
            "uwb_baud": None,
            "uwb_parse_ok": False,
            "uwb_last_raw": "",
            "uwb_raw": "",
            "uwb_error": "",
            "esp32_wifi_count": 0,
            "esp32_wifi_last_seen": None,
            "esp32_wifi_last_raw": "",
            "sensor_t": None,
            "uwb_t": None,
            "imu_t": None,
            "imu_ax": None,
            "imu_ay": None,
            "imu_az": None,
            "imu_gx": None,
            "imu_gy": None,
            "imu_gz": None,
            "imu_ok": False,
            "imu_last_seen": None,
            "imu_last_seen_monotonic": None,
            "imu_last_raw": "",
            "imu_source": "esp32_json",
            "imu_command": COMM_SETTINGS.get("ESP32_IMU_COMMAND", "IMU_ON"),
            "imu_command_last_sent": None,
            "imu_command_count": 0,
            "imu_no_response_sec": None,
            "imu_error": "",
            "enc_l": 0,
            "enc_r": 0,
        }

    def _port_candidates(self):
        candidates = []
        patterns = [
            "/dev/ttyUSB*",
            "/dev/ttyACM*",
            "/dev/serial/by-id/*",
        ]

        for port in self.ports:
            if port in self.excluded_ports:
                continue
            if port not in candidates:
                candidates.append(port)

        for pattern in patterns:
            for port in sorted(glob.glob(pattern)):
                if port in self.excluded_ports:
                    continue
                if port not in candidates:
                    candidates.append(port)

        return candidates

    def connect(self):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial is required for ESP32/UWB serial input.") from exc

        for port in self._port_candidates():
            if not os.path.exists(port):
                print(f"Serial skip {port}: not found", flush=True)
                continue

            for baudrate in self.baudrates:
                try:
                    try:
                        os.chmod(port, 0o666)
                    except OSError:
                        pass

                    candidate = self._open_serial(serial, port, baudrate, timeout=0.05)
                    candidate.reset_input_buffer()
                    self.serial = candidate
                    with self.lock:
                        self.data["uwb_port"] = port
                        self.data["uwb_baud"] = baudrate
                        self.data["uwb_error"] = "waiting for UWB serial data"

                    if self._probe_serial_data(port, baudrate):
                        print(f"ESP32/UWB serial ready: {port} @ {baudrate}", flush=True)
                        return True

                    try:
                        candidate.close()
                    except Exception:
                        pass
                    self.serial = None
                    with self.lock:
                        self.data["uwb_connected"] = False
                        self.data["uwb_error"] = "no UWB serial data"
                    print(f"Serial skip {port} @ {baudrate}: no data", flush=True)
                except Exception as exc:
                    with self.lock:
                        self.data["uwb_error"] = str(exc)
                    print(f"Serial skip {port} @ {baudrate}: {exc}", flush=True)

        print("ESP32/UWB serial not found.", flush=True)
        with self.lock:
            self.data["uwb_connected"] = False
            self.data["uwb_port"] = None
            self.data["uwb_baud"] = None
            self.data["uwb_error"] = "serial port not found"
        return False

    @staticmethod
    def _open_serial(serial_module, port, baudrate, timeout=0.05):
        try:
            return serial_module.Serial(port, baudrate, timeout=timeout, exclusive=True)
        except TypeError:
            return serial_module.Serial(port, baudrate, timeout=timeout)

    def _probe_serial_data(self, port, baudrate):
        deadline = time.monotonic() + DRIVE_SETTINGS.get("UWB_CONNECT_PROBE_SEC", 2.5)
        last_probe_command = 0.0
        while time.monotonic() < deadline and self.serial:
            try:
                now = time.monotonic()
                if now - last_probe_command >= 0.4:
                    self._send_esp32_imu_command(force=True)
                    last_probe_command = now

                if self.serial.in_waiting <= 0:
                    time.sleep(0.02)
                    continue

                line = self.serial.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                with self.lock:
                    self.data["uwb_connected"] = True
                    self.data["uwb_port"] = port
                    self.data["uwb_baud"] = baudrate
                    self.data["uwb_last_raw"] = line
                    self.data["uwb_raw"] = line
                    self.data["uwb_error"] = ""

                self._process_serial_line(line)
                return True
            except Exception as exc:
                with self.lock:
                    self.data["uwb_error"] = str(exc)
                return False

        return False

    def start(self):
        self.running = True
        threading.Thread(target=self.read_loop, daemon=True).start()

    def stop(self):
        self.running = False
        if self.serial:
            self.serial.close()
        with self.lock:
            self.data["uwb_connected"] = False

    def get_position(self):
        with self.lock:
            return self.data.get("drive_x"), self.data.get("drive_y")

    def get_display_position(self):
        with self.lock:
            return self.data.get("display_x"), self.data.get("display_y")

    def set_sample_position(self, x, y):
        with self.lock:
            self.data["sample_x"] = round(float(x), 3) if x is not None else None
            self.data["sample_y"] = round(float(y), 3) if y is not None else None

    def snapshot(self):
        with self.lock:
            return dict(self.data)

    def calibrate_origin(self, raw_x=None, raw_y=None, map_x=0.0, map_y=0.0):
        with self.lock:
            if raw_x is None:
                raw_x = self.data.get("raw_x")
            if raw_y is None:
                raw_y = self.data.get("raw_y")
            if raw_x is None or raw_y is None:
                return False

            map_x = 0.0 if map_x is None else float(map_x)
            map_y = 0.0 if map_y is None else float(map_y)
            scale_x = float(DRIVE_SETTINGS.get("UWB_MAP_SCALE_X", 1.0)) or 1.0
            scale_y = float(DRIVE_SETTINGS.get("UWB_MAP_SCALE_Y", 1.0)) or 1.0
            self.x_offset = float(raw_x) - (map_x / scale_x)
            self.y_offset = float(raw_y) - (map_y / scale_y)
            self.origin_calibrated = True
            self.last_valid_x = map_x
            self.last_valid_y = map_y
            self.lowpass_x = map_x
            self.lowpass_y = map_y
            self.drive_x = map_x
            self.drive_y = map_y
            self.display_x = map_x
            self.display_y = map_y
            self.invalid_position_count = 0
            self.position_outlier_count = 0
            self.position_outlier_reason = ""
            self.last_position_clamped = False
            self.last_position_holding = False
            self.data["x_offset"] = round(self.x_offset, 3)
            self.data["y_offset"] = round(self.y_offset, 3)
            self.data["x"] = round(map_x, 3)
            self.data["y"] = round(map_y, 3)
            self.data["display_x"] = round(map_x, 3)
            self.data["display_y"] = round(map_y, 3)
            self.data["drive_x"] = round(map_x, 3)
            self.data["drive_y"] = round(map_y, 3)
            self.data["sample_x"] = round(map_x, 3)
            self.data["sample_y"] = round(map_y, 3)
            self.data["map_x"] = round(map_x, 3)
            self.data["map_y"] = round(map_y, 3)
            self.data["unbounded_map_x"] = round(map_x, 3)
            self.data["unbounded_map_y"] = round(map_y, 3)
            self.data["uwb_origin_calibrated"] = True
            self.data["uwb_position_geofence_valid"] = True
            self.data["uwb_invalid_position_count"] = 0
            self.data["uwb_position_outlier_count"] = 0
            self.data["uwb_position_outlier_reason"] = ""
            self.data["uwb_position_clamped"] = False
            self.data["uwb_position_holding"] = False
            return True

    def clear_origin_calibration(self):
        with self.lock:
            self.x_offset = 0.0
            self.y_offset = 0.0
            self.origin_calibrated = False
            self.lowpass_x = None
            self.lowpass_y = None
            self.drive_x = None
            self.drive_y = None
            self.display_x = None
            self.display_y = None
            self.data["x_offset"] = 0.0
            self.data["y_offset"] = 0.0
            self.data["uwb_origin_calibrated"] = False
            return True

    def reset_position_status(self):
        with self.lock:
            self.invalid_position_count = 0
            self.lowpass_x = self.last_valid_x
            self.lowpass_y = self.last_valid_y
            self.position_outlier_count = 0
            self.position_outlier_reason = ""
            self.last_position_clamped = False
            self.last_position_holding = False
            self.data["uwb_invalid_position_count"] = 0
            self.data["uwb_position_outlier_count"] = 0
            self.data["uwb_position_outlier_reason"] = ""
            self.data["uwb_position_clamped"] = False
            self.data["uwb_position_holding"] = False
            return True

    def set_wifi_collection_enabled(self, enabled):
        with self.lock:
            self.wifi_collection_enabled = bool(enabled)
            if enabled:
                self.esp32_wifi_aps.clear()
                self.data["esp32_wifi_count"] = 0

    def get_esp32_wifi_aps(self, max_age_sec=10.0):
        now = time.monotonic()
        with self.lock:
            self.esp32_wifi_aps = [
                ap for ap in self.esp32_wifi_aps
                if now - ap.get("_seen_monotonic", now) <= max_age_sec
            ]
            aps = []
            for ap in self.esp32_wifi_aps:
                clean = dict(ap)
                clean.pop("_seen_monotonic", None)
                aps.append(clean)
            self.data["esp32_wifi_count"] = len(self.esp32_wifi_aps)
            return aps

    def _store_esp32_wifi_ap(self, ap, raw_line):
        now = time.monotonic()
        ap = dict(ap)
        ap["_seen_monotonic"] = now

        key = (
            str(ap.get("bssid", "")).lower(),
            str(ap.get("ssid", "")),
            str(ap.get("band", "")),
        )

        with self.lock:
            if not self.wifi_collection_enabled:
                return
            self.esp32_wifi_aps = [
                existing for existing in self.esp32_wifi_aps
                if (
                    str(existing.get("bssid", "")).lower(),
                    str(existing.get("ssid", "")),
                    str(existing.get("band", "")),
                ) != key and now - existing.get("_seen_monotonic", now) <= 10.0
            ]
            self.esp32_wifi_aps.append(ap)
            self.data["esp32_wifi_count"] = len(self.esp32_wifi_aps)
            self.data["esp32_wifi_last_seen"] = datetime.now().strftime("%H:%M:%S")
            self.data["esp32_wifi_last_raw"] = raw_line

    def read_loop(self):
        last_rx_time = 0.0
        serial_open_time = 0.0

        while self.running:
            try:
                if not self.serial:
                    now = time.monotonic()
                    if now - self.last_connect_attempt > 2.0:
                        self.last_connect_attempt = now
                        if self.connect():
                            serial_open_time = time.monotonic()
                            self._send_esp32_imu_command(force=True)

                elif self.serial:
                    self._send_esp32_imu_command()
                    self._update_imu_no_response_status()

                    if self.serial.in_waiting <= 0:
                        if last_rx_time > 0 and time.monotonic() - last_rx_time > DRIVE_SETTINGS["UWB_STALE_SEC"]:
                            with self.lock:
                                self.data["uwb_valid"] = False
                                self.data["uwb_error"] = "UWB serial data stale"

                        elif last_rx_time == 0 and serial_open_time > 0:
                            no_data_sec = time.monotonic() - serial_open_time
                            if no_data_sec > DRIVE_SETTINGS.get("UWB_NO_DATA_RECONNECT_SEC", 4.0):
                                with self.lock:
                                    self.data["uwb_connected"] = False
                                    self.data["uwb_error"] = "no UWB serial data"
                                    self.data["uwb_parse_ok"] = False
                                try:
                                    self.serial.close()
                                except Exception:
                                    pass
                                self.serial = None
                                serial_open_time = 0.0

                        time.sleep(0.02)
                        continue

                    line = self.serial.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        last_rx_time = time.monotonic()
                        with self.lock:
                            self.data["uwb_connected"] = True
                            self.data["uwb_last_raw"] = line
                            self.data["uwb_raw"] = line
                            self.data["uwb_error"] = ""
                        self._process_serial_line(line)

                time.sleep(0.02)

            except Exception as exc:
                print(f"UWB read error: {exc}", flush=True)
                try:
                    if self.serial:
                        self.serial.close()
                except Exception:
                    pass
                self.serial = None
                with self.lock:
                    self.data["uwb_connected"] = False
                    self.data["uwb_error"] = str(exc)
                time.sleep(0.2)

    def _send_esp32_imu_command(self, force=False):
        if not COMM_SETTINGS.get("ESP32_IMU_ENABLE", False):
            return False
        if not COMM_SETTINGS.get("ESP32_IMU_COMMAND_ENABLE", False):
            return False
        if not self.serial:
            return False

        now = time.monotonic()
        interval = COMM_SETTINGS.get("ESP32_IMU_COMMAND_INTERVAL_SEC", 2.0)
        if not force and now - self._last_imu_command < interval:
            return False

        command = self._next_esp32_imu_command()
        try:
            self.serial.write((command + "\n").encode("utf-8"))
            self.serial.flush()
        except Exception as exc:
            with self.lock:
                self.data["imu_error"] = f"IMU command failed: {exc}"
            return False

        self._last_imu_command = now
        if self._imu_command_first_sent is None:
            self._imu_command_first_sent = now
        with self.lock:
            self.data["imu_command"] = command
            self.data["imu_command_last_sent"] = datetime.now().strftime("%H:%M:%S")
            self.data["imu_command_count"] = int(self.data.get("imu_command_count") or 0) + 1
        return True

    def _next_esp32_imu_command(self):
        commands = COMM_SETTINGS.get("ESP32_IMU_COMMANDS")
        if not commands:
            return COMM_SETTINGS.get("ESP32_IMU_COMMAND", "IMU_ON")

        command = commands[self._imu_command_index % len(commands)]
        self._imu_command_index += 1
        return command

    def _update_imu_no_response_status(self, now=None):
        if not COMM_SETTINGS.get("ESP32_IMU_ENABLE", False):
            return

        now = time.monotonic() if now is None else now
        with self.lock:
            last_seen = self.data.get("imu_last_seen_monotonic")
            first_sent = self._imu_command_first_sent

        if last_seen is not None:
            no_response_sec = max(0.0, now - float(last_seen))
        elif first_sent is not None:
            no_response_sec = max(0.0, now - float(first_sent))
        else:
            no_response_sec = None

        with self.lock:
            self.data["imu_no_response_sec"] = round(no_response_sec, 2) if no_response_sec is not None else None
            if (
                no_response_sec is not None
                and no_response_sec >= COMM_SETTINGS.get("ESP32_IMU_NO_RESPONSE_WARN_SEC", 6.0)
                and not self.data.get("imu_last_seen")
            ):
                self.data["imu_error"] = "ESP32 IMU response not received"

    def _process_serial_line(self, line):
        sensor_data = self.parse_sensor_json_line(line)
        if sensor_data:
            with self.lock:
                self.data.update(sensor_data)
                self.data["uwb_parse_ok"] = True
            return True

        wifi_ap = self.parse_wifi_line(line)
        if wifi_ap is not None:
            if wifi_ap:
                self._store_esp32_wifi_ap(wifi_ap, line)
            return True

        status_data = self.parse_status_line(line)
        if status_data:
            with self.lock:
                self.data.update(status_data)
            return True

        imu_data = self.parse_imu_line(line)
        if imu_data:
            with self.lock:
                self.data.update(imu_data)
            return True

        parsed = self.parse_line(line)

        if parsed:
            with self.lock:
                self.data.update(parsed)
                self.data["uwb_parse_ok"] = True
            return True

        if line:
            with self.lock:
                self.data["uwb_parse_ok"] = False
        return False

    def parse_sensor_json_line(self, line):
        if not line:
            return None

        raw_line = line.strip()
        if not raw_line.startswith("{"):
            return None

        try:
            frame = json.loads(raw_line)
        except json.JSONDecodeError:
            return None

        if not isinstance(frame, dict):
            return None

        has_sensor_key = any(
            key in frame
            for key in (
                "t",
                "uwb_x",
                "uwb_y",
                "uwb_d1",
                "imu_ax",
                "imu_ay",
                "imu_az",
                "imu_gx",
                "imu_gy",
                "imu_gz",
                "imu_ok",
            )
        )
        has_esp32_imu_values = any(
            self._json_number(frame.get(key)) is not None
            for key in ("imu_ax", "imu_ay", "imu_az", "imu_gx", "imu_gy", "imu_gz", "ax", "ay", "az", "gx", "gy", "gz")
        )
        if not has_sensor_key and has_esp32_imu_values:
            imu_ok = bool(frame.get("imu_ok", frame.get("ok", True)))
            now_mono = time.monotonic()
            return {
                "imu_t": self._json_int(frame.get("imu_t")),
                "imu_ax": self._json_number(frame.get("imu_ax", frame.get("ax"))),
                "imu_ay": self._json_number(frame.get("imu_ay", frame.get("ay"))),
                "imu_az": self._json_number(frame.get("imu_az", frame.get("az"))),
                "imu_gx": self._json_number(frame.get("imu_gx", frame.get("gx"))),
                "imu_gy": self._json_number(frame.get("imu_gy", frame.get("gy"))),
                "imu_gz": self._json_number(frame.get("imu_gz", frame.get("gz"))),
                "imu_source": "esp32_json",
                "imu_ok": imu_ok,
                "imu_last_seen": datetime.now().strftime("%H:%M:%S"),
                "imu_last_seen_monotonic": now_mono,
                "imu_last_raw": raw_line,
                "imu_raw": raw_line,
                "imu_error": "",
            }

        if not has_sensor_key:
            return None

        distances = [
            self._json_number(frame.get("uwb_d1")),
            self._json_number(frame.get("uwb_d2")),
            self._json_number(frame.get("uwb_d3")),
            self._json_number(frame.get("uwb_d4")),
        ]
        rssis = [
            self._json_number(frame.get("rssi1")),
            self._json_number(frame.get("rssi2")),
            self._json_number(frame.get("rssi3")),
            self._json_number(frame.get("rssi4")),
        ]
        uwb_x = self._json_number(frame.get("uwb_x"))
        uwb_y = self._json_number(frame.get("uwb_y"))
        has_imu_values = any(
            self._json_number(frame.get(key)) is not None
            for key in ("imu_ax", "imu_ay", "imu_az", "imu_gx", "imu_gy", "imu_gz", "ax", "ay", "az", "gx", "gy", "gz")
        )
        has_uwb_values = (
            uwb_x is not None
            or uwb_y is not None
            or any(value is not None for value in distances)
            or any(value is not None for value in rssis)
            or "uwb_ok" in frame
        )

        now_text = datetime.now().strftime("%H:%M:%S")
        now_mono = time.monotonic()
        imu_ok = bool(frame.get("imu_ok", frame.get("ok", has_imu_values)))
        imu_data = {
            "imu_t": self._json_int(frame.get("imu_t")),
            "imu_ax": self._json_number(frame.get("imu_ax", frame.get("ax"))),
            "imu_ay": self._json_number(frame.get("imu_ay", frame.get("ay"))),
            "imu_az": self._json_number(frame.get("imu_az", frame.get("az"))),
            "imu_gx": self._json_number(frame.get("imu_gx", frame.get("gx"))),
            "imu_gy": self._json_number(frame.get("imu_gy", frame.get("gy"))),
            "imu_gz": self._json_number(frame.get("imu_gz", frame.get("gz"))),
            "imu_source": "esp32_json",
            "imu_ok": imu_ok,
            "imu_raw": line,
            "imu_last_seen_monotonic": now_mono,
        }

        if has_imu_values and not has_uwb_values:
            return imu_data

        if uwb_x is not None and uwb_y is not None:
            parsed = self._build_direct_position_result(
                uwb_x,
                uwb_y,
                distances,
                rssis,
                raw_line,
                frame.get("uwb_ok", True),
            )
        else:
            parsed = self._build_parsed_result(uwb_x, uwb_y, distances, rssis, raw_line)

        uwb_ok = bool(frame.get("uwb_ok", parsed.get("uwb_valid", False)))
        parsed.update(imu_data)
        parsed.update(
            {
                "uwb_t": self._json_int(frame.get("uwb_t")),
                "uwb_valid": uwb_ok and parsed.get("uwb_valid", False),
                "uwb_last_seen": now_text if uwb_ok else parsed.get("uwb_last_seen"),
                "uwb_last_raw": raw_line,
                "uwb_raw": raw_line,
            }
        )
        return parsed

    def parse_status_line(self, line):
        if not line:
            return None

        raw_line = line.strip()
        parts = [part.strip() for part in raw_line.split(",", 1)]
        kind = parts[0].upper() if parts else ""
        if kind != "IMU_STATUS":
            return None

        data = {
            "imu_last_raw": raw_line,
            "imu_last_seen": datetime.now().strftime("%H:%M:%S"),
            "imu_last_seen_monotonic": time.monotonic(),
            "imu_source": "esp32_serial",
        }
        if "ready=0" in raw_line.lower():
            data["imu_error"] = raw_line
        elif "ready=1" in raw_line.lower():
            data["imu_error"] = ""
            data["imu_ok"] = True
        return data

    def parse_wifi_line(self, line):
        if not line:
            return None

        raw_line = line.strip()
        parts = [part.strip() for part in raw_line.split(",")]
        if not parts:
            return None

        kind = parts[0].upper()
        if kind in ("WIFI_SCAN_START", "WIFI_START", "AP_SCAN_START"):
            with self.lock:
                if self.wifi_collection_enabled:
                    self.esp32_wifi_aps.clear()
                    self.data["esp32_wifi_count"] = 0
            return {}

        if kind not in ("WIFI", "WIFI_AP", "AP", "ESP32_WIFI"):
            return None

        labeled = self._parse_wifi_labeled_values(raw_line)
        if labeled:
            return self._build_wifi_ap(labeled)

        if len(parts) < 4:
            return None

        fields = {
            "ssid": parts[1] if len(parts) > 1 else "",
            "bssid": parts[2] if len(parts) > 2 else "",
            "rssi": parts[3] if len(parts) > 3 else "",
            "channel": parts[4] if len(parts) > 4 else "",
        }
        return self._build_wifi_ap(fields)

    @classmethod
    def _parse_wifi_labeled_values(cls, line):
        values = {}
        aliases = {
            "ssid": "ssid",
            "bssid": "bssid",
            "mac": "bssid",
            "rssi": "rssi",
            "signal": "rssi",
            "ch": "channel",
            "chan": "channel",
            "channel": "channel",
            "freq": "freq",
            "frequency": "freq",
        }
        pattern = r"([A-Za-z][A-Za-z0-9_]*)\s*[:=]\s*([^,]+)"
        for key, raw_value in re.findall(pattern, line):
            normalized_key = aliases.get(key.lower())
            if normalized_key:
                values[normalized_key] = raw_value.strip().strip('"')
        return values

    @classmethod
    def _build_wifi_ap(cls, fields):
        rssi = cls._parse_float(str(fields.get("rssi", "")))
        if rssi is None:
            return None

        channel = cls._parse_float(str(fields.get("channel", "")))
        freq = cls._parse_float(str(fields.get("freq", "")))
        band = "2.4GHz"
        if freq is not None:
            band = "5GHz" if freq >= 5000 or freq >= 5.0 else "2.4GHz"
        elif channel is not None:
            band = "5GHz" if channel > 14 else "2.4GHz"

        if band != "2.4GHz":
            return None

        return {
            "ssid": str(fields.get("ssid", "")),
            "bssid": str(fields.get("bssid", "")),
            "rssi": float(rssi),
            "band": band,
            "channel": int(channel) if channel is not None else None,
            "interface": "esp32",
        }

    def parse_line(self, line):
        if not line:
            return None

        raw_line = line.strip()
        anchor_line_result = self._parse_anchor_distance_line(raw_line)
        if anchor_line_result:
            return anchor_line_result

        parts = [part.strip() for part in raw_line.split(",") if part.strip() != ""]
        if not parts:
            return None

        kind = parts[0].upper()
        x = y = None
        distances = [None, None, None, None]
        rssis = [None, None, None, None]

        # 0) 라벨형 우선 처리: X=1.2,Y=0.8,D1=...,RSSI1=-72
        labeled = self._parse_labeled_values(raw_line)
        if labeled:
            x = labeled.get("x")
            y = labeled.get("y")
            distances = [
                labeled.get("d1"),
                labeled.get("d2"),
                labeled.get("d3"),
                labeled.get("d4"),
            ]
            rssis = [
                labeled.get("rssi1"),
                labeled.get("rssi2"),
                labeled.get("rssi3"),
                labeled.get("rssi4"),
            ]
            if kind in ("POS", "POSITION", "XY") and x is not None and y is not None:
                return self._build_direct_position_result(x, y, distances, rssis, raw_line, labeled.get("valid"))
            if x is None or y is None:
                x, y = self._estimate_position_from_distances(distances)
            return self._build_parsed_result(x, y, distances, rssis, raw_line)

        # 1) POS,x,y 또는 POS,x,y,d1,d2,d3,d4[,rssi1,rssi2,rssi3,rssi4]
        # 중요: ESP32에서 POS,NAN,NAN,d1,d2,d3,d4 처럼
        # 좌표는 NAN이고 거리값만 들어오는 경우가 있음.
        # 이 경우 유효 앵커가 3개 이상이면 라즈베리파이에서 2D 좌표를 계산한다.
        if kind in ("POS", "POSITION", "XY") and len(parts) >= 3:
            x = self._parse_float(parts[1])
            y = self._parse_float(parts[2])
            distances = self._normalize_distances(parts[3:7])
            rssis = self._normalize_rssis(parts[7:11])

            if x is None or y is None:
                est_x, est_y = self._estimate_position_from_distances(distances)
                if est_x is not None and est_y is not None:
                    x, y = est_x, est_y

            return self._build_parsed_result(x, y, distances, rssis, raw_line)

        # 2) DATA 계열 처리
        if kind == "DATA":
            values = [self._parse_float(v) for v in parts[1:]]
            values = [v for v in values if v is not None]

            if len(values) >= 11:
                # DATA,id,x,y,d1,d2,d3,d4,rssi1,rssi2,rssi3,rssi4
                x, y = values[1], values[2]
                distances = self._normalize_distances([str(v) for v in values[3:7]])
                rssis = self._normalize_rssis([str(v) for v in values[7:11]])
            elif len(values) >= 10:
                # DATA,x,y,d1,d2,d3,d4,rssi1,rssi2,rssi3,rssi4
                x, y = values[0], values[1]
                distances = self._normalize_distances([str(v) for v in values[2:6]])
                rssis = self._normalize_rssis([str(v) for v in values[6:10]])
            elif len(values) >= 8:
                # DATA,d1,d2,d3,d4,rssi1,rssi2,rssi3,rssi4
                distances = self._normalize_distances([str(v) for v in values[0:4]])
                rssis = self._normalize_rssis([str(v) for v in values[4:8]])
                x, y = self._estimate_position_from_distances(distances)
            elif len(values) >= 7:
                # DATA,id,x,y,d1,d2,d3,d4 형식까지 허용
                x, y = values[1], values[2]
                distances = self._normalize_distances([str(v) for v in values[3:7]])
            elif len(values) >= 6:
                # DATA,x,y,d1,d2,d3,d4 형식
                x, y = values[0], values[1]
                distances = self._normalize_distances([str(v) for v in values[2:6]])
            elif len(values) == 5:
                # 두 가지가 섞여 있을 수 있음:
                # DATA,x,y,d1,d2,d3  또는 DATA,id,d1,d2,d3,d4
                # 현재 코드에서 좌표 표시가 우선이라 앞 2개를 좌표 후보로 표시하고,
                # 동시에 뒤쪽 4개는 거리 후보로 저장한다.
                x, y = values[0], values[1]
                distances = self._normalize_distances([str(v) for v in values[1:5]])
                # 좌표 후보가 방 범위를 크게 벗어나면 거리값으로 좌표 재계산
                if not self._position_in_reasonable_room(x, y):
                    x, y = self._estimate_position_from_distances(distances)
            elif len(values) == 4:
                # DATA,d1,d2,d3,d4
                distances = self._normalize_distances([str(v) for v in values])
                x, y = self._estimate_position_from_distances(distances)
            elif len(values) == 3:
                # DATA,d1,d2,d3
                distances = self._normalize_distances([str(v) for v in values])
                x, y = self._estimate_position_from_distances(distances)
            elif len(values) >= 2:
                # DATA,x,y 최소 좌표 표시
                x, y = values[0], values[1]
                distances = [None, None, None, None]
            else:
                return None

            return self._build_parsed_result(x, y, distances, rssis, raw_line)

        # 3) 접두사 없는 CSV: x,y,d1,d2,d3,d4[,rssi1..4] 또는 d1,d2,d3,d4[,rssi1..4]
        numeric_parts = [self._parse_float(v) for v in parts]
        numeric_parts = [v for v in numeric_parts if v is not None]

        if len(numeric_parts) >= 10:
            x, y = numeric_parts[0], numeric_parts[1]
            distances = self._normalize_distances([str(v) for v in numeric_parts[2:6]])
            rssis = self._normalize_rssis([str(v) for v in numeric_parts[6:10]])
        elif len(numeric_parts) >= 8:
            distances = self._normalize_distances([str(v) for v in numeric_parts[0:4]])
            rssis = self._normalize_rssis([str(v) for v in numeric_parts[4:8]])
            x, y = self._estimate_position_from_distances(distances)
        elif len(numeric_parts) >= 6:
            x, y = numeric_parts[0], numeric_parts[1]
            distances = self._normalize_distances([str(v) for v in numeric_parts[2:6]])
        elif len(numeric_parts) == 4:
            distances = self._normalize_distances([str(v) for v in numeric_parts])
            x, y = self._estimate_position_from_distances(distances)
        elif len(numeric_parts) >= 2:
            x, y = numeric_parts[0], numeric_parts[1]
            distances = [None, None, None, None]
        else:
            return None

        return self._build_parsed_result(x, y, distances, rssis, raw_line)

    def _parse_anchor_distance_line(self, line):
        match = re.search(
            r"ANCHOR\s+(?:0x)?([0-9A-Fa-f]+)\s*:\s*([-+]?\d+(?:\.\d+)?)\s*m?",
            line,
            re.IGNORECASE,
        )
        if not match:
            return None

        try:
            anchor_number = int(match.group(1), 16)
        except ValueError:
            return None

        if not 1 <= anchor_number <= 4:
            return None

        distance_m = self._parse_float(match.group(2))
        if distance_m is None:
            return None

        anchor_id = f"d{anchor_number}"
        now = time.monotonic()
        self.anchor_line_distances[anchor_id] = distance_m
        self.anchor_line_seen_monotonic[anchor_id] = now
        distances = self._fresh_anchor_line_distances(now)
        valid_count = sum(1 for value in distances if self._is_valid_distance(value))
        required_count = DRIVE_SETTINGS.get("MIN_UWB_ANCHORS_FOR_POSITION", 3)
        if valid_count < required_count:
            return self._build_anchor_waiting_result(distances, line)
        return self._build_parsed_result(None, None, distances, [None, None, None, None], line)

    def _fresh_anchor_line_distances(self, now=None):
        now = time.monotonic() if now is None else now
        max_age = DRIVE_SETTINGS.get("UWB_ANCHOR_LINE_MAX_AGE_SEC", 0.8)
        distances = []
        for anchor_id in ("d1", "d2", "d3", "d4"):
            seen_at = self.anchor_line_seen_monotonic.get(anchor_id)
            if seen_at is None or now - seen_at > max_age:
                distances.append(None)
            else:
                distances.append(self.anchor_line_distances.get(anchor_id))
        return distances

    def _build_anchor_waiting_result(self, distances, line):
        raw_distance_dict = self._distance_dict(distances)
        valid_anchor_count = sum(1 for value in distances if value is not None and value > 0.05)
        return {
            "x": None,
            "y": None,
            "display_x": None,
            "display_y": None,
            "drive_x": None,
            "drive_y": None,
            "map_x": None,
            "map_y": None,
            "unbounded_map_x": None,
            "unbounded_map_y": None,
            "raw_x": None,
            "raw_y": None,
            "raw_distances": raw_distance_dict,
            "filtered_distances": raw_distance_dict,
            "d1": distances[0],
            "d2": distances[1],
            "d3": distances[2],
            "d4": distances[3],
            "uwb_valid": False,
            "uwb_anchor_count": valid_anchor_count,
            "uwb_last_seen": datetime.now().strftime("%H:%M:%S"),
            "uwb_last_raw": line,
            "uwb_raw": line,
            "uwb_parse_ok": True,
            "uwb_error": "waiting for fresh distances from at least 3 anchors",
        }

    def _build_direct_position_result(self, x, y, distances, rssis, line, valid_flag=None):
        raw_distances = self._normalize_distances(["" if v is None else str(v) for v in distances])
        distances = self._filter_distances(raw_distances)
        rssis = self._normalize_rssis(["" if v is None else str(v) for v in rssis])
        valid_rssis = [value for value in rssis if value is not None]
        valid_anchor_count = sum(
            1 for value in distances
            if value is not None and value > 0.05
        )

        direct_valid = valid_flag is None or bool(valid_flag)
        raw_x = float(x) if x is not None else None
        raw_y = float(y) if y is not None else None
        if raw_x is not None and DRIVE_SETTINGS.get("UWB_CLAMP_NEGATIVE_X_TO_ZERO", False):
            raw_x = max(0.0, raw_x)

        if raw_x is not None and raw_y is not None:
            unbounded_map_x = (raw_x - self.x_offset) * DRIVE_SETTINGS.get("UWB_MAP_SCALE_X", 1.0)
            unbounded_map_y = (raw_y - self.y_offset) * DRIVE_SETTINGS.get("UWB_MAP_SCALE_Y", 1.0)
        else:
            unbounded_map_x = None
            unbounded_map_y = None

        if direct_valid and raw_x is not None and raw_y is not None:
            map_x, map_y = self._apply_origin_and_geofence(raw_x, raw_y)
            unbounded_map_x = (raw_x - self.x_offset) * DRIVE_SETTINGS.get("UWB_MAP_SCALE_X", 1.0)
            unbounded_map_y = (raw_y - self.y_offset) * DRIVE_SETTINGS.get("UWB_MAP_SCALE_Y", 1.0)
        elif self.last_valid_x is not None and self.last_valid_y is not None:
            self.last_position_holding = True
            self.last_position_clamped = False
            map_x, map_y = self.last_valid_x, self.last_valid_y
        else:
            self.last_position_holding = True
            self.last_position_clamped = False
            map_x, map_y = None, None

        geofence_valid = direct_valid and map_x is not None and map_y is not None and not self.last_position_holding
        distance_error_m, distance_error_max_m = self._distance_error_from_position(raw_x, raw_y, distances)
        raw_distance_dict = self._distance_dict(raw_distances)
        filtered_distance_dict = self._distance_dict(distances)
        self._log_distance_filter(raw_distance_dict, filtered_distance_dict, valid_anchor_count)

        display_x = self.display_x if self.display_x is not None else map_x
        display_y = self.display_y if self.display_y is not None else map_y

        return {
            "x": round(display_x, 3) if display_x is not None else None,
            "y": round(display_y, 3) if display_y is not None else None,
            "display_x": round(display_x, 3) if display_x is not None else None,
            "display_y": round(display_y, 3) if display_y is not None else None,
            "drive_x": round(map_x, 3) if map_x is not None else None,
            "drive_y": round(map_y, 3) if map_y is not None else None,
            "map_x": round(map_x, 3) if map_x is not None else None,
            "map_y": round(map_y, 3) if map_y is not None else None,
            "unbounded_map_x": round(unbounded_map_x, 3) if unbounded_map_x is not None else None,
            "unbounded_map_y": round(unbounded_map_y, 3) if unbounded_map_y is not None else None,
            "raw_x": round(raw_x, 3) if raw_x is not None else None,
            "raw_y": round(raw_y, 3) if raw_y is not None else None,
            "raw_distances": raw_distance_dict,
            "filtered_distances": filtered_distance_dict,
            "x_offset": round(self.x_offset, 3),
            "y_offset": round(self.y_offset, 3),
            "uwb_map_scale_x": DRIVE_SETTINGS.get("UWB_MAP_SCALE_X", 1.0),
            "uwb_map_scale_y": DRIVE_SETTINGS.get("UWB_MAP_SCALE_Y", 1.0),
            "d1": distances[0],
            "d2": distances[1],
            "d3": distances[2],
            "d4": distances[3],
            "rssi1": rssis[0],
            "rssi2": rssis[1],
            "rssi3": rssis[2],
            "rssi4": rssis[3],
            "uwb_rssi": round(sum(valid_rssis) / len(valid_rssis), 1) if valid_rssis else None,
            "uwb_valid": geofence_valid,
            "uwb_origin_calibrated": self.origin_calibrated,
            "uwb_position_geofence_valid": geofence_valid,
            "uwb_position_clamped": self.last_position_clamped,
            "uwb_position_holding": self.last_position_holding,
            "uwb_invalid_position_count": self.invalid_position_count,
            "uwb_position_outlier_count": self.position_outlier_count,
            "uwb_position_outlier_reason": self.position_outlier_reason,
            "uwb_geofence_margin_m": DRIVE_SETTINGS.get("UWB_GEOFENCE_MARGIN_M", 0.0),
            "uwb_anchor_count": valid_anchor_count,
            "uwb_rssi_count": len(valid_rssis),
            "uwb_distance_error_m": distance_error_m,
            "uwb_distance_error_max_m": distance_error_max_m,
            "uwb_last_seen": datetime.now().strftime("%H:%M:%S"),
            "uwb_last_raw": line,
            "uwb_raw": line,
        }

    def parse_imu_line(self, line):
        if not line:
            return None

        raw_line = line.strip()
        parts = [part.strip() for part in raw_line.split(",")]
        if not parts or parts[0].upper() not in ("IMU", "YPR", "ACCEL", "GYRO"):
            return None

        labeled = self._parse_imu_labeled_values(raw_line)
        values = {
            "heading": labeled.get("heading"),
            "pitch": labeled.get("pitch"),
            "roll": labeled.get("roll"),
            "imu_ax": labeled.get("imu_ax"),
            "imu_ay": labeled.get("imu_ay"),
            "imu_az": labeled.get("imu_az"),
            "imu_gx": labeled.get("imu_gx"),
            "imu_gy": labeled.get("imu_gy"),
            "imu_gz": labeled.get("imu_gz"),
        }

        if not labeled and len(parts) >= 4:
            numbers = [self._parse_float(value) for value in parts[1:]]
            if parts[0].upper() == "YPR":
                values["heading"], values["pitch"], values["roll"] = numbers[:3]
            elif parts[0].upper() == "ACCEL":
                values["imu_ax"], values["imu_ay"], values["imu_az"] = numbers[:3]
            elif parts[0].upper() == "GYRO":
                values["imu_gx"], values["imu_gy"], values["imu_gz"] = numbers[:3]
            elif len(numbers) >= 6:
                values["imu_ax"], values["imu_ay"], values["imu_az"] = numbers[:3]
                values["imu_gx"], values["imu_gy"], values["imu_gz"] = numbers[3:6]
            else:
                values["heading"], values["pitch"], values["roll"] = numbers[:3]

        values = {key: value for key, value in values.items() if value is not None}
        if not values:
            return None

        values.update({
            "imu_ok": True,
            "imu_last_seen": datetime.now().strftime("%H:%M:%S"),
            "imu_last_seen_monotonic": time.monotonic(),
            "imu_source": "esp32_serial",
            "imu_last_raw": raw_line,
            "imu_error": "",
        })
        return values

    @classmethod
    def _parse_imu_labeled_values(cls, line):
        values = {}
        aliases = {
            "h": "heading",
            "heading": "heading",
            "yaw": "heading",
            "y": "heading",
            "p": "pitch",
            "pitch": "pitch",
            "r": "roll",
            "roll": "roll",
            "ax": "imu_ax",
            "accel_x": "imu_ax",
            "imu_ax": "imu_ax",
            "ay": "imu_ay",
            "accel_y": "imu_ay",
            "imu_ay": "imu_ay",
            "az": "imu_az",
            "accel_z": "imu_az",
            "imu_az": "imu_az",
            "gx": "imu_gx",
            "gyro_x": "imu_gx",
            "imu_gx": "imu_gx",
            "gy": "imu_gy",
            "gyro_y": "imu_gy",
            "imu_gy": "imu_gy",
            "gz": "imu_gz",
            "gyro_z": "imu_gz",
            "imu_gz": "imu_gz",
        }
        pattern = r"([A-Za-z][A-Za-z0-9_]*)\s*[:=]\s*(-?(?:\d+(?:\.\d*)?|\.\d+)|NAN)"
        for key, raw_value in re.findall(pattern, line):
            normalized_key = aliases.get(key.lower())
            if normalized_key:
                values[normalized_key] = cls._parse_float(raw_value)
        return values

    def _build_parsed_result(self, x, y, distances, rssis, line):
        raw_distances = self._normalize_distances(["" if v is None else str(v) for v in distances])
        distances = self._filter_distances(raw_distances)
        rssis = self._normalize_rssis(["" if v is None else str(v) for v in rssis])

        valid_rssis = [value for value in rssis if value is not None]

        valid_anchor_count = sum(
            1 for value in distances
            if value is not None and value > 0.05
        )
        est_x, est_y = self._estimate_position_from_distances(distances) if valid_anchor_count >= 3 else (None, None)
        raw_x = est_x if est_x is not None else x
        raw_y = est_y if est_y is not None else y
        if raw_x is not None and DRIVE_SETTINGS.get("UWB_CLAMP_NEGATIVE_X_TO_ZERO", False):
            raw_x = max(0.0, float(raw_x))
        position_valid = raw_x is not None and raw_y is not None and valid_anchor_count >= 3
        if position_valid:
            map_x, map_y = self._apply_origin_and_geofence(raw_x, raw_y)
            unbounded_map_x = (float(raw_x) - self.x_offset) * DRIVE_SETTINGS.get("UWB_MAP_SCALE_X", 1.0)
            unbounded_map_y = (float(raw_y) - self.y_offset) * DRIVE_SETTINGS.get("UWB_MAP_SCALE_Y", 1.0)
        elif self.last_valid_x is not None and self.last_valid_y is not None:
            self.last_position_holding = True
            self.last_position_clamped = False
            map_x, map_y = self.last_valid_x, self.last_valid_y
            unbounded_map_x, unbounded_map_y = None, None
        else:
            self.last_position_holding = True
            self.last_position_clamped = False
            map_x, map_y = None, None
            unbounded_map_x, unbounded_map_y = None, None
        geofence_valid = position_valid and not self.last_position_holding
        distance_error_m, distance_error_max_m = self._distance_error_from_position(raw_x, raw_y, distances)
        raw_distance_dict = self._distance_dict(raw_distances)
        filtered_distance_dict = self._distance_dict(distances)
        self._log_distance_filter(raw_distance_dict, filtered_distance_dict, valid_anchor_count)

        display_x = self.display_x if self.display_x is not None else map_x
        display_y = self.display_y if self.display_y is not None else map_y

        return {
            "x": round(display_x, 3) if display_x is not None else None,
            "y": round(display_y, 3) if display_y is not None else None,
            "display_x": round(display_x, 3) if display_x is not None else None,
            "display_y": round(display_y, 3) if display_y is not None else None,
            "drive_x": round(map_x, 3) if map_x is not None else None,
            "drive_y": round(map_y, 3) if map_y is not None else None,
            "map_x": round(map_x, 3) if map_x is not None else None,
            "map_y": round(map_y, 3) if map_y is not None else None,
            "unbounded_map_x": round(unbounded_map_x, 3) if unbounded_map_x is not None else None,
            "unbounded_map_y": round(unbounded_map_y, 3) if unbounded_map_y is not None else None,
            "raw_x": round(raw_x, 3) if raw_x is not None else None,
            "raw_y": round(raw_y, 3) if raw_y is not None else None,
            "raw_distances": raw_distance_dict,
            "filtered_distances": filtered_distance_dict,
            "x_offset": round(self.x_offset, 3),
            "y_offset": round(self.y_offset, 3),
            "uwb_map_scale_x": DRIVE_SETTINGS.get("UWB_MAP_SCALE_X", 1.0),
            "uwb_map_scale_y": DRIVE_SETTINGS.get("UWB_MAP_SCALE_Y", 1.0),
            "d1": distances[0],
            "d2": distances[1],
            "d3": distances[2],
            "d4": distances[3],
            "rssi1": rssis[0],
            "rssi2": rssis[1],
            "rssi3": rssis[2],
            "rssi4": rssis[3],
            "uwb_rssi": round(sum(valid_rssis) / len(valid_rssis), 1) if valid_rssis else None,
            # 화면 좌표 표시용으로는 x,y가 있으면 유효 처리.
            # 앵커 개수는 별도 표시해서 확인한다.
            "uwb_valid": geofence_valid,
            "uwb_origin_calibrated": self.origin_calibrated,
            "uwb_position_geofence_valid": geofence_valid,
            "uwb_position_clamped": self.last_position_clamped,
            "uwb_position_holding": self.last_position_holding,
            "uwb_invalid_position_count": self.invalid_position_count,
            "uwb_position_outlier_count": self.position_outlier_count,
            "uwb_position_outlier_reason": self.position_outlier_reason,
            "uwb_geofence_margin_m": DRIVE_SETTINGS.get("UWB_GEOFENCE_MARGIN_M", 0.0),
            "uwb_anchor_count": valid_anchor_count,
            "uwb_rssi_count": len(valid_rssis),
            "uwb_distance_error_m": distance_error_m,
            "uwb_distance_error_max_m": distance_error_max_m,
            "uwb_last_seen": datetime.now().strftime("%H:%M:%S"),
            "uwb_last_raw": line,
            "uwb_raw": line,
        }

    @staticmethod
    def _distance_dict(distances):
        return {
            "d1": distances[0],
            "d2": distances[1],
            "d3": distances[2],
            "d4": distances[3],
        }

    @staticmethod
    def _is_valid_distance(value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return False
        return 0.0 < value <= DRIVE_SETTINGS.get("UWB_MAX_VALID_DISTANCE_M", 5.0)

    def _filter_distances(self, raw_distances):
        return [
            float(distance_m) if self._is_valid_distance(distance_m) else None
            for distance_m in raw_distances
        ]

    def _log_distance_filter(self, raw_distances, filtered_distances, valid_anchor_count):
        if not DRIVE_SETTINGS.get("UWB_DISTANCE_LOG_ENABLE", False):
            return
        now = time.monotonic()
        if now - self._last_distance_log < 1.0:
            return
        self._last_distance_log = now
        print(f"[UWB DIST] anchors={valid_anchor_count}, raw={raw_distances}, filtered={filtered_distances}", flush=True)

    def _apply_origin_and_geofence(self, raw_x, raw_y):
        if not self.origin_calibrated and not DRIVE_SETTINGS.get("UWB_AUTO_ORIGIN_CALIBRATE", True):
            self.x_offset = 0.0
            self.y_offset = 0.0
        if DRIVE_SETTINGS.get("UWB_AUTO_ORIGIN_CALIBRATE", True) and not self.origin_calibrated:
            self.x_offset = float(raw_x)
            self.y_offset = float(raw_y)
            self.origin_calibrated = True
            print(f"[UWB] origin calibrated: x_offset={self.x_offset:.3f}, y_offset={self.y_offset:.3f}", flush=True)

        map_x = (float(raw_x) - self.x_offset) * DRIVE_SETTINGS.get("UWB_MAP_SCALE_X", 1.0)
        map_y = (float(raw_y) - self.y_offset) * DRIVE_SETTINGS.get("UWB_MAP_SCALE_Y", 1.0)
        return self._geofence_position(map_x, map_y)

    def _geofence_position(self, map_x, map_y):
        self.last_position_clamped = False
        self.last_position_holding = False
        self.position_outlier_reason = ""

        if DRIVE_SETTINGS.get("UWB_CLAMP_NEGATIVE_X_TO_ZERO", False) and map_x < 0.0:
            map_x = 0.0
        if not DRIVE_SETTINGS.get("UWB_GEOFENCE_ENABLE", True):
            drive_x, drive_y = float(map_x), float(map_y)
            self._update_position_channels(drive_x, drive_y)
            self.last_valid_x = drive_x
            self.last_valid_y = drive_y
            self.invalid_position_count = 0
            return drive_x, drive_y

        width = APP_SETTINGS.get("ROOM_WIDTH_M", 3.0)
        height = APP_SETTINGS.get("ROOM_HEIGHT_M", 3.0)
        is_strict_valid = 0.0 <= map_x <= width and 0.0 <= map_y <= height
        is_valid = self._position_inside_geofence(map_x, map_y)

        if is_strict_valid:
            drive_x, drive_y = float(map_x), float(map_y)
            self._update_position_channels(drive_x, drive_y)
            self.last_valid_x = drive_x
            self.last_valid_y = drive_y
            self.invalid_position_count = 0
            return drive_x, drive_y

        if is_valid:
            clamped_x = max(0.0, min(width, map_x))
            clamped_y = max(0.0, min(height, map_y))
            self._update_position_channels(clamped_x, clamped_y)
            self.last_valid_x = clamped_x
            self.last_valid_y = clamped_y
            self.invalid_position_count = 0
            self.last_position_clamped = True
            return clamped_x, clamped_y

        self.invalid_position_count += 1
        self.last_position_holding = True
        clamped_x = max(0.0, min(width, map_x))
        clamped_y = max(0.0, min(height, map_y))
        self.last_valid_x = clamped_x
        self.last_valid_y = clamped_y
        self.last_position_clamped = True
        self._update_position_channels(clamped_x, clamped_y)
        warn_count = DRIVE_SETTINGS.get("UWB_INVALID_WARN_COUNT", 5)
        if self.invalid_position_count >= warn_count:
            print(
                f"[UWB WARNING] invalid position repeated: count={self.invalid_position_count}, "
                f"x={map_x:.2f}, y={map_y:.2f}",
                flush=True,
            )

        return clamped_x, clamped_y

    def _update_position_channels(self, drive_x, drive_y):
        self.drive_x = float(drive_x)
        self.drive_y = float(drive_y)
        self.display_x, self.display_y = self._lowpass_position(self.drive_x, self.drive_y)
        return self.drive_x, self.drive_y

    def _lowpass_position(self, map_x, map_y):
        if not DRIVE_SETTINGS.get("UWB_POSITION_LOWPASS_ENABLE", True):
            self.lowpass_x = float(map_x)
            self.lowpass_y = float(map_y)
            return float(map_x), float(map_y)

        alpha = float(
            DRIVE_SETTINGS.get(
                "UWB_DISPLAY_POSITION_LOWPASS_ALPHA",
                DRIVE_SETTINGS.get("UWB_POSITION_LOWPASS_ALPHA", 0.35),
            )
        )
        alpha = max(0.0, min(1.0, alpha))
        x = float(map_x)
        y = float(map_y)
        if self.lowpass_x is None or self.lowpass_y is None:
            self.lowpass_x = x
            self.lowpass_y = y
        else:
            self.lowpass_x = alpha * x + (1.0 - alpha) * self.lowpass_x
            self.lowpass_y = alpha * y + (1.0 - alpha) * self.lowpass_y
        return self.lowpass_x, self.lowpass_y

    @staticmethod
    def _position_inside_geofence(map_x, map_y):
        width = APP_SETTINGS.get("ROOM_WIDTH_M", 3.0)
        height = APP_SETTINGS.get("ROOM_HEIGHT_M", 3.0)
        margin = DRIVE_SETTINGS.get("UWB_GEOFENCE_MARGIN_M", 0.0)
        return -margin <= map_x <= width + margin and -margin <= map_y <= height + margin

    def _distance_error_from_position(self, x, y, distances):
        if x is None or y is None:
            return None, None

        anchors = [
            UWB_ANCHORS_M["d1"],
            UWB_ANCHORS_M["d2"],
            UWB_ANCHORS_M["d3"],
            UWB_ANCHORS_M["d4"],
        ]
        errors = []
        for anchor, measured in zip(anchors, distances):
            if measured is None or measured <= 0.05:
                continue
            expected = math.hypot(x - anchor[0], y - anchor[1])
            errors.append(abs(expected - measured))

        if not errors:
            return None, None

        rmse = math.sqrt(sum(error * error for error in errors) / len(errors))
        return round(rmse, 3), round(max(errors), 3)

    @staticmethod
    def _position_in_reasonable_room(x, y):
        if x is None or y is None:
            return False
        width = APP_SETTINGS.get("ROOM_WIDTH_M", 3.0)
        height = APP_SETTINGS.get("ROOM_HEIGHT_M", 3.0)
        return -1.0 <= x <= width + 1.0 and -1.0 <= y <= height + 1.0

    def _estimate_position_from_distances(self, distances):
        """
        4개 앵커 거리값으로 2D 좌표 계산.
        앵커 위치는 config.py의 UWB_ANCHORS_M 사용.
        """

        anchors = [
            UWB_ANCHORS_M["d1"],
            UWB_ANCHORS_M["d2"],
            UWB_ANCHORS_M["d3"],
            UWB_ANCHORS_M["d4"],
        ]

        valid = []

        for anchor, dist in zip(anchors, distances):
            if dist is not None and dist > 0.05:
                valid.append((anchor[0], anchor[1], dist))

        # 2D 좌표 계산은 최소 3개 앵커가 필요하다.
        # config.py의 MIN_UWB_ANCHORS_FOR_POSITION 기본값은 3.
        if len(valid) < DRIVE_SETTINGS["MIN_UWB_ANCHORS_FOR_POSITION"]:
            return None, None

        # 첫 번째 유효 앵커를 기준으로 least squares trilateration
        x1, y1, r1 = valid[0]

        a_rows = []
        b_rows = []

        for xi, yi, ri in valid[1:]:
            a_rows.append([
                2 * (xi - x1),
                2 * (yi - y1)
            ])
            b_rows.append(
                (r1 ** 2 - ri ** 2)
                + (xi ** 2 - x1 ** 2)
                + (yi ** 2 - y1 ** 2)
            )

        if len(a_rows) < 2:
            return None, None

        # 2x2 normal equation 직접 계산
        # x = (A^T A)^-1 A^T b
        sxx = sum(row[0] * row[0] for row in a_rows)
        sxy = sum(row[0] * row[1] for row in a_rows)
        syy = sum(row[1] * row[1] for row in a_rows)

        sxb = sum(row[0] * b for row, b in zip(a_rows, b_rows))
        syb = sum(row[1] * b for row, b in zip(a_rows, b_rows))

        det = sxx * syy - sxy * sxy

        if abs(det) < 1e-9:
            return None, None

        x = (sxb * syy - syb * sxy) / det
        y = (sxx * syb - sxy * sxb) / det

        return x, y

    @staticmethod
    def _parse_float(value):
        if value is None:
            return None
        value = str(value)
        if value.strip().upper() == "NAN":
            return None

        try:
            return float(value)
        except ValueError:
            return None

    @classmethod
    def _json_number(cls, value):
        if value is None or isinstance(value, bool):
            return None
        parsed = cls._parse_float(value)
        if parsed is None or not math.isfinite(parsed):
            return None
        return parsed

    @classmethod
    def _json_int(cls, value, default=None):
        parsed = cls._json_number(value)
        if parsed is None:
            return default
        return int(parsed)

    @classmethod
    def _normalize_distances(cls, values):
        distances = [cls._parse_float(value) for value in values[:4]]
        while len(distances) < 4:
            distances.append(None)
        return distances

    @classmethod
    def _normalize_rssis(cls, values):
        rssis = [cls._parse_float(value) for value in values[:4]]
        while len(rssis) < 4:
            rssis.append(None)
        return rssis

    @classmethod
    def _parse_labeled_values(cls, line):
        values = {}
        aliases = {
            "x": "x",
            "y": "y",
            "posx": "x",
            "posy": "y",
            "d1": "d1",
            "d2": "d2",
            "d3": "d3",
            "d4": "d4",
            "a1": "d1",
            "a2": "d2",
            "a3": "d3",
            "a4": "d4",
            "rssi": "rssi1",
            "rssi1": "rssi1",
            "rssi2": "rssi2",
            "rssi3": "rssi3",
            "rssi4": "rssi4",
            "r1": "rssi1",
            "r2": "rssi2",
            "r3": "rssi3",
            "r4": "rssi4",
            "a1rssi": "rssi1",
            "a2rssi": "rssi2",
            "a3rssi": "rssi3",
            "a4rssi": "rssi4",
            "d1rssi": "rssi1",
            "d2rssi": "rssi2",
            "d3rssi": "rssi3",
            "d4rssi": "rssi4",
            "valid": "valid",
        }

        pattern = r"([A-Za-z][A-Za-z0-9_]*)\s*[:=]\s*(-?(?:\d+(?:\.\d*)?|\.\d+)|NAN)"
        for key, raw_value in re.findall(pattern, line):
            normalized_key = aliases.get(key.lower())
            if normalized_key:
                values[normalized_key] = cls._parse_float(raw_value)

        return values
