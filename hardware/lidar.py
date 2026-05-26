import glob
import math
import os
import subprocess
import threading
import time

from config import DRIVE_SETTINGS


class LidarReader:
    def __init__(self, ports=None, baudrates=None, excluded_ports=None):
        self.ports = list(ports or [])
        self.baudrates = list(baudrates or [115200])
        self.excluded_ports = set(excluded_ports or [])
        self.lidar = None
        self.port = None
        self.baudrate = None
        self.running = False
        self.lock = threading.Lock()
        self.points = []
        self._scan_count = 0
        self._last_obstacle = {}
        self.data = {
            "lidar_connected": False,
            "lidar_port": None,
            "lidar_baud": None,
            "lidar_parse_ok": False,
            "lidar_point_count": 0,
            "lidar_last_seen": None,
            "lidar_last_raw": "",
            "lidar_error": "",
            "lidar_driver": "rplidar",
            "lidar_info": None,
            "lidar_scan_count": 0,
            "lidar_obstacle": {},
            "lidar_avoidance": {},
            "lidar_wall_assist": {},
        }

    def start(self):
        if not DRIVE_SETTINGS.get("LIDAR_ENABLE", False):
            return False
        if self.running:
            return True
        self.running = True
        threading.Thread(target=self.read_loop, daemon=True).start()
        return True

    def stop(self):
        self.running = False
        self._close_lidar()
        with self.lock:
            self.data["lidar_connected"] = False

    def snapshot(self):
        if DRIVE_SETTINGS.get("LIDAR_OBSTACLE_ENABLE", False):
            try:
                self.obstacle_ahead()
            except Exception:
                pass

        with self.lock:
            data = dict(self.data)
            data["lidar_wall_assist"] = dict(data.get("lidar_wall_assist") or {})
            data["lidar_obstacle"] = dict(data.get("lidar_obstacle") or {})
            data["lidar_avoidance"] = dict(data.get("lidar_avoidance") or {})
            return data

    def read_loop(self):
        while self.running:
            try:
                if self.lidar is None:
                    if not self.connect():
                        time.sleep(DRIVE_SETTINGS.get("LIDAR_RECONNECT_SEC", 2.0))
                        continue

                for scan in self._iter_rplidar_scans():
                    if not self.running:
                        break

                    points = []
                    for quality, angle, distance_mm in scan:
                        point = self._make_point(angle, distance_mm / 1000.0, quality)
                        if point:
                            points.append(point)

                    if points:
                        self._store_points(points, f"rplidar scan points={len(points)}")
                    else:
                        self._set_parse_error("empty RPLIDAR scan")

                self._close_lidar()
            except Exception as exc:
                self._set_error(str(exc))
                self._close_lidar()
                time.sleep(DRIVE_SETTINGS.get("LIDAR_RECONNECT_SEC", 2.0))

    def _iter_rplidar_scans(self):
        lidar = self.lidar
        if not lidar:
            return

        try:
            lidar.start_motor()
        except Exception:
            pass

        max_buf = int(DRIVE_SETTINGS.get("LIDAR_RPLIDAR_MAX_BUF_MEAS", 500))
        min_len = int(DRIVE_SETTINGS.get("LIDAR_RPLIDAR_MIN_SCAN_LEN", 5))

        if not hasattr(lidar, "iter_measures"):
            raise RuntimeError("RPLidar.iter_measures() is required. Install/use rplidar-roboticia.")

        batch = []
        for new_scan, quality, angle, distance_mm in lidar.iter_measures():
            if new_scan and len(batch) >= min_len:
                yield batch
                batch = []
            if quality > 0 and distance_mm > 0:
                batch.append((quality, angle, distance_mm))
            if len(batch) >= max_buf:
                yield batch
                batch = []
        if batch:
            yield batch

    def connect(self):
        try:
            from rplidar import RPLidar
        except ImportError:
            self._set_error("rplidar package is required for RPLIDAR input")
            return False

        ports = self._candidate_ports()
        if not ports:
            self._set_error("no LiDAR serial ports configured")
            return False

        for port in ports:
            if not os.path.exists(port):
                continue
            if not self._ensure_port_permission(port):
                continue

            for baudrate in self.baudrates:
                lidar = None
                try:
                    lidar = RPLidar(
                        port,
                        baudrate=int(baudrate),
                        timeout=float(DRIVE_SETTINGS.get("LIDAR_SERIAL_TIMEOUT_SEC", 1.0)),
                    )
                    info = self._read_lidar_info(lidar)
                    self.lidar = lidar
                    self.port = port
                    self.baudrate = int(baudrate)
                    with self.lock:
                        self.data["lidar_connected"] = True
                        self.data["lidar_port"] = port
                        self.data["lidar_baud"] = self.baudrate
                        self.data["lidar_info"] = info
                        self.data["lidar_error"] = ""
                        self.data["lidar_last_raw"] = f"RPLIDAR connected info={info}"
                    print(f"RPLIDAR ready: {port} @ {self.baudrate}, info={info}", flush=True)
                    return True
                except Exception as exc:
                    self._set_error(f"{port} @ {baudrate}: {exc}")
                    try:
                        lidar.stop()
                    except Exception:
                        pass
                    try:
                        lidar.stop_motor()
                    except Exception:
                        pass
                    try:
                        lidar.disconnect()
                    except Exception:
                        pass

        self._set_error("RPLIDAR serial data not found")
        return False

    def _candidate_ports(self):
        ports = []
        for item in self.ports:
            if "*" in item:
                ports.extend(glob.glob(item))
            else:
                ports.append(item)

        if DRIVE_SETTINGS.get("LIDAR_AUTO_FIND_CP210X", True):
            ports.extend(self._cp210x_ports())

        return [
            port for port in dict.fromkeys(ports)
            if port and port not in self.excluded_ports
        ]

    @staticmethod
    def _cp210x_ports():
        try:
            import serial.tools.list_ports
        except Exception:
            return []

        found = []
        for port in serial.tools.list_ports.comports():
            hwid = str(getattr(port, "hwid", "") or "").upper()
            description = str(getattr(port, "description", "") or "").lower()
            if "10C4:EA60" in hwid or "CP210" in hwid or "cp210" in description:
                found.append(port.device)
        return found

    @staticmethod
    def _ensure_port_permission(port):
        if not DRIVE_SETTINGS.get("LIDAR_CHMOD_BEFORE_CONNECT", True):
            return True

        mode_text = str(DRIVE_SETTINGS.get("LIDAR_CHMOD_MODE", "666"))
        try:
            os.chmod(port, int(mode_text, 8))
            return True
        except PermissionError:
            pass
        except Exception:
            return True

        try:
            result = subprocess.run(
                ["sudo", "-n", "chmod", mode_text, port],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0,
            )
            if result.returncode == 0:
                return True
            print(f"LiDAR permission warning: run `sudo chmod {mode_text} {port}`", flush=True)
            return True
        except Exception:
            print(f"LiDAR permission warning: run `sudo chmod {mode_text} {port}`", flush=True)
            return True

    @staticmethod
    def _read_lidar_info(lidar):
        try:
            return lidar.get_info()
        except Exception as exc:
            return {"error": str(exc)}

    def _close_lidar(self):
        lidar = self.lidar
        self.lidar = None
        if not lidar:
            return
        try:
            lidar.stop()
        except Exception:
            pass
        try:
            lidar.stop_motor()
        except Exception:
            pass
        try:
            lidar.disconnect()
        except Exception:
            pass
        with self.lock:
            self.data["lidar_connected"] = False

    def _set_error(self, error):
        with self.lock:
            self.data["lidar_error"] = error
            self.data["lidar_connected"] = False
            self.data["lidar_parse_ok"] = False

    def _set_parse_error(self, error):
        with self.lock:
            self.data["lidar_error"] = error
            self.data["lidar_parse_ok"] = False

    def _make_point(self, angle, distance, intensity=None):
        try:
            angle = float(angle) % 360.0
            distance = float(distance)
            intensity = float(intensity) if intensity is not None else None
        except (TypeError, ValueError):
            return None

        min_distance = DRIVE_SETTINGS.get("LIDAR_MIN_DISTANCE_M", 0.08)
        max_distance = DRIVE_SETTINGS.get("LIDAR_MAX_DISTANCE_M", 12.0)
        if not min_distance <= distance <= max_distance:
            return None
        return {"angle": angle, "distance": distance, "intensity": intensity}

    def _store_points(self, points, raw_line):
        now = time.monotonic()
        max_age = DRIVE_SETTINGS.get("LIDAR_POINT_MAX_AGE_SEC", 0.60)
        max_points = DRIVE_SETTINGS.get("LIDAR_MAX_POINTS", 1200)
        stamped = [
            (point["angle"], point["distance"], point.get("intensity"), now)
            for point in points
        ]
        with self.lock:
            self.points.extend(stamped)
            self.points = [
                point for point in self.points
                if now - point[3] <= max_age
            ][-max_points:]
            self._scan_count += 1
            self.data["lidar_connected"] = True
            self.data["lidar_parse_ok"] = True
            self.data["lidar_point_count"] = len(self.points)
            self.data["lidar_last_seen"] = time.strftime("%H:%M:%S")
            self.data["_lidar_last_seen_monotonic"] = now
            self.data["lidar_last_raw"] = raw_line[:240]
            self.data["lidar_error"] = ""
            self.data["lidar_scan_count"] = self._scan_count

    def wall_assist(self):
        if not DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_ENABLE", False):
            return self._store_wall_assist({"valid": False, "reason": "disabled"})

        now = time.monotonic()
        max_age = DRIVE_SETTINGS.get("LIDAR_POINT_MAX_AGE_SEC", 0.60)
        with self.lock:
            points = [point for point in self.points if now - point[3] <= max_age]

        if not points:
            return self._store_wall_assist({"valid": False, "reason": "no_points"})

        side = str(DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_SIDE", "auto")).strip().lower()
        candidates = []
        if side in ("auto", "left"):
            candidates.append(self._wall_candidate(points, "left"))
        if side in ("auto", "right"):
            candidates.append(self._wall_candidate(points, "right"))
        candidates = [candidate for candidate in candidates if candidate.get("valid")]
        if not candidates:
            return self._store_wall_assist({"valid": False, "reason": "no_near_wall"})

        if side == "auto":
            candidate = min(candidates, key=lambda item: item["mid_distance_m"])
        else:
            candidate = candidates[0]

        return self._store_wall_assist(candidate)

    def _store_wall_assist(self, assist):
        with self.lock:
            self.data["lidar_wall_assist"] = dict(assist)
        return assist

    def obstacle_ahead(self, distance_m=None, center_deg=None, width_deg=None):
        distance_m = float(distance_m if distance_m is not None else DRIVE_SETTINGS.get("LIDAR_OBSTACLE_DISTANCE_M", 0.30))
        center_deg = float(center_deg if center_deg is not None else DRIVE_SETTINGS.get("LIDAR_OBSTACLE_FRONT_CENTER_DEG", 0.0))
        width_deg = float(width_deg if width_deg is not None else DRIVE_SETTINGS.get("LIDAR_OBSTACLE_FRONT_WIDTH_DEG", 60.0))

        now = time.monotonic()
        max_age = DRIVE_SETTINGS.get("LIDAR_POINT_MAX_AGE_SEC", 0.60)
        with self.lock:
            points = [point for point in self.points if now - point[3] <= max_age]

        if width_deg >= 360.0:
            candidates = [
                (angle, distance, intensity)
                for angle, distance, intensity, _ts in points
            ]
        else:
            half_width = width_deg / 2.0
            candidates = [
                (angle, distance, intensity)
                for angle, distance, intensity, _ts in points
                if abs(self._angle_error_deg(angle, center_deg)) <= half_width
            ]

        nearest = min(candidates, key=lambda item: item[1]) if candidates else None
        obstacle = nearest is not None and nearest[1] <= distance_m
        result = {
            "valid": bool(points),
            "obstacle": bool(obstacle),
            "distance_m": round(nearest[1], 3) if nearest else None,
            "angle_deg": round(nearest[0], 1) if nearest else None,
            "quality": nearest[2] if nearest else None,
            "front_points": len(candidates),
            "fresh_points": len(points),
            "threshold_m": round(distance_m, 3),
            "center_deg": round(center_deg, 1),
            "width_deg": round(width_deg, 1),
        }
        with self.lock:
            self._last_obstacle = dict(result)
            self.data["lidar_obstacle"] = dict(result)
        return result

    def avoidance_direction(self):
        now = time.monotonic()
        max_age = DRIVE_SETTINGS.get("LIDAR_POINT_MAX_AGE_SEC", 0.60)
        with self.lock:
            points = [point for point in self.points if now - point[3] <= max_age]

        default_direction = str(DRIVE_SETTINGS.get("LIDAR_AVOIDANCE_DEFAULT_DIRECTION", "right")).strip().lower()
        if default_direction not in ("left", "right"):
            default_direction = "right"

        if not points:
            return self._store_avoidance({
                "valid": False,
                "direction": default_direction,
                "reason": "no_points",
                "left_score_m": None,
                "right_score_m": None,
            })

        width = float(DRIVE_SETTINGS.get("LIDAR_AVOIDANCE_SECTOR_WIDTH_DEG", 90.0))
        open_score = float(DRIVE_SETTINGS.get("LIDAR_AVOIDANCE_OPEN_SCORE_M", 3.0))
        left = self._avoidance_sector(
            points,
            "left",
            DRIVE_SETTINGS.get("LIDAR_AVOIDANCE_LEFT_CENTER_DEG", 90.0),
            width,
            open_score,
        )
        right = self._avoidance_sector(
            points,
            "right",
            DRIVE_SETTINGS.get("LIDAR_AVOIDANCE_RIGHT_CENTER_DEG", 270.0),
            width,
            open_score,
        )
        deadband = float(DRIVE_SETTINGS.get("LIDAR_AVOIDANCE_DIRECTION_DEADBAND_M", 0.10))
        if left["score_m"] > right["score_m"] + deadband:
            direction = "left"
        elif right["score_m"] > left["score_m"] + deadband:
            direction = "right"
        else:
            direction = default_direction

        return self._store_avoidance({
            "valid": True,
            "direction": direction,
            "reason": "sector_clearance",
            "left_score_m": round(left["score_m"], 3),
            "right_score_m": round(right["score_m"], 3),
            "left_nearest_m": left["nearest_m"],
            "right_nearest_m": right["nearest_m"],
            "left_points": left["points"],
            "right_points": right["points"],
            "sector_width_deg": round(width, 1),
        })

    def _store_avoidance(self, result):
        with self.lock:
            self.data["lidar_avoidance"] = dict(result)
        return result

    def _avoidance_sector(self, points, side, center_deg, width_deg, open_score):
        half_width = width_deg / 2.0
        distances = [
            distance for angle, distance, _intensity, _ts in points
            if abs(self._angle_error_deg(angle, float(center_deg))) <= half_width
        ]
        if not distances:
            return {
                "side": side,
                "score_m": open_score,
                "nearest_m": None,
                "points": 0,
            }
        nearest = min(distances)
        return {
            "side": side,
            "score_m": nearest,
            "nearest_m": round(nearest, 3),
            "points": len(distances),
        }

    def _wall_candidate(self, points, side):
        width = DRIVE_SETTINGS.get("LIDAR_WALL_SECTOR_WIDTH_DEG", 18.0)
        if side == "left":
            mid_deg = DRIVE_SETTINGS.get("LIDAR_WALL_LEFT_MID_DEG", 90.0)
            front_deg = DRIVE_SETTINGS.get("LIDAR_WALL_LEFT_FRONT_DEG", 65.0)
            rear_deg = DRIVE_SETTINGS.get("LIDAR_WALL_LEFT_REAR_DEG", 115.0)
            side_sign = 1.0
        else:
            mid_deg = DRIVE_SETTINGS.get("LIDAR_WALL_RIGHT_MID_DEG", 270.0)
            front_deg = DRIVE_SETTINGS.get("LIDAR_WALL_RIGHT_FRONT_DEG", 295.0)
            rear_deg = DRIVE_SETTINGS.get("LIDAR_WALL_RIGHT_REAR_DEG", 245.0)
            side_sign = -1.0

        mid = self._sector_median(points, mid_deg, width)
        front = self._sector_median(points, front_deg, width)
        rear = self._sector_median(points, rear_deg, width)
        if mid is None:
            return {"valid": False, "side": side, "reason": "no_mid"}

        near_min = DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_NEAR_MIN_M", 0.12)
        near_max = DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_NEAR_MAX_M", 0.85)
        if not near_min <= mid <= near_max:
            return {"valid": False, "side": side, "reason": "wall_not_near", "mid_distance_m": round(mid, 3)}

        target = DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_TARGET_M", 0.40)
        distance_error = target - mid
        parallel_error = 0.0
        if front is not None and rear is not None:
            parallel_error = rear - front

        correction = side_sign * (
            distance_error * DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_DISTANCE_GAIN", 18.0)
            + parallel_error * DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_PARALLEL_GAIN", 12.0)
        )
        correction *= -1.0 if DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_STEERING_SIGN", 1.0) < 0 else 1.0
        max_correction = DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_MAX_STEERING_DEG", 6.0)
        correction = max(-max_correction, min(max_correction, correction))
        return {
            "valid": True,
            "side": side,
            "mid_distance_m": round(mid, 3),
            "front_distance_m": round(front, 3) if front is not None else None,
            "rear_distance_m": round(rear, 3) if rear is not None else None,
            "target_distance_m": round(target, 3),
            "distance_error_m": round(distance_error, 3),
            "parallel_error_m": round(parallel_error, 3),
            "steering_correction_deg": round(correction, 2),
        }

    @staticmethod
    def _sector_median(points, center_deg, width_deg):
        half_width = width_deg / 2.0
        distances = [
            distance for angle, distance, _intensity, _ts in points
            if abs(((angle - center_deg + 180.0) % 360.0) - 180.0) <= half_width
        ]
        if not distances:
            return None
        distances.sort()
        middle = len(distances) // 2
        if len(distances) % 2:
            return distances[middle]
        return (distances[middle - 1] + distances[middle]) / 2.0

    @staticmethod
    def _angle_error_deg(angle, center):
        return ((angle - center + 180.0) % 360.0) - 180.0
