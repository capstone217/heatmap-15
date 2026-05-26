import threading
import time
import math

from config import APP_SETTINGS, DRIVE_SETTINGS


class Navigator:
    def __init__(self, robot, uwb, imu=None, camera=None, scanner=None, ultrasonic=None, lidar=None):
        self.robot = robot
        self.uwb = uwb
        self.imu = imu
        self.camera = camera
        self.scanner = scanner
        self.ultrasonic = ultrasonic
        self.lidar = lidar
        self.rf_db = None
        self.running = False
        self.stop_requested = False
        self.lock = threading.Lock()
        self.obstacle_count = 0
        self.points = []
        self.status = "Idle"
        self.current_target = None
        self.current_waypoint_index = 0
        self.total_waypoints = 0
        self.path_mode = self._auto_path_mode()
        self.visited_count = 0
        self.waypoint_states = []
        self.target_bearing = None
        self.current_heading = None
        self.heading_error = None
        self.distance_to_target = None
        self.x_boundary_recovery_reason = None
        self.last_waypoint_arrived = False
        self.waypoint_arrive_count = 0
        self.last_position_clamped = False
        self._x_boundary_recovery_active = False
        self._x_boundary_recovery_side = None
        self._last_x_boundary_recovery_log = 0.0
        self._last_waypoint_log = 0.0
        self._last_lidar_obstacle_log = 0.0
        self._lidar_obstacle_active = False
        self._last_lidar_obstacle = {}
        self._last_ultrasonic_obstacle_log = 0.0
        self._ultrasonic_obstacle_active = False
        self._last_ultrasonic_distance_cm = None
        self._last_obstacle_avoidance_at = 0.0
        self._last_obstacle_sequence_at = 0.0
        self._obstacle_sequence_count = 0
        self._last_avoidance_direction = None
        self._target_reorient_block_until = 0.0
        self.lidar_avoidance = {}
        self._rssi_power_stage_active = False
        self.rssi_power_state = {
            "stage": "idle",
            "dongle_power_enabled": False,
            "lidar_running": bool(getattr(lidar, "running", False)),
            "exclusive": DRIVE_SETTINGS.get("RSSI_POWER_EXCLUSIVE_LIDAR_WIFI", True),
            "simultaneous_power_blocked": False,
            "message": "RSSI dongle idle",
        }
        self._last_motion_update = time.monotonic()
        self._last_uwb_pos = None
        self._last_auto_steering = DRIVE_SETTINGS["FORWARD_STEERING"]
        self._last_heading = None
        self._estimated_heading = None
        self._imu_yaw_heading = None
        self._imu_yaw_bias = None
        self._imu_yaw_last_update = None
        self._last_heading_position = None
        self._turn_start_heading = None
        self._last_segment_drive_start = None
        self._last_segment_drive_end = None
        self.lidar_wall_assist = {}
        self.motion_metrics = {
            "odometry_distance_m": 0.0,
            "uwb_distance_m": 0.0,
            "wheel_distance_error_m": 0.0,
            "heading_drift_deg": 0.0,
            "last_turn_error_deg": None,
            "motion_mode": "idle",
        }

    def start(self):
        if self.running:
            return
        self._force_rssi_dongle_off("drive_start")
        self._center_steering_before_start()
        self._reset_motion_tracking()
        self.stop_requested = False
        self.running = True
        threading.Thread(target=self._drive_loop, daemon=True).start()

    @property
    def is_running(self):
        return self.running

    def stop(self):
        self.stop_requested = True
        self.running = False
        self.status = "Stopping"
        try:
            if self.robot:
                self.robot.stop()
        except Exception:
            pass
        self._force_rssi_dongle_off("nav_stop")
        self.status = "Stopped"

    def run_imu_pivot_test(self, direction="right", degrees=90.0):
        if self.running or not self.robot:
            return False
        direction = str(direction or "right").strip().lower()
        turn_error = abs(float(degrees))
        if direction == "left":
            turn_error = -turn_error

        self._force_rssi_dongle_off("imu_pivot_start")
        self.stop_requested = False
        self.running = True
        self.status = f"IMU {direction} pivot test {abs(turn_error):.0f}deg"
        try:
            ok = self._imu_integrated_pivot(turn_error, self.status)
            if not ok:
                self._time_based_pivot(turn_error, f"Timed {direction} pivot test {abs(turn_error):.0f}deg")
            self.status = f"IMU pivot test done ({direction})"
            return True
        finally:
            if self.robot:
                self.robot.stop()
            self._force_rssi_dongle_off("imu_pivot_finish")
            self.motion_metrics["motion_mode"] = "idle"
            self.running = False

    def run_compact_arc_test(self, direction="right", degrees=165.0):
        if self.running or not self.robot:
            return False
        direction = str(direction or "right").strip().lower()
        degrees = abs(float(degrees))
        speed = DRIVE_SETTINGS.get("COMPACT_ARC_TEST_SPEED", 10)
        steering_abs = DRIVE_SETTINGS.get(
            "COMPACT_ARC_TEST_STEERING_DEG",
            DRIVE_SETTINGS.get("AUTO_MAX_STEERING_DEG", 35),
        )
        seconds_165 = DRIVE_SETTINGS.get("COMPACT_ARC_TEST_165_SEC", 2.2)
        seconds = seconds_165 * max(0.0, degrees / 165.0)
        steering = steering_abs if direction == "right" else -steering_abs

        self._force_rssi_dongle_off("compact_arc_start")
        self.stop_requested = False
        self.running = True
        self.status = f"Compact arc {direction} {degrees:.0f}deg"
        try:
            self.robot.forward(
                speed,
                left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
                right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
                steering=steering,
            )
            self._sleep_with_uwb_recording(seconds, "turning", speed)
            self.status = f"Compact arc done ({direction})"
            return True
        finally:
            if self.robot:
                self.robot.stop()
            self._force_rssi_dongle_off("compact_arc_finish")
            self.motion_metrics["motion_mode"] = "idle"
            self.running = False

    def shutdown(self):
        self.stop()
        self._force_rssi_dongle_off("shutdown")
        if self.uwb:
            self.uwb.stop()
        if self.lidar:
            self.lidar.stop()
        if self.imu:
            self.imu.stop()

    def get_filtered_position(self):
        if not self.uwb:
            return None, None
        return self.uwb.get_position()

    def _waypoint_position(self):
        x, y = self.get_filtered_position()
        window = int(DRIVE_SETTINGS.get("UWB_AVG_WINDOW", 1) or 1)
        if window <= 1:
            return x, y

        samples = []
        with self.lock:
            for point in self.points[-window:]:
                px = point.get("x")
                py = point.get("y")
                if px is not None and py is not None:
                    samples.append((float(px), float(py)))
        if x is not None and y is not None:
            samples.append((float(x), float(y)))
        samples = samples[-window:]
        if not samples:
            return x, y

        avg_x = sum(px for px, _ in samples) / len(samples)
        avg_y = sum(py for _, py in samples) / len(samples)
        return avg_x, avg_y

    def get_display_position(self):
        if not self.uwb:
            return None, None
        if hasattr(self.uwb, "get_display_position"):
            return self.uwb.get_display_position()
        return self.uwb.get_position()

    def snapshot(self):
        data = {"status": self.status, "new_points": []}

        if self.uwb:
            data.update(self.uwb.snapshot())
        if self.lidar:
            data["lidar"] = self.lidar.snapshot()
        if self.imu:
            data.update(self.imu.snapshot())
        if self.camera:
            data.update(self.camera.snapshot())
        data.update(self._motion_snapshot())
        data["path_mode"] = self.path_mode
        data["current_target"] = self.current_target
        data["current_waypoint_index"] = self.current_waypoint_index
        data["total_waypoints"] = self.total_waypoints
        data["visited_count"] = self.visited_count
        data["waypoint_states"] = [dict(item) for item in getattr(self, "waypoint_states", [])]
        data["target_bearing"] = self.target_bearing
        data["current_heading"] = self.current_heading
        data["heading_error"] = self.heading_error
        data["auto_timed_waypoint_enable"] = DRIVE_SETTINGS.get("AUTO_TIMED_WAYPOINT_ENABLE", False)
        data["distance_to_target"] = self.distance_to_target
        data["x_boundary_recovery_reason"] = self.x_boundary_recovery_reason
        data["waypoint_arrived"] = self.last_waypoint_arrived
        data["waypoint_arrive_count"] = self.waypoint_arrive_count
        data["position_clamped"] = self.last_position_clamped

        with self.lock:
            data["new_points"] = list(self.points)
            self.points.clear()

        return data

    def _drive_loop(self):
        while self.running and not self.stop_requested:
            try:
                if self._obstacle_detected():
                    self._handle_obstacle_detected()
                else:
                    self.status = "Running"
                    if self.robot:
                        self.robot.forward(
                            DRIVE_SETTINGS["FORWARD_SPEED"],
                            left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
                            right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
                            steering=DRIVE_SETTINGS["FORWARD_STEERING"],
                        )
                    self._update_motion_tracking("forward", DRIVE_SETTINGS["FORWARD_SPEED"])
                    self._record_position()

                time.sleep(DRIVE_SETTINGS["CONTROL_PERIOD_SEC"])
            except Exception as exc:
                self.status = "Main Error"
                print(f"Navigation error: {exc}", flush=True)
                if self.robot:
                    self.robot.stop()
                time.sleep(1.0)

    def _obstacle_detected(self):
        camera_hit = False
        ultrasonic_hit = False
        lidar_hit = False

        if DRIVE_SETTINGS["CAMERA_OBSTACLE_ENABLE"] and self.camera:
            camera_data = self.camera.snapshot()
            camera_hit = camera_data["camera_ready"] and camera_data["camera_obstacle"]

        if DRIVE_SETTINGS.get("ULTRASONIC_ENABLE", False) and self.ultrasonic:
            ultrasonic_hit = self._ultrasonic_obstacle_detected()
        else:
            self._ultrasonic_obstacle_active = False
            self._last_ultrasonic_distance_cm = None

        lidar_obstacle_enabled = DRIVE_SETTINGS.get("LIDAR_OBSTACLE_ENABLE", False)

        if self.path_mode == "diamond":
            lidar_obstacle_enabled = (
                lidar_obstacle_enabled
                and DRIVE_SETTINGS.get("DIAMOND_LIDAR_OBSTACLE_ENABLE", True)
            )

        if lidar_obstacle_enabled and self.lidar:
            lidar_hit = self._lidar_obstacle_detected()
        else:
            self._lidar_obstacle_active = False
            self._last_lidar_obstacle = {}

        if lidar_hit:
            self.obstacle_count = max(1, DRIVE_SETTINGS["OBSTACLE_CONFIRM_COUNT"])
            return True

        if camera_hit or ultrasonic_hit:
            self.obstacle_count += 1
        else:
            self.obstacle_count = 0

        return self.obstacle_count >= DRIVE_SETTINGS["OBSTACLE_CONFIRM_COUNT"]

    def _ultrasonic_obstacle_detected(self):
        if not self.ultrasonic or not getattr(self.ultrasonic, "ready", False):
            self._ultrasonic_obstacle_active = False
            self._last_ultrasonic_distance_cm = None
            return False

        distance_cm = self.ultrasonic.read_distance()
        self._last_ultrasonic_distance_cm = distance_cm
        threshold_cm = DRIVE_SETTINGS.get("ULTRASONIC_THRESHOLD_CM", 32)
        hit = distance_cm is not None and distance_cm <= threshold_cm
        self._ultrasonic_obstacle_active = hit

        now = time.monotonic()
        log_sec = DRIVE_SETTINGS.get("ULTRASONIC_OBSTACLE_LOG_SEC", 0.5)
        if hit and now - self._last_ultrasonic_obstacle_log >= log_sec:
            self._last_ultrasonic_obstacle_log = now
            print(
                f"[ULTRASONIC_OBSTACLE] detected distance={distance_cm:.1f}cm, "
                f"threshold={threshold_cm}cm",
                flush=True,
            )
        return hit

    def _lidar_obstacle_detected(self):
        if not hasattr(self.lidar, "obstacle_ahead"):
            self._lidar_obstacle_active = False
            self._last_lidar_obstacle = {}
            return False

        try:
            result = self.lidar.obstacle_ahead(
                distance_m=DRIVE_SETTINGS.get("LIDAR_OBSTACLE_DISTANCE_M", 0.30),
                center_deg=DRIVE_SETTINGS.get("LIDAR_OBSTACLE_FRONT_CENTER_DEG", 0.0),
                width_deg=DRIVE_SETTINGS.get("LIDAR_OBSTACLE_FRONT_WIDTH_DEG", 360.0),
            )
            if not result.get("obstacle"):
                rear_result = self.lidar.obstacle_ahead(
                    distance_m=DRIVE_SETTINGS.get("LIDAR_REAR_OBSTACLE_DISTANCE_M", 0.40),
                    center_deg=DRIVE_SETTINGS.get("LIDAR_REAR_OBSTACLE_CENTER_DEG", 180.0),
                    width_deg=DRIVE_SETTINGS.get("LIDAR_REAR_OBSTACLE_WIDTH_DEG", 60.0),
                )
                if rear_result.get("obstacle"):
                    rear_result["zone"] = "rear"
                    result = rear_result
                else:
                    result["rear_obstacle"] = False
                    result["rear_distance_m"] = rear_result.get("distance_m")
                    result["rear_threshold_m"] = rear_result.get("threshold_m")
            else:
                result["zone"] = "front"
        except Exception as exc:
            self._lidar_obstacle_active = False
            self._last_lidar_obstacle = {"valid": False, "error": str(exc)}
            return False

        hit = bool(result.get("obstacle"))
        self._lidar_obstacle_active = hit
        self._last_lidar_obstacle = dict(result)
        now = time.monotonic()
        log_sec = DRIVE_SETTINGS.get("LIDAR_OBSTACLE_LOG_SEC", 0.5)
        if hit and now - self._last_lidar_obstacle_log >= log_sec:
            self._last_lidar_obstacle_log = now
            print(
                f"[LIDAR_OBSTACLE] detected distance={result.get('distance_m')}m, "
                f"angle={result.get('angle_deg')}deg, "
                f"zone={result.get('zone') or '-'}, "
                f"front_points={result.get('front_points')}, "
                f"threshold={result.get('threshold_m')}m",
                flush=True,
            )
        return hit

    def _rear_obstacle_detected(self):
        if not self.lidar or not hasattr(self.lidar, "obstacle_ahead"):
            return False
        try:
            result = self.lidar.obstacle_ahead(
                distance_m=DRIVE_SETTINGS.get("LIDAR_REAR_OBSTACLE_DISTANCE_M", 0.55),
                center_deg=DRIVE_SETTINGS.get("LIDAR_REAR_OBSTACLE_CENTER_DEG", 180.0),
                width_deg=DRIVE_SETTINGS.get("LIDAR_REAR_OBSTACLE_WIDTH_DEG", 60.0),
            )
        except Exception as exc:
            print(f"Rear LiDAR guard check failed: {exc}", flush=True)
            return False
        hit = bool(result.get("obstacle"))
        if hit:
            self._last_lidar_obstacle = dict(result, zone="rear")
            self.status = f"Reverse blocked: rear {result.get('distance_m')}m"
            print(
                f"[LIDAR_REAR_GUARD] rear obstacle distance={result.get('distance_m')}m, "
                f"angle={result.get('angle_deg')}deg, threshold={result.get('threshold_m')}m",
                flush=True,
            )
        return hit

    def _sleep_backward_with_rear_guard(self, seconds, speed, use_uwb_recording=False):
        end_time = time.monotonic() + max(0, seconds)
        while self.running and not self.stop_requested and time.monotonic() < end_time:
            if use_uwb_recording and self._uwb_failsafe_triggered():
                break
            if self._rear_obstacle_detected():
                if self.robot:
                    self.robot.stop()
                self._update_motion_tracking("idle", 0)
                return False
            self._enforce_power_exclusion_for_motion("backward")
            self._update_motion_tracking("backward", speed)
            if use_uwb_recording:
                self._record_position()
            time.sleep(min(0.05, max(0, end_time - time.monotonic())))
        if self.stop_requested and self.robot:
            self.robot.stop()
        return not self.stop_requested

    def _escape_forward_from_rear_obstacle(self):
        if not self.robot:
            return
        speed = DRIVE_SETTINGS.get("REAR_OBSTACLE_ESCAPE_FORWARD_SPEED", 12)
        seconds = float(DRIVE_SETTINGS.get("REAR_OBSTACLE_ESCAPE_FORWARD_SEC", 0.35) or 0.0)
        if seconds <= 0.0:
            return
        self.status = "Rear obstacle escape forward"
        print(
            f"[REVERSE_GUARD] rear obstacle: escape forward speed={speed}, sec={seconds}",
            flush=True,
        )
        if self.robot.forward(
            speed,
            left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
            right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
            steering=DRIVE_SETTINGS["FORWARD_STEERING"],
        ) is not False:
            self._sleep_with_motion(seconds, "forward", speed)
        self.robot.stop()

    def _handle_obstacle_detected(self):
        if self._lidar_obstacle_active and DRIVE_SETTINGS.get("LIDAR_OBSTACLE_STOP_ONLY", True):
            self._stop_for_lidar_obstacle()
            return "lidar_stop"

        self._avoid_obstacle()
        return "avoidance"

    def _stop_for_lidar_obstacle(self):
        if self.robot:
            self.robot.stop()
        self._last_obstacle_avoidance_at = time.monotonic()

        obstacle = self._last_lidar_obstacle or {}
        distance = obstacle.get("distance_m")
        angle = obstacle.get("angle_deg")
        prefix = "Diamond LiDAR stop" if self.path_mode == "diamond" else "LiDAR stop"
        if distance is not None and angle is not None:
            self.status = f"{prefix}: obstacle {distance:.2f}m @ {angle:.1f}deg"
        else:
            self.status = f"{prefix}: obstacle"
        self._update_motion_tracking("idle", 0)
        self._record_position()

    def _avoid_obstacle(self):
        if not self.robot:
            return

        now = time.monotonic()
        self._last_obstacle_avoidance_at = now
        window_sec = max(0.0, float(DRIVE_SETTINGS.get("CORNER_ESCAPE_WINDOW_SEC", 8.0)))
        if now - self._last_obstacle_sequence_at <= window_sec:
            self._obstacle_sequence_count += 1
        else:
            self._obstacle_sequence_count = 1
        self._last_obstacle_sequence_at = now

        lidar_avoidance = self._lidar_obstacle_active and DRIVE_SETTINGS.get("LIDAR_AVOIDANCE_ENABLE", True)
        corner_escape = (
            DRIVE_SETTINGS.get("CORNER_ESCAPE_ENABLE", True)
            and self._obstacle_sequence_count >= max(1, int(DRIVE_SETTINGS.get("CORNER_ESCAPE_TRIGGER_COUNT", 3)))
        )
        self.status = "LiDAR avoiding" if lidar_avoidance else (
            "Ultrasonic avoiding" if self._ultrasonic_obstacle_active else "Avoiding"
        )
        if corner_escape:
            self.status = "Corner escape"
            print(
                f"Corner escape triggered after {self._obstacle_sequence_count} obstacle events.",
                flush=True,
            )
        else:
            print("Obstacle detected. Reverse before choosing turn direction.", flush=True)

        self.robot.stop()
        self._sleep_with_motion(0.4, "idle", 0)
        if not DRIVE_SETTINGS.get("AUTO_REVERSE_ENABLE", True):
            self.status = "Reverse skipped"
            self._update_motion_tracking("idle", 0)
            print("[REVERSE_GUARD] automatic reverse skipped", flush=True)
        elif self._rear_obstacle_detected():
            self.robot.stop()
            self._update_motion_tracking("idle", 0)
            self._escape_forward_from_rear_obstacle()
        elif self.robot.backward(DRIVE_SETTINGS["REVERSE_SPEED"]) is False:
            self.status = "Reverse blocked"
            self._update_motion_tracking("idle", 0)
        else:
            reverse_sec = (
                DRIVE_SETTINGS.get("CORNER_ESCAPE_REVERSE_SEC", DRIVE_SETTINGS["REVERSE_TIME_SEC"])
                if corner_escape
                else DRIVE_SETTINGS["REVERSE_TIME_SEC"]
            )
            self._sleep_backward_with_rear_guard(reverse_sec, DRIVE_SETTINGS["REVERSE_SPEED"])
        self.robot.stop()
        self._sleep_with_motion(0.3, "idle", 0)

        direction = self._avoidance_turn_direction(lidar_avoidance)
        if (
            corner_escape
            and self._last_avoidance_direction in ("left", "right")
            and self._avoidance_scores_close()
        ):
            direction = self._last_avoidance_direction
        self.status = f"LiDAR avoiding {direction}" if lidar_avoidance else f"Avoiding {direction}"
        if corner_escape:
            self.status = f"Corner escape {direction}"
            print(f"Corner escape turn {direction}.", flush=True)
        else:
            print(f"Obstacle avoidance turn {direction}.", flush=True)

        self._start_turn_tracking()
        if direction == "left":
            self.robot.turn_left(DRIVE_SETTINGS["TURN_SPEED"])
        else:
            self.robot.turn_right(DRIVE_SETTINGS["TURN_SPEED"])

        turn_sec = (
            DRIVE_SETTINGS.get("CORNER_ESCAPE_TURN_SEC", DRIVE_SETTINGS["TURN_TIME_SEC"])
            if corner_escape
            else DRIVE_SETTINGS["TURN_TIME_SEC"]
        )
        self._sleep_with_motion(turn_sec, "turning", 0)
        self.robot.stop()
        self._finish_turn_tracking()

        forward_sec = max(
            0.0,
            float(
                DRIVE_SETTINGS.get(
                    "CORNER_ESCAPE_FORWARD_SEC" if corner_escape else "OBSTACLE_AVOID_FORWARD_SEC",
                    0.0,
                )
            ),
        )
        if forward_sec > 0.0:
            if self.robot.forward(
                DRIVE_SETTINGS["FORWARD_SPEED"],
                left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
                right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
                steering=DRIVE_SETTINGS["FORWARD_STEERING"],
            ) is not False:
                self._sleep_with_motion(forward_sec, "forward", DRIVE_SETTINGS["FORWARD_SPEED"])
            self.robot.stop()

        delay_sec = max(
            0.0,
            float(
                DRIVE_SETTINGS.get(
                    "CORNER_ESCAPE_REORIENT_DELAY_SEC"
                    if corner_escape
                    else "OBSTACLE_AVOID_REORIENT_DELAY_SEC",
                    0.0,
                )
            ),
        )
        if delay_sec > 0.0:
            self._target_reorient_block_until = time.monotonic() + delay_sec

        if corner_escape:
            self._obstacle_sequence_count = max(
                0,
                int(DRIVE_SETTINGS.get("CORNER_ESCAPE_TRIGGER_COUNT", 3)) - 1,
            )

        self._last_avoidance_direction = direction
        self.obstacle_count = 0
        self._last_obstacle_avoidance_at = time.monotonic()

    def _avoidance_scores_close(self):
        data = self.lidar_avoidance or {}
        left = data.get("left_score_m")
        right = data.get("right_score_m")
        if left is None or right is None:
            return True
        try:
            deadband = float(DRIVE_SETTINGS.get("LIDAR_AVOIDANCE_DIRECTION_DEADBAND_M", 0.10))
            return abs(float(left) - float(right)) <= deadband
        except (TypeError, ValueError):
            return True

    def _avoidance_turn_direction(self, use_lidar):
        if use_lidar and self.lidar and hasattr(self.lidar, "avoidance_direction"):
            try:
                result = self.lidar.avoidance_direction()
                self.lidar_avoidance = dict(result or {})
                direction = str(self.lidar_avoidance.get("direction") or "").strip().lower()
                if direction in ("left", "right"):
                    print(
                        "[LIDAR_AVOIDANCE] "
                        f"direction={direction}, "
                        f"left={self.lidar_avoidance.get('left_score_m')}, "
                        f"right={self.lidar_avoidance.get('right_score_m')}",
                        flush=True,
                    )
                    return direction
            except Exception as exc:
                self.lidar_avoidance = {"valid": False, "reason": str(exc)}
                print(f"[LIDAR_AVOIDANCE] direction failed: {exc}", flush=True)

        if self.camera:
            direction = str(self.camera.snapshot().get("camera_turn", "right")).strip().lower()
            if direction in ("left", "right"):
                return direction

        direction = str(DRIVE_SETTINGS.get("LIDAR_AVOIDANCE_DEFAULT_DIRECTION", "right")).strip().lower()
        return direction if direction in ("left", "right") else "right"

    def _record_position(self):
        if not self.uwb:
            return

        data = self.uwb.snapshot()
        if not data.get("uwb_valid"):
            return

        point = {
            "x": data.get("display_x", data.get("x")),
            "y": data.get("display_y", data.get("y")),
            "timestamp": data["uwb_last_seen"],
            "uwb": self._uwb_measurement_from_snapshot(data),
            "motion": self._motion_snapshot(),
        }

        with self.lock:
            self.points.append(point)
            if len(self.points) > 500:
                self.points.pop(0)

    @staticmethod
    def _uwb_measurement_from_snapshot(data):
        return {
            "d1": data.get("d1"),
            "d2": data.get("d2"),
            "d3": data.get("d3"),
            "d4": data.get("d4"),
            "rssi1": data.get("rssi1"),
            "rssi2": data.get("rssi2"),
            "rssi3": data.get("rssi3"),
            "rssi4": data.get("rssi4"),
            "rssi_avg": data.get("uwb_rssi"),
            "anchor_count": data.get("uwb_anchor_count", 0),
            "rssi_count": data.get("uwb_rssi_count", 0),
            "distance_error_m": data.get("uwb_distance_error_m"),
            "distance_error_max_m": data.get("uwb_distance_error_max_m"),
            "raw": data.get("uwb_last_raw", ""),
        }

    def capture_fingerprint_with_uwb(self, scanner, x=None, y=None, rssi_sample_sec=0.0, manage_power=True):
        uwb_data = self.uwb.snapshot() if self.uwb else {}
        if x is None:
            x = uwb_data.get("sample_x", uwb_data.get("drive_x", uwb_data.get("x")))
        if y is None:
            y = uwb_data.get("sample_y", uwb_data.get("drive_y", uwb_data.get("y")))

        if x is None or y is None:
            x, y = self.get_filtered_position()

        power_stage = self._begin_rssi_power_stage(scanner) if manage_power else None
        try:
            if manage_power and self.robot:
                self.robot.stop()
            fingerprint = scanner.capture_fingerprint(
                x,
                y,
                duration_sec=rssi_sample_sec,
                average_recent_count=DRIVE_SETTINGS.get("AUTO_RSSI_AVG_COUNT", 5),
            )
        finally:
            if manage_power:
                self._end_rssi_power_stage(scanner, power_stage)
        fingerprint["uwb"] = self._uwb_measurement_from_snapshot(uwb_data)
        fingerprint["motion"] = self._motion_snapshot()
        return fingerprint

    def _reset_motion_tracking(self):
        self._last_motion_update = time.monotonic()
        self._last_uwb_pos = self.get_filtered_position()
        self._last_heading_position = self._last_uwb_pos
        self._estimated_heading = None
        self._imu_yaw_heading = None
        self._imu_yaw_bias = None
        self._imu_yaw_last_update = None
        self._initialize_relative_imu_heading()
        self._last_heading = self._get_heading()
        self._turn_start_heading = None
        self.motion_metrics.update({
            "odometry_distance_m": 0.0,
            "uwb_distance_m": 0.0,
            "wheel_distance_error_m": 0.0,
            "heading_drift_deg": 0.0,
            "last_turn_error_deg": None,
            "motion_mode": "idle",
        })

    def _motion_snapshot(self):
        turn_error = self.motion_metrics["last_turn_error_deg"]
        return {
            "odometry_distance_m": round(self.motion_metrics["odometry_distance_m"], 3),
            "uwb_distance_m": round(self.motion_metrics["uwb_distance_m"], 3),
            "wheel_distance_error_m": round(self.motion_metrics["wheel_distance_error_m"], 3),
            "heading_drift_deg": round(self.motion_metrics["heading_drift_deg"], 1),
            "last_turn_error_deg": round(turn_error, 1) if turn_error is not None else None,
            "motion_mode": self.motion_metrics["motion_mode"],
            "lidar_wall_assist": dict(self.lidar_wall_assist or {}),
            "lidar_avoidance": dict(self.lidar_avoidance or {}),
            "rssi_power": self._rssi_power_snapshot(),
        }

    def _get_heading(self):
        if DRIVE_SETTINGS.get("AUTO_IMU_RELATIVE_HEADING_ENABLE", False):
            heading = self._update_imu_yaw_heading()
            if heading is not None:
                return heading
        heading = self._get_imu_heading(DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_STALE_SEC", 3.0))
        if heading is not None:
            return heading
        heading = self._update_imu_yaw_heading()
        if heading is not None:
            return heading
        return self._estimate_heading_from_uwb_motion()

    def _initialize_relative_imu_heading(self):
        if not DRIVE_SETTINGS.get("AUTO_IMU_RELATIVE_HEADING_ENABLE", False):
            return
        if not DRIVE_SETTINGS.get("AUTO_IMU_YAW_ENABLE", False):
            return

        initial_heading = float(DRIVE_SETTINGS.get("AUTO_INITIAL_HEADING_DEG", 0.0)) % 360.0
        bias = self._sample_imu_gyro_bias()
        now = time.monotonic()
        self._estimated_heading = initial_heading
        self._imu_yaw_heading = initial_heading
        self._imu_yaw_bias = bias
        self._imu_yaw_last_update = now
        self._last_heading = initial_heading
        self._last_heading_position = self._current_position_or_none()
        print(
            f"[IMU_HEADING] relative heading initialized: heading={initial_heading:.1f}deg, "
            f"bias={'ok' if bias is not None else 'unavailable'}",
            flush=True,
        )

    def _update_imu_yaw_heading(self):
        if not DRIVE_SETTINGS.get("AUTO_IMU_YAW_ENABLE", False):
            return None
        raw = self._imu_gyro_raw()
        if raw is None:
            return self._imu_yaw_heading

        now = time.monotonic()
        if self._imu_yaw_last_update is None:
            self._imu_yaw_last_update = now
            return self._imu_yaw_heading

        if self._imu_yaw_bias is None:
            self._imu_yaw_bias = tuple(raw)
            self._imu_yaw_last_update = now
            if self._imu_yaw_heading is None:
                self._imu_yaw_heading = float(DRIVE_SETTINGS.get("AUTO_INITIAL_HEADING_DEG", 0.0)) % 360.0
            return self._imu_yaw_heading

        dt = max(0.0, min(0.2, now - self._imu_yaw_last_update))
        self._imu_yaw_last_update = now
        yaw_rate_dps = self._imu_gyro_axis_dps(raw, self._imu_yaw_bias, signed=True)
        if self._imu_yaw_heading is None:
            self._imu_yaw_heading = (
                self._estimated_heading
                if self._estimated_heading is not None
                else float(DRIVE_SETTINGS.get("AUTO_INITIAL_HEADING_DEG", 0.0)) % 360.0
            )
        if self._imu_yaw_heading is not None:
            self._imu_yaw_heading = (self._imu_yaw_heading + yaw_rate_dps * dt) % 360.0
        return self._imu_yaw_heading

    def _get_imu_heading(self, max_age_sec=None):
        if self.imu:
            data = self.imu.snapshot()
            heading = data.get("heading")
            if heading is not None and self._sensor_fresh(data.get("imu_last_seen_monotonic"), max_age_sec):
                return heading
        return None

    def _get_uwb_serial_heading(self, max_age_sec=None):
        # IMU disabled.
        return None
        """
        if not self.uwb:
            return None
        data = self.uwb.snapshot()
        heading = data.get("heading")
        if heading is not None and self._sensor_fresh(data.get("imu_last_seen_monotonic"), max_age_sec):
            return heading
        return None
        """

    @staticmethod
    def _sensor_fresh(last_seen_monotonic, max_age_sec):
        if max_age_sec is None or last_seen_monotonic is None:
            return True
        return time.monotonic() - float(last_seen_monotonic) <= max_age_sec

    def _estimate_heading_from_uwb_motion(self):
        pos = self.get_filtered_position()
        if pos[0] is None or pos[1] is None:
            return self._estimated_heading

        if self._last_heading_position is None:
            self._last_heading_position = pos
            return self._estimated_heading

        prev_x, prev_y = self._last_heading_position
        if prev_x is None or prev_y is None:
            self._last_heading_position = pos
            return self._estimated_heading

        dx = pos[0] - prev_x
        dy = pos[1] - prev_y
        distance = math.hypot(dx, dy)
        min_move = DRIVE_SETTINGS.get("AUTO_UWB_HEADING_MIN_MOVE_M", 0.10)
        if distance >= min_move:
            self._estimated_heading = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
            self._last_heading_position = pos

        return self._estimated_heading

    def _update_motion_tracking(self, mode="idle", speed=0):
        if not DRIVE_SETTINGS.get("ODOMETRY_ENABLE", True):
            return

        now = time.monotonic()
        dt = max(0.0, now - self._last_motion_update)
        self._last_motion_update = now
        self.motion_metrics["motion_mode"] = mode

        if mode in ("forward", "backward"):
            speed_to_mps = DRIVE_SETTINGS.get("MOTOR_SPEED_TO_MPS", 0.012)
            self.motion_metrics["odometry_distance_m"] += abs(speed) * speed_to_mps * dt

        pos = self.get_filtered_position()
        if pos[0] is not None and pos[1] is not None:
            if self._last_uwb_pos and self._last_uwb_pos[0] is not None and self._last_uwb_pos[1] is not None:
                self.motion_metrics["uwb_distance_m"] += math.hypot(
                    pos[0] - self._last_uwb_pos[0],
                    pos[1] - self._last_uwb_pos[1],
                )
            self._last_uwb_pos = pos

        heading = self._get_heading()
        if heading is not None and self._last_heading is not None and mode in ("forward", "backward"):
            self.motion_metrics["heading_drift_deg"] += abs(self._angle_diff(heading, self._last_heading))
        if heading is not None:
            self._last_heading = heading

        self.motion_metrics["wheel_distance_error_m"] = (
            self.motion_metrics["odometry_distance_m"] - self.motion_metrics["uwb_distance_m"]
        )

    def _start_turn_tracking(self):
        self._update_motion_tracking("turning", 0)
        self._turn_start_heading = self._get_heading()

    def _finish_turn_tracking(self):
        current = self._get_heading()
        if current is not None and self._turn_start_heading is not None:
            turned = abs(self._angle_diff(current, self._turn_start_heading))
            expected = DRIVE_SETTINGS.get("TURN_EXPECTED_DEG", 90.0)
            self.motion_metrics["last_turn_error_deg"] = turned - expected
        self._turn_start_heading = None
        self._update_motion_tracking("idle", 0)

    def _sleep_with_motion(self, seconds, mode="idle", speed=0, step=0.05):
        end_time = time.monotonic() + max(0, seconds)
        while self.running and not self.stop_requested and time.monotonic() < end_time:
            self._enforce_power_exclusion_for_motion(mode)
            self._update_motion_tracking(mode, speed)
            time.sleep(min(step, max(0, end_time - time.monotonic())))
        if self.stop_requested and self.robot:
            self.robot.stop()

    def _sleep_without_correction(self, seconds, step=0.02):
        end_time = time.monotonic() + max(0, seconds)
        while self.running and not self.stop_requested and time.monotonic() < end_time:
            time.sleep(min(step, max(0, end_time - time.monotonic())))

    @staticmethod
    def _angle_diff(a, b):
        return (a - b + 180) % 360 - 180

    @staticmethod
    def _auto_path_mode():
        return str(DRIVE_SETTINGS.get("AUTO_PATH_MODE", "zigzag")).strip().lower()

    def _center_steering_before_start(self):
        if not self.robot or not DRIVE_SETTINGS.get("CENTER_STEERING_BEFORE_START", True):
            return

        steering = float(DRIVE_SETTINGS.get("FORWARD_STEERING", 0))
        settle_sec = max(0.0, DRIVE_SETTINGS.get("CENTER_STEERING_SETTLE_SEC", 0.4))
        self.status = f"Centering steering ({steering:.1f}deg)"
        try:
            self.robot.stop()
            if DRIVE_SETTINGS.get("CENTER_STEERING_SWEEP_ENABLE", False):
                sweep_deg = abs(float(DRIVE_SETTINGS.get("CENTER_STEERING_SWEEP_DEG", 10.0)))
                sweep_cycles = max(1, int(DRIVE_SETTINGS.get("CENTER_STEERING_SWEEP_CYCLES", 1)))
                sweep_step_sec = max(0.0, DRIVE_SETTINGS.get("CENTER_STEERING_SWEEP_STEP_SEC", 0.10))
                for _ in range(sweep_cycles):
                    self.robot.set_steering(steering - sweep_deg)
                    if sweep_step_sec > 0:
                        time.sleep(sweep_step_sec)
                    self.robot.set_steering(steering + sweep_deg)
                    if sweep_step_sec > 0:
                        time.sleep(sweep_step_sec)
            self.robot.set_steering(steering)
            if settle_sec > 0:
                time.sleep(settle_sec)
        except Exception as exc:
            print(f"Steering center before start failed: {exc}", flush=True)

    def autonomous_drive_grid(self, width, height, scanner, fingerprint_db):
        if self.running:
            return

        self.stop_requested = False
        self.running = True
        self.path_mode = self._auto_path_mode()
        self.status = f"Auto Mapping {width:.1f}x{height:.1f}m ({self.path_mode})"
        final_target = None
        completed = False

        try:
            self._force_rssi_dongle_off("auto_start")
            if (
                DRIVE_SETTINGS.get("LIDAR_START_ON_AUTO_START", True)
                and self.lidar
                and not getattr(self.lidar, "running", False)
            ):
                if self.lidar.start():
                    print("LiDAR started for autonomous exploration.", flush=True)
            self._center_steering_before_start()
            self._reset_motion_tracking()
            if DRIVE_SETTINGS.get("DIAMOND_MODE_ENABLE", False) or self.path_mode == "diamond":
                final_target, completed = self._autonomous_drive_diamond(width, height, scanner, fingerprint_db)
            elif self.path_mode == "nearest":
                final_target, completed = self._drive_nearest_coverage(width, height, scanner, fingerprint_db)
            elif self.path_mode == "right_1p5_sequence":
                final_target, completed = self._drive_right_angle_sequence(scanner, fingerprint_db)
            elif self.path_mode == "short_90_return":
                final_target, completed = self._drive_short_90_return(scanner, fingerprint_db)
            else:
                final_target, completed = self._drive_ordered_coverage(width, height, scanner, fingerprint_db)
        finally:
            if self.robot:
                self.robot.stop()
            if DRIVE_SETTINGS.get("LIDAR_STOP_ON_AUTO_FINISH", True) and self.lidar:
                try:
                    self.lidar.stop()
                    print("LiDAR stopped after autonomous exploration.", flush=True)
                except Exception as exc:
                    print(f"LiDAR stop after auto failed: {exc}", flush=True)
            self._force_rssi_dongle_off("auto_finish")
            self.running = False
            self.current_target = None if self.path_mode == "nearest" else (final_target if completed else None)
            if not str(self.status).startswith("Fail-safe"):
                if completed and self.path_mode == "nearest":
                    self.status = "Completed"
                elif completed and final_target:
                    self.status = f"Completed at ({final_target[0]:.2f}, {final_target[1]:.2f})"
                elif not str(self.status).startswith(("Target not reached", "Transition target not reached")):
                    self.status = "Stopped"

    def _drive_ordered_coverage(self, width, height, scanner, fingerprint_db, path=None):
        if path is None:
            path = self._build_auto_waypoints(width, height)
        transition_indices = self._zigzag_transition_indices(path) if self.path_mode == "zigzag" else set()
        self.total_waypoints = len(path)
        self.visited_count = 0
        final_target = path[-1] if path else None
        last_direction = None
        completed = False

        for index, target in enumerate(path):
            if not self.running:
                break
            self.current_target = target
            self.current_waypoint_index = self.visited_count + 1

            previous_target = path[index - 1] if index > 0 else None
            lane_change_target = False
            if previous_target and self._is_lane_change_segment(previous_target, target):
                lane_change_target = True
                self._lane_change_turn(previous_target, target)
                if not self.running:
                    break
                self._lane_change_reacquire_x(target)
                if not self.running:
                    break
                last_direction = self._planned_segment_direction(previous_target, target)
            if previous_target:
                direction = self._planned_segment_direction(previous_target, target)
            else:
                current = self._current_position_or_none()
                start = self._axis_locked_start(target, current) if current is not None else None
                direction = self._planned_segment_direction(start, target) if start is not None else None
            if lane_change_target:
                self.last_waypoint_arrived = True
                self.distance_to_target = None
                self.status = f"Lane ready at ({target[0]:.2f}, {target[1]:.2f})"
            elif index in transition_indices:
                next_target = path[index + 1] if index + 1 < len(path) else None
                transition_exit_direction = self._drive_zigzag_transition(
                    previous_target,
                    target,
                    next_target,
                    last_direction,
                )
                if not self.running:
                    break
                if not self.last_waypoint_arrived:
                    self.status = f"Transition target not reached ({target[0]:.2f}, {target[1]:.2f})"
                    break
                last_direction = transition_exit_direction or direction or last_direction
                self._scan_current_point(scanner, fingerprint_db, settle=True)
                self.visited_count += 1
                continue
            elif index > 0 or not self._at_waypoint(target):
                if index > 0 and self.path_mode == "zigzag":
                    if last_direction and direction and direction != last_direction:
                        self._pivot_for_direction_change(last_direction, direction)
                        if not self.running:
                            break
                        self._pivot_nudge_forward(target)
                reached = self._drive_to_waypoint(
                    target,
                    planned_start=previous_target,
                    scanner=scanner,
                    fingerprint_db=fingerprint_db,
                )
                if not self.running:
                    break
                if not reached:
                    self.status = f"Target not reached ({target[0]:.2f}, {target[1]:.2f})"
                    break

            next_is_transition = index + 1 in transition_indices
            if next_is_transition and self.path_mode == "zigzag":
                self.status = f"Corner ready at ({target[0]:.2f}, {target[1]:.2f})"
                self.last_waypoint_arrived = True
                if DRIVE_SETTINGS.get("AUTO_SCAN_ZIGZAG_CORNERS", True):
                    self._scan_current_point(scanner, fingerprint_db, settle=True)
            else:
                self._scan_current_point(scanner, fingerprint_db, settle=True)
            self.visited_count += 1
            last_direction = direction or last_direction
        else:
            completed = True

        if completed and self.running and DRIVE_SETTINGS.get("AUTO_FINISH_AT_BOTTOM_RIGHT", True):
            final_target = self._inner_bottom_right_target(width, height)
            self.current_target = final_target
            self.current_waypoint_index = self.total_waypoints
            if not path or math.hypot(path[-1][0] - final_target[0], path[-1][1] - final_target[1]) > 1e-6:
                completed = self._drive_to_waypoint(
                    final_target,
                    scanner=scanner,
                    fingerprint_db=fingerprint_db,
                )

        return final_target, self.running and completed

    def _autonomous_drive_diamond(self, width, height, scanner, fingerprint_db):
        previous_mode = self.path_mode
        self.path_mode = "diamond"
        path = self._build_diamond_waypoints(width, height)
        print(f"[PATH] using diamond waypoints: {path}", flush=True)
        print(
            "[DIAMOND] LiDAR obstacle guard "
            f"{'enabled' if DRIVE_SETTINGS.get('DIAMOND_LIDAR_OBSTACLE_ENABLE', True) and DRIVE_SETTINGS.get('LIDAR_OBSTACLE_ENABLE', False) else 'disabled'}",
            flush=True,
        )
        self.status = f"Diamond Mapping {width:.1f}x{height:.1f}m"
        try:
            return self._drive_ordered_coverage(width, height, scanner, fingerprint_db, path=path)
        finally:
            self.path_mode = previous_mode

    def _drive_right_angle_sequence(self, scanner, fingerprint_db):
        distance = float(DRIVE_SETTINGS.get("AUTO_RIGHT_SEQUENCE_DISTANCE_M", 1.5))
        turn_deg = abs(float(DRIVE_SETTINGS.get("AUTO_RIGHT_SEQUENCE_TURN_DEG", 90.0)))
        start = self._right_sequence_leg_start()
        if start is None:
            start = (
                float(DRIVE_SETTINGS.get("AUTO_RIGHT_SEQUENCE_START_FALLBACK_X_M", 0.45)),
                float(DRIVE_SETTINGS.get("AUTO_RIGHT_SEQUENCE_START_FALLBACK_Y_M", 0.0)),
            )

        progress_arrival = bool(DRIVE_SETTINGS.get("AUTO_RIGHT_SEQUENCE_PROGRESS_ARRIVAL_ENABLE", True))
        progress_tolerance = float(DRIVE_SETTINGS.get("AUTO_RIGHT_SEQUENCE_PROGRESS_TOLERANCE_M", 0.05))

        self.total_waypoints = 3
        self.visited_count = 0

        self.current_waypoint_index = 1
        first_target = self._right_sequence_target(start, "north", distance)
        self.current_target = first_target
        self.status = f"Sequence forward {distance:.2f}m"
        if not self._drive_to_waypoint(
            first_target,
            planned_start=start,
            progress_arrival=progress_arrival,
            progress_tolerance=progress_tolerance,
        ):
            return first_target, False
        self.visited_count = 1
        self._scan_current_point(scanner, fingerprint_db, settle=True)
        if not self.running:
            return first_target, False

        if not self._right_sequence_pivot("north", "east", turn_deg):
            return first_target, False

        self.current_waypoint_index = 2
        second_start = self._right_sequence_leg_start() or first_target
        second_target = self._right_sequence_target(second_start, "east", distance)
        self.current_target = second_target
        self.status = f"Sequence right leg {distance:.2f}m"
        if not self._drive_to_waypoint(
            second_target,
            planned_start=second_start,
            progress_arrival=progress_arrival,
            progress_tolerance=progress_tolerance,
        ):
            return second_target, False
        self.visited_count = 2
        self._scan_current_point(scanner, fingerprint_db, settle=True)
        if not self.running:
            return second_target, False

        if not self._right_sequence_pivot("east", "south", turn_deg):
            return second_target, False

        self.current_waypoint_index = 3
        final_start = self._right_sequence_leg_start() or second_target
        final_target = self._right_sequence_target(final_start, "south", distance)
        self.current_target = final_target
        self.status = f"Sequence final leg {distance:.2f}m"
        if not self._drive_to_waypoint(
            final_target,
            planned_start=final_start,
            progress_arrival=progress_arrival,
            progress_tolerance=progress_tolerance,
        ):
            return final_target, False
        self.visited_count = 3
        self._scan_current_point(scanner, fingerprint_db, settle=True)
        return final_target, self.running

    def _right_sequence_leg_start(self):
        sample_sec = max(0.0, float(DRIVE_SETTINGS.get("AUTO_RIGHT_SEQUENCE_START_SAMPLE_SEC", 0.35)))
        if sample_sec > 0.0:
            x, y = self._sample_uwb_position(sample_sec)
            if x is not None and y is not None:
                return (float(x), float(y))
        return self._current_position_or_none()

    @staticmethod
    def _right_sequence_target(start, direction, distance):
        sx, sy = start
        if direction == "east":
            return (round(sx + distance, 3), round(sy, 3))
        if direction == "south":
            return (round(sx, 3), round(sy - distance, 3))
        if direction == "west":
            return (round(sx - distance, 3), round(sy, 3))
        return (round(sx, 3), round(sy + distance, 3))

    def _right_sequence_pivot(self, from_direction, to_direction, turn_deg):
        if not self.robot:
            return False
        pause_sec = max(0.0, DRIVE_SETTINGS.get("AUTO_SHORT_90_PAUSE_BEFORE_TURN_SEC", 0.20))
        if pause_sec > 0:
            self.status = "Sequence pause before right turn"
            self.robot.stop()
            self._sleep_with_uwb_recording(pause_sec, "idle", 0)
        if not self.running:
            return False

        turn_error = abs(float(turn_deg))
        self.status = f"Sequence right turn {turn_error:.0f}deg"
        used_imu = self._imu_integrated_pivot(turn_error, self.status)
        if not used_imu and DRIVE_SETTINGS.get("AUTO_TURN_IMU_FALLBACK_TO_TIMED", True):
            self._time_based_pivot(turn_error, self.status)

        to_heading = self._planned_direction_heading(to_direction)
        if to_heading is not None:
            self._estimated_heading = to_heading
            self._imu_yaw_heading = to_heading
            self._imu_yaw_bias = None
            self._imu_yaw_last_update = None
            self._last_heading = to_heading
            self._last_heading_position = self._current_position_or_none()
        return self.running

    def _drive_short_90_return(self, scanner, fingerprint_db):
        start_pose_x = float(DRIVE_SETTINGS.get("AUTO_SHORT_90_START_POSE_X_M", 0.00))
        start_pose_y = float(DRIVE_SETTINGS.get("AUTO_SHORT_90_START_POSE_Y_M", 0.00))
        start_x = float(DRIVE_SETTINGS.get("AUTO_SHORT_90_START_X_M", 0.30))
        turn_y = float(DRIVE_SETTINGS.get("AUTO_SHORT_90_TURN_Y_M", 1.50))
        end_x = float(DRIVE_SETTINGS.get("AUTO_SHORT_90_END_X_M", 1.50))
        end_y = float(DRIVE_SETTINGS.get("AUTO_SHORT_90_END_Y_M", 0.00))
        second_turn_x = float(DRIVE_SETTINGS.get("AUTO_SHORT_90_SECOND_TURN_TRIGGER_X_M", end_x))
        first_target = (round(start_x, 3), round(turn_y, 3))
        second_target = (round(second_turn_x, 3), round(turn_y, 3))
        final_target = (round(end_x, 3), round(end_y, 3))
        path = [first_target, second_target, final_target]

        original_tolerance = DRIVE_SETTINGS.get("AUTO_WAYPOINT_TOLERANCE_M", 0.12)
        DRIVE_SETTINGS["AUTO_WAYPOINT_TOLERANCE_M"] = DRIVE_SETTINGS.get(
            "AUTO_SHORT_90_WAYPOINT_TOLERANCE_M",
            original_tolerance,
        )
        try:
            self.total_waypoints = len(path)
            self.visited_count = 0
            self.current_waypoint_index = 1
            self.current_target = first_target
            first_segment_start = self._current_position_or_none() or (start_pose_x, start_pose_y)
            self.status = f"Short 90 to ({first_target[0]:.2f}, {first_target[1]:.2f})"
            if not self._drive_to_waypoint(first_target):
                return first_target, False
            first_segment_end = self._current_position_or_none() or first_target
            self.visited_count = 1
            self._scan_current_point(scanner, fingerprint_db, settle=True)
            if not self.running:
                return first_target, False

            pause_sec = max(0.0, DRIVE_SETTINGS.get("AUTO_SHORT_90_PAUSE_BEFORE_TURN_SEC", 0.20))
            if pause_sec > 0:
                self.status = "Short 90 pause before IMU turn"
                self.robot.stop()
                self._sleep_with_uwb_recording(pause_sec, "idle", 0)

            direction = str(DRIVE_SETTINGS.get("AUTO_SHORT_90_TURN_DIRECTION", "right")).strip().lower()
            turn_deg = abs(float(DRIVE_SETTINGS.get("AUTO_SHORT_90_TURN_DEG", 90.0)))
            turn_error = self._adaptive_turn_error(
                first_segment_start,
                first_segment_end,
                desired_heading=90.0,
                fallback_deg=turn_deg,
                direction=direction,
            )
            self.status = f"Short 90 IMU pivot {direction} {abs(turn_error):.0f}deg"
            used_imu = self._imu_integrated_pivot(turn_error, self.status)
            if not used_imu and DRIVE_SETTINGS.get("AUTO_TURN_IMU_FALLBACK_TO_TIMED", True):
                self._time_based_pivot(turn_error, self.status)
            if not self.running:
                return first_target, False

            self.current_waypoint_index = 2
            self.current_target = second_target
            second_segment_start = self._current_position_or_none() or first_segment_end
            self.status = f"Short 90 drive until x={second_turn_x:.2f}"
            self._drive_forward_until_map_x(second_turn_x, "east")
            if not self.last_waypoint_arrived and self.running:
                self.last_waypoint_arrived = True
            if not self.running:
                return second_target, False
            second_segment_end = self._current_position_or_none() or second_target
            self.visited_count = 2
            self._scan_current_point(scanner, fingerprint_db, settle=True)
            if not self.running:
                return second_target, False

            if DRIVE_SETTINGS.get("AUTO_SHORT_90_SECOND_TURN_ENABLE", True):
                if pause_sec > 0:
                    self.status = "Short 90 pause before second IMU turn"
                    self.robot.stop()
                    self._sleep_with_uwb_recording(pause_sec, "idle", 0)
                second_direction = str(DRIVE_SETTINGS.get("AUTO_SHORT_90_SECOND_TURN_DIRECTION", "right")).strip().lower()
                second_deg = abs(float(DRIVE_SETTINGS.get("AUTO_SHORT_90_SECOND_TURN_DEG", 90.0)))
                second_error = self._adaptive_turn_error(
                    second_segment_start,
                    second_segment_end,
                    desired_heading=180.0,
                    fallback_deg=second_deg,
                    direction=second_direction,
                )
                self.status = f"Short 90 second IMU pivot {second_direction} {abs(second_error):.0f}deg"
                used_imu = self._imu_integrated_pivot(second_error, self.status)
                if not used_imu and DRIVE_SETTINGS.get("AUTO_TURN_IMU_FALLBACK_TO_TIMED", True):
                    self._time_based_pivot(second_error, self.status)

            if not self.running:
                return second_target, False

            self.current_waypoint_index = 3
            self.current_target = final_target
            self.status = f"Short 90 down to ({final_target[0]:.2f}, {final_target[1]:.2f})"
            if not self._drive_to_waypoint(final_target, planned_start=second_target):
                return final_target, False
            self.visited_count = 3
            self._scan_current_point(scanner, fingerprint_db, settle=True)

            return final_target, self.running
        finally:
            DRIVE_SETTINGS["AUTO_WAYPOINT_TOLERANCE_M"] = original_tolerance

    def _adaptive_turn_error(self, segment_start, segment_end, desired_heading, fallback_deg, direction):
        fallback = abs(float(fallback_deg))
        sign = 1.0 if str(direction).strip().lower() == "right" else -1.0
        fallback_error = sign * fallback
        if not DRIVE_SETTINGS.get("AUTO_SHORT_90_ADAPTIVE_TURN_ENABLE", True):
            return fallback_error
        if segment_start is None or segment_end is None:
            return fallback_error

        sx, sy = segment_start
        ex, ey = segment_end
        move_m = math.hypot(ex - sx, ey - sy)
        min_move = DRIVE_SETTINGS.get("AUTO_SHORT_90_ADAPTIVE_MIN_MOVE_M", 0.35)
        if move_m < min_move:
            return fallback_error

        actual_heading = self._bearing_to_target(segment_start, segment_end)
        if actual_heading is None:
            return fallback_error

        raw_error = self._angle_diff(float(desired_heading), actual_heading)
        if sign > 0 and raw_error < 0:
            raw_error += 360.0
        elif sign < 0 and raw_error > 0:
            raw_error -= 360.0

        max_correction = max(0.0, DRIVE_SETTINGS.get("AUTO_SHORT_90_ADAPTIVE_MAX_CORRECTION_DEG", 25.0))
        min_error = fallback - max_correction
        max_error = fallback + max_correction
        adjusted_abs = max(min_error, min(max_error, abs(raw_error)))
        adjusted_error = sign * adjusted_abs
        print(
            f"[SHORT_90_ADAPT] start=({sx:.2f},{sy:.2f}), end=({ex:.2f},{ey:.2f}), "
            f"heading={actual_heading:.1f}, desired={desired_heading:.1f}, "
            f"turn={adjusted_error:.1f}, fallback={fallback_error:.1f}",
            flush=True,
        )
        return adjusted_error

    def _zigzag_transition_indices(self, path):
        transitions = set()
        if len(path) < 3:
            return transitions

        spacing = max(0.1, DRIVE_SETTINGS.get("AUTO_GRID_SPACING_M", 0.5))
        for index in range(1, len(path) - 1):
            previous = path[index - 1]
            target = path[index]
            next_target = path[index + 1]
            dx = abs(target[0] - previous[0])
            dy = abs(target[1] - previous[1])
            next_dx = abs(next_target[0] - target[0])
            next_dy = abs(next_target[1] - target[1])
            if (
                abs(dx - spacing) <= 1e-6
                and dy <= 1e-6
                and next_dx <= 1e-6
                and 1e-6 < next_dy <= spacing + 1e-6
            ):
                transitions.add(index)
        return transitions

    def _drive_zigzag_transition(self, start, target, next_target=None, entry_direction=None):
        if not self.robot:
            return None
        if start is None:
            self._drive_to_waypoint(target)
            return None

        transition_direction = self._planned_segment_direction(start, target)
        exit_direction = self._planned_segment_direction(target, next_target) if next_target else None
        if DRIVE_SETTINGS.get("AUTO_CORNER_OPEN_LOOP_ENABLE", True):
            return self._run_open_loop_corner(
                entry_direction,
                transition_direction,
                exit_direction,
                target,
                next_target,
            )

        if self._use_double_pivot_forward_sequence(target, entry_direction, transition_direction, exit_direction):
            return self._run_double_pivot_forward_sequence(entry_direction)

        tolerance = DRIVE_SETTINGS.get("AUTO_ZIGZAG_TRANSITION_X_TOLERANCE_M", 0.25)
        timeout = DRIVE_SETTINGS.get("AUTO_ZIGZAG_TRANSITION_MAX_SEC", 1.8)
        speed = DRIVE_SETTINGS.get("AUTO_LANE_CHANGE_SPEED", DRIVE_SETTINGS["FORWARD_SPEED"])
        step_sec = 0.05
        deadline = time.monotonic() + timeout
        self.status = f"Transition to x={target[0]:.2f}"
        if entry_direction and transition_direction and entry_direction != transition_direction:
            self._pivot_for_direction_change(entry_direction, transition_direction)
            if not self.running or self.stop_requested:
                return transition_direction

        while self.running and not self.stop_requested and time.monotonic() < deadline:
            if self._uwb_failsafe_triggered():
                break

            x, y = self.get_filtered_position()
            if x is None or y is None or not self._current_position_valid_for_motion():
                self.robot.stop()
                self.status = f"UWB settling for transition x={target[0]:.2f}"
                self._sleep_with_uwb_recording(
                    DRIVE_SETTINGS.get("AUTO_STOP_GO_SETTLE_SEC", 0.45),
                    "idle",
                    0,
                )
                continue

            transition_x = x
            transition_target_x = target[0]
            transition_label = "map"
            if DRIVE_SETTINGS.get("AUTO_ZIGZAG_TRANSITION_X_USE_RAW", False) and self.uwb:
                raw_x = self.uwb.snapshot().get("raw_x")
                raw_target_x = DRIVE_SETTINGS.get("AUTO_ZIGZAG_TRANSITION_RAW_X_M")
                if raw_x is not None and raw_target_x is not None:
                    transition_x = float(raw_x)
                    transition_target_x = float(raw_target_x)
                    transition_label = "raw"

            x_error = transition_target_x - transition_x
            self.distance_to_target = round(abs(x_error), 3)
            if transition_label == "raw" and transition_direction == "east":
                self.last_waypoint_arrived = transition_x >= transition_target_x - tolerance
            elif transition_label == "raw" and transition_direction == "west":
                self.last_waypoint_arrived = transition_x <= transition_target_x + tolerance
            else:
                self.last_waypoint_arrived = abs(x_error) <= tolerance
            if self.last_waypoint_arrived:
                break

            steering = DRIVE_SETTINGS["FORWARD_STEERING"]
            self.status = f"Transition {transition_label}_x {transition_x:.2f}->{transition_target_x:.2f}"
            self.robot.forward(
                speed,
                left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
                right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
                steering=steering,
            )
            self._sleep_with_uwb_recording(step_sec, "forward", speed)

        self.robot.stop()
        self._sleep_with_motion(DRIVE_SETTINGS["AUTO_TURN_PAUSE_SEC"], "idle", 0)
        if transition_direction and exit_direction and transition_direction != exit_direction:
            self._pivot_for_direction_change(transition_direction, exit_direction)
            return exit_direction
        return transition_direction

    def _use_double_pivot_forward_sequence(self, target, entry_direction, transition_direction, exit_direction):
        if not DRIVE_SETTINGS.get("AUTO_CORNER_DOUBLE_PIVOT_FORWARD_SEQUENCE_ENABLE", False):
            return False
        if not (entry_direction and transition_direction and exit_direction):
            return False

        turn_y = DRIVE_SETTINGS.get("AUTO_TURN_Y_M")
        if turn_y is None:
            return False
        return abs(target[1] - float(turn_y)) <= 1e-6

    def _run_double_pivot_forward_sequence(self, entry_direction):
        current_direction = entry_direction
        self.status = "Double pivot corner sequence"
        print("[CORNER] sequence start: map_y turn -> right pivot -> map_x drive -> right pivot", flush=True)

        first_pivot_sec = DRIVE_SETTINGS.get(
            "RIGHT_PIVOT_FIRST_90_SEC",
            DRIVE_SETTINGS.get("RIGHT_PIVOT_90_SEC", DRIVE_SETTINGS["TURN_TIME_SEC"]),
        )
        second_pivot_sec = DRIVE_SETTINGS.get(
            "RIGHT_PIVOT_SECOND_90_SEC",
            DRIVE_SETTINGS.get("RIGHT_PIVOT_90_SEC", DRIVE_SETTINGS["TURN_TIME_SEC"]),
        )

        next_direction = self._right_direction(current_direction)
        if next_direction is None:
            return current_direction
        print("[CORNER] step 1/3: first right 90 pivot", flush=True)
        self._pivot_for_direction_change(current_direction, next_direction, first_pivot_sec)
        current_direction = next_direction
        if not self.running:
            return current_direction

        second_pivot_x = DRIVE_SETTINGS.get("AUTO_CORNER_SECOND_PIVOT_X_M")
        if second_pivot_x is None:
            return current_direction
        print(f"[CORNER] step 2/3: drive to map_x={float(second_pivot_x):.2f}", flush=True)
        self._drive_forward_until_map_x(float(second_pivot_x), current_direction)
        if not self.running:
            return current_direction

        next_direction = self._right_direction(current_direction)
        if next_direction is None:
            return current_direction
        print("[CORNER] step 3/3: second right 90 pivot", flush=True)
        self._pivot_for_direction_change(current_direction, next_direction, second_pivot_sec)
        current_direction = next_direction
        if not self.running:
            return current_direction

        print("[CORNER] sequence complete: resume next straight drive", flush=True)
        return current_direction

    def _drive_forward_until_map_x(self, target_x, direction):
        if not self.robot:
            return

        tolerance = DRIVE_SETTINGS.get("AUTO_CORNER_SECOND_PIVOT_X_TOLERANCE_M", 0.05)
        timeout = DRIVE_SETTINGS.get("AUTO_CORNER_SECOND_PIVOT_X_MAX_SEC", 4.0)
        speed = DRIVE_SETTINGS["FORWARD_SPEED"]
        step_sec = 0.05
        deadline = time.monotonic() + timeout

        self.status = f"Forward to map_x={target_x:.2f}"
        self.robot.stop()
        self._sleep_without_correction(DRIVE_SETTINGS.get("TIME_BASED_PIVOT_SETTLE_SEC", 0.5))

        while self.running and not self.stop_requested and time.monotonic() < deadline:
            if self._uwb_failsafe_triggered():
                break

            x, y = self.get_filtered_position()
            if x is None or y is None or not self._current_position_valid_for_motion():
                self.robot.stop()
                self.status = f"UWB settling before map_x={target_x:.2f}"
                self._sleep_with_uwb_recording(
                    DRIVE_SETTINGS.get("AUTO_STOP_GO_SETTLE_SEC", 0.45),
                    "idle",
                    0,
                )
                continue

            if direction == "east":
                arrived = x >= target_x - tolerance
            elif direction == "west":
                arrived = x <= target_x + tolerance
            else:
                arrived = abs(target_x - x) <= tolerance
            self.distance_to_target = round(abs(target_x - x), 3)
            print(
                f"[CORNER] map_x drive: current={x:.2f}, target={target_x:.2f}, arrived={arrived}",
                flush=True,
            )
            if arrived:
                break

            self.robot.forward(
                speed,
                left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
                right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
                steering=DRIVE_SETTINGS["FORWARD_STEERING"],
            )
            self.motion_metrics["motion_mode"] = "forward"
            self._sleep_with_uwb_recording(step_sec, "forward", speed)

        self.robot.stop()
        self.motion_metrics["motion_mode"] = "idle"
        self._sleep_without_correction(DRIVE_SETTINGS.get("TIME_BASED_PIVOT_SETTLE_SEC", 0.5))

    def _run_open_loop_corner(self, entry_direction, transition_direction, exit_direction, transition_target, next_target):
        if not self.robot:
            return exit_direction or transition_direction

        self.waypoint_arrive_count = 0
        self.last_waypoint_arrived = False
        self.distance_to_target = None
        self.status = f"Open-loop corner at ({transition_target[0]:.2f}, {transition_target[1]:.2f})"
        self.robot.stop()
        self._sleep_with_motion(DRIVE_SETTINGS["AUTO_TURN_PAUSE_SEC"], "idle", 0)

        if entry_direction and transition_direction and entry_direction != transition_direction:
            self._open_loop_turn_between(entry_direction, transition_direction)
            if not self.running:
                return transition_direction

        nudge_sec = DRIVE_SETTINGS.get("AUTO_CORNER_NUDGE_SEC", 0.15)
        if nudge_sec > 0:
            speed = DRIVE_SETTINGS.get("AUTO_CORNER_NUDGE_SPEED", 8)
            self.status = f"Open-loop corner nudge to x={transition_target[0]:.2f}"
            self.robot.forward(
                speed,
                left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
                right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
                steering=DRIVE_SETTINGS["FORWARD_STEERING"],
            )
            self._sleep_with_uwb_recording(nudge_sec, "forward", speed)
            self.robot.stop()
            self._sleep_with_motion(DRIVE_SETTINGS["AUTO_TURN_PAUSE_SEC"], "idle", 0)

        if transition_direction and exit_direction and transition_direction != exit_direction:
            self._open_loop_turn_between(transition_direction, exit_direction)
            if not self.running:
                return exit_direction

        settle_sec = DRIVE_SETTINGS.get("AUTO_CORNER_SETTLE_SEC", 0.5)
        if settle_sec > 0:
            self.status = "Open-loop corner UWB settle"
            self.robot.stop()
            self._sleep_with_uwb_recording(settle_sec, "idle", 0)

        if next_target is not None:
            self.status = f"Corner ready for ({next_target[0]:.2f}, {next_target[1]:.2f})"
        self.last_waypoint_arrived = False
        self.waypoint_arrive_count = 0
        return exit_direction or transition_direction

    def _open_loop_turn_between(self, from_direction, to_direction):
        from_heading = self._planned_direction_heading(from_direction)
        to_heading = self._planned_direction_heading(to_direction)
        if from_heading is None or to_heading is None:
            return

        turn_error = self._angle_diff(to_heading, from_heading)
        if abs(turn_error) < 1e-6:
            return

        status = f"Open-loop turn {from_direction}->{to_direction}"
        used_imu = False
        if DRIVE_SETTINGS.get("AUTO_TURN_USE_IMU_PIVOT", True):
            used_imu = self._imu_integrated_pivot(turn_error, status)
        if not used_imu and DRIVE_SETTINGS.get("AUTO_TURN_IMU_FALLBACK_TO_TIMED", True):
            self._time_based_pivot(turn_error, status)

    @staticmethod
    def _planned_direction_heading(direction):
        headings = {
            "north": 0.0,
            "east": 90.0,
            "south": 180.0,
            "west": 270.0,
        }
        return headings.get(direction)

    @staticmethod
    def _right_direction(direction):
        directions = ["north", "east", "south", "west"]
        if direction not in directions:
            return None
        return directions[(directions.index(direction) + 1) % len(directions)]

    def _pivot_for_direction_change(self, from_direction, to_direction, seconds_90_override=None):
        if not DRIVE_SETTINGS.get("AUTO_ZIGZAG_CORNER_PIVOT_ENABLE", True):
            return
        if not self.robot:
            return

        from_heading = self._planned_direction_heading(from_direction)
        to_heading = self._planned_direction_heading(to_direction)
        if from_heading is None or to_heading is None:
            return

        measured_from_heading = self._uwb_trajectory_heading_for_last_segment(from_heading)
        turn_error = self._angle_diff(to_heading, measured_from_heading)
        if abs(turn_error) < 1e-6:
            return

        status = f"Corner pivot {from_direction}->{to_direction}"
        used_imu = False
        if DRIVE_SETTINGS.get("AUTO_TURN_USE_IMU_PIVOT", True):
            used_imu = self._imu_integrated_pivot(turn_error, status, seconds_90_override)
        if not used_imu and DRIVE_SETTINGS.get("AUTO_TURN_IMU_FALLBACK_TO_TIMED", True):
            self._time_based_pivot(turn_error, status, seconds_90_override)
        self._estimated_heading = to_heading
        self._imu_yaw_heading = to_heading
        self._imu_yaw_bias = None
        self._imu_yaw_last_update = None
        self._last_heading = to_heading
        self._last_heading_position = self._current_position_or_none()

    def _uwb_trajectory_heading_for_last_segment(self, fallback_heading):
        if not DRIVE_SETTINGS.get("AUTO_TURN_USE_UWB_TRAJECTORY_HEADING", True):
            return fallback_heading

        start = self._last_segment_drive_start
        end = self._last_segment_drive_end or self._current_position_or_none()
        if start is None or end is None:
            return fallback_heading

        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        min_move = DRIVE_SETTINGS.get("AUTO_TURN_UWB_TRAJECTORY_MIN_MOVE_M", 0.25)
        if distance < min_move:
            return fallback_heading

        actual_heading = self._bearing_to_target(start, end)
        if actual_heading is None:
            return fallback_heading

        max_correction = max(
            0.0,
            DRIVE_SETTINGS.get("AUTO_TURN_UWB_TRAJECTORY_MAX_CORRECTION_DEG", 25.0),
        )
        correction = self._angle_diff(actual_heading, fallback_heading)
        if abs(correction) > max_correction:
            correction = max(-max_correction, min(max_correction, correction))

        measured_heading = (fallback_heading + correction) % 360.0
        print(
            f"[PIVOT_UWB] planned={fallback_heading:.1f}, actual={actual_heading:.1f}, "
            f"used={measured_heading:.1f}, move={distance:.2f}m",
            flush=True,
        )
        return measured_heading

    def _imu_integrated_pivot(self, turn_error, status, seconds_90_override=None):
        if not DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_ENABLE", False):
            return False
        if not self.robot or not self.uwb:
            return False

        initial_gyro = self._imu_gyro_axis_dps()
        if initial_gyro is None:
            print("[PIVOT_IMU] gyro unavailable; using timed pivot fallback.", flush=True)
            return False

        direction = "right" if turn_error > 0 else "left"
        if seconds_90_override is not None:
            seconds_90 = float(seconds_90_override)
        else:
            seconds_90 = DRIVE_SETTINGS.get(
                "RIGHT_PIVOT_90_SEC" if direction == "right" else "LEFT_PIVOT_90_SEC",
                DRIVE_SETTINGS.get("AUTO_ZIGZAG_CORNER_PIVOT_SEC", DRIVE_SETTINGS["TURN_TIME_SEC"]),
            )
        target_deg = abs(turn_error)
        configured_target = DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_TARGET_DEG")
        if configured_target is not None and abs(target_deg - 90.0) <= 1e-6:
            target_deg = float(configured_target)
        tolerance = max(0.0, DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_TOLERANCE_DEG", 7.0))
        stop_deg = max(0.0, target_deg - tolerance)
        max_sec = max(
            0.4,
            seconds_90 * max(0.0, abs(turn_error) / 90.0) * DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_MAX_SEC_FACTOR", 1.6),
        )
        speed = DRIVE_SETTINGS.get("AUTO_ZIGZAG_CORNER_PIVOT_SPEED", DRIVE_SETTINGS["TURN_SPEED"])
        step_sec = max(0.005, DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_STEP_SEC", 0.02))
        settle_sec = DRIVE_SETTINGS.get("TIME_BASED_PIVOT_SETTLE_SEC", 0.5)

        self.status = f"{status} IMU {target_deg:.0f}deg"
        print(
            f"[PIVOT_IMU] {direction}: axis={DRIVE_SETTINGS.get('AUTO_ZIGZAG_PIVOT_IMU_AXIS', 'z')}, "
            f"target={target_deg:.1f}deg, stop={stop_deg:.1f}deg, max={max_sec:.2f}s, speed={speed}",
            flush=True,
        )
        self.robot.stop()
        self.motion_metrics["motion_mode"] = "idle"
        self._sleep_without_correction(settle_sec)
        if not self.running:
            return True
        gyro_bias = self._sample_imu_gyro_bias()
        if gyro_bias is None:
            print("[PIVOT_IMU] gyro bias sample failed; using timed pivot fallback.", flush=True)
            return False

        turned_deg = 0.0
        last_t = time.monotonic()
        deadline = last_t + max_sec
        min_gyro_dps = max(0.0, DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_MIN_GYRO_DPS", 3.0))
        if direction == "right":
            self.robot.turn_right(speed)
        else:
            self.robot.turn_left(speed)
        self.motion_metrics["motion_mode"] = "turning"

        try:
            while self.running and not self.stop_requested and time.monotonic() < deadline:
                now = time.monotonic()
                gyro_dps = self._imu_gyro_axis_dps(bias=gyro_bias)
                dt = max(0.0, now - last_t)
                last_t = now
                if gyro_dps is not None and gyro_dps >= min_gyro_dps:
                    turned_deg += gyro_dps * dt
                    self.heading_error = round(max(0.0, target_deg - turned_deg), 1)
                if turned_deg >= stop_deg:
                    break
                time.sleep(step_sec)
        finally:
            self.robot.stop()
            self.motion_metrics["motion_mode"] = "idle"
            self._sleep_without_correction(settle_sec)

        ok = turned_deg >= stop_deg
        print(
            f"[PIVOT_IMU] done: turned={turned_deg:.1f}deg, ok={ok}, "
            f"bias=({gyro_bias[0]:.1f},{gyro_bias[1]:.1f},{gyro_bias[2]:.1f})",
            flush=True,
        )
        return ok

    def _sample_imu_gyro_bias(self):
        sample_sec = max(0.0, DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_BIAS_SAMPLE_SEC", 0.35))
        deadline = time.monotonic() + sample_sec
        samples = []
        while self.running and not self.stop_requested and time.monotonic() < deadline:
            raw = self._imu_gyro_raw()
            if raw is not None:
                samples.append(raw)
            time.sleep(0.02)
        if not samples:
            return None
        return tuple(sum(sample[index] for sample in samples) / len(samples) for index in range(3))

    def _imu_gyro_raw(self):
        if not self.uwb:
            return None
        data = self.uwb.snapshot()
        last_seen = data.get("imu_last_seen_monotonic")
        max_age_sec = DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_STALE_SEC", 3.0)
        if last_seen is None or not self._sensor_fresh(last_seen, max_age_sec):
            return None

        values = []
        for key in ("imu_gx", "imu_gy", "imu_gz"):
            value = data.get(key)
            if value is None:
                return None
            try:
                values.append(float(value))
            except (TypeError, ValueError):
                return None
        return values

    def _imu_gyro_magnitude_dps(self, bias=None):
        values = self._imu_gyro_raw()
        if values is None:
            return None
        if bias is not None:
            values = [value - bias[index] for index, value in enumerate(values)]
        scale = max(1e-6, DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_GYRO_SCALE", 131.0))
        return math.sqrt(sum((value / scale) ** 2 for value in values))

    def _imu_gyro_axis_dps(self, values=None, bias=None, signed=False):
        if values is None:
            values = self._imu_gyro_raw()
        if values is None:
            return None

        axis = str(DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_AXIS", "z")).strip().lower()
        axis_index = {"x": 0, "y": 1, "z": 2}.get(axis, 2)
        value = float(values[axis_index])
        if bias is not None:
            value -= float(bias[axis_index])
        scale = max(1e-6, DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_GYRO_SCALE", 131.0))
        dps = value / scale
        return dps if signed else abs(dps)

    def _time_based_pivot(self, turn_error, status, seconds_90_override=None):
        if not self.robot:
            return

        direction = "right" if turn_error > 0 else "left"
        if seconds_90_override is not None:
            seconds_90 = float(seconds_90_override)
        else:
            seconds_90 = DRIVE_SETTINGS.get(
                "RIGHT_PIVOT_90_SEC" if direction == "right" else "LEFT_PIVOT_90_SEC",
                DRIVE_SETTINGS.get("AUTO_ZIGZAG_CORNER_PIVOT_SEC", DRIVE_SETTINGS["TURN_TIME_SEC"]),
            )
        seconds = seconds_90 * max(0.0, abs(turn_error) / 90.0)
        speed = DRIVE_SETTINGS.get("AUTO_ZIGZAG_CORNER_PIVOT_SPEED", DRIVE_SETTINGS["TURN_SPEED"])
        settle_sec = DRIVE_SETTINGS.get("TIME_BASED_PIVOT_SETTLE_SEC", 0.5)

        self.status = status
        print(f"[PIVOT] {direction}: speed={speed}, duration={seconds:.2f}s", flush=True)
        self.robot.stop()
        self.motion_metrics["motion_mode"] = "idle"
        self._sleep_without_correction(settle_sec)
        if not self.running:
            return

        pulse_count = 1
        pulse_pause_sec = 0.0
        if direction == "right":
            pulse_count = max(1, int(DRIVE_SETTINGS.get("RIGHT_PIVOT_PULSE_COUNT", 1)))
            pulse_pause_sec = max(0.0, DRIVE_SETTINGS.get("RIGHT_PIVOT_PULSE_PAUSE_SEC", 0.0))
        pulse_seconds = seconds / pulse_count

        for index in range(pulse_count):
            print(
                f"[PIVOT] {direction} pulse {index + 1}/{pulse_count}: "
                f"speed={speed}, duration={pulse_seconds:.2f}s",
                flush=True,
            )
            if direction == "right":
                self.robot.turn_right(speed)
            else:
                self.robot.turn_left(speed)
            self.motion_metrics["motion_mode"] = "turning"
            self._sleep_without_correction(pulse_seconds)
            self.robot.stop()
            self.motion_metrics["motion_mode"] = "idle"
            if not self.running or index == pulse_count - 1:
                break
            self._sleep_without_correction(pulse_pause_sec)

        self.robot.stop()
        self.motion_metrics["motion_mode"] = "idle"
        self._sleep_without_correction(settle_sec)

    def _pivot_to_heading(self, target_heading, status):
        # IMU disabled.
        return False
        """
        if not self.robot:
            return False

        max_age_sec = DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_STALE_SEC", 1.0)
        heading = self._get_imu_heading(max_age_sec)
        if heading is None:
            heading = self._get_uwb_serial_heading(max_age_sec)
        if heading is None:
            self.status = "IMU heading unavailable for pivot"
            print("[PIVOT] IMU heading unavailable; using timed fallback if configured.", flush=True)
            return False

        tolerance = DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_TOLERANCE_DEG", 6.0)
        deadline = time.monotonic() + DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_MAX_SEC", 2.2)
        speed = DRIVE_SETTINGS.get("AUTO_ZIGZAG_CORNER_PIVOT_SPEED", DRIVE_SETTINGS["TURN_SPEED"])
        step_sec = DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_STEP_SEC", 0.04)
        settle_sec = DRIVE_SETTINGS.get("AUTO_ZIGZAG_PIVOT_IMU_SETTLE_SEC", 0.15)

        self.status = status
        self.robot.stop()
        self._sleep_with_motion(DRIVE_SETTINGS["AUTO_TURN_PAUSE_SEC"], "idle", 0)
        self._start_turn_tracking()

        while self.running and not self.stop_requested and time.monotonic() < deadline:
            heading = self._get_imu_heading(max_age_sec)
            if heading is None:
                heading = self._get_uwb_serial_heading(max_age_sec)
            if heading is None:
                break

            error = self._angle_diff(target_heading, heading)
            self.current_heading = round(heading, 1)
            self.target_bearing = round(target_heading, 1)
            self.heading_error = round(error, 1)
            if abs(error) <= tolerance:
                self.robot.stop()
                self._sleep_with_uwb_recording(settle_sec, "idle", 0)
                self._finish_turn_tracking()
                self._estimated_heading = target_heading
                self._last_heading = target_heading
                self._last_heading_position = self._current_position_or_none()
                return True

            if error > 0:
                self.robot.turn_right(speed)
            else:
                self.robot.turn_left(speed)
            self._sleep_with_uwb_recording(step_sec, "turning", speed)

        self.robot.stop()
        self._finish_turn_tracking()
        final_heading = self._get_imu_heading(max_age_sec) or self._get_uwb_serial_heading(max_age_sec)
        if final_heading is None:
            return False

        final_error = self._angle_diff(target_heading, final_heading)
        self.current_heading = round(final_heading, 1)
        self.target_bearing = round(target_heading, 1)
        self.heading_error = round(final_error, 1)
        ok = abs(final_error) <= tolerance * 1.5
        if ok:
            self._estimated_heading = target_heading
            self._last_heading = target_heading
            self._last_heading_position = self._current_position_or_none()
        else:
            print(f"[PIVOT] IMU pivot timed out: error={final_error:.1f}deg", flush=True)
        return ok
        """

    def _drive_nearest_coverage(self, width, height, scanner, fingerprint_db):
        waypoints = self._build_nearest_waypoints(width, height)
        waypoint_states = [
            {"x": point[0], "y": point[1], "visited": False, "failed_count": 0, "defer_until": 0.0}
            for point in waypoints
        ]
        self.waypoint_states = waypoint_states
        self.total_waypoints = len(waypoints)
        self.visited_count = 0
        final_target = None
        preferred_next_target_index = None
        max_failed_count = max(1, int(DRIVE_SETTINGS.get("TARGET_MAX_FAILED_COUNT", 3)))
        retry_defer_sec = max(0.0, float(DRIVE_SETTINGS.get("TARGET_RETRY_DEFER_SEC", 5.0)))
        print(f"[NEAREST] unvisited waypoint mode: {waypoint_states}", flush=True)

        if DRIVE_SETTINGS.get("COVERAGE_WANDER_ENABLE", True):
            return self._drive_waypoint_coverage_wander(waypoints, waypoint_states, scanner, fingerprint_db)

        while self.running and not self.stop_requested and self.visited_count < self.total_waypoints:
            if self._uwb_failsafe_triggered():
                break

            if self._current_position_valid_for_motion():
                start_index = self._nearest_arrived_unvisited_waypoint_index(waypoints, waypoint_states)
                if start_index is not None:
                    target = waypoints[start_index]
                    if self.robot:
                        self.robot.stop()
                    self._scan_current_point(scanner, fingerprint_db, settle=True)
                    waypoint_states[start_index]["visited"] = True
                    waypoint_states[start_index]["state"] = "visited"
                    self.visited_count = sum(1 for item in waypoint_states if item["visited"])
                    self.current_target = None
                    preferred_next_target_index = self._next_inward_waypoint_index(waypoints, waypoint_states, start_index)
                    self.status = f"Start waypoint visited {self.visited_count}/{self.total_waypoints}"
                    print(
                        f"[NEAREST] current position is inside waypoint {start_index + 1}: "
                        f"({target[0]:.2f}, {target[1]:.2f}), "
                        f"visited={self.visited_count}/{self.total_waypoints}",
                        flush=True,
                    )
                    continue

                visited = [state["visited"] for state in waypoint_states]
                if (
                    preferred_next_target_index is not None
                    and not waypoint_states[preferred_next_target_index]["visited"]
                ):
                    target_index = preferred_next_target_index
                    preferred_next_target_index = None
                else:
                    preferred_next_target_index = None
                    target_index = self._nearest_unvisited_waypoint_index(waypoints, visited)
            else:
                target_index = None
            if target_index is None:
                if all(state["visited"] for state in waypoint_states):
                    break
                self.status = "Waiting for valid UWB before target select"
                self.distance_to_target = None
                if self.robot:
                    self.robot.stop()
                self._sleep_with_uwb_recording(
                    DRIVE_SETTINGS.get("AUTO_STOP_GO_SETTLE_SEC", 0.45),
                    "idle",
                    0,
                )
                continue

            target = waypoints[target_index]
            final_target = target
            self.current_target = target
            self.current_waypoint_index = target_index + 1
            self.status = f"Nearest target ({target[0]:.2f}, {target[1]:.2f})"
            print(
                f"[NEAREST] selected target {target_index + 1}/{self.total_waypoints}: "
                f"({target[0]:.2f}, {target[1]:.2f})",
                flush=True,
            )

            if not self._nearest_arrived(target):
                if self._get_heading() is not None and not self._align_heading_to_target(target):
                    continue
                if not self.running:
                    break
                self._drive_to_waypoint(target, scanner=scanner, fingerprint_db=fingerprint_db)
                if not self.running:
                    break

            if self._nearest_arrived(target):
                if self.robot:
                    self.robot.stop()
                self._scan_current_point(scanner, fingerprint_db, settle=True)
                waypoint_states[target_index]["visited"] = True
                waypoint_states[target_index]["state"] = "visited"
                self.visited_count = sum(1 for item in waypoint_states if item["visited"])
                self.last_waypoint_arrived = True
                self.current_target = None
                self.status = f"Visited {self.visited_count}/{self.total_waypoints}"
                print(
                    f"[NEAREST] arrived target {target_index + 1}: "
                    f"({target[0]:.2f}, {target[1]:.2f}), "
                    f"visited={self.visited_count}/{self.total_waypoints}",
                    flush=True,
                )
            else:
                self.last_waypoint_arrived = False
                if self.robot:
                    self.robot.stop()
                self._scan_current_point(scanner, fingerprint_db, settle=True)
                waypoint_states[target_index]["failed_count"] += 1
                final_failed = waypoint_states[target_index]["failed_count"] >= max_failed_count
                waypoint_states[target_index]["visited"] = final_failed
                waypoint_states[target_index]["state"] = "failed" if final_failed else "retry"
                waypoint_states[target_index]["defer_until"] = 0.0 if final_failed else time.monotonic() + retry_defer_sec
                self.visited_count = sum(1 for item in waypoint_states if item["visited"])
                self.current_target = None
                self.status = f"Nearest target {'failed' if final_failed else 'deferred'} {self.visited_count}/{self.total_waypoints}"
                print(
                    f"[NEAREST] target miss: ({target[0]:.2f}, {target[1]:.2f}), "
                    f"failed_count={waypoint_states[target_index]['failed_count']}/{max_failed_count}, "
                    f"final_failed={final_failed}; current position saved and moving on",
                    flush=True,
                )

        return final_target, self.running and self.visited_count >= self.total_waypoints

    def _drive_waypoint_coverage_wander(self, waypoints, waypoint_states, scanner, fingerprint_db):
        if not self.robot:
            return None, False

        self.current_target = None
        self.current_waypoint_index = 0
        final_target = None
        locked_target_index = None
        started = time.monotonic()
        last_sample_at = time.monotonic()
        last_visit_at = time.monotonic()
        last_reorient_at = time.monotonic()
        drive_step_sec = DRIVE_SETTINGS.get("CONTROL_PERIOD_SEC", 0.05)
        max_sec = float(DRIVE_SETTINGS.get("COVERAGE_MAX_SEC", 120.0) or 0.0)
        print("[COVERAGE] waypoint coordinates are visit checkpoints, not steering targets", flush=True)

        while self.running and not self.stop_requested and self.visited_count < self.total_waypoints:
            if self._uwb_failsafe_triggered():
                break

            if max_sec > 0.0 and time.monotonic() - started >= max_sec:
                self.status = f"Coverage timeout {self.visited_count}/{self.total_waypoints}"
                print(f"[COVERAGE] timeout: visited={self.visited_count}/{self.total_waypoints}", flush=True)
                break

            visited_now = self._mark_current_waypoint_visits(waypoints, waypoint_states, scanner, fingerprint_db)
            if visited_now:
                last_visit_at = time.monotonic()
                final_target = waypoints[visited_now[-1]]
                if locked_target_index in visited_now:
                    locked_target_index = None
                last_sample_at = time.monotonic()
                continue

            locked_target_index = self._coverage_locked_target_index(
                waypoints,
                waypoint_states,
                locked_target_index,
            )

            if self._obstacle_detected():
                self._handle_obstacle_detected()
                if not self._target_reorient_blocked():
                    self._coverage_reorient(
                        waypoints=waypoints,
                        waypoint_states=waypoint_states,
                        target_index=locked_target_index,
                    )
                last_reorient_at = time.monotonic()
                continue

            if (
                time.monotonic() - last_sample_at >= DRIVE_SETTINGS.get("AUTO_STOP_GO_RUN_SEC", 3.0)
                and scanner
                and fingerprint_db is not None
            ):
                blocked, reason = self._rssi_sampling_blocked_reason()
                if blocked:
                    print(f"[RSSI] coverage sample skipped: {reason}", flush=True)
                else:
                    skip_visited, visited_reason = self._visited_sample_blocked_reason(waypoints, waypoint_states)
                    if skip_visited:
                        print(f"[RSSI] coverage sample skipped: {visited_reason}", flush=True)
                        last_sample_at = time.monotonic()
                        continue
                    self.status = "Coverage RSSI Sampling"
                    self._scan_current_point(scanner, fingerprint_db, settle=True)
                last_sample_at = time.monotonic()
                continue

            reorient_sec = float(DRIVE_SETTINGS.get("COVERAGE_REORIENT_SEC", 6.0) or 0.0)
            if (
                reorient_sec > 0.0
                and time.monotonic() - last_visit_at >= reorient_sec
                and time.monotonic() - last_reorient_at >= reorient_sec
                and not self._target_reorient_blocked()
            ):
                self._coverage_reorient(
                    waypoints=waypoints,
                    waypoint_states=waypoint_states,
                    target_index=locked_target_index,
                )
                last_reorient_at = time.monotonic()
                continue

            speed = DRIVE_SETTINGS["FORWARD_SPEED"]
            steering = DRIVE_SETTINGS["FORWARD_STEERING"]
            if locked_target_index is not None:
                target = waypoints[locked_target_index]
                self.current_target = target
                self.current_waypoint_index = locked_target_index + 1
                self.status = (
                    f"Coverage driving to {self.current_waypoint_index} "
                    f"{self.visited_count}/{self.total_waypoints}"
                )
                if not self._target_reorient_blocked():
                    current = self._waypoint_position()
                    if current[0] is not None and current[1] is not None:
                        max_steering = DRIVE_SETTINGS.get("AUTO_MAX_STEERING_DEG", 30)
                        steering = self._limit_auto_steering(
                            self._relaxed_target_steering(current, target, max_steering),
                            max_steering,
                        )
            else:
                self.current_target = None
                self.current_waypoint_index = 0
                self.status = f"Coverage driving {self.visited_count}/{self.total_waypoints}"
                self._last_auto_steering = DRIVE_SETTINGS["FORWARD_STEERING"]
            self.robot.forward(
                speed,
                left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
                right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
                steering=steering,
            )
            self._sleep_with_uwb_recording(drive_step_sec, "forward", speed)

        if self.robot:
            self.robot.stop()
        self.current_target = None
        return final_target, self.running and self.visited_count >= self.total_waypoints

    def _target_reorient_blocked(self):
        return time.monotonic() < getattr(self, "_target_reorient_block_until", 0.0)

    def _coverage_locked_target_index(self, waypoints, waypoint_states, locked_target_index=None):
        if (
            locked_target_index is not None
            and 0 <= locked_target_index < len(waypoints)
            and not waypoint_states[locked_target_index].get("visited")
        ):
            return locked_target_index

        nearest = self._nearest_unvisited_checkpoint(waypoints, waypoint_states)
        if nearest is None:
            return None

        index, target, distance, _ = nearest
        print(
            f"[COVERAGE] locked target {index + 1}: "
            f"({target[0]:.2f}, {target[1]:.2f}), distance={distance:.2f}m",
            flush=True,
        )
        return index

    def _mark_current_waypoint_visits(self, waypoints, waypoint_states, scanner, fingerprint_db):
        if not self._current_position_valid_for_arrival():
            return []

        x, y = self._waypoint_position()
        if x is None or y is None:
            return []

        recent_positions = self._recent_waypoint_positions(
            int(DRIVE_SETTINGS.get("WAYPOINT_RECENT_VISIT_WINDOW", 6) or 0)
        )
        visited_indices = []
        for index, waypoint in enumerate(waypoints):
            if waypoint_states[index].get("visited"):
                continue
            tolerance = self._waypoint_visit_tolerance(waypoint)
            distance = math.hypot(waypoint[0] - x, waypoint[1] - y)
            recent_hits = sum(
                1
                for px, py in recent_positions
                if math.hypot(waypoint[0] - px, waypoint[1] - py) <= tolerance
            )
            min_recent_hits = max(1, int(DRIVE_SETTINGS.get("WAYPOINT_RECENT_VISIT_MIN_HITS", 2)))
            arrived = distance <= tolerance or recent_hits >= min_recent_hits
            if arrived:
                waypoint_states[index]["visited"] = True
                waypoint_states[index]["state"] = "visited"
                self.visited_count = sum(1 for item in waypoint_states if item["visited"])
                self.last_waypoint_arrived = True
                self.distance_to_target = round(distance, 3)
                self.current_target = None
                self.current_waypoint_index = index + 1
                self.status = f"Coverage visited {self.visited_count}/{self.total_waypoints}"
                print(
                    f"[COVERAGE] visited waypoint {index + 1}: "
                    f"({waypoint[0]:.2f}, {waypoint[1]:.2f}), "
                    f"distance={distance:.2f}m, tolerance={tolerance:.2f}m, "
                    f"recent_hits={recent_hits}/{len(recent_positions)}",
                    flush=True,
                )
                visited_indices.append(index)

        if visited_indices:
            if self.robot:
                self.robot.stop()
            self._scan_current_point(scanner, fingerprint_db, settle=True)
        return visited_indices

    def _recent_waypoint_positions(self, window):
        if window <= 0:
            return []
        positions = []
        with self.lock:
            for point in self.points[-window:]:
                px = point.get("x")
                py = point.get("y")
                if px is None or py is None:
                    continue
                try:
                    positions.append((float(px), float(py)))
                except (TypeError, ValueError):
                    continue
        return positions

    def _waypoint_visit_tolerance(self, waypoint):
        base = float(DRIVE_SETTINGS.get("TARGET_REACHED_RADIUS_M", 0.5))
        edge_tolerance = float(DRIVE_SETTINGS.get("EDGE_TARGET_REACHED_RADIUS_M", base))
        edge_margin = float(DRIVE_SETTINGS.get("EDGE_TARGET_MARGIN_M", 0.6))
        width = float(APP_SETTINGS.get("ROOM_WIDTH_M", 3.0))
        height = float(APP_SETTINGS.get("ROOM_HEIGHT_M", 3.0))
        x, y = waypoint
        is_edge = (
            x <= edge_margin
            or x >= width - edge_margin
            or y <= edge_margin
            or y >= height - edge_margin
        )
        return edge_tolerance if is_edge else base

    def _visited_sample_blocked_reason(self, waypoints, waypoint_states):
        if not DRIVE_SETTINGS.get("VISITED_SAMPLE_SKIP_ENABLE", True):
            return False, ""
        if not self._current_position_valid_for_arrival():
            return False, ""

        x, y = self._waypoint_position()
        if x is None or y is None:
            return False, ""

        radius = float(DRIVE_SETTINGS.get("VISITED_SAMPLE_SKIP_RADIUS_M", 0.45) or 0.0)
        if radius <= 0.0:
            return False, ""

        for index, waypoint in enumerate(waypoints):
            if not waypoint_states[index].get("visited"):
                continue
            distance = math.hypot(waypoint[0] - x, waypoint[1] - y)
            if distance <= radius:
                return True, (
                    f"near visited waypoint {index + 1} "
                    f"({distance:.2f}m <= {radius:.2f}m)"
                )
        return False, ""

    def _nearest_unvisited_checkpoint(self, waypoints, waypoint_states):
        if not self._current_position_valid_for_motion():
            return None
        x, y = self._waypoint_position()
        if x is None or y is None:
            return None

        candidates = []
        for index, waypoint in enumerate(waypoints):
            if waypoint_states[index].get("visited"):
                continue
            distance = math.hypot(waypoint[0] - x, waypoint[1] - y)
            candidates.append((distance, index, waypoint))
        if not candidates:
            return None
        distance, index, waypoint = min(candidates, key=lambda item: (item[0], item[1]))
        return index, waypoint, distance, (x, y)

    def _coverage_reorient(self, waypoints=None, waypoint_states=None, target_index=None):
        if not self.robot:
            return
        direction = "right"
        target = None
        distance = None
        heading_error = None
        if (
            DRIVE_SETTINGS.get("COVERAGE_REORIENT_TO_UNVISITED_ENABLE", True)
            and waypoints is not None
            and waypoint_states is not None
        ):
            nearest = None
            if (
                target_index is not None
                and 0 <= target_index < len(waypoints)
                and not waypoint_states[target_index].get("visited")
                and self._current_position_valid_for_motion()
            ):
                current = self._waypoint_position()
                if current[0] is not None and current[1] is not None:
                    target = waypoints[target_index]
                    nearest = (
                        target_index,
                        target,
                        math.hypot(target[0] - current[0], target[1] - current[1]),
                        current,
                    )
            if nearest is None:
                nearest = self._nearest_unvisited_checkpoint(waypoints, waypoint_states)
            heading = self._get_heading()
            if nearest is not None and heading is not None:
                index, target, distance, current = nearest
                bearing = self._bearing_to_target(current, target)
                heading_error = self._angle_diff(bearing, heading) if bearing is not None else None
                self.current_waypoint_index = index + 1
                self.target_bearing = round(bearing, 1) if bearing is not None else None
                self.current_heading = round(heading, 1)
                self.heading_error = round(heading_error, 1) if heading_error is not None else None
                tolerance = float(DRIVE_SETTINGS.get("COVERAGE_REORIENT_HEADING_TOLERANCE_DEG", 25.0))
                if heading_error is not None and abs(heading_error) > tolerance:
                    direction = "right" if heading_error > 0 else "left"
                elif target is not None:
                    print(
                        f"[COVERAGE] already roughly facing unvisited waypoint {index + 1}: "
                        f"error={heading_error:.1f}deg",
                        flush=True,
                    )
                    return

        if self.lidar and hasattr(self.lidar, "avoidance_direction"):
            try:
                if target is None:
                    decision = self.lidar.avoidance_direction()
                    if decision.get("direction") in ("left", "right"):
                        direction = decision["direction"]
            except Exception:
                pass

        speed = DRIVE_SETTINGS.get("TURN_SPEED", 12)
        seconds = float(
            DRIVE_SETTINGS.get(
                "COVERAGE_REORIENT_TO_UNVISITED_TURN_SEC" if target is not None else "COVERAGE_REORIENT_TURN_SEC",
                DRIVE_SETTINGS.get("TURN_TIME_SEC", 0.65),
            )
        )
        if target is not None:
            self.status = f"Coverage bias {direction} to unvisited ({target[0]:.2f}, {target[1]:.2f})"
            heading_error_text = f"{heading_error:.1f}deg" if heading_error is not None else "-"
            print(
                f"[COVERAGE] reorient toward unvisited waypoint {self.current_waypoint_index}: "
                f"target=({target[0]:.2f}, {target[1]:.2f}), distance={distance:.2f}m, "
                f"heading_error={heading_error_text}, turn={direction}",
                flush=True,
            )
        else:
            self.status = f"Coverage reorient {direction}"
        self._start_turn_tracking()
        if direction == "left":
            self.robot.turn_left(speed)
        else:
            self.robot.turn_right(speed)
        self._sleep_with_uwb_recording(seconds, "turning", speed)
        self.robot.stop()
        self._finish_turn_tracking()
        self._sleep_with_uwb_recording(DRIVE_SETTINGS.get("AUTO_TURN_PAUSE_SEC", 0.03), "idle", 0)

    def _nearest_arrived_unvisited_waypoint_index(self, waypoints, waypoint_states):
        if not self._current_position_valid_for_arrival():
            return None

        x, y = self._waypoint_position()
        if x is None or y is None:
            return None

        tolerance = float(DRIVE_SETTINGS.get("TARGET_REACHED_RADIUS_M", 0.5))
        candidates = []
        for index, waypoint in enumerate(waypoints):
            if waypoint_states[index].get("visited"):
                continue
            distance = math.hypot(waypoint[0] - x, waypoint[1] - y)
            if distance <= tolerance:
                candidates.append((distance, index))
        if not candidates:
            return None
        return min(candidates, key=lambda item: item[0])[1]

    @staticmethod
    def _next_inward_waypoint_index(waypoints, waypoint_states, start_index):
        sx, sy = waypoints[start_index]
        candidates = []
        for index, waypoint in enumerate(waypoints):
            if waypoint_states[index].get("visited"):
                continue
            wx, wy = waypoint
            if wy <= sy:
                continue
            x_error = abs(wx - sx)
            y_step = wy - sy
            candidates.append((x_error, y_step, index))
        if not candidates:
            return None
        return min(candidates, key=lambda item: (round(item[0], 2), round(item[1], 2), item[2]))[2]

    @staticmethod
    def _first_unvisited_waypoint_index(visited):
        for index, item in enumerate(visited):
            if not item:
                return index
        return None

    def _build_auto_waypoints(self, width, height):
        mode = self._auto_path_mode()
        if mode == "diamond":
            return self._build_diamond_waypoints(width, height)
        if mode == "nearest":
            return self._apply_target_offset(self._build_nearest_waypoints(width, height), width, height)
        if mode == "spiral":
            return self._apply_target_offset(self._build_spiral_waypoints(width, height), width, height)
        return self._apply_target_offset(self._build_zigzag_waypoints(width, height), width, height)

    def _build_diamond_waypoints(self, width, height):
        key = "DIAMOND_WAYPOINTS_QUICK" if DRIVE_SETTINGS.get("DIAMOND_QUICK_MODE", True) else "DIAMOND_WAYPOINTS"
        raw_points = DRIVE_SETTINGS.get(key) or DRIVE_SETTINGS.get("DIAMOND_WAYPOINTS") or []
        waypoints = []
        for point in raw_points:
            if len(point) != 2:
                continue
            x = min(max(0.0, float(point[0])), float(width))
            y = min(max(0.0, float(point[1])), float(height))
            rounded = (round(x, 3), round(y, 3))
            if not waypoints or waypoints[-1] != rounded:
                waypoints.append(rounded)

        if waypoints:
            return self._apply_target_offset(waypoints, width, height)

        center_x = float(width) / 2.0
        center_y = float(height) / 2.0
        margin = max(0.25, float(DRIVE_SETTINGS.get("AUTO_EDGE_MARGIN_M", 0.0)))
        fallback = [
            (center_x, margin),
            (margin, center_y),
            (center_x, max(margin, float(height) - margin)),
            (max(margin, float(width) - margin), center_y),
            (center_x, margin),
        ]
        return self._apply_target_offset(
            [(round(x, 3), round(y, 3)) for x, y in fallback],
            width,
            height,
        )

    @staticmethod
    def _apply_target_offset(waypoints, width, height):
        offset_x = float(DRIVE_SETTINGS.get("AUTO_TARGET_X_OFFSET_M", 0.0) or 0.0)
        offset_y = float(DRIVE_SETTINGS.get("AUTO_TARGET_Y_OFFSET_M", 0.0) or 0.0)
        if abs(offset_x) < 1e-9 and abs(offset_y) < 1e-9:
            return waypoints

        shifted = []
        for x, y in waypoints:
            shifted_x = min(max(0.0, x + offset_x), float(width))
            shifted_y = min(max(0.0, y + offset_y), float(height))
            shifted.append((round(shifted_x, 3), round(shifted_y, 3)))
        return shifted

    def _build_nearest_waypoints(self, width, height):
        configured = (
            DRIVE_SETTINGS.get("UNVISITED_WAYPOINTS")
            if DRIVE_SETTINGS.get("UNVISITED_WAYPOINTS_ENABLE", False)
            else None
        )
        if configured:
            waypoints = []
            for point in configured:
                if len(point) != 2:
                    continue
                x = min(max(0.0, float(point[0])), float(width))
                y = min(max(0.0, float(point[1])), float(height))
                rounded = (round(x, 3), round(y, 3))
                if rounded not in waypoints:
                    waypoints.append(rounded)
            if waypoints:
                return waypoints

        spacing = max(0.1, DRIVE_SETTINGS.get("AUTO_GRID_SPACING_M", 0.5))
        margin = max(0.0, DRIVE_SETTINGS.get("AUTO_EDGE_MARGIN_M", 0.5))
        min_x = min(margin, max(0.0, width / 2.0))
        min_y = min(margin, max(0.0, height / 2.0))
        max_x = max(min_x, width - margin)
        max_y = max(min_y, height - margin)

        xs = self._axis_points_between(min_x, max_x, spacing)
        ys = self._axis_points_between(min_y, max_y, spacing)
        return [(x, y) for y in ys for x in xs]

    def _nearest_unvisited_waypoint_index(self, waypoints, visited):
        if not self._current_position_valid_for_motion():
            return None

        x, y = self._waypoint_position()
        if x is None or y is None:
            return None

        heading = self._get_heading()
        candidates = []
        for index, waypoint in enumerate(waypoints):
            if visited[index]:
                continue
            distance = math.hypot(waypoint[0] - x, waypoint[1] - y)
            bearing = self._bearing_to_target((x, y), waypoint)
            heading_error = self._angle_diff(bearing, heading) if bearing is not None and heading is not None else None
            # Tie-break equal-distance targets toward the room interior first.
            # From start #2 (1.5, 0.5), this chooses #5 before #1/#3.
            inward_priority = 0 if waypoint[1] > y + 0.05 else 1
            lateral_error = abs(waypoint[0] - x)
            candidates.append((distance, inward_priority, lateral_error, index, bearing, heading_error))
        if not candidates:
            return None

        distance, _inward_priority, _lateral_error, nearest_index, bearing, heading_error = min(
            candidates,
            key=lambda item: (round(item[0], 2), item[1], item[2], item[3]),
        )
        self.distance_to_target = round(distance, 3)
        self.target_bearing = round(bearing, 1) if bearing is not None else None
        self.current_heading = round(heading, 1) if heading is not None else None
        self.heading_error = round(heading_error, 1) if heading_error is not None else None
        return nearest_index

    def _nearest_arrived(self, target):
        if not self._current_position_valid_for_arrival():
            return False

        x, y = self.get_filtered_position()
        if x is None or y is None:
            return False

        distance = math.hypot(target[0] - x, target[1] - y)
        self.distance_to_target = round(distance, 3)
        tolerance = max(
            DRIVE_SETTINGS.get("TARGET_REACHED_RADIUS_M", DRIVE_SETTINGS.get("AUTO_WAYPOINT_TOLERANCE_M", 0.12)),
            DRIVE_SETTINGS.get("AUTO_NEAREST_VISIT_TOLERANCE_M", 0.30),
        )
        return distance <= tolerance

    @staticmethod
    def _bearing_to_target(current, target):
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return None
        return (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0

    def _target_heading_state(self, target):
        x, y = self.get_filtered_position()
        heading = self._get_heading()
        if x is None or y is None or heading is None:
            self.current_heading = round(heading, 1) if heading is not None else None
            self.target_bearing = None
            self.heading_error = None
            return None, heading, None

        bearing = self._bearing_to_target((x, y), target)
        error = self._angle_diff(bearing, heading) if bearing is not None else None
        self.target_bearing = round(bearing, 1) if bearing is not None else None
        self.current_heading = round(heading, 1)
        self.heading_error = round(error, 1) if error is not None else None
        return bearing, heading, error

    def _align_heading_to_target(self, target):
        if not DRIVE_SETTINGS.get("AUTO_HEADING_ALIGN_ENABLE", True):
            return True
        if not self.robot:
            return False

        tolerance = DRIVE_SETTINGS.get("AUTO_HEADING_ALIGN_TOLERANCE_DEG", 20)
        deadline = time.monotonic() + DRIVE_SETTINGS.get("AUTO_HEADING_ALIGN_MAX_SEC", 2.5)
        speed = DRIVE_SETTINGS.get("AUTO_HEADING_ALIGN_TURN_SPEED", 10)
        step_sec = DRIVE_SETTINGS.get("AUTO_HEADING_ALIGN_STEP_SEC", 0.08)

        while self.running and not self.stop_requested and time.monotonic() < deadline:
            if self._uwb_failsafe_triggered():
                break
            if not self._current_position_valid_for_motion():
                self.robot.stop()
                self.status = f"UWB settling before heading align ({target[0]:.2f}, {target[1]:.2f})"
                self._sleep_with_uwb_recording(
                    DRIVE_SETTINGS.get("AUTO_STOP_GO_SETTLE_SEC", 0.45),
                    "idle",
                    0,
                )
                continue

            _, heading, error = self._target_heading_state(target)
            if heading is None:
                print(
                    "[NEAREST] heading unavailable during align; stopping before waypoint drive.",
                    flush=True,
                )
                self.status = "Nearest blocked: no heading"
                self.robot.stop()
                return False
            if error is None or abs(error) <= tolerance:
                self.robot.stop()
                return True

            self.status = f"Heading align {error:.1f}deg to ({target[0]:.2f}, {target[1]:.2f})"
            if error > 0:
                self.robot.turn_right(speed)
            else:
                self.robot.turn_left(speed)
            self._sleep_with_uwb_recording(step_sec, "turning", speed)

        self.robot.stop()
        return False

    def _pivot_nudge_forward(self, target):
        if not DRIVE_SETTINGS.get("AUTO_PIVOT_NUDGE_ENABLE", True):
            return
        if not self.robot:
            return

        speed = DRIVE_SETTINGS.get("AUTO_PIVOT_NUDGE_SPEED", 8)
        seconds = DRIVE_SETTINGS.get("AUTO_PIVOT_NUDGE_SEC", 0.18)
        if seconds <= 0:
            return

        self.status = f"Pivot nudge to ({target[0]:.2f}, {target[1]:.2f})"
        self.robot.forward(
            speed,
            left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
            right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
            steering=DRIVE_SETTINGS["FORWARD_STEERING"],
        )
        self._sleep_with_uwb_recording(seconds, "forward", speed)
        self.robot.stop()

    def _build_zigzag_waypoints(self, width, height):
        if DRIVE_SETTINGS.get("AUTO_ZIGZAG_CORNER_TEST_PATH_ENABLE", False):
            waypoints = []
            for point in DRIVE_SETTINGS.get("AUTO_ZIGZAG_CORNER_TEST_WAYPOINTS", []):
                if len(point) != 2:
                    continue
                x = min(max(0.0, float(point[0])), float(width))
                y = min(max(0.0, float(point[1])), float(height))
                waypoints.append((round(x, 3), round(y, 3)))
            if waypoints:
                waypoints = self._shift_waypoints_to_current_start_x(waypoints, width)
                print(f"[PATH] using map corner test waypoints: {waypoints}", flush=True)
                return waypoints

        spacing = max(0.1, DRIVE_SETTINGS.get("AUTO_GRID_SPACING_M", 0.5))
        margin = max(0.0, DRIVE_SETTINGS.get("AUTO_EDGE_MARGIN_M", 0.0))
        margin_x = max(0.0, DRIVE_SETTINGS.get("AUTO_EDGE_MARGIN_X_M", margin))
        margin_y = max(0.0, DRIVE_SETTINGS.get("AUTO_EDGE_MARGIN_Y_M", margin))
        turn_early_margin = max(0.0, DRIVE_SETTINGS.get("AUTO_TURN_EARLY_MARGIN_M", 0.0))
        turn_start = DRIVE_SETTINGS.get("AUTO_TURN_START_M")
        min_x = min(margin_x, max(0.0, width / 2.0))
        min_y = min(margin_y, max(0.0, height / 2.0))
        max_x = max(min_x, width - margin_x)
        max_y = max(min_y, height - margin_y)
        if max_y - min_y > 2.0 * turn_early_margin:
            min_y += turn_early_margin
            max_y -= turn_early_margin
        turn_y = DRIVE_SETTINGS.get("AUTO_TURN_Y_M")
        if turn_y is not None:
            max_y = max(min_y, min(max_y, float(turn_y)))
        if turn_start:
            max_x = min(max_x, max(min_x, turn_start))
            max_y = min(max_y, max(min_y, turn_start))

        xs = self._axis_points_between(min_x, max_x, spacing)
        ys = self._axis_points_between(min_y, max_y, spacing)
        waypoints = []

        for col, x in enumerate(xs):
            col_ys = ys if col % 2 == 0 else list(reversed(ys))
            if DRIVE_SETTINGS.get("AUTO_ZIGZAG_VERTICAL_ENDPOINTS_ONLY", False) and len(col_ys) > 1:
                col_ys = [col_ys[0], col_ys[-1]]
            if col > 0 and waypoints and col_ys and abs(waypoints[-1][1] - col_ys[0]) <= 1e-6:
                col_ys = col_ys[1:]

            for y in col_ys:
                waypoints.append((x, y))

            if col + 1 < len(xs) and col_ys:
                waypoints.append((xs[col + 1], col_ys[-1]))

        return self._shift_waypoints_to_current_start_x(waypoints, width) or [(round(min_x, 3), round(min_y, 3))]

    def _shift_waypoints_to_current_start_x(self, waypoints, width):
        if not waypoints or not DRIVE_SETTINGS.get("AUTO_ZIGZAG_SHIFT_TO_START_X_ENABLE", False):
            return waypoints

        current = self._current_position_or_none()
        if current is None:
            return waypoints

        current_x, _ = current
        first_x = waypoints[0][0]
        delta = float(current_x) - float(first_x)
        max_shift = max(0.0, DRIVE_SETTINGS.get("AUTO_ZIGZAG_START_X_SHIFT_MAX_M", 0.0))
        if abs(delta) < 1e-3 or abs(delta) > max_shift:
            return waypoints

        width = float(width)
        shifted = [
            (round(min(max(0.0, x + delta), width), 3), y)
            for x, y in waypoints
        ]
        print(
            f"[PATH] shifted zigzag x by {delta:.2f}m to match start x={current_x:.2f}",
            flush=True,
        )
        return shifted

    def _build_spiral_waypoints(self, width, height):
        spacing = max(0.1, DRIVE_SETTINGS.get("AUTO_GRID_SPACING_M", 0.5))
        margin = max(0.0, DRIVE_SETTINGS.get("AUTO_EDGE_MARGIN_M", 0.0))
        margin_x = max(0.0, DRIVE_SETTINGS.get("AUTO_EDGE_MARGIN_X_M", margin))
        margin_y = max(0.0, DRIVE_SETTINGS.get("AUTO_EDGE_MARGIN_Y_M", margin))
        turn_start = DRIVE_SETTINGS.get("AUTO_TURN_START_M")
        min_x = min(margin_x, max(0.0, width / 2.0))
        min_y = min(margin_y, max(0.0, height / 2.0))
        max_x = max(min_x, width - margin_x)
        max_y = max(min_y, height - margin_y)
        if turn_start:
            max_x = min(max_x, max(min_x, turn_start))
            max_y = min(max_y, max(min_y, turn_start))

        waypoints = []
        left = min_x
        right = max_x
        bottom = min_y
        top = max_y

        def add(point):
            rounded = (round(point[0], 3), round(point[1], 3))
            if not waypoints or waypoints[-1] != rounded:
                waypoints.append(rounded)

        while left <= right + 1e-6 and bottom <= top + 1e-6:
            add((left, bottom))
            add((left, top))
            if left >= right - 1e-6:
                break

            add((right, top))
            if bottom >= top - 1e-6:
                break

            add((right, bottom))
            left += spacing
            right -= spacing
            bottom += spacing
            top -= spacing

            if left <= right + 1e-6 and bottom <= top + 1e-6:
                add((left, bottom))

        return waypoints or [(round(min_x, 3), round(min_y, 3))]

    @staticmethod
    def _axis_points_between(start, limit, spacing):
        if limit <= start:
            return [round(start, 3)]

        count = max(0, int(math.floor((limit - start) / spacing)))
        points = [round(start + i * spacing, 3) for i in range(count + 1)]
        if not points or abs(points[-1] - limit) > 1e-6:
            points.append(round(limit, 3))
        return points

    def _inner_bottom_right_target(self, width, height):
        margin = max(0.0, DRIVE_SETTINGS.get("AUTO_EDGE_MARGIN_M", 0.0))
        margin_x = max(0.0, DRIVE_SETTINGS.get("AUTO_EDGE_MARGIN_X_M", margin))
        margin_y = max(0.0, DRIVE_SETTINGS.get("AUTO_EDGE_MARGIN_Y_M", margin))
        x = max(0.0, width - margin_x)
        y = min(max(0.0, margin_y), max(0.0, height))
        return (round(x, 3), round(y, 3))

    def _at_waypoint(self, target):
        x, y = self.get_filtered_position()
        if x is None or y is None:
            return False
        tolerance = DRIVE_SETTINGS.get("AUTO_WAYPOINT_TOLERANCE_M", 0.12)
        return math.hypot(target[0] - x, target[1] - y) <= tolerance

    def _vertical_segment_y_reached(self, start, target, current):
        if start is None:
            return False
        sx, sy = start
        tx, ty = target
        x, y = current
        if abs(tx - sx) > 1e-6 or abs(ty - sy) < 1e-6:
            return False

        y_for_arrival = y
        turn_y = DRIVE_SETTINGS.get("AUTO_TURN_Y_M")
        if (
            DRIVE_SETTINGS.get("AUTO_TURN_Y_USE_RAW", False)
            and turn_y is not None
            and abs(ty - float(turn_y)) <= 1e-6
            and self.uwb
        ):
            raw_y = self.uwb.snapshot().get("raw_y")
            if raw_y is not None:
                y_for_arrival = float(raw_y)

        tolerance = DRIVE_SETTINGS.get("AUTO_VERTICAL_Y_TOLERANCE_M", 0.12)
        if ty > sy:
            y_reached = y_for_arrival >= ty - tolerance
        else:
            y_reached = y_for_arrival <= ty + tolerance

        if not y_reached:
            return False

        if DRIVE_SETTINGS.get("AUTO_VERTICAL_IGNORE_X_FOR_ARRIVAL", False):
            return True

        x_tolerance = DRIVE_SETTINGS.get("AUTO_VERTICAL_X_TOLERANCE_M", tolerance)
        return abs(x - tx) <= x_tolerance

    def _axis_stop_at_target_reached(self, start, target, current):
        if not DRIVE_SETTINGS.get("AUTO_AXIS_STOP_AT_TARGET_ENABLE", False):
            return False
        if start is None:
            return False

        sx, sy = start
        tx, ty = target
        x, y = current
        dx = tx - sx
        dy = ty - sy
        if abs(dx) >= abs(dy):
            x_tolerance = DRIVE_SETTINGS.get("AUTO_HORIZONTAL_X_TOLERANCE_M", 0.05)
            y_tolerance = DRIVE_SETTINGS.get("AUTO_HORIZONTAL_Y_TOLERANCE_M", 0.20)
            if dx >= 0:
                x_reached = x >= tx - x_tolerance
            else:
                x_reached = x <= tx + x_tolerance
            reached = x_reached and abs(y - ty) <= y_tolerance
            if x_reached:
                print(
                    f"[AXIS_STOP] horizontal x reached: current=({x:.2f},{y:.2f}), "
                    f"target=({tx:.2f},{ty:.2f}), y_ok={abs(y - ty) <= y_tolerance}",
                    flush=True,
                )
            return reached

        y_tolerance = DRIVE_SETTINGS.get("AUTO_VERTICAL_Y_TOLERANCE_M", 0.12)
        x_tolerance = DRIVE_SETTINGS.get("AUTO_VERTICAL_X_TOLERANCE_M", 0.10)
        if dy >= 0:
            y_reached = y >= ty - y_tolerance
        else:
            y_reached = y <= ty + y_tolerance
        if DRIVE_SETTINGS.get("AUTO_VERTICAL_IGNORE_X_FOR_ARRIVAL", False):
            reached = y_reached
        else:
            reached = y_reached and abs(x - tx) <= x_tolerance
        if y_reached:
            print(
                f"[AXIS_STOP] vertical y reached: current=({x:.2f},{y:.2f}), "
                f"target=({tx:.2f},{ty:.2f}), x_ok={abs(x - tx) <= x_tolerance}",
                flush=True,
            )
        return reached

    @staticmethod
    def _segment_is_vertical(start, target):
        if start is None or target is None:
            return False
        return abs(target[0] - start[0]) <= 1e-6 and abs(target[1] - start[1]) > 1e-6

    def _segment_direction(self, target):
        x, y = self.get_filtered_position()
        if x is None or y is None:
            return None

        dx = target[0] - x
        dy = target[1] - y
        if math.hypot(dx, dy) < DRIVE_SETTINGS.get("AUTO_WAYPOINT_TOLERANCE_M", 0.12):
            return None
        if abs(dx) >= abs(dy):
            return "east" if dx >= 0 else "west"
        return "north" if dy >= 0 else "south"

    @staticmethod
    def _planned_segment_direction(start, target):
        dx = target[0] - start[0]
        dy = target[1] - start[1]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return None
        if abs(dx) >= abs(dy):
            return "east" if dx >= 0 else "west"
        return "north" if dy >= 0 else "south"

    def _grid_line_recovery_target(self, start, target, current):
        if not DRIVE_SETTINGS.get("AUTO_GRID_LINE_RECOVERY_ENABLE", True):
            return None
        if self.path_mode != "zigzag" or start is None:
            return None

        sx, sy = start
        tx, ty = target
        x, y = current
        trigger = DRIVE_SETTINGS.get("AUTO_GRID_LINE_RECOVERY_TRIGGER_M", 0.10)
        max_error = DRIVE_SETTINGS.get("AUTO_GRID_LINE_RECOVERY_MAX_ERROR_M", 0.15)

        if abs(tx - sx) >= abs(ty - sy):
            line_error = ty - y
            if trigger <= abs(line_error) <= max_error:
                return (round(x, 3), round(ty, 3))
        else:
            line_error = tx - x
            if trigger <= abs(line_error) <= max_error:
                return (round(tx, 3), round(y, 3))

        return None

    def _segment_follow_target(self, start, target, current):
        if not DRIVE_SETTINGS.get("AUTO_SEGMENT_FOLLOW_ENABLE", False):
            return None
        if start is None:
            return None

        sx, sy = start
        tx, ty = target
        x, y = current
        vx = tx - sx
        vy = ty - sy
        length = math.hypot(vx, vy)
        if length < 1e-6:
            return None

        ux = vx / length
        uy = vy / length
        progress_m = (x - sx) * ux + (y - sy) * uy
        lookahead_m = max(0.0, DRIVE_SETTINGS.get("AUTO_SEGMENT_LOOKAHEAD_M", 0.30))
        follow_progress = max(0.0, min(length, progress_m + lookahead_m))
        return (
            round(sx + ux * follow_progress, 3),
            round(sy + uy * follow_progress, 3),
        )

    def _axis_locked_start(self, target, current):
        if not DRIVE_SETTINGS.get("AUTO_AXIS_START_LOCK_ENABLE", False):
            return current
        if self.path_mode != "zigzag" or target is None or current is None:
            return current

        x, y = current
        dx = abs(target[0] - x)
        dy = abs(target[1] - y)
        if dy >= dx:
            return (target[0], y)
        return (x, target[1])

    def _target_reacquire_reason(self, start, target, current):
        if not DRIVE_SETTINGS.get("AUTO_TARGET_REACQUIRE_ENABLE", False):
            return None
        if start is None or target is None or current is None:
            return None

        sx, sy = start
        tx, ty = target
        x, y = current
        vx = tx - sx
        vy = ty - sy
        length = math.hypot(vx, vy)
        if length < 1e-6:
            return None

        progress_m = ((x - sx) * vx + (y - sy) * vy) / length
        cross_track_m = abs((vx * (y - sy) - vy * (x - sx)) / length)
        deviation_m = DRIVE_SETTINGS.get("AUTO_TARGET_REACQUIRE_DEVIATION_M", 0.55)
        overshoot_m = DRIVE_SETTINGS.get("AUTO_TARGET_REACQUIRE_OVERSHOOT_M", 0.30)

        if cross_track_m >= deviation_m:
            return f"cross_track {cross_track_m:.2f}m"
        if progress_m > length + overshoot_m:
            return f"overshoot {progress_m - length:.2f}m"
        if progress_m < -overshoot_m:
            return f"behind_start {-progress_m:.2f}m"
        return None

    def _drive_to_waypoint(
        self,
        target,
        planned_start=None,
        progress_arrival=False,
        progress_tolerance=None,
        scanner=None,
        fingerprint_db=None,
    ):
        if not self.robot:
            return False
        self._force_rssi_dongle_off("waypoint_drive")

        tolerance = DRIVE_SETTINGS.get("AUTO_WAYPOINT_TOLERANCE_M", 0.12)
        confirm_count = max(1, DRIVE_SETTINGS.get("AUTO_WAYPOINT_CONFIRM_COUNT", 5))
        timeout = DRIVE_SETTINGS.get("AUTO_WAYPOINT_TIMEOUT_SEC", 8.0)
        if self.path_mode == "nearest":
            tolerance = DRIVE_SETTINGS.get("TARGET_REACHED_RADIUS_M", tolerance)
            confirm_count = max(1, DRIVE_SETTINGS.get("TARGET_REACHED_CONFIRM_COUNT", 1))
            timeout = DRIVE_SETTINGS.get("TARGET_TIMEOUT_SEC", timeout)
        steering_gain = DRIVE_SETTINGS.get("AUTO_STEERING_GAIN", 22.0)
        max_steering = DRIVE_SETTINGS.get(
            "AUTO_LINE_MAX_STEERING_DEG",
            DRIVE_SETTINGS.get("AUTO_MAX_STEERING_DEG", 25),
        )
        deadline = None if timeout is None or float(timeout) <= 0.0 else time.monotonic() + float(timeout)
        has_planned_start = planned_start is not None
        axis_start_locked = False
        segment_start = planned_start or self._current_position_or_none()
        arrival_segment_start = segment_start
        segment_started = time.monotonic()
        drive_elapsed_sec = 0.0
        drive_since_stop_sec = 0.0
        last_stop_go_sample_at = time.monotonic()
        best_distance = None
        last_progress_at = time.monotonic()
        drive_step_sec = 0.05
        planned_start = segment_start
        self._last_segment_drive_start = self._current_position_or_none() or segment_start
        self._last_segment_drive_end = None
        timed_arrival_sec = None
        if DRIVE_SETTINGS.get("AUTO_TIMED_WAYPOINT_ENABLE", False):
            if planned_start is not None:
                planned_distance = math.hypot(target[0] - planned_start[0], target[1] - planned_start[1])
            else:
                planned_distance = DRIVE_SETTINGS.get("AUTO_GRID_SPACING_M", 1.0)
            timed_arrival_sec = max(
                DRIVE_SETTINGS.get("AUTO_TIMED_WAYPOINT_MIN_SEC", 1.2),
                planned_distance * DRIVE_SETTINGS.get("AUTO_TIMED_WAYPOINT_SEC_PER_M", 2.2),
            )
        self._last_auto_steering = DRIVE_SETTINGS["FORWARD_STEERING"]

        self.status = f"Moving to ({target[0]:.2f}, {target[1]:.2f})"
        self.waypoint_arrive_count = 0
        self.last_waypoint_arrived = False

        while self.running and not self.stop_requested and (deadline is None or time.monotonic() < deadline):
            if self._uwb_failsafe_triggered():
                break

            stop_go_due = (
                DRIVE_SETTINGS.get("AUTO_STOP_GO_ENABLE", False)
                and time.monotonic() - last_stop_go_sample_at >= DRIVE_SETTINGS.get("AUTO_STOP_GO_RUN_SEC", 0.9)
            )
            if stop_go_due:
                blocked, reason = self._rssi_sampling_blocked_reason()
                if blocked:
                    print(f"[RSSI] timed sample skipped: {reason}", flush=True)
                    last_stop_go_sample_at = time.monotonic()
                    drive_since_stop_sec = 0.0
                    continue
                if (
                    DRIVE_SETTINGS.get("AUTO_STOP_GO_RSSI_SAMPLE_ENABLE", True)
                    and scanner
                    and fingerprint_db is not None
                ):
                    x_now, y_now = self.get_filtered_position()
                    self.status = f"Timed RSSI sample near ({x_now}, {y_now})"
                    print(
                        f"[RSSI] timed sample trigger by waypoint timer "
                        f"elapsed={time.monotonic() - last_stop_go_sample_at:.2f}s "
                        f"near ({x_now}, {y_now}); dwell={DRIVE_SETTINGS.get('AUTO_SCAN_DWELL_SEC', 2.0)}s",
                        flush=True,
                    )
                    self._scan_current_point(scanner, fingerprint_db, settle=True)
                    if self.stop_requested:
                        break
                    self.status = f"Moving to ({target[0]:.2f}, {target[1]:.2f})"
                else:
                    self.robot.stop()
                    self.status = f"Timed UWB settle near ({target[0]:.2f}, {target[1]:.2f})"
                    self._sleep_with_uwb_recording(
                        DRIVE_SETTINGS.get("AUTO_STOP_GO_SETTLE_SEC", 0.45),
                        "idle",
                        0,
                    )
                last_stop_go_sample_at = time.monotonic()
                drive_since_stop_sec = 0.0

            if self._obstacle_detected():
                action = self._handle_obstacle_detected()
                if action == "lidar_stop":
                    self._sleep_with_uwb_recording(
                        DRIVE_SETTINGS.get("CONTROL_PERIOD_SEC", 0.05),
                        "idle",
                        0,
                    )
                    drive_since_stop_sec = 0.0
                else:
                    segment_start = planned_start or self._current_position_or_none()
                    segment_started = time.monotonic()
                    drive_since_stop_sec = 0.0
                    self._last_auto_steering = DRIVE_SETTINGS["FORWARD_STEERING"]
                continue

            x, y = self._waypoint_position() if self.path_mode == "nearest" else self.get_filtered_position()
            if x is None or y is None:
                self.waypoint_arrive_count = 0
                self.distance_to_target = None
                self.last_waypoint_arrived = False
                self.status = f"Waiting for UWB position ({target[0]:.2f}, {target[1]:.2f})"
                self.robot.stop()
                self._sleep_with_uwb_recording(
                    DRIVE_SETTINGS.get("AUTO_STOP_GO_SETTLE_SEC", 0.45),
                    "idle",
                    0,
                )
                continue

            if segment_start is None:
                segment_start = (x, y)
            if arrival_segment_start is None:
                arrival_segment_start = segment_start
            if self._last_segment_drive_start is None:
                self._last_segment_drive_start = (x, y)
            if not has_planned_start and not axis_start_locked:
                segment_start = self._axis_locked_start(target, (x, y))
                planned_start = segment_start
                axis_start_locked = True

            dx = target[0] - x
            dy = target[1] - y
            distance = math.hypot(dx, dy)
            if self.path_mode == "nearest":
                progress_delta = float(DRIVE_SETTINGS.get("NO_PROGRESS_MIN_DELTA_M", 0.05))
                if best_distance is None or distance < best_distance - progress_delta:
                    best_distance = distance
                    last_progress_at = time.monotonic()
                no_progress_sec = float(DRIVE_SETTINGS.get("NO_PROGRESS_TIMEOUT_SEC", 0.0) or 0.0)
                if no_progress_sec > 0.0 and time.monotonic() - last_progress_at >= no_progress_sec:
                    self.status = f"No progress to ({target[0]:.2f}, {target[1]:.2f})"
                    print(
                        f"[NEAREST] no progress for {no_progress_sec:.1f}s: "
                        f"target=({target[0]:.2f},{target[1]:.2f}), "
                        f"distance={distance:.2f}m, best={best_distance:.2f}m",
                        flush=True,
                    )
                    self.last_waypoint_arrived = False
                    break
            clamped = self._current_position_clamped()
            valid_for_motion = self._current_position_valid_for_motion()
            valid_for_arrival = self._current_position_valid_for_arrival()
            if not valid_for_motion:
                self.waypoint_arrive_count = 0
                self.distance_to_target = round(distance, 3)
                self.last_position_clamped = clamped
                self.last_waypoint_arrived = False
                self.status = f"UWB invalid settling for ({target[0]:.2f}, {target[1]:.2f})"
                self.robot.stop()
                self._sleep_with_uwb_recording(
                    DRIVE_SETTINGS.get("AUTO_STOP_GO_SETTLE_SEC", 0.45),
                    "idle",
                    0,
                )
                continue

            if self._handle_x_boundary_recovery():
                drive_since_stop_sec = 0.0
                if not self._segment_is_vertical(segment_start, target):
                    segment_start = planned_start or self._current_position_or_none()
                segment_started = time.monotonic()
                continue

            vertical_arrived = (
                self.path_mode != "nearest"
                and
                valid_for_arrival
                and self._vertical_segment_y_reached(segment_start, target, (x, y))
            )
            axis_stop_arrived = (
                self.path_mode != "nearest"
                and
                valid_for_arrival
                and self._axis_stop_at_target_reached(segment_start, target, (x, y))
            )
            progress_info = (
                self._waypoint_progress(arrival_segment_start, target, (x, y))
                if progress_arrival and arrival_segment_start is not None
                else None
            )
            progress_arrived = (
                progress_info is not None
                and valid_for_motion
                and progress_info[0] >= progress_info[1] - (
                    tolerance if progress_tolerance is None else progress_tolerance
                )
            )
            near_target = (
                (distance <= tolerance and valid_for_arrival)
                or vertical_arrived
                or axis_stop_arrived
                or progress_arrived
            )
            if near_target:
                self.waypoint_arrive_count += 1
            else:
                self.waypoint_arrive_count = 0
            timed_arrived = (
                timed_arrival_sec is not None
                and drive_elapsed_sec >= timed_arrival_sec
                and valid_for_arrival
                and self._timed_waypoint_target_close(target, (x, y))
                and self._timed_waypoint_position_ok(segment_start, target, (x, y), tolerance)
            )
            arrived = (
                vertical_arrived
                or axis_stop_arrived
                or progress_arrived
                or self.waypoint_arrive_count >= confirm_count
                or timed_arrived
            )
            arrival_reason = None
            if progress_arrived:
                arrival_reason = "progress"
            elif vertical_arrived:
                arrival_reason = "vertical"
            elif axis_stop_arrived:
                arrival_reason = "axis"
            elif timed_arrived:
                arrival_reason = "timed"
            elif self.waypoint_arrive_count >= confirm_count:
                arrival_reason = "distance"
            self.distance_to_target = round(distance, 3)
            self.last_position_clamped = clamped
            self.last_waypoint_arrived = arrived
            self._log_waypoint_state(
                (x, y),
                target,
                distance,
                arrived,
                clamped,
                self.waypoint_arrive_count,
                confirm_count,
                timed_arrived or vertical_arrived or axis_stop_arrived or progress_arrived,
                arrival_reason,
                progress_info,
            )
            if arrived:
                self._last_segment_drive_end = (x, y)
                break

            reacquire_reason = self._target_reacquire_reason(segment_start, target, (x, y))
            if reacquire_reason is not None:
                self.waypoint_arrive_count = 0
                self.last_waypoint_arrived = False
                self.status = f"Reacquiring target: {reacquire_reason}"
                print(
                    f"[TARGET_REACQUIRE] {reacquire_reason}, "
                    f"start=({segment_start[0]:.2f},{segment_start[1]:.2f}), "
                    f"target=({target[0]:.2f},{target[1]:.2f}), "
                    f"current=({x:.2f},{y:.2f})",
                    flush=True,
                )
                segment_start = (x, y)
                planned_start = segment_start
                segment_started = time.monotonic()
                drive_since_stop_sec = 0.0
                self._last_auto_steering = DRIVE_SETTINGS["FORWARD_STEERING"]

            line_recovery_target = self._grid_line_recovery_target(segment_start, target, (x, y))
            if line_recovery_target is not None:
                self.status = f"Grid line recovery to ({line_recovery_target[0]:.2f}, {line_recovery_target[1]:.2f})"
                self._align_heading_to_target(line_recovery_target)
                segment_start = planned_start or (x, y)
                segment_started = time.monotonic()
                drive_since_stop_sec = 0.0
                target_for_motion = line_recovery_target
            else:
                segment_follow_target = self._segment_follow_target(segment_start, target, (x, y))
                target_for_motion = segment_follow_target or target

            vertical_segment = self._segment_is_vertical(segment_start, target)
            if vertical_segment and self._x_axis_recovery_requires_stop(target, (x, y)):
                self.waypoint_arrive_count = 0
                recovery_steering = self._x_axis_recovery_steering(
                    target,
                    (x, y),
                    self._planned_segment_direction(segment_start, target),
                )
                if recovery_steering is None:
                    self.robot.stop()
                    self.status = f"X line hold for ({target[0]:.2f}, {target[1]:.2f})"
                    self._sleep_with_uwb_recording(
                        DRIVE_SETTINGS.get("AUTO_STOP_GO_SETTLE_SEC", 0.45),
                        "idle",
                        0,
                    )
                else:
                    speed = DRIVE_SETTINGS.get("AUTO_X_RECOVERY_SPEED", DRIVE_SETTINGS["FORWARD_SPEED"])
                    self.status = f"Hard X recovery to x={target[0]:.2f}"
                    self.robot.forward(
                        speed,
                        left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
                        right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
                        steering=recovery_steering,
                    )
                    self._sleep_with_uwb_recording(drive_step_sec, "forward", speed)
                continue

            horizontal_recovery = self._horizontal_y_recovery_steering(segment_start, target_for_motion, (x, y))
            recovery_steering = (
                self._x_axis_recovery_steering(
                    target_for_motion,
                    (x, y),
                    self._planned_segment_direction(segment_start, target),
                )
                if vertical_segment
                else None
            )
            speed = DRIVE_SETTINGS["FORWARD_SPEED"]
            if self.path_mode == "nearest" and self._get_heading() is None:
                speed = DRIVE_SETTINGS.get("AUTO_NO_HEADING_TEST_SPEED", speed)
            if (
                horizontal_recovery is None
                and recovery_steering is None
                and self._target_align_if_needed((x, y), target_for_motion)
            ):
                drive_since_stop_sec = 0.0
                segment_started = time.monotonic()
                continue
            if horizontal_recovery is not None:
                steering, y_error = horizontal_recovery
                speed = DRIVE_SETTINGS.get("AUTO_HORIZONTAL_Y_RECOVERY_SPEED", speed)
                self.status = f"Recovering y line {y:.2f}->{target_for_motion[1]:.2f}"
                print(
                    f"[Y_RECOVERY] horizontal segment: y_error={y_error:.3f}, "
                    f"steering={steering:.1f}, current=({x:.2f},{y:.2f}), "
                    f"target=({target_for_motion[0]:.2f},{target_for_motion[1]:.2f})",
                    flush=True,
                )
            elif recovery_steering is not None:
                steering = recovery_steering
                speed = DRIVE_SETTINGS.get("AUTO_X_RECOVERY_SPEED", speed)
                direction = "right" if recovery_steering > DRIVE_SETTINGS["FORWARD_STEERING"] else "left"
                self.status = f"Recovering {direction} to ({target[0]:.2f}, {target[1]:.2f})"
            elif time.monotonic() - segment_started < DRIVE_SETTINGS.get("AUTO_STEERING_WARMUP_SEC", 0.35):
                steering = DRIVE_SETTINGS["FORWARD_STEERING"]
            else:
                if target_for_motion != target:
                    self.status = f"Following line to ({target_for_motion[0]:.2f}, {target_for_motion[1]:.2f})"
                else:
                    self.status = f"Moving to ({target[0]:.2f}, {target[1]:.2f})"
                if self.path_mode == "nearest" and DRIVE_SETTINGS.get("RELAXED_TARGET_FOLLOW_ENABLE", True):
                    steering = self._relaxed_target_steering((x, y), target_for_motion, max_steering)
                else:
                    steering = self._uwb_segment_follow_steering(
                        segment_start,
                        target,
                        (x, y),
                        target_for_motion,
                        steering_gain,
                        max_steering,
                    )
                steering = self._lidar_wall_assist_steering(steering, max_steering)
                steering = self._limit_auto_steering(steering, max_steering)

            self.robot.forward(
                speed,
                left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
                right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
                steering=steering,
            )
            self._sleep_with_uwb_recording(drive_step_sec, "forward", speed)
            drive_elapsed_sec += drive_step_sec
            drive_since_stop_sec += drive_step_sec

            if (
                DRIVE_SETTINGS.get("AUTO_STOP_GO_ENABLE", False)
                and drive_since_stop_sec >= DRIVE_SETTINGS.get("AUTO_STOP_GO_RUN_SEC", 0.9)
            ):
                blocked, reason = self._rssi_sampling_blocked_reason()
                if blocked:
                    print(f"[RSSI] timed sample skipped: {reason}", flush=True)
                    drive_since_stop_sec = 0.0
                    last_stop_go_sample_at = time.monotonic()
                    continue
                if (
                    DRIVE_SETTINGS.get("AUTO_STOP_GO_RSSI_SAMPLE_ENABLE", True)
                    and scanner
                    and fingerprint_db is not None
                ):
                    self.status = f"Timed RSSI sample near ({x:.2f}, {y:.2f})"
                    print(
                        f"[RSSI] timed sample trigger after {drive_since_stop_sec:.2f}s "
                        f"near ({x:.2f}, {y:.2f}); dwell={DRIVE_SETTINGS.get('AUTO_SCAN_DWELL_SEC', 2.0)}s",
                        flush=True,
                    )
                    self._scan_current_point(scanner, fingerprint_db, settle=True)
                else:
                    self.robot.stop()
                    self.status = f"UWB settling for ({target[0]:.2f}, {target[1]:.2f})"
                    self._sleep_with_uwb_recording(
                        DRIVE_SETTINGS.get("AUTO_STOP_GO_SETTLE_SEC", 0.45),
                        "idle",
                        0,
                    )
                if self.stop_requested:
                    break
                self.status = f"Moving to ({target[0]:.2f}, {target[1]:.2f})"
                drive_since_stop_sec = 0.0
                last_stop_go_sample_at = time.monotonic()

        self.robot.stop()
        if self._last_segment_drive_end is None:
            self._last_segment_drive_end = self._current_position_or_none()
        self._sleep_with_motion(DRIVE_SETTINGS["AUTO_TURN_PAUSE_SEC"], "idle", 0)
        return bool(self.last_waypoint_arrived)

    def _target_align_if_needed(self, current, target):
        if not DRIVE_SETTINGS.get("AUTO_TARGET_ALIGN_ENABLE", False):
            return False
        if not self.robot:
            return False

        heading = self._get_heading()
        bearing = self._bearing_to_target(current, target)
        if heading is None or bearing is None:
            return False

        error = self._angle_diff(bearing, heading)
        self.target_bearing = round(bearing, 1)
        self.current_heading = round(heading, 1)
        self.heading_error = round(error, 1)

        trigger = DRIVE_SETTINGS.get("AUTO_TARGET_ALIGN_TRIGGER_DEG", 45)
        if abs(error) < trigger:
            return False

        release = DRIVE_SETTINGS.get("AUTO_TARGET_ALIGN_RELEASE_DEG", 18)
        max_sec = DRIVE_SETTINGS.get("AUTO_TARGET_ALIGN_MAX_SEC", 0.8)
        speed = DRIVE_SETTINGS.get("AUTO_TARGET_ALIGN_TURN_SPEED", DRIVE_SETTINGS.get("TURN_SPEED", 10))
        step_sec = DRIVE_SETTINGS.get("AUTO_TARGET_ALIGN_STEP_SEC", 0.05)
        deadline = time.monotonic() + max(0.0, max_sec)

        self.status = f"Target align {error:.1f}deg"
        print(
            f"[TARGET_ALIGN] current=({current[0]:.2f},{current[1]:.2f}), "
            f"target=({target[0]:.2f},{target[1]:.2f}), heading={heading:.1f}, "
            f"bearing={bearing:.1f}, error={error:.1f}",
            flush=True,
        )
        self.robot.stop()
        self._sleep_with_uwb_recording(
            DRIVE_SETTINGS.get("AUTO_STOP_GO_SETTLE_SEC", 0.45),
            "idle",
            0,
        )

        while self.running and not self.stop_requested and time.monotonic() < deadline:
            heading = self._get_heading()
            bearing = self._bearing_to_target(current, target)
            if heading is None or bearing is None:
                break
            error = self._angle_diff(bearing, heading)
            self.target_bearing = round(bearing, 1)
            self.current_heading = round(heading, 1)
            self.heading_error = round(error, 1)
            if abs(error) <= release:
                break
            if error > 0:
                self.robot.turn_right(speed)
            else:
                self.robot.turn_left(speed)
            self._sleep_with_uwb_recording(step_sec, "turning", speed)

        self.robot.stop()
        self._sleep_with_uwb_recording(
            DRIVE_SETTINGS.get("AUTO_STOP_GO_SETTLE_SEC", 0.45),
            "idle",
            0,
        )
        self._last_auto_steering = DRIVE_SETTINGS["FORWARD_STEERING"]
        return True

    def _current_position_or_none(self):
        x, y = self.get_filtered_position()
        if x is None or y is None:
            return None
        return (x, y)

    def _current_position_clamped(self):
        if not self.uwb:
            return False
        return bool(self.uwb.snapshot().get("uwb_position_clamped"))

    def _current_position_valid_for_arrival(self):
        if not self.uwb:
            return True
        data = self.uwb.snapshot()
        return bool(data.get("uwb_valid")) and not bool(data.get("uwb_position_clamped"))

    def _current_position_valid_for_motion(self):
        if not self.uwb:
            return True
        data = self.uwb.snapshot()
        return (
            bool(data.get("uwb_valid"))
            and not bool(data.get("uwb_position_holding"))
            and int(data.get("uwb_invalid_position_count") or 0) == 0
        )

    def _x_boundary_recovery_action(self):
        if not DRIVE_SETTINGS.get("AUTO_X_BOUNDARY_RECOVERY_ENABLE", True):
            self._x_boundary_recovery_active = False
            self._x_boundary_recovery_side = None
            self.x_boundary_recovery_reason = None
            return None
        if not self.uwb:
            return None

        data = self.uwb.snapshot()
        unbounded_x = data.get("unbounded_map_x")
        if unbounded_x is None:
            return None

        width = APP_SETTINGS.get("ROOM_WIDTH_M", 3.0)
        exit_margin = DRIVE_SETTINGS.get("AUTO_X_BOUNDARY_RECOVERY_EXIT_MARGIN_M", 0.10)

        if self._x_boundary_recovery_active:
            if self._x_boundary_recovery_side == "underflow":
                if unbounded_x >= exit_margin:
                    self._x_boundary_recovery_active = False
                    self._x_boundary_recovery_side = None
                    self.x_boundary_recovery_reason = None
                    return None
                return "x_underflow_unbounded", unbounded_x
            if self._x_boundary_recovery_side == "overflow":
                if unbounded_x <= width - exit_margin:
                    self._x_boundary_recovery_active = False
                    self._x_boundary_recovery_side = None
                    self.x_boundary_recovery_reason = None
                    return None
                return "x_overflow_unbounded", unbounded_x

        if unbounded_x < 0.0:
            self._x_boundary_recovery_active = True
            self._x_boundary_recovery_side = "underflow"
            return "x_underflow_unbounded", unbounded_x
        if unbounded_x > width:
            self._x_boundary_recovery_active = True
            self._x_boundary_recovery_side = "overflow"
            return "x_overflow_unbounded", unbounded_x

        self.x_boundary_recovery_reason = None
        return None

    def _handle_x_boundary_recovery(self):
        action = self._x_boundary_recovery_action()
        if action is None:
            return False
        if not self.robot:
            return True

        reason, unbounded_x = action
        self.x_boundary_recovery_reason = reason
        self.waypoint_arrive_count = 0
        self.last_waypoint_arrived = False
        self.distance_to_target = None
        self.status = f"X boundary recovery: {reason}"

        if reason == "x_underflow_unbounded":
            steering = DRIVE_SETTINGS.get("AUTO_X_BOUNDARY_UNDERFLOW_STEERING", 28)
            action_name = "steer_to_positive_x"
        else:
            steering = DRIVE_SETTINGS.get("AUTO_X_BOUNDARY_OVERFLOW_STEERING", -28)
            action_name = "steer_to_negative_x"
        steering *= self._steering_correction_sign()

        now = time.monotonic()
        if now - self._last_x_boundary_recovery_log >= 0.5:
            print(
                f"[X_BOUNDARY_RECOVERY] unbounded_x={unbounded_x:.2f}, action={action_name}, steering={steering}",
                flush=True,
            )
            self._last_x_boundary_recovery_log = now

        speed = DRIVE_SETTINGS.get("AUTO_X_BOUNDARY_RECOVERY_SPEED", DRIVE_SETTINGS.get("AUTO_X_RECOVERY_SPEED", 8))
        self.robot.forward(
            speed,
            left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
            right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
            steering=steering,
        )
        self._sleep_with_uwb_recording(
            DRIVE_SETTINGS.get("AUTO_GEOFENCE_RECOVERY_STEP_SEC", 0.08),
            "forward",
            speed,
        )
        return True

    def _x_axis_recovery_steering(self, target, current, direction=None):
        if not DRIVE_SETTINGS.get("AUTO_X_RECOVERY_ENABLE", True):
            return None
        if not self._current_position_valid_for_motion():
            return None

        x, _ = current
        data = self.uwb.snapshot() if self.uwb else {}
        unbounded_x = data.get("unbounded_map_x")
        if (
            DRIVE_SETTINGS.get("UWB_NEGATIVE_X_RECOVERY_ENABLE", True)
            and unbounded_x is not None
            and unbounded_x < -DRIVE_SETTINGS.get("UWB_NEGATIVE_X_RECOVERY_MARGIN_M", 0.05)
        ):
            x = unbounded_x

        desired_x = target[0]
        trigger_m = DRIVE_SETTINGS.get("AUTO_X_RECOVERY_TRIGGER_M", 0.18)
        x_error_m = desired_x - x

        if abs(x_error_m) < trigger_m:
            return None

        correction = DRIVE_SETTINGS.get("AUTO_X_RECOVERY_STEERING", 16)
        steering = correction if x_error_m > 0 else -correction
        if direction == "south":
            steering = -steering
        steering *= self._steering_correction_sign()
        max_steering = DRIVE_SETTINGS.get("AUTO_MAX_STEERING_DEG", 30)
        return max(-max_steering, min(max_steering, steering))

    def _x_axis_recovery_requires_stop(self, target, current):
        if not DRIVE_SETTINGS.get("AUTO_X_RECOVERY_ENABLE", True):
            return False
        if not self._current_position_valid_for_motion():
            return False

        x, _ = current
        stop_error_m = DRIVE_SETTINGS.get("AUTO_X_RECOVERY_STOP_ERROR_M", 0.75)
        return abs(target[0] - x) >= stop_error_m

    def _horizontal_y_recovery_steering(self, start, target, current):
        if not DRIVE_SETTINGS.get("AUTO_HORIZONTAL_Y_RECOVERY_ENABLE", False):
            return None
        if start is None or target is None:
            return None

        sx, sy = start
        tx, ty = target
        x, y = current
        if abs(tx - sx) <= abs(ty - sy):
            return None

        y_error = ty - y
        trigger = DRIVE_SETTINGS.get("AUTO_HORIZONTAL_Y_RECOVERY_TRIGGER_M", 0.18)
        if abs(y_error) < trigger:
            return None

        direction = "east" if tx >= sx else "west"
        gain = DRIVE_SETTINGS.get("AUTO_HORIZONTAL_Y_RECOVERY_GAIN", 45.0)
        correction = -y_error * gain if direction == "east" else y_error * gain
        correction *= self._steering_correction_sign()
        max_steering = DRIVE_SETTINGS.get("AUTO_LINE_MAX_STEERING_DEG", DRIVE_SETTINGS.get("AUTO_MAX_STEERING_DEG", 30))
        base = DRIVE_SETTINGS["FORWARD_STEERING"]
        steering = base + correction
        steering = max(base - max_steering, min(base + max_steering, steering))
        return steering, y_error

    @staticmethod
    def _steering_correction_sign():
        return -1.0 if DRIVE_SETTINGS.get("AUTO_STEERING_CORRECTION_SIGN", 1.0) < 0 else 1.0

    def _uwb_failsafe_triggered(self):
        if not self.uwb:
            return False

        data = self.uwb.snapshot()
        invalid_count = int(data.get("uwb_invalid_position_count") or 0)
        limit = DRIVE_SETTINGS.get("AUTO_FAILSAFE_INVALID_UWB_COUNT", 3)
        holding = bool(data.get("uwb_position_holding"))
        clamped = bool(data.get("uwb_position_clamped"))

        if holding and invalid_count >= limit:
            if DRIVE_SETTINGS.get("AUTO_GEOFENCE_RECOVERY_ENABLE", True):
                print(
                    f"[RECOVERY] UWB outside map. holding={holding}, clamped={clamped}, "
                    f"invalid={invalid_count}, raw=({data.get('raw_x')},{data.get('raw_y')}), "
                    f"map=({data.get('map_x')},{data.get('map_y')})",
                    flush=True,
                )
                self._recover_inside_geofence(data)
                return False

            if not DRIVE_SETTINGS.get("AUTO_UWB_OUTSIDE_MAP_FAILSAFE_ENABLE", True):
                now = time.monotonic()
                if now - getattr(self, "_last_geofence_ignore_log", 0.0) >= 1.0:
                    self._last_geofence_ignore_log = now
                    print(
                        f"[GEOFENCE] UWB outside map ignored. LiDAR obstacle handling remains active. "
                        f"invalid={invalid_count}, raw=({data.get('raw_x')},{data.get('raw_y')}), "
                        f"map=({data.get('map_x')},{data.get('map_y')})",
                        flush=True,
                    )
                return False

            self.status = f"Fail-safe: UWB outside map ({invalid_count})"
            print(
                f"[FAILSAFE] UWB outside map. holding={holding}, clamped={clamped}, "
                f"invalid={invalid_count}, raw=({data.get('raw_x')},{data.get('raw_y')}), "
                f"map=({data.get('map_x')},{data.get('map_y')})",
                flush=True,
            )
            if self.robot:
                self.robot.stop()
            self.running = False
            return True

        return False

    def _recover_inside_geofence(self, data):
        if not self.robot:
            return

        width = APP_SETTINGS.get("ROOM_WIDTH_M", 3.0)
        height = APP_SETTINGS.get("ROOM_HEIGHT_M", 3.0)
        margin = max(0.0, DRIVE_SETTINGS.get("AUTO_EDGE_MARGIN_M", 0.5))
        current = self._recovery_position_from_snapshot(data, width, height)
        target = (
            round(min(max(current[0], margin), max(margin, width - margin)), 3),
            round(min(max(current[1], margin), max(margin, height - margin)), 3),
        )
        self.current_target = target
        self.distance_to_target = round(math.hypot(target[0] - current[0], target[1] - current[1]), 3)

        deadline = time.monotonic() + DRIVE_SETTINGS.get("AUTO_GEOFENCE_RECOVERY_MAX_SEC", 2.0)
        step_sec = DRIVE_SETTINGS.get("AUTO_GEOFENCE_RECOVERY_STEP_SEC", 0.08)
        speed = DRIVE_SETTINGS.get("AUTO_GEOFENCE_RECOVERY_SPEED", 8)
        tolerance = DRIVE_SETTINGS.get("AUTO_WAYPOINT_TOLERANCE_M", 0.25)

        while self.running and not self.stop_requested and time.monotonic() < deadline:
            snapshot = self.uwb.snapshot() if self.uwb else {}
            invalid_count = int(snapshot.get("uwb_invalid_position_count") or 0)
            holding = bool(snapshot.get("uwb_position_holding"))
            if not holding and invalid_count == 0:
                self.status = "Recovered inside map"
                self.robot.stop()
                return

            current = self._recovery_position_from_snapshot(snapshot, width, height)
            distance = math.hypot(target[0] - current[0], target[1] - current[1])
            self.distance_to_target = round(distance, 3)
            self.status = f"Recovering inside map to ({target[0]:.2f}, {target[1]:.2f})"
            if distance <= tolerance:
                self.robot.stop()
                self._sleep_recovery_step(step_sec, "idle", 0)
                continue

            bearing = self._bearing_to_target(current, target)
            heading = self._get_heading()
            error = self._angle_diff(bearing, heading) if bearing is not None and heading is not None else None
            self.target_bearing = round(bearing, 1) if bearing is not None else None
            self.current_heading = round(heading, 1) if heading is not None else None
            self.heading_error = round(error, 1) if error is not None else None

            if error is not None and abs(error) > DRIVE_SETTINGS.get("AUTO_HEADING_ALIGN_TOLERANCE_DEG", 20):
                if error > 0:
                    self.robot.turn_right(speed)
                else:
                    self.robot.turn_left(speed)
                self._sleep_recovery_step(step_sec, "turning", speed)
            else:
                self.robot.forward(
                    speed,
                    left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
                    right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
                    steering=DRIVE_SETTINGS["FORWARD_STEERING"],
                )
                self._sleep_recovery_step(step_sec, "forward", speed)

        self.robot.stop()

    def _recovery_position_from_snapshot(self, data, width, height):
        x, y = self.get_filtered_position()
        if x is None:
            x = data.get("map_x", data.get("x", width / 2.0))
        if y is None:
            y = data.get("map_y", data.get("y", height / 2.0))
        if x is None:
            x = width / 2.0
        if y is None:
            y = height / 2.0
        return (float(x), float(y))

    def _sleep_recovery_step(self, seconds, mode="idle", speed=0):
        end_time = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end_time and self.running and not self.stop_requested:
            self._update_motion_tracking(mode, speed)
            self._record_position()
            time.sleep(min(0.02, max(0.0, end_time - time.monotonic())))

    def _log_waypoint_state(
        self,
        current,
        target,
        distance,
        arrived,
        clamped,
        arrive_count,
        confirm_count,
        timed_arrived=False,
        arrival_reason=None,
        progress_info=None,
    ):
        now = time.monotonic()
        if not arrived and now - self._last_waypoint_log < 0.5:
            return
        self._last_waypoint_log = now
        uwb = self.uwb.snapshot() if self.uwb else {}
        progress_text = ""
        if progress_info is not None:
            progress_text = f"progress={progress_info[0]:.2f}/{progress_info[1]:.2f}m, "
        print(
            f"[WAYPOINT] current=({current[0]:.2f},{current[1]:.2f}), "
            f"target=({target[0]:.2f},{target[1]:.2f}), "
            f"distance={distance:.3f}, arrived={arrived}, clamped={clamped}, "
            f"{progress_text}"
            f"confirm={arrive_count}/{confirm_count}, timed={timed_arrived}, "
            f"reason={arrival_reason}, "
            f"raw=({uwb.get('raw_x')},{uwb.get('raw_y')}), "
            f"map=({uwb.get('map_x')},{uwb.get('map_y')}), "
            f"unbounded_map=({uwb.get('unbounded_map_x')},{uwb.get('unbounded_map_y')})",
            flush=True,
        )

    def _timed_waypoint_position_ok(self, start, target, current, tolerance):
        if start is None:
            return False

        sx, sy = start
        tx, ty = target
        x, y = current
        vx = tx - sx
        vy = ty - sy
        length = math.hypot(vx, vy)
        if length < 1e-6:
            return True

        progress_m = ((x - sx) * vx + (y - sy) * vy) / length
        cross_track_m = abs((vx * (y - sy) - vy * (x - sx)) / length)
        progress_margin = max(tolerance, DRIVE_SETTINGS.get("AUTO_TIMED_WAYPOINT_PROGRESS_MARGIN_M", 0.25))
        max_cross_track = DRIVE_SETTINGS.get("AUTO_TIMED_WAYPOINT_CROSSTRACK_M", 0.45)

        return progress_m >= length - progress_margin and cross_track_m <= max_cross_track

    def _timed_waypoint_target_close(self, target, current):
        x, y = current
        tolerance = DRIVE_SETTINGS.get("AUTO_TIMED_WAYPOINT_TARGET_TOLERANCE_M", 0.35)
        return abs(target[0] - x) <= tolerance and abs(target[1] - y) <= tolerance

    def _uwb_line_follow_steering(self, start, target, current, gain, max_steering):
        sx, sy = start
        tx, ty = target
        x, y = current
        vx = tx - sx
        vy = ty - sy
        length = math.hypot(vx, vy)
        if length < 1e-6:
            return DRIVE_SETTINGS["FORWARD_STEERING"]

        # Positive cross-track means the tag is left of the planned path.
        # The correction sign is configurable because servo direction can vary by car.
        cross_track_m = (vx * (y - sy) - vy * (x - sx)) / length
        clamp_m = DRIVE_SETTINGS.get("AUTO_CROSSTRACK_CLAMP_M", 0.35)
        cross_track_m = max(-clamp_m, min(clamp_m, cross_track_m))
        deadband_m = DRIVE_SETTINGS.get("AUTO_CROSSTRACK_DEADBAND_M", 0.0)
        if abs(cross_track_m) < deadband_m:
            return DRIVE_SETTINGS["FORWARD_STEERING"]
        correction = cross_track_m * gain * self._steering_correction_sign()
        steering = DRIVE_SETTINGS["FORWARD_STEERING"] + correction
        return max(-max_steering, min(max_steering, steering))

    def _uwb_segment_follow_steering(self, start, segment_target, current, follow_target, gain, max_steering):
        steering = self._uwb_line_follow_steering(
            start,
            segment_target,
            current,
            gain,
            max_steering,
        )
        heading = self._get_heading()
        bearing = self._bearing_to_target(current, follow_target)
        if heading is None or bearing is None:
            return steering

        heading_error = self._angle_diff(bearing, heading)
        self.target_bearing = round(bearing, 1)
        self.current_heading = round(heading, 1)
        self.heading_error = round(heading_error, 1)

        heading_gain = DRIVE_SETTINGS.get("AUTO_SEGMENT_HEADING_GAIN", 0.18)
        steering += heading_error * heading_gain * self._steering_correction_sign()
        return max(-max_steering, min(max_steering, steering))

    def _relaxed_target_steering(self, current, target, max_steering):
        heading = self._get_heading()
        bearing = self._bearing_to_target(current, target)
        if heading is None or bearing is None:
            return DRIVE_SETTINGS["FORWARD_STEERING"]

        heading_error = self._angle_diff(bearing, heading)
        self.target_bearing = round(bearing, 1)
        self.current_heading = round(heading, 1)
        self.heading_error = round(heading_error, 1)

        relaxed_limit = min(
            max_steering,
            float(DRIVE_SETTINGS.get("RELAXED_TARGET_MAX_STEERING_DEG", max_steering)),
        )
        steering = (
            DRIVE_SETTINGS["FORWARD_STEERING"]
            + heading_error
            * float(DRIVE_SETTINGS.get("RELAXED_TARGET_HEADING_GAIN", 0.18))
            * self._steering_correction_sign()
        )
        return max(-relaxed_limit, min(relaxed_limit, steering))

    def _limit_auto_steering(self, steering, max_steering):
        base = DRIVE_SETTINGS["FORWARD_STEERING"]
        rate_limit = DRIVE_SETTINGS.get("AUTO_STEERING_RATE_LIMIT_DEG", 2.0)
        steering = max(base - max_steering, min(base + max_steering, steering))
        delta = steering - self._last_auto_steering
        if delta > rate_limit:
            steering = self._last_auto_steering + rate_limit
        elif delta < -rate_limit:
            steering = self._last_auto_steering - rate_limit
        self._last_auto_steering = steering
        return steering

    def _lidar_wall_assist_steering(self, steering, max_steering):
        if not DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_ENABLE", False) or not self.lidar:
            self.lidar_wall_assist = {"valid": False, "reason": "disabled"}
            return steering
        if not hasattr(self.lidar, "wall_assist"):
            self.lidar_wall_assist = {"valid": False, "reason": "unsupported"}
            return steering

        assist = self.lidar.wall_assist()
        self.lidar_wall_assist = dict(assist or {})
        if not assist or not assist.get("valid"):
            now = time.monotonic()
            log_sec = DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_LOG_SEC", 0.8)
            if now - getattr(self, "_last_lidar_wall_log", 0.0) >= log_sec:
                self._last_lidar_wall_log = now
                print(
                    f"[LIDAR_WALL] invalid reason={self.lidar_wall_assist.get('reason')}, "
                    f"side={self.lidar_wall_assist.get('side')}, "
                    f"mid={self.lidar_wall_assist.get('mid_distance_m')}",
                    flush=True,
                )
            return steering

        base = DRIVE_SETTINGS["FORWARD_STEERING"]
        wall_steering = base + float(assist.get("steering_correction_deg", 0.0))
        weight = float(DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_WEIGHT", 0.85))
        weight = max(0.0, min(1.0, weight))
        uwb_delta = steering - base
        wall_delta = wall_steering - base
        conflict_deadband = DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_CONFLICT_DEADBAND_DEG", 1.0)
        lidar_uwb_conflict = (
            abs(uwb_delta) >= conflict_deadband
            and abs(wall_delta) >= conflict_deadband
            and uwb_delta * wall_delta < 0
        )
        if lidar_uwb_conflict:
            weight = min(
                weight,
                DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_CONFLICT_WEIGHT", 0.25),
            )
            self.lidar_wall_assist["uwb_conflict"] = True
            self.lidar_wall_assist["effective_weight"] = round(weight, 2)
        else:
            self.lidar_wall_assist["uwb_conflict"] = False
            self.lidar_wall_assist["effective_weight"] = round(weight, 2)
        steering = (1.0 - weight) * steering + weight * wall_steering

        lidar_limit = DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_MAX_STEERING_DEG", max_steering)
        steering_limit = min(max_steering, lidar_limit)
        steering = max(base - steering_limit, min(base + steering_limit, steering))

        now = time.monotonic()
        log_sec = DRIVE_SETTINGS.get("LIDAR_WALL_ASSIST_LOG_SEC", 0.8)
        if now - getattr(self, "_last_lidar_wall_log", 0.0) >= log_sec:
            self._last_lidar_wall_log = now
            print(
                f"[LIDAR_WALL] side={assist.get('side')}, "
                f"mid={assist.get('mid_distance_m')}m, "
                f"parallel={assist.get('parallel_error_m')}m, "
                f"correction={assist.get('steering_correction_deg')}deg, "
                f"weight={weight:.2f}, steering={steering:.1f}",
                flush=True,
            )
        return steering

    def _waypoint_progress_reached(self, start, target, current, tolerance):
        sx, sy = start
        tx, ty = target
        x, y = current
        vx = tx - sx
        vy = ty - sy
        length = math.hypot(vx, vy)
        if length < 1e-6:
            return True

        progress_m = ((x - sx) * vx + (y - sy) * vy) / length
        return progress_m >= length - tolerance

    @staticmethod
    def _waypoint_progress(start, target, current):
        sx, sy = start
        tx, ty = target
        x, y = current
        vx = tx - sx
        vy = ty - sy
        length = math.hypot(vx, vy)
        if length < 1e-6:
            return (0.0, 0.0)
        progress_m = ((x - sx) * vx + (y - sy) * vy) / length
        return (progress_m, length)

    @staticmethod
    def _is_lane_change_segment(start, target):
        if not DRIVE_SETTINGS.get("AUTO_LANE_CHANGE_ENABLE", True):
            return False
        if str(DRIVE_SETTINGS.get("AUTO_PATH_MODE", "zigzag")).strip().lower() != "zigzag":
            return False
        dx = abs(target[0] - start[0])
        dy = abs(target[1] - start[1])
        lane_spacing = max(
            0.1,
            DRIVE_SETTINGS.get(
                "AUTO_LANE_SPACING_M",
                DRIVE_SETTINGS.get("AUTO_GRID_SPACING_M", 0.5),
            ),
        )
        return abs(dx - lane_spacing) <= 1e-6 and dy <= 1e-6

    def _lane_change_turn(self, start, target):
        if not self.robot:
            return

        self.status = "Lane Change Turn"
        self.robot.stop()
        self._sleep_with_motion(DRIVE_SETTINGS.get("AUTO_TURN_PAUSE_SEC", 0.12), "idle", 0)
        if self._uwb_failsafe_triggered():
            return

        if not DRIVE_SETTINGS.get("AUTO_REVERSE_ENABLE", True):
            self.status = "Reverse skipped"
            self._update_motion_tracking("idle", 0)
            print("[REVERSE_GUARD] lane-change reverse skipped", flush=True)
        elif self._rear_obstacle_detected():
            self.robot.stop()
            self._update_motion_tracking("idle", 0)
            self._escape_forward_from_rear_obstacle()
        elif self.robot.backward(
            DRIVE_SETTINGS["REVERSE_SPEED"],
            steering=DRIVE_SETTINGS["FORWARD_STEERING"],
        ) is False:
            self.status = "Reverse blocked"
        else:
            self._sleep_backward_with_rear_guard(
                DRIVE_SETTINGS.get("AUTO_LANE_CHANGE_BACKUP_SEC", 0.30),
                DRIVE_SETTINGS["REVERSE_SPEED"],
                use_uwb_recording=True,
            )
        self.robot.stop()
        self._sleep_with_motion(DRIVE_SETTINGS.get("AUTO_TURN_PAUSE_SEC", 0.12), "idle", 0)
        if self._uwb_failsafe_triggered():
            return

        steering_abs = DRIVE_SETTINGS.get("AUTO_LANE_CHANGE_STEERING_DEG", 24)
        speed = DRIVE_SETTINGS.get("AUTO_LANE_CHANGE_SPEED", DRIVE_SETTINGS["TURN_SPEED"])
        moving_to_right_lane = target[0] > start[0]
        moving_from_top_to_bottom = target[1] < start[1]

        first_sign = 1 if moving_to_right_lane else -1
        if not moving_from_top_to_bottom:
            first_sign *= -1

        self.robot.forward(
            speed,
            left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
            right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
            steering=first_sign * steering_abs,
        )
        self._sleep_with_uwb_recording(
            DRIVE_SETTINGS.get("AUTO_LANE_CHANGE_ARC_SEC", 0.65),
            "turning",
            speed,
        )
        if self._uwb_failsafe_triggered():
            return

        self.robot.forward(
            speed,
            left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
            right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
            steering=-first_sign * steering_abs,
        )
        self._sleep_with_uwb_recording(
            DRIVE_SETTINGS.get("AUTO_LANE_CHANGE_ALIGN_SEC", 0.35),
            "turning",
            speed,
        )

        self.robot.stop()
        self._sleep_with_uwb_recording(
            DRIVE_SETTINGS.get("AUTO_LANE_CHANGE_SETTLE_SEC", 0.50),
            "idle",
            0,
        )

    def _lane_change_reacquire_x(self, target):
        if not self.robot:
            return

        tolerance = DRIVE_SETTINGS.get("AUTO_LANE_REACQUIRE_X_TOLERANCE_M", 0.18)
        deadline = time.monotonic() + DRIVE_SETTINGS.get("AUTO_LANE_REACQUIRE_MAX_SEC", 2.2)
        step_sec = 0.05

        while self.running and not self.stop_requested and time.monotonic() < deadline:
            if self._uwb_failsafe_triggered():
                break

            x, y = self.get_filtered_position()
            if x is None or y is None or not self._current_position_valid_for_motion():
                self.robot.stop()
                self.status = f"UWB settling for ({target[0]:.2f}, {target[1]:.2f})"
                self._sleep_with_uwb_recording(
                    DRIVE_SETTINGS.get("AUTO_STOP_GO_SETTLE_SEC", 0.45),
                    "idle",
                    0,
                )
                continue

            x_error = target[0] - x
            if abs(x_error) <= tolerance:
                break

            steering = self._x_axis_recovery_steering(target, (x, y))
            if steering is None:
                break

            direction = "right" if x_error > 0 else "left"
            speed = DRIVE_SETTINGS.get("AUTO_X_RECOVERY_SPEED", DRIVE_SETTINGS["FORWARD_SPEED"])
            self.status = f"Lane align {direction} to x={target[0]:.2f}"
            self.robot.forward(
                speed,
                left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
                right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
                steering=steering,
            )
            self._sleep_with_uwb_recording(step_sec, "forward", speed)

        self.robot.stop()
        self._sleep_with_uwb_recording(
            DRIVE_SETTINGS.get("AUTO_LANE_CHANGE_SETTLE_SEC", 0.50),
            "idle",
            0,
        )

    def _zigzag_turn(self, next_direction):
        if not self.robot:
            return

        self.status = "Zigzag Turn"
        self.robot.stop()
        self._sleep_with_motion(DRIVE_SETTINGS["AUTO_TURN_PAUSE_SEC"], "idle", 0)
        if self._uwb_failsafe_triggered():
            return

        if not DRIVE_SETTINGS.get("AUTO_REVERSE_ENABLE", True):
            self.status = "Reverse skipped"
            self._update_motion_tracking("idle", 0)
            print("[REVERSE_GUARD] zigzag reverse skipped", flush=True)
        elif self._rear_obstacle_detected():
            self.robot.stop()
            self._update_motion_tracking("idle", 0)
            self._escape_forward_from_rear_obstacle()
        elif self.robot.backward(DRIVE_SETTINGS["REVERSE_SPEED"], steering=DRIVE_SETTINGS["FORWARD_STEERING"]) is False:
            self.status = "Reverse blocked"
        else:
            self._sleep_backward_with_rear_guard(
                DRIVE_SETTINGS.get("AUTO_TURN_BACKUP_TIME_SEC", 0.35),
                DRIVE_SETTINGS["REVERSE_SPEED"],
                use_uwb_recording=True,
            )
        if self._uwb_failsafe_triggered():
            return
        self.robot.stop()
        self._sleep_with_motion(DRIVE_SETTINGS["AUTO_TURN_PAUSE_SEC"], "idle", 0)
        if self._uwb_failsafe_triggered():
            return

        turn_left = next_direction in ("west", "north")
        self._start_turn_tracking()
        turn_steering = DRIVE_SETTINGS.get(
            "AUTO_TURN_STEERING_DEG",
            DRIVE_SETTINGS.get("AUTO_MAX_STEERING_DEG", 25),
        )
        steering = -turn_steering if turn_left else turn_steering
        self.robot.forward(
            DRIVE_SETTINGS["TURN_SPEED"],
            left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
            right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
            steering=steering,
        )
        self._sleep_with_uwb_recording(
            DRIVE_SETTINGS.get("AUTO_TURN_FORWARD_TIME_SEC", 0.35),
            "turning",
            DRIVE_SETTINGS["TURN_SPEED"],
        )
        if self._uwb_failsafe_triggered():
            return
        self.robot.stop()
        self._finish_turn_tracking()
        self._sleep_with_motion(DRIVE_SETTINGS["AUTO_TURN_PAUSE_SEC"], "idle", 0)
        if self._uwb_failsafe_triggered():
            return

        straighten_sec = DRIVE_SETTINGS.get("AUTO_TURN_STRAIGHTEN_SEC", 0.0)
        if straighten_sec > 0:
            self.robot.forward(
                DRIVE_SETTINGS["FORWARD_SPEED"],
                left_offset=DRIVE_SETTINGS["LEFT_SPEED_OFFSET"],
                right_scale=DRIVE_SETTINGS["RIGHT_SPEED_SCALE"],
                steering=DRIVE_SETTINGS["FORWARD_STEERING"],
            )
            self._sleep_with_uwb_recording(straighten_sec, "forward", DRIVE_SETTINGS["FORWARD_SPEED"])
            self.robot.stop()
        self._sleep_with_motion(DRIVE_SETTINGS["AUTO_TURN_PAUSE_SEC"], "idle", 0)

    def _scan_current_point(self, scanner, fingerprint_db, settle=True):
        if self.stop_requested:
            if self.robot:
                self.robot.stop()
            return

        if not settle:
            self._record_position()
            return

        if settle and self.robot:
            self.robot.stop()
            self.status = "Sampling"
            self._sleep_with_motion(DRIVE_SETTINGS["AUTO_SETTLE_TIME_SEC"], "idle", 0)

        power_stage = self._begin_rssi_power_stage(scanner)
        try:
            print(
                "[RSSI] sampling start: LiDAR must be OFF before dongle scan",
                flush=True,
            )
            sample_x, sample_y = self._sample_uwb_position(
                DRIVE_SETTINGS.get("AUTO_UWB_SAMPLE_SEC", 0.7)
            )

            if scanner:
                self.status = "RSSI Sampling"
                record = self.capture_fingerprint_with_uwb(
                    scanner,
                    x=sample_x,
                    y=sample_y,
                    rssi_sample_sec=DRIVE_SETTINGS.get("AUTO_SCAN_DWELL_SEC", 2.0),
                    manage_power=False,
                )
                ap_count = len(record.get("aps") or [])
                if ap_count <= 0:
                    print(
                        "[RSSI] no APs captured; saving raw scan row with 0 APs. "
                        "Measurements/fingerprints need a matching AP.",
                        flush=True,
                    )
                fingerprint_db.append(record)
                self._save_rf_fingerprint_record(record)
                print(
                    f"[RSSI] sampling done: ap_count={ap_count}, "
                    f"pos=({sample_x}, {sample_y})",
                    flush=True,
                )
        finally:
            self._end_rssi_power_stage(scanner, power_stage)

    def _save_rf_fingerprint_record(self, record):
        rf_db = getattr(self, "rf_db", None)
        if not rf_db:
            return
        try:
            scan_id = rf_db.save_fingerprint_record(record)
            grid_id = getattr(rf_db, "last_grid_id", None)
            ap_count = len(record.get("aps") or [])
            print(
                f"Saved RF scan to SQLite: scan_id={scan_id}, "
                f"grid_id={grid_id or '-'}, ap_count={ap_count}",
                flush=True,
            )
        except Exception as exc:
            print(f"SQLite RF fingerprint save failed: {exc}", flush=True)

    def _rssi_sampling_blocked_reason(self):
        if not DRIVE_SETTINGS.get("AUTO_RSSI_SKIP_DURING_LIDAR_OBSTACLE", True):
            return False, ""

        if self._lidar_obstacle_active:
            return True, "LiDAR obstacle active"

        skip_sec = max(0.0, float(DRIVE_SETTINGS.get("AUTO_RSSI_SKIP_AFTER_OBSTACLE_SEC", 0.0)))
        if skip_sec > 0.0 and self._last_obstacle_avoidance_at:
            elapsed = time.monotonic() - self._last_obstacle_avoidance_at
            if elapsed < skip_sec:
                return True, f"recent obstacle avoidance {elapsed:.1f}s ago"

        status = str(self.status or "").lower()
        if "avoid" in status or "lidar stop" in status:
            return True, f"navigation status={self.status}"

        return False, ""

    def _set_rssi_power_state(self, stage, dongle_power=False, message=""):
        lidar_running = bool(getattr(self.lidar, "running", False))
        simultaneous = bool(dongle_power and lidar_running)
        self.rssi_power_state = {
            "stage": stage,
            "stage_active": self._rssi_power_stage_active,
            "dongle_power_enabled": bool(dongle_power),
            "lidar_running": lidar_running,
            "exclusive": DRIVE_SETTINGS.get("RSSI_POWER_EXCLUSIVE_LIDAR_WIFI", True),
            "simultaneous_power_blocked": simultaneous,
            "message": message,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        if simultaneous:
            print("[POWER] BLOCK: RSSI dongle and LiDAR must not be on together", flush=True)

    def _enforce_power_exclusion_for_motion(self, mode):
        if not DRIVE_SETTINGS.get("RSSI_POWER_EXCLUSIVE_LIDAR_WIFI", True):
            return
        if mode in ("idle", "sampling", "rssi_sampling"):
            return

        scanner = self.scanner
        power_enabled = False
        if scanner and hasattr(scanner, "get_power_status"):
            try:
                power_enabled = bool(scanner.get_power_status().get("local_wifi_power_enabled"))
            except Exception:
                power_enabled = False
        if power_enabled:
            print(
                f"[POWER] SAFETY: RSSI dongle was ON during motion mode={mode}; forcing OFF",
                flush=True,
            )
            self._force_rssi_dongle_off(f"motion_{mode}", scanner=scanner)

    def _rssi_power_snapshot(self):
        state = dict(self.rssi_power_state or {})
        scanner = self.scanner
        if scanner and hasattr(scanner, "get_power_status"):
            scanner_status = scanner.get_power_status()
            state["scanner"] = scanner_status
            state["dongle_power_enabled"] = bool(scanner_status.get("local_wifi_power_enabled"))
            state["dongle_interfaces"] = scanner_status.get("local_wifi_power_interfaces", [])
        state["lidar_running"] = bool(getattr(self.lidar, "running", False))
        state["stage_active"] = self._rssi_power_stage_active
        state["exclusive"] = DRIVE_SETTINGS.get("RSSI_POWER_EXCLUSIVE_LIDAR_WIFI", True)
        state["safe"] = not (state.get("dongle_power_enabled") and state.get("lidar_running"))
        return state

    def _force_rssi_dongle_off(self, reason, scanner=None):
        scanner = scanner or self.scanner
        self._set_wifi_collection_enabled(False)
        power_enabled = False
        if scanner and hasattr(scanner, "get_power_status"):
            power_enabled = bool(scanner.get_power_status().get("local_wifi_power_enabled"))
        if scanner and hasattr(scanner, "set_local_wifi_power"):
            if power_enabled or reason not in ("drive_loop", "waypoint_drive"):
                scanner.set_local_wifi_power(False)
        self._set_rssi_power_state(
            reason,
            dongle_power=False,
            message=f"RSSI dongle forced off ({reason})",
        )

    def _begin_rssi_power_stage(self, scanner):
        if not DRIVE_SETTINGS.get("RSSI_POWER_EXCLUSIVE_LIDAR_WIFI", True):
            self._set_wifi_collection_enabled(True)
            return None

        self._rssi_power_stage_active = True
        self._set_rssi_power_state(
            "preparing",
            dongle_power=False,
            message="RSSI sample prep: dongle off, stopping LiDAR",
        )
        self._set_wifi_collection_enabled(False)
        self._force_rssi_dongle_off("rssi_precheck", scanner=scanner)
        lidar_was_running = self._stop_lidar_for_rssi()
        scanner_state = self._configure_scanner_for_rssi(scanner)
        return {
            "lidar_was_running": lidar_was_running,
            "scanner_state": scanner_state,
        }

    def _end_rssi_power_stage(self, scanner, stage):
        if not DRIVE_SETTINGS.get("RSSI_POWER_EXCLUSIVE_LIDAR_WIFI", True):
            self._set_wifi_collection_enabled(False)
            return
        stage = stage or {}
        self._restore_scanner_after_rssi(scanner, stage.get("scanner_state"))
        self._restore_lidar_after_rssi(bool(stage.get("lidar_was_running")))
        self._rssi_power_stage_active = False
        self._set_rssi_power_state(
            "idle",
            dongle_power=False,
            message="RSSI sample done: dongle off, LiDAR restored",
        )

    def _stop_lidar_for_rssi(self):
        if not self.lidar:
            return False
        was_running = bool(getattr(self.lidar, "running", False))
        if was_running:
            self.status = "RSSI prep: LiDAR off"
            print("[POWER] LiDAR OFF before RSSI dongle scan", flush=True)
            self.lidar.stop()
            self.lidar_wall_assist = {"valid": False, "reason": "rssi_lidar_off"}
            self._sleep_with_motion(DRIVE_SETTINGS.get("RSSI_LIDAR_STOP_SETTLE_SEC", 0.7), "idle", 0)
        self._set_rssi_power_state(
            "lidar_off",
            dongle_power=False,
            message="LiDAR is off; RSSI dongle may be enabled",
        )
        return was_running

    def _restore_lidar_after_rssi(self, was_running):
        if not was_running or not self.lidar:
            return
        if not DRIVE_SETTINGS.get("RSSI_RESTART_LIDAR_AFTER_SCAN", True):
            return
        self._sleep_with_motion(DRIVE_SETTINGS.get("RSSI_WIFI_TO_LIDAR_GUARD_SEC", 0.3), "idle", 0)
        print("[POWER] RSSI scan done; LiDAR restart", flush=True)
        if self.lidar.start():
            self._wait_for_lidar_ready_after_restart()
        self._set_rssi_power_state(
            "lidar_on",
            dongle_power=False,
            message="LiDAR restarted after RSSI dongle power down",
        )

    def _wait_for_lidar_ready_after_restart(self):
        wait_sec = max(0.0, float(DRIVE_SETTINGS.get("LIDAR_RESTART_READY_WAIT_SEC", 0.0)))
        if wait_sec <= 0 or not self.lidar:
            return

        deadline = time.monotonic() + wait_sec
        self.status = "LiDAR restarting"
        while self.running and not self.stop_requested and time.monotonic() < deadline:
            try:
                data = self.lidar.snapshot() if hasattr(self.lidar, "snapshot") else {}
                connected = bool(data.get("lidar_connected"))
                point_count = int(data.get("lidar_point_count") or 0)
                if connected and point_count > 0:
                    print(
                        f"[POWER] LiDAR ready after restart: points={point_count}",
                        flush=True,
                    )
                    return
            except Exception as exc:
                print(f"[POWER] LiDAR ready check failed: {exc}", flush=True)
                return
            time.sleep(0.05)

        print("[POWER] LiDAR restart wait ended before fresh points.", flush=True)

    def _configure_scanner_for_rssi(self, scanner):
        self._set_wifi_collection_enabled(False)
        if not scanner:
            return None

        state = {
            "use_esp32_wifi": getattr(scanner, "use_esp32_wifi", None),
            "use_local_wifi": getattr(scanner, "use_local_wifi", None),
        }
        use_local = DRIVE_SETTINGS.get("RSSI_USE_LOCAL_WIFI", True)
        use_esp32 = DRIVE_SETTINGS.get("RSSI_USE_ESP32_WIFI", False)

        if hasattr(scanner, "set_sources"):
            scanner.set_sources(use_esp32_wifi=use_esp32, use_local_wifi=use_local)
        else:
            scanner.use_esp32_wifi = bool(use_esp32)
            scanner.use_local_wifi = bool(use_local)

        lidar_still_running = bool(self.lidar and getattr(self.lidar, "running", False))
        if use_local and lidar_still_running:
            print("[POWER] BLOCK: LiDAR still running, RSSI dongle will stay OFF", flush=True)
            if hasattr(scanner, "set_sources"):
                scanner.set_sources(use_local_wifi=False)
            else:
                scanner.use_local_wifi = False
            self._set_rssi_power_state(
                "blocked",
                dongle_power=False,
                message="RSSI dongle blocked because LiDAR is still running",
            )
            return state

        if use_esp32:
            self._set_wifi_collection_enabled(True)
        if use_local and hasattr(scanner, "set_local_wifi_power"):
            if hasattr(scanner, "get_network_status"):
                net_status = scanner.get_network_status(log=True)
                if not net_status.get("scan_interfaces"):
                    print("[RSSI] sample skipped: no safe Wi-Fi dongle scan interface", flush=True)
                    self._set_rssi_power_state(
                        "wifi_unavailable",
                        dongle_power=False,
                        message="No safe Wi-Fi dongle scan interface; wlan0 remains untouched",
                    )
                    return state
            if bool(self.lidar and getattr(self.lidar, "running", False)):
                print("[POWER] SAFETY: LiDAR is running; RSSI dongle will NOT turn on", flush=True)
                self._set_rssi_power_state(
                    "blocked",
                    dongle_power=False,
                    message="RSSI dongle blocked because LiDAR is running",
                )
                return state
            print("[POWER] RSSI dongle ON; LiDAR must remain OFF", flush=True)
            scanner.set_local_wifi_power(True)
            if bool(self.lidar and getattr(self.lidar, "running", False)):
                print("[POWER] SAFETY: LiDAR restarted unexpectedly; forcing RSSI dongle OFF", flush=True)
                scanner.set_local_wifi_power(False)
                self._set_rssi_power_state(
                    "blocked",
                    dongle_power=False,
                    message="RSSI dongle forced off because LiDAR is running",
                )
                return state
            power_status = scanner.get_power_status() if hasattr(scanner, "get_power_status") else {}
            if not power_status.get("local_wifi_power_enabled"):
                print("[RSSI] sample skipped: RSSI dongle did not power on", flush=True)
                self._set_rssi_power_state(
                    "wifi_unavailable",
                    dongle_power=False,
                    message="RSSI dongle did not power on; LiDAR remains protected",
                )
                return state
            self._set_rssi_power_state(
                "sampling",
                dongle_power=True,
                message="RSSI dongle on for sampling; LiDAR must remain off",
            )
        return state

    def _restore_scanner_after_rssi(self, scanner, state):
        self._set_wifi_collection_enabled(False)
        if not scanner:
            return
        if hasattr(scanner, "set_local_wifi_power"):
            scanner.set_local_wifi_power(False)
            print("[POWER] RSSI dongle OFF", flush=True)
            self._set_rssi_power_state(
                "dongle_off",
                dongle_power=False,
                message="RSSI dongle off before LiDAR restart",
            )
        if state:
            if hasattr(scanner, "set_sources"):
                scanner.set_sources(
                    use_esp32_wifi=state.get("use_esp32_wifi"),
                    use_local_wifi=state.get("use_local_wifi"),
                )
            else:
                scanner.use_esp32_wifi = bool(state.get("use_esp32_wifi"))
                scanner.use_local_wifi = bool(state.get("use_local_wifi"))

    def _sample_uwb_position(self, seconds, step=0.05):
        end_time = time.monotonic() + max(0.0, seconds)
        samples = []

        while time.monotonic() < end_time and self.running and not self.stop_requested:
            self._record_position()
            x, y = self.get_filtered_position()
            if x is not None and y is not None and self._current_position_valid_for_arrival():
                samples.append((x, y))
            time.sleep(min(step, max(0.0, end_time - time.monotonic())))

        if samples:
            method = str(DRIVE_SETTINGS.get("AUTO_UWB_SAMPLE_METHOD", "median")).strip().lower()
            if method == "mean":
                sample_x = sum(x for x, _ in samples) / len(samples)
                sample_y = sum(y for _, y in samples) / len(samples)
            else:
                sample_x = self._median([x for x, _ in samples])
                sample_y = self._median([y for _, y in samples])
            sample_x, sample_y = round(sample_x, 3), round(sample_y, 3)
            if self.uwb and hasattr(self.uwb, "set_sample_position"):
                self.uwb.set_sample_position(sample_x, sample_y)
            return sample_x, sample_y

        return self.get_filtered_position()

    @staticmethod
    def _median(values):
        values = sorted(values)
        count = len(values)
        middle = count // 2
        if count % 2:
            return values[middle]
        return (values[middle - 1] + values[middle]) / 2.0

    def _set_wifi_collection_enabled(self, enabled):
        if self.uwb and hasattr(self.uwb, "set_wifi_collection_enabled"):
            self.uwb.set_wifi_collection_enabled(enabled)

    def _sleep_with_uwb_recording(self, seconds, mode="idle", speed=0, step=0.05):
        end_time = time.monotonic() + max(0, seconds)
        while time.monotonic() < end_time and self.running and not self.stop_requested:
            if self._uwb_failsafe_triggered():
                break
            self._enforce_power_exclusion_for_motion(mode)
            self._update_motion_tracking(mode, speed)
            self._record_position()
            time.sleep(min(step, max(0, end_time - time.monotonic())))
        if self.stop_requested and self.robot:
            self.robot.stop()
