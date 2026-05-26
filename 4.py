import time

from config import COMM_SETTINGS, DRIVE_SETTINGS


def angle_error_deg(angle, center):
    return (angle - center + 180.0) % 360.0 - 180.0


def in_sector(angle, center, width):
    return abs(angle_error_deg(angle, center)) <= width / 2.0


def main():
    try:
        from rplidar import RPLidar
    except ImportError as exc:
        print(f"rplidar package is required: {exc}", flush=True)
        return

    ports = COMM_SETTINGS.get("LIDAR_SERIAL_PORTS", ["/dev/ttyUSB1"])
    baudrates = COMM_SETTINGS.get("LIDAR_SERIAL_BAUDS", [115200])
    port = ports[0]
    baudrate = int(baudrates[0])

    rear_center = float(DRIVE_SETTINGS.get("LIDAR_REAR_OBSTACLE_CENTER_DEG", 180.0))
    rear_width = float(DRIVE_SETTINGS.get("LIDAR_REAR_OBSTACLE_WIDTH_DEG", 60.0))
    rear_threshold_m = float(DRIVE_SETTINGS.get("LIDAR_REAR_OBSTACLE_DISTANCE_M", 0.55))
    min_distance_m = float(DRIVE_SETTINGS.get("LIDAR_MIN_DISTANCE_M", 0.08))
    max_distance_m = float(DRIVE_SETTINGS.get("LIDAR_MAX_DISTANCE_M", 12.0))

    print("LiDAR rear angle check")
    print(f"Port: {port} @ {baudrate}")
    print(
        f"Current rear sector: center={rear_center:.1f}deg, "
        f"width={rear_width:.1f}deg, range={rear_center - rear_width / 2:.1f}~{rear_center + rear_width / 2:.1f}deg"
    )
    print(f"Rear obstacle threshold: {rear_threshold_m * 100:.0f}cm")
    print("Put a wall/box behind the car at 30~60cm and watch the nearest angle.")
    print("Ctrl+C to stop.\n")

    lidar = RPLidar(port, baudrate=baudrate, timeout=1.0)
    points = []
    last_print = 0.0

    try:
        try:
            lidar.start_motor()
        except Exception:
            pass
        time.sleep(1.5)

        for _new_scan, quality, angle, distance_mm in lidar.iter_measures():
            distance_m = float(distance_mm) / 1000.0
            if quality <= 0 or distance_m < min_distance_m or distance_m > max_distance_m:
                continue

            points.append((float(angle) % 360.0, distance_m, int(quality)))
            if len(points) > 600:
                del points[:-600]

            now = time.monotonic()
            if now - last_print < 0.35 or len(points) < 20:
                continue
            last_print = now

            nearest = min(points, key=lambda item: item[1])
            rear_points = [
                item for item in points
                if in_sector(item[0], rear_center, rear_width)
            ]
            rear_nearest = min(rear_points, key=lambda item: item[1]) if rear_points else None

            angle, distance, q = nearest
            rear_text = "rear:none"
            if rear_nearest:
                r_angle, r_distance, r_q = rear_nearest
                rear_hit = r_distance <= rear_threshold_m
                rear_text = (
                    f"rear:{r_distance * 100:.1f}cm @ {r_angle:.1f}deg "
                    f"q={r_q} hit={'YES' if rear_hit else 'no'}"
                )

            print(
                f"nearest:{distance * 100:.1f}cm @ {angle:.1f}deg q={q} | "
                f"{rear_text}",
                flush=True,
            )

    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
    finally:
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


if __name__ == "__main__":
    main()
