import socket
import sys
import time

import RPi.GPIO as GPIO

from config import APP_SETTINGS, COMM_SETTINGS, DRIVE_SETTINGS, PINS
from core.navigation import Navigator
from core.rf_fingerprint_db import RFFingerprintDB
from hardware.camera import USBCamera
from hardware.imu import IMUSensor
from hardware.motor_driver import RobotController
from hardware.scanner import WiFiScanner
from hardware.lidar import LidarReader
from hardware.ultrasonic import UltrasonicSensor
from hardware.uwb_reader import UWBReceiver
from web.server import start_web_server


navigator = None
scanner = None
camera = None
ultrasonic = None
lidar = None
imu = None


def wait_for_uwb_port(uwb, timeout_sec):
    deadline = time.monotonic() + max(0.0, timeout_sec)
    last_port = None
    while time.monotonic() < deadline:
        snapshot = uwb.snapshot()
        last_port = snapshot.get("uwb_port") or last_port
        if snapshot.get("uwb_connected") and snapshot.get("uwb_port"):
            return snapshot.get("uwb_port")
        time.sleep(0.05)
    return last_port


def initialize_system():
    # Clean up GPIO pins before initialization
    try:
        GPIO.cleanup()
    except Exception as e:
        print(f"GPIO cleanup warning: {e}", flush=True)
    
    try:
        GPIO.setmode(GPIO.BCM)
        print("GPIO initialized successfully.", flush=True)
    except Exception as e:
        print(f"GPIO initialization failed: {e}", flush=True)
        print("Hardware sensors may not work properly.", flush=True)
    
    global navigator, scanner, camera, ultrasonic, lidar, imu

    try:
        camera = USBCamera(device_id=COMM_SETTINGS["CAMERA_INDEX"])
        if camera.connect():
            camera.start_capture_thread()
            print("Web camera ready.", flush=True)
        else:
            print("Web camera not found. Camera obstacle detection is disabled.", flush=True)

        # Initialize robot controller first (before sensors that may share Robot HAT resources)
        robot = None
        try:
            robot = RobotController()
            print("Robot controller ready.", flush=True)
        except Exception as e:
            print(f"Robot controller initialization failed: {e}. Robot control disabled.", flush=True)

        ultrasonic = UltrasonicSensor(
            trig_pin=COMM_SETTINGS.get("ULTRASONIC_TRIGGER_PIN"),
            echo_pin=COMM_SETTINGS.get("ULTRASONIC_ECHO_PIN"),
            threshold_cm=DRIVE_SETTINGS.get("ULTRASONIC_THRESHOLD_CM", 32),
            robot_hat_ultrasonic=getattr(getattr(robot, "px", None), "ultrasonic", None),
        )
        ultrasonic_ready = False
        if DRIVE_SETTINGS.get("ULTRASONIC_ENABLE", False):
            try:
                ultrasonic_ready = ultrasonic.connect()
            except Exception as exc:
                print(f"Ultrasonic sensor initialization failed: {exc}", flush=True)
                ultrasonic_ready = False
        if ultrasonic_ready:
            print("Ultrasonic sensor ready.", flush=True)
        else:
            print("Ultrasonic sensor disabled.", flush=True)

        imu = None
        imu_excluded_ports = []
        if COMM_SETTINGS.get("LOCAL_IMU_ENABLE", False):
            imu = IMUSensor(
                port=COMM_SETTINGS.get("IMU_SERIAL_PORT"),
                ports=COMM_SETTINGS.get("IMU_SERIAL_PORTS"),
                baudrate=COMM_SETTINGS.get("IMU_SERIAL_BAUD", COMM_SETTINGS["SERIAL_BAUD"]),
                bus_num=PINS.get("IMU_BUS", 1),
                i2c_enable=COMM_SETTINGS.get("IMU_I2C_ENABLE", True),
                i2c_addrs=COMM_SETTINGS.get("IMU_I2C_ADDRS", [0x68, 0x69]),
                rtimu_enable=COMM_SETTINGS.get("IMU_RTIMU_ENABLE", True),
            )
            imu.start()
            imu_excluded_ports = [imu.port] if getattr(imu, "serial", None) and imu.port else []
            print("IMU enabled.", flush=True)
        else:
            print("IMU disabled.", flush=True)

        uwb = UWBReceiver(
            ports=COMM_SETTINGS.get("UWB_SERIAL_PORTS", COMM_SETTINGS["SERIAL_PORTS"]),
            baudrate=COMM_SETTINGS.get(
                "UWB_SERIAL_BAUDS",
                [COMM_SETTINGS.get("UWB_SERIAL_BAUD", COMM_SETTINGS["SERIAL_BAUD"])],
            ),
            excluded_ports=imu_excluded_ports,
        )
        uwb.start()

        lidar = None
        if DRIVE_SETTINGS.get("LIDAR_ENABLE", False):
            uwb_port = None
            if DRIVE_SETTINGS.get("LIDAR_START_AFTER_UWB_READY", True):
                uwb_port = wait_for_uwb_port(
                    uwb,
                    DRIVE_SETTINGS.get("LIDAR_WAIT_FOR_UWB_SEC", 3.0),
                )
            excluded_lidar_ports = [uwb_port] if uwb_port else []
            if DRIVE_SETTINGS.get("LIDAR_START_AFTER_UWB_READY", True) and not uwb_port:
                print("LiDAR delayed: UWB serial port is not ready.", flush=True)
            else:
                lidar = LidarReader(
                    ports=COMM_SETTINGS.get("LIDAR_SERIAL_PORTS", []),
                    baudrates=COMM_SETTINGS.get("LIDAR_SERIAL_BAUDS", [230400, 115200]),
                    excluded_ports=excluded_lidar_ports,
                )
                if not DRIVE_SETTINGS.get("LIDAR_START_ON_SYSTEM_INIT", True):
                    print("LiDAR start delayed until autonomous exploration starts.", flush=True)
                elif lidar.start():
                    print("LiDAR obstacle detection enabled.", flush=True)
                else:
                    print("LiDAR obstacle detection disabled.", flush=True)
        else:
            print("LiDAR obstacle detection disabled.", flush=True)

        scanner = WiFiScanner(
            interface=COMM_SETTINGS["WIFI_INTERFACE"],
            interfaces=COMM_SETTINGS.get("WIFI_INTERFACES"),
            scan_interfaces=COMM_SETTINGS.get("WIFI_SCAN_INTERFACES"),
            scan_all_interfaces=COMM_SETTINGS.get("WIFI_SCAN_ALL_INTERFACES", True),
            auto_discover_usb_dongle=COMM_SETTINGS.get("WIFI_AUTO_DISCOVER_USB_DONGLE", True),
            exclude_connected_interfaces=COMM_SETTINGS.get("WIFI_EXCLUDE_CONNECTED_INTERFACES", True),
            interface_power_control_enable=COMM_SETTINGS.get("WIFI_INTERFACE_POWER_CONTROL_ENABLE", False),
            esp32_wifi_source=uwb,
            use_esp32_wifi=COMM_SETTINGS.get("ESP32_WIFI_SCAN_ENABLE", True),
            use_local_wifi=COMM_SETTINGS.get("LOCAL_WIFI_SCAN_ENABLE", True),
            target_ssids=COMM_SETTINGS.get("TARGET_WIFI_SSIDS"),
        )
        net_status = scanner.get_network_status(log=True)
        print(f"[WiFi] configured scan interfaces: {net_status.get('configured_scan_interfaces', [])}", flush=True)
        scanner.local_wifi_power_enabled = False
        scanner.local_wifi_power_interfaces = []
        print("RSSI Wi-Fi dongle marked OFF at startup without changing network links.", flush=True)
        rf_db = RFFingerprintDB()
        rf_db.create_tables()
        print(f"RF fingerprint SQLite DB ready: {rf_db.db_path}", flush=True)
        navigator = Navigator(
            robot=robot,
            uwb=uwb,
            imu=imu,
            camera=camera,
            scanner=scanner,
            ultrasonic=ultrasonic,
            lidar=lidar,
        )
        navigator.rf_db = rf_db

        print("Robot system initialized.", flush=True)
        return True
    except Exception as exc:
        print(f"System initialization failed: {exc}", flush=True)
        return False


def is_port_available(host, port):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.5)
            return sock.connect_ex((host, port)) != 0
    except OSError as exc:
        print(f"Port check skipped: {exc}", flush=True)
        return True


def cleanup():
    print("Cleaning up resources...", flush=True)

    if navigator:
        navigator.shutdown()

    if camera:
        camera.release()

    if scanner:
        scanner.close()

    if ultrasonic:
        try:
            ultrasonic.cleanup()
        except Exception:
            pass

    if lidar:
        try:
            lidar.stop()
        except Exception:
            pass

    if imu:
        try:
            imu.stop()
        except Exception:
            pass

    print("Shutdown complete.", flush=True)


def open_browser_safely(url):
    try:
        import webbrowser

        webbrowser.open(url)
    except Exception as exc:
        print(f"Browser auto-open skipped: {exc}", flush=True)


def main():
    port = APP_SETTINGS["WEB_PORT"]
    host = APP_SETTINGS["WEB_HOST"]
    bind_url = f"http://{host}:{port}"
    browser_url = f"http://127.0.0.1:{port}" if host == "0.0.0.0" else bind_url

    host_check = "127.0.0.1" if host == "0.0.0.0" else host
    if not is_port_available(host_check, port):
        print(
            f"Port {port} is already in use on {host}. Please stop any running server before starting.",
            flush=True,
        )
        sys.exit(1)

    if not initialize_system():
        cleanup()
        sys.exit(1)

    print(f"Dashboard: {browser_url}", flush=True)
    if host == "0.0.0.0":
        print(f"Server bind: {bind_url}", flush=True)

    if APP_SETTINGS.get("AUTO_OPEN_BROWSER", False):
        import threading
        print("브라우저를 열고 있습니다...", flush=True)
        threading.Timer(2.0, open_browser_safely, args=(browser_url,)).start()

    try:
        start_web_server(navigator=navigator, scanner=scanner, camera=camera)
    except KeyboardInterrupt:
        print("\nStopped by user.", flush=True)
    finally:
        cleanup()


if __name__ == "__main__":
    main()
