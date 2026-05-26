import threading
import time
import os
import math


class IMUSensor:
    def __init__(
        self,
        port=None,
        baudrate=115200,
        bus_num=1,
        ports=None,
        i2c_enable=True,
        i2c_addrs=None,
        rtimu_enable=True,
    ):
        self.port = port
        self.ports = ports or []
        self.baudrate = baudrate
        self.bus_num = bus_num
        self.serial = None
        self.bus = None
        self.i2c_enable = i2c_enable
        self.i2c_addrs = i2c_addrs or [0x68, 0x69]
        self.i2c_addr = None
        self.rtimu_enable = rtimu_enable
        self.rtimu = None
        self.rtimu_poll_sec = 0.02
        self.source = None
        self._last_i2c_update = None
        self._heading = 0.0
        self.running = False
        self.lock = threading.Lock()
        self.data = {
            "heading": None,
            "roll": None,
            "pitch": None,
            "imu_last_seen": None,
            "imu_last_seen_monotonic": None,
            "imu_source": None,
            "imu_error": "",
        }

    def start(self):
        serial_ready = self._start_serial()
        if not serial_ready and self.i2c_enable:
            self._start_i2c()
        if not self.serial and not self.bus and self.rtimu_enable:
            self._start_rtimu()

        if not self.serial and not self.bus and not self.rtimu:
            print("IMU not available.", flush=True)
            return

        self.running = True
        threading.Thread(target=self._read_loop, daemon=True).start()

    def _start_serial(self):
        try:
            import serial
        except Exception as exc:
            print(f"IMU serial disabled: {exc}", flush=True)
            return False

        for port in self._port_candidates():
            if not os.path.exists(port):
                continue
            try:
                self.serial = serial.Serial(port, self.baudrate, timeout=0.02)
                self.port = port
                self.source = "serial"
                with self.lock:
                    self.data["imu_source"] = self.source
                    self.data["imu_error"] = ""
                print(f"IMU serial ready: {port} @ {self.baudrate}", flush=True)
                return True
            except Exception as exc:
                print(f"IMU serial skip {port}: {exc}", flush=True)

        print("IMU serial not found. Trying I2C IMU.", flush=True)
        return False

    def _start_i2c(self):
        try:
            import smbus
        except Exception:
            try:
                import smbus2 as smbus
            except Exception as exc:
                print(f"IMU I2C disabled: {exc}", flush=True)
                with self.lock:
                    self.data["imu_error"] = str(exc)
                return False

        try:
            bus = smbus.SMBus(self.bus_num)
        except Exception as exc:
            print(f"IMU I2C bus open failed: {exc}", flush=True)
            with self.lock:
                self.data["imu_error"] = str(exc)
            return False

        for addr in self.i2c_addrs:
            try:
                bus.write_byte_data(addr, 0x6B, 0x00)
                time.sleep(0.05)
                bus.read_byte_data(addr, 0x75)
                self.bus = bus
                self.i2c_addr = addr
                self.source = "i2c"
                self._last_i2c_update = time.monotonic()
                with self.lock:
                    self.data["heading"] = 0.0
                    self.data["imu_source"] = self.source
                    self.data["imu_error"] = ""
                print(f"IMU I2C ready: bus={self.bus_num}, addr=0x{addr:02x}", flush=True)
                return True
            except Exception as exc:
                print(f"IMU I2C skip addr 0x{addr:02x}: {exc}", flush=True)

        try:
            bus.close()
        except Exception:
            pass
        print("IMU I2C not found.", flush=True)
        with self.lock:
            self.data["imu_error"] = "I2C IMU not found"
        return False

    def _start_rtimu(self):
        try:
            import RTIMU
        except Exception as exc:
            print(f"IMU RTIMU disabled: {exc}", flush=True)
            with self.lock:
                self.data["imu_error"] = str(exc)
            return False

        try:
            settings = RTIMU.Settings("RTIMULib")
            imu = RTIMU.RTIMU(settings)
            if not imu.IMUInit():
                print("IMU RTIMU init failed.", flush=True)
                with self.lock:
                    self.data["imu_error"] = "RTIMU init failed"
                return False
            imu_name = str(imu.IMUName())
            if "null" in imu_name.lower():
                print("IMU RTIMU found Null IMU. Treating as unavailable.", flush=True)
                with self.lock:
                    self.data["imu_error"] = "RTIMU Null IMU"
                return False

            imu.setSlerpPower(0.02)
            imu.setGyroEnable(True)
            imu.setAccelEnable(True)
            imu.setCompassEnable(True)
            self.rtimu_poll_sec = max(0.005, imu.IMUGetPollInterval() / 1000.0)
            self.rtimu = imu
            self.source = "rtimu"
            with self.lock:
                self.data["heading"] = 0.0
                self.data["imu_source"] = self.source
                self.data["imu_error"] = ""
            print(f"IMU RTIMU ready: {imu_name}", flush=True)
            return True
        except Exception as exc:
            print(f"IMU RTIMU failed: {exc}", flush=True)
            with self.lock:
                self.data["imu_error"] = str(exc)
            return False

    def _port_candidates(self):
        candidates = []
        for port in [self.port, *self.ports]:
            if port and port not in candidates:
                candidates.append(port)
        return candidates

    def stop(self):
        self.running = False
        if self.serial:
            self.serial.close()
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass

    def snapshot(self):
        with self.lock:
            return dict(self.data)

    def get_heading(self):
        with self.lock:
            return self.data["heading"]

    def _read_loop(self):
        while self.running:
            try:
                if self.serial and self.serial.in_waiting > 0:
                    line = self.serial.readline().decode("utf-8", errors="ignore").strip()
                    parsed = self._parse_line(line)
                    if parsed:
                        with self.lock:
                            parsed["imu_source"] = "serial"
                            self.data.update(parsed)
                elif self.bus:
                    parsed = self._read_i2c_mpu6050()
                    if parsed:
                        with self.lock:
                            self.data.update(parsed)
                elif self.rtimu:
                    parsed = self._read_rtimu()
                    if parsed:
                        with self.lock:
                            self.data.update(parsed)
                time.sleep(self.rtimu_poll_sec if self.rtimu else 0.02)
            except Exception as exc:
                print(f"IMU read error: {exc}", flush=True)
                with self.lock:
                    self.data["imu_error"] = str(exc)
                time.sleep(0.2)

    def _parse_line(self, line):
        parts = [part.strip() for part in line.split(",")]
        if len(parts) < 4 or parts[0].upper() not in ("IMU", "YPR"):
            return None

        heading = self._parse_float(parts[1])
        pitch = self._parse_float(parts[2])
        roll = self._parse_float(parts[3])
        return {
            "heading": heading,
            "pitch": pitch,
            "roll": roll,
            "imu_last_seen": time.strftime("%H:%M:%S"),
            "imu_last_seen_monotonic": time.monotonic(),
        }

    def _read_i2c_mpu6050(self):
        now = time.monotonic()
        dt = 0.0 if self._last_i2c_update is None else max(0.0, now - self._last_i2c_update)
        self._last_i2c_update = now

        accel_x = self._read_i2c_word(0x3B) / 16384.0
        accel_y = self._read_i2c_word(0x3D) / 16384.0
        accel_z = self._read_i2c_word(0x3F) / 16384.0
        gyro_z = self._read_i2c_word(0x47) / 131.0

        self._heading = (self._heading + gyro_z * dt) % 360.0
        roll = math.degrees(math.atan2(accel_y, accel_z))
        pitch = math.degrees(math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z)))

        return {
            "heading": round(self._heading, 2),
            "pitch": round(pitch, 2),
            "roll": round(roll, 2),
            "imu_last_seen": time.strftime("%H:%M:%S"),
            "imu_last_seen_monotonic": time.monotonic(),
            "imu_source": "i2c",
            "imu_error": "",
        }

    def _read_i2c_word(self, reg):
        high = self.bus.read_byte_data(self.i2c_addr, reg)
        low = self.bus.read_byte_data(self.i2c_addr, reg + 1)
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value

    def _read_rtimu(self):
        if not self.rtimu.IMURead():
            return None

        data = self.rtimu.getIMUData()
        fusion = data.get("fusionPose") or self.rtimu.getFusionData()
        if not fusion or len(fusion) < 3:
            return None

        roll = math.degrees(fusion[0])
        pitch = math.degrees(fusion[1])
        heading = math.degrees(fusion[2]) % 360.0
        return {
            "heading": round(heading, 2),
            "pitch": round(pitch, 2),
            "roll": round(roll, 2),
            "imu_last_seen": time.strftime("%H:%M:%S"),
            "imu_last_seen_monotonic": time.monotonic(),
            "imu_source": "rtimu",
            "imu_error": "",
        }

    @staticmethod
    def _parse_float(value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return None
