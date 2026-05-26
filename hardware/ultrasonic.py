import time


class UltrasonicSensor:
    def __init__(
        self,
        trig_pin=None,
        echo_pin=None,
        max_distance_cm=120,
        threshold_cm=30,
        robot_hat_ultrasonic=None,
    ):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.max_distance_cm = max_distance_cm
        self.threshold_cm = threshold_cm
        self.robot_hat_ultrasonic = robot_hat_ultrasonic
        self.backend = ""
        self.ready = False
        self.GPIO = None
        self.error = ""

    def connect(self):
        if self.robot_hat_ultrasonic is not None:
            self.backend = "robot_hat"
            self.ready = True
            self.error = ""
            print("Ultrasonic sensor using PiCar-X robot_hat backend.", flush=True)
            return True

        if self.trig_pin is None or self.echo_pin is None:
            return False

        try:
            import RPi.GPIO as GPIO
        except ImportError:
            print("Ultrasonic sensor: RPi.GPIO unavailable, ultrasonic obstacle detection disabled.", flush=True)
            return False

        self.GPIO = GPIO
        try:
            # Check if GPIO mode is already set (e.g., by Picarx)
            current_mode = GPIO.getmode()
            if current_mode is None:
                # GPIO not initialized yet, set it
                GPIO.setmode(GPIO.BCM)
            elif current_mode != GPIO.BCM:
                print(f"GPIO already set to mode {current_mode}, ultrasonic sensor needs BCM mode.", flush=True)
                return False
            
            self._setup_pins()
            time.sleep(0.05)
            self.ready = True
            self.error = ""
            return True
        except Exception as exc:
            try:
                self.GPIO.cleanup([self.trig_pin, self.echo_pin])
            except Exception:
                pass
            try:
                self._setup_pins()
                time.sleep(0.05)
                self.ready = True
                self.error = ""
                print("Ultrasonic sensor GPIO setup recovered after cleanup.", flush=True)
                return True
            except Exception as retry_exc:
                self.ready = False
                self.error = str(retry_exc or exc)
                print(
                    f"Ultrasonic sensor GPIO setup failed: {self.error}",
                    flush=True,
                )
                print("Ultrasonic sensor disabled. Other obstacle sensors will still run.", flush=True)
                return False

    def _setup_pins(self):
        self.GPIO.setup(self.trig_pin, self.GPIO.OUT)
        self.GPIO.setup(self.echo_pin, self.GPIO.IN)
        self.GPIO.output(self.trig_pin, False)

    def read_distance(self):
        if not self.ready:
            return None

        if self.backend == "robot_hat":
            try:
                distance_cm = self.robot_hat_ultrasonic.read()
            except Exception as exc:
                self.ready = False
                self.error = str(exc)
                print(f"Ultrasonic sensor read failed, disabling ultrasonic: {self.error}", flush=True)
                return None

            try:
                distance_cm = float(distance_cm)
            except (TypeError, ValueError):
                return None
            if distance_cm <= 0 or distance_cm > self.max_distance_cm:
                return None
            return distance_cm

        try:
            self.GPIO.output(self.trig_pin, False)
            time.sleep(0.0002)
            self.GPIO.output(self.trig_pin, True)
            time.sleep(0.00001)
            self.GPIO.output(self.trig_pin, False)

            start_time = time.time()
            timeout = start_time + 0.02
            while self.GPIO.input(self.echo_pin) == 0 and time.time() < timeout:
                start_time = time.time()

            stop_time = time.time()
            while self.GPIO.input(self.echo_pin) == 1 and time.time() < timeout:
                stop_time = time.time()
        except Exception as exc:
            self.ready = False
            self.error = str(exc)
            print(f"Ultrasonic sensor read failed, disabling ultrasonic: {self.error}", flush=True)
            return None

        elapsed = stop_time - start_time
        distance_cm = (elapsed * 34300) / 2
        if distance_cm <= 0 or distance_cm > self.max_distance_cm:
            return None
        return distance_cm

    def is_obstacle(self):
        distance = self.read_distance()
        return distance is not None and distance <= self.threshold_cm

    def cleanup(self):
        if self.GPIO and self.ready:
            try:
                self.GPIO.cleanup([self.trig_pin, self.echo_pin])
            except Exception:
                pass
