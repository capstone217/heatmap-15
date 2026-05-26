class RobotController:
    def __init__(self):
        try:
            from picarx import Picarx
        except ImportError as exc:
            raise RuntimeError("picarx library is required on the robot.") from exc

        try:
            try:
                from config import COMM_SETTINGS
                ultrasonic_pins = COMM_SETTINGS.get("PICARX_ULTRASONIC_PINS", ["D2", "D3"])
            except Exception:
                ultrasonic_pins = ["D2", "D3"]
            self.px = Picarx(ultrasonic_pins=ultrasonic_pins)
            self.stop()
            print(f"Picarx initialized successfully. Ultrasonic pins reserved at {ultrasonic_pins}.")
        except Exception as e:
            print(f"Picarx initialization failed: {e}")
            raise RuntimeError(f"Failed to initialize Picarx: {e}") from e

    def stop(self):
        self.px.stop()
        self.set_steering(0)

    def set_steering(self, angle):
        self.px.set_dir_servo_angle(angle)

    def set_motor_speeds(self, left, right):
        self.px.set_motor_speed(1, left)
        self.px.set_motor_speed(2, right)
        return True

    def forward(self, speed, left_offset=1.0, right_scale=1.0, steering=-5):
        self.set_steering(steering)
        return self.set_motor_speeds(speed * left_offset, -speed * right_scale)

    def backward(self, speed, steering=0):
        self.set_steering(steering)
        return self.set_motor_speeds(-speed, speed)

    def turn_left(self, speed):
        self.set_steering(0)
        return self.set_motor_speeds(-speed, -speed)

    def turn_right(self, speed):
        self.set_steering(0)
        return self.set_motor_speeds(speed, speed)

    def cleanup(self):
        self.stop()
