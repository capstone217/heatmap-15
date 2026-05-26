import argparse
import signal
import sys
import time

from config import DRIVE_SETTINGS
from hardware.motor_driver import RobotController


running = True


def handle_signal(signum, frame):
    global running
    running = False


def sleep_with_stop_check(seconds):
    end_time = time.monotonic() + max(0.0, seconds)
    while running and time.monotonic() < end_time:
        time.sleep(0.02)


def run_pivot(robot, direction, speed, seconds):
    if direction == "left":
        print(f"Pivot left: speed={speed}, duration={seconds:.2f}s", flush=True)
        robot.turn_left(speed)
    else:
        print(f"Pivot right: speed={speed}, duration={seconds:.2f}s", flush=True)
        robot.turn_right(speed)

    sleep_with_stop_check(seconds)
    robot.stop()
    print("Stopped.", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Run a standalone PiCar-X pivot turn test.")
    parser.add_argument(
        "--direction",
        choices=("left", "right", "both", "right-down"),
        default="right-down",
        help="Pivot direction or preset sequence to test.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=DRIVE_SETTINGS.get("AUTO_ZIGZAG_CORNER_PIVOT_SPEED", DRIVE_SETTINGS.get("TURN_SPEED", 18)),
        help="Motor speed used for pivot.",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=2.4,
        help="Pivot duration per 90-degree turn in seconds.",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="Number of pivot cycles.",
    )
    parser.add_argument(
        "--pause",
        type=float,
        default=0.6,
        help="Pause between repeated pivots.",
    )
    args = parser.parse_args()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    robot = None
    try:
        robot = RobotController()
        if args.direction == "both":
            directions = ["left", "right"]
        elif args.direction == "right-down":
            directions = ["right", "right"]
            print("Preset: right 90deg, then right 90deg again to face down.", flush=True)
        else:
            directions = [args.direction]

        for cycle in range(max(1, args.repeat)):
            if not running:
                break

            print(f"Cycle {cycle + 1}/{max(1, args.repeat)}", flush=True)
            for direction in directions:
                if not running:
                    break
                run_pivot(robot, direction, args.speed, args.seconds)
                if running and args.pause > 0:
                    sleep_with_stop_check(args.pause)

    except Exception as exc:
        print(f"Pivot test failed: {exc}", flush=True)
        return 1
    finally:
        if robot:
            try:
                robot.stop()
                robot.cleanup()
            except Exception as exc:
                print(f"Cleanup warning: {exc}", flush=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())