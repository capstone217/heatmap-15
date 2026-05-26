import json
import time
import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)

    print(f"IMU serial connected: {PORT} @ {BAUD}")

    latest = {}

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        if not line.startswith("{"):
            print("skip:", line)
            continue

        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            print("bad json:", line)
            continue

        if not data.get("ok", False):
            print("not ok:", data)
            continue

        latest = {
            "ok": True,
            "source": "esp32_json",
            "ax": data.get("ax"),
            "ay": data.get("ay"),
            "az": data.get("az"),
            "gx": data.get("gx"),
            "gy": data.get("gy"),
            "gz": data.get("gz"),
            "last_update": time.time(),
            "raw": line,
        }

        print(latest)

if __name__ == "__main__":
    main()
