# Autonomous Radio Heatmap Mapper

This project runs a Picar-X based autonomous radio heatmap mapper with:

- USB webcam video streaming
- ESP32/UWB serial input
- IMU serial input placeholder
- AC1300/Wi-Fi dongle scanning for 2.4 GHz and 5 GHz RSSI
- FastAPI web dashboard at port 8000

## Run

```bash
cd heatmap-15
sudo python3 1.py
```

Open the dashboard:

```text
http://localhost:8000
```

From another PC on the same network:

```text
http://<raspberry-pi-ip>:8000
```

## Install Dependencies

On Raspberry Pi OS, try apt packages first:

```bash
sudo apt update
sudo apt install -y python3-fastapi python3-uvicorn python3-jinja2 python3-serial python3-opencv wireless-tools iw
```

If using a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python 1.py
```

## Wi-Fi Dongle

The scanner automatically tries `wlan1`, `wlan0`, and detected wireless interfaces such as `wlx...`.

To check the AC1300 interface name:

```bash
iw dev
```

If needed, edit `config.py`:

```python
"WIFI_INTERFACES": ["wlan1", "wlan0", "wlx..."]
```

## Notes

The web page starts the car only when the dashboard button is pressed. Run the server first, then open the dashboard and press the exploration/start button.

## 2.85m x 2.85m UWB zigzag mapping

`config.py` defaults to a 2.85m x 2.85m room with four UWB anchors on the corners:

```text
d1: (0.0, 0.0)
d2: (0.0, 2.85)
d3: (2.85, 2.85)
d4: (2.85, 0.0)
```

The auto mapping button drives the robot in a zigzag grid over the selected area. The default dashboard size is 2.85m x 2.85m and the default grid spacing is 0.3m.

Recommended ESP32/UWB serial line with mandatory RSSI:

```text
POS,NAN,NAN,d1,d2,d3,d4,rssi1,rssi2,rssi3,rssi4
```

Example:

```text
POS,NAN,NAN,1.20,0.81,2.48,2.10,-62,-68,-71,-74
```

Label format also works:

```text
X=1.23,Y=0.85,D1=1.40,D2=0.81,D3=2.48,D4=2.10,RSSI1=-62,RSSI2=-68,RSSI3=-71,RSSI4=-74
```

Each scan record and exported CSV now includes UWB distances and UWB RSSI values.


## 2026-05-06 3-anchor UWB coordinate fix

This bundle fixes the case where the ESP32/UWB tag sends lines like:

```text
POS,NAN,NAN,d1,d2,d3,d4
```

If `x` and `y` are `NAN` but at least 3 anchor distances are valid, the Raspberry Pi now calculates `(x, y)` using trilateration from `config.py -> UWB_ANCHORS_M`.

Important:
- At least 3 valid distances are required.
- If only 2 anchors are valid, the HTML will correctly show `좌표 없음`.
- Set the real anchor coordinates in `config.py` before final testing.

Quick parser test:

```bash
python3 test_uwb_parse.py
```

Run:

```bash
sudo python3 1.py
```
