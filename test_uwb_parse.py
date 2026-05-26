from hardware.uwb_reader import UWBReceiver

receiver = UWBReceiver([], baudrate=115200)

tests = [
    "POS,NAN,NAN,1.20,0.81,2.48,NAN",
    "POS,NAN,NAN,1.20,0.81,2.48,2.10,-62,-68,-71,-74",
    "POS,NAN,NAN,NAN,0.81,2.48,NAN",
    "POS,1.23,0.85,1.40,0.81,2.48,2.10",
    "X=1.23,Y=0.85,D1=1.40,D2=0.81,D3=2.48,D4=2.10,RSSI1=-62,RSSI2=-68,RSSI3=-71,RSSI4=-74",
    "DATA,1.20,0.81,2.48,2.10,-62,-68,-71,-74",
    "DATA,1.20,0.81,2.48,NAN",
]

for line in tests:
    print("RAW:", line)
    print(receiver.parse_line(line))
    print()

wifi_tests = [
    "WIFI,TestAP,aa:bb:cc:11:22:33,-57,6",
    "AP,SSID=Office24,BSSID=11:22:33:44:55:66,RSSI=-63,CH=11",
    "ESP32_WIFI,SSID=FiveG,BSSID=11:22:33:44:55:77,RSSI=-40,CH=36",
]

for line in wifi_tests:
    print("WIFI RAW:", line)
    print(receiver.parse_wifi_line(line))
    print()

json_tests = [
    '{"t":123456,"uwb_x":1.23,"uwb_y":0.85,"uwb_d1":1.40,"uwb_d2":0.81,"uwb_d3":2.48,"uwb_d4":2.10,"uwb_t":123450,"imu_ax":0.01,"imu_ay":-0.02,"imu_az":9.81,"imu_gx":0.001,"imu_gy":-0.002,"imu_gz":0.015,"imu_t":123455,"enc_l":1234,"enc_r":1240,"imu_ok":true,"uwb_ok":true}',
]

for line in json_tests:
    print("JSON RAW:", line)
    print(receiver.parse_sensor_json_line(line))
    print()

# IMU disabled.
# imu_tests = [
#     "IMU,123.4,1.2,-0.5",
#     "YPR,125.0,1.0,-0.4",
# ]
#
# for line in imu_tests:
#     print("IMU RAW:", line)
#     print(receiver.parse_imu_line(line))
#     print()
