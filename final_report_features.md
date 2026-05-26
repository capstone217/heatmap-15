# 최종보고서용 시스템 기능 정리

## 1. 시스템 개요

본 프로젝트는 Raspberry Pi 기반 자율주행 로봇을 이용해 실내 공간의 Wi-Fi RSSI를 위치별로 수집하고, RF Fingerprinting 데이터베이스와 RSSI Heatmap을 생성하는 시스템이다.

로봇은 UWB로 현재 좌표를 추정하고, USB Wi-Fi 동글로 주변 AP의 SSID/BSSID/RSSI를 측정한다. 측정된 데이터는 위치 좌표와 함께 저장되며, 웹 대시보드에서 실시간 히트맵, 로봇 위치, 측정 데이터, RF Fingerprint DB 상태를 확인할 수 있다.

## 2. 주요 하드웨어

| 장치 | 역할 |
|---|---|
| Raspberry Pi 4 | 전체 제어, 웹 서버, 센서 데이터 통합 |
| SunFounder PiCar-X | 로봇 주행 플랫폼 |
| UWB 모듈 | 로봇의 실내 x, y 좌표 추정 |
| RPLIDAR | 장애물 감지 및 회피 방향 판단 |
| USB Wi-Fi 동글 | 주변 AP RSSI 스캔 |
| Camera | 웹 대시보드 실시간 영상 표시 |
| IMU | 회전/heading 보조 정보 수집 |

## 3. 자율주행 방식

현재 기본 주행은 고정 지그재그 경로가 아니라 `coverage wander` 방식이다.

로봇은 정해진 좌표를 정밀하게 따라가기보다, 실내 공간을 주행하면서 UWB 좌표가 미리 정의된 웨이포인트 반경 안에 들어오면 해당 지점을 방문 처리한다. 이 방식은 타겟 좌표를 정확히 찍는 것보다 넓은 영역을 안정적으로 커버하면서 RSSI 데이터를 수집하는 데 목적이 있다.

### 사용 중인 웨이포인트

3m x 3m 공간 기준 9개 지점을 사용한다.

| 번호 | 좌표 (x, y) |
|---|---|
| 1 | (0.5, 0.5) |
| 2 | (1.5, 0.5) |
| 3 | (2.5, 0.5) |
| 4 | (0.5, 1.5) |
| 5 | (1.5, 1.5) |
| 6 | (2.5, 1.5) |
| 7 | (0.5, 2.5) |
| 8 | (1.5, 2.5) |
| 9 | (2.5, 2.5) |

### 방문 판정

- 현재 UWB 좌표와 웨이포인트 간 거리를 계산한다.
- 거리가 `TARGET_REACHED_RADIUS_M` 이하이면 방문 처리한다.
- 현재 설정값은 약 `0.50 m`이다.
- 한 번 방문한 웨이포인트는 다시 방문 대상으로 사용하지 않는다.

### Coverage Wander 동작

- 기본적으로 전진한다.
- LiDAR가 장애물을 감지하면 회피한다.
- 주행 중 UWB 위치가 웨이포인트 반경 안에 들어오면 방문 처리한다.
- 일정 시간 새 웨이포인트 방문이 없으면 LiDAR 기준으로 더 열린 방향으로 재정렬한다.
- 모든 웨이포인트를 방문하면 탐사를 종료한다.

## 4. UWB 위치 추정 및 필터

UWB는 로봇의 실내 위치를 추정하는 핵심 센서이다. 단일 AP의 위치를 찾는 용도가 아니라, 로봇이 RSSI를 측정한 위치 좌표를 제공하는 역할이다.

### 사용 기능

- UWB 앵커 거리 기반 위치 추정
- 최소 앵커 수 검사
- 잘못된 좌표에 대한 유효성 검사
- 좌표 low-pass filtering
- 표시용 좌표 smoothing
- 최근 좌표 평균 기반 안정화

### 주요 설정

| 설정 | 의미 |
|---|---|
| `MIN_UWB_ANCHORS_FOR_POSITION` | 위치 계산에 필요한 최소 앵커 수 |
| `UWB_POSITION_LOWPASS_ENABLE` | UWB 좌표 저역통과 필터 사용 |
| `UWB_POSITION_LOWPASS_ALPHA` | 주행용 좌표 smoothing 계수 |
| `UWB_DISPLAY_POSITION_LOWPASS_ALPHA` | 화면 표시용 좌표 smoothing 계수 |
| `UWB_AVG_WINDOW` | 최근 UWB 좌표 평균에 사용할 샘플 수 |
| `UWB_MAX_VALID_DISTANCE_M` | 유효 거리 상한 |
| `UWB_GEOFENCE_ENABLE` | 지도 영역 기준 좌표 검사 |

### Low-pass filter 목적

UWB 좌표는 순간적으로 튀는 값이 발생할 수 있으므로, 저역통과 필터를 적용해 위치 변화가 급격하게 튀지 않도록 한다. 이를 통해 로봇 위치 표시와 웨이포인트 방문 판정이 안정적으로 동작한다.

## 5. LiDAR 장애물 감지 및 회피

LiDAR는 주행 중 충돌 방지와 장애물 회피에 사용한다.

### 장애물 감지

- RPLIDAR를 `/dev/ttyUSB1`, `115200 baud`로 연결한다.
- `iter_measures()` 방식으로 거리와 각도 데이터를 읽는다.
- 현재는 0~360도 전체에서 가장 가까운 물체를 기준으로 장애물을 판단한다.
- 장애물 임계거리는 `0.30 m`이다.

### 회피 동작

장애물이 감지되면 다음 순서로 회피한다.

1. 즉시 정지
2. 짧게 후진
3. LiDAR로 왼쪽/오른쪽 열린 공간 비교
4. 더 열린 방향으로 회전
5. 다시 주행 재개

### 주요 설정

| 설정 | 의미 |
|---|---|
| `LIDAR_ENABLE` | LiDAR 사용 여부 |
| `LIDAR_OBSTACLE_ENABLE` | 장애물 감지 사용 여부 |
| `LIDAR_OBSTACLE_DISTANCE_M` | 장애물 감지 거리 |
| `LIDAR_OBSTACLE_FRONT_WIDTH_DEG` | 감지 각도 범위 |
| `LIDAR_AVOIDANCE_ENABLE` | 장애물 회피 사용 여부 |
| `LIDAR_AVOIDANCE_LEFT_CENTER_DEG` | 왼쪽 회피 영역 중심각 |
| `LIDAR_AVOIDANCE_RIGHT_CENTER_DEG` | 오른쪽 회피 영역 중심각 |
| `LIDAR_AVOIDANCE_SECTOR_WIDTH_DEG` | 회피 방향 판단 섹터 폭 |

## 6. LiDAR와 Wi-Fi 동글 전원 배타 제어

본 시스템에서 가장 중요한 안전 조건 중 하나는 LiDAR와 Wi-Fi 동글이 동시에 동작하지 않도록 하는 것이다. 두 장치가 동시에 동작하면 전력 피크가 커져 Raspberry Pi 연결이 끊길 수 있기 때문이다.

### 전원 정책

- 주행 중에는 Wi-Fi 동글 스캔을 하지 않는다.
- RSSI 샘플링 단계에서는 먼저 LiDAR를 정지한다.
- LiDAR 정지 후 안정화 시간을 기다린다.
- 그 다음 Wi-Fi 동글을 켜고 RSSI를 측정한다.
- RSSI 측정이 끝나면 Wi-Fi 동글을 끈다.
- guard time 후 LiDAR를 다시 시작한다.

### 주요 설정

| 설정 | 의미 |
|---|---|
| `RSSI_POWER_EXCLUSIVE_LIDAR_WIFI` | LiDAR와 Wi-Fi 동글 동시 사용 금지 |
| `RSSI_LIDAR_STOP_SETTLE_SEC` | LiDAR 정지 후 대기 시간 |
| `RSSI_WIFI_TO_LIDAR_GUARD_SEC` | Wi-Fi 종료 후 LiDAR 재시작 전 대기 시간 |
| `RSSI_RESTART_LIDAR_AFTER_SCAN` | 샘플링 후 LiDAR 재시작 |
| `RSSI_LOCAL_WIFI_DOWN_AFTER_SCAN` | 샘플링 후 Wi-Fi 동글 비활성화 |

## 7. Wi-Fi RSSI 스캔

Wi-Fi 스캔은 RF Fingerprinting 데이터를 만들기 위한 핵심 기능이다.

### 스캔 대상

라즈베리파이 내장 Wi-Fi는 SSH와 노트북 핫스팟 연결 유지에 사용한다. 따라서 RSSI 스캔은 USB Wi-Fi 동글만 사용하도록 분리한다.

### 사용 정보

각 AP에 대해 다음 정보를 저장한다.

- SSID
- BSSID/MAC
- RSSI
- Frequency
- Channel
- Band
- Interface
- Timestamp

### 인터페이스 분리

| 설정 | 의미 |
|---|---|
| `WIFI_INTERFACE` | 기본 스캔 인터페이스 |
| `WIFI_SCAN_INTERFACES` | 수동 지정 스캔 인터페이스 목록 |
| `WIFI_SCAN_ALL_INTERFACES` | 전체 인터페이스 스캔 여부 |
| `WIFI_AUTO_DISCOVER_USB_DONGLE` | USB 동글 자동 탐색 |
| `WIFI_EXCLUDE_CONNECTED_INTERFACES` | 연결 중인 인터페이스 스캔 제외 |

현재 구조에서는 `wlan0`처럼 핫스팟 연결에 사용 중인 인터페이스를 스캔하지 않고, `wlan1` 또는 `wlx...` 형태의 USB Wi-Fi 동글만 스캔 대상으로 사용한다.

## 8. RSSI 샘플링 방식

로봇은 계속 Wi-Fi를 스캔하지 않고, 일정 주행 후 짧게 정지하여 RSSI를 샘플링한다.

### 현재 샘플링 설정

| 설정 | 값 | 의미 |
|---|---:|---|
| `AUTO_STOP_GO_RUN_SEC` | 5.0초 | 샘플링 사이 주행 시간 |
| `AUTO_SCAN_DWELL_SEC` | 1.5초 | RSSI 샘플링 대기 시간 |
| `AUTO_RSSI_AVG_COUNT` | 3개 | 최근 RSSI 샘플 평균 개수 |

### 샘플링 제외 조건

장애물 회피 중에는 RSSI 샘플링을 하지 않는다. 회피 중 측정된 좌표는 벽 근처에서 반복되거나 불안정할 수 있기 때문이다.

| 설정 | 의미 |
|---|---|
| `AUTO_RSSI_SKIP_DURING_LIDAR_OBSTACLE` | LiDAR 회피 중 RSSI 샘플링 생략 |
| `AUTO_RSSI_SKIP_AFTER_OBSTACLE_SEC` | 장애물 회피 후 일정 시간 샘플링 지연 |

## 9. RF Fingerprinting DB

RF Fingerprinting DB는 단순히 RSSI 하나를 저장하는 것이 아니라, 각 위치에서 관측된 여러 AP의 BSSID-RSSI 조합을 저장한다.

### Fingerprint 구조

하나의 측정 위치는 다음과 같은 형태를 가진다.

```json
{
  "x": 1.0,
  "y": 2.0,
  "aps": [
    {
      "ssid": "Example_AP",
      "bssid": "AA:BB:CC:11:22:33",
      "rssi": -45,
      "band": "2.4GHz"
    }
  ],
  "timestamp": "..."
}
```

즉 AP의 실제 물리 위치를 추정하는 것이 아니라, 로봇이 특정 좌표에서 관측한 AP별 RSSI 조합을 저장한다.

### SQLite DB 테이블

| 테이블 | 역할 |
|---|---|
| `scans` | 한 번의 위치 측정 기록 |
| `access_points` | 관측된 AP 정보 |
| `measurements` | scan별 AP RSSI raw 데이터 |
| `fingerprints` | grid별 BSSID 평균 RSSI 통계 |

### Grid ID

좌표는 일정 grid 크기로 나누어 관리한다.

예:

```text
x=1.25, y=0.82, grid_size=0.3
grid_x=4, grid_y=2
grid_id=F1_4_2
```

### Fingerprint 통계

Grid별, BSSID별로 다음 통계를 계산한다.

- 평균 RSSI
- 최소 RSSI
- 최대 RSSI
- 표준편차
- 샘플 수

## 10. Wi-Fi Fingerprint 기반 위치 추정

저장된 fingerprint DB를 이용해 현재 Wi-Fi scan 결과와 가장 유사한 grid를 찾는 위치 추정 기능이 있다.

### 방식

- 현재 측정된 `{BSSID: RSSI}` 벡터를 만든다.
- DB에 저장된 각 grid의 fingerprint vector와 비교한다.
- 현재 scan에 없는 AP 또는 DB에 없는 AP는 `-100 dBm`으로 처리한다.
- Euclidean distance 기반 k-NN 방식으로 가장 유사한 grid를 찾는다.

### 특징

- AP의 물리적 위치를 계산하지 않는다.
- “현재 RSSI 조합이 과거 어느 위치의 RSSI 조합과 가장 비슷한가”를 찾는다.

## 11. 웹 대시보드 기능

웹 대시보드는 FastAPI 서버와 WebSocket/API를 통해 실시간 정보를 표시한다.

### 표시 정보

- 로봇 상태
- UWB 좌표
- LiDAR 상태
- Wi-Fi 스캔 상태
- 카메라 영상
- RSSI Heatmap
- Contour 등고선
- 측정 지점
- 현재 로봇 위치
- RF DB 요약
- 최근 측정 데이터
- CSV 다운로드
- Fingerprint DB 다운로드

### 주요 API

| API | 기능 |
|---|---|
| `/api/state` | 현재 시스템 상태 조회 |
| `/ws` | 실시간 상태 WebSocket |
| `/api/start_auto` | 자율 탐사 시작 |
| `/api/stop_auto` | 자율 탐사 정지 |
| `/api/reset_data` | 측정 데이터 및 RF DB 초기화 |
| `/api/calibrate_origin` | 현재 UWB 위치 기준점 설정 |
| `/api/video_feed` | 카메라 스트리밍 |
| `/api/export_csv` | 메모리 기반 CSV export |
| `/api/rf_db/export_fingerprints_csv` | RF Fingerprint DB CSV export |
| `/api/rf_db/summary` | RF DB 요약 |
| `/api/rf_db/locate` | Wi-Fi fingerprint 기반 위치 추정 |

## 12. 안전 및 예외 처리

### 주행 안전

- LiDAR 장애물 감지 시 즉시 정지
- 열린 방향 판단 후 회피
- 탐사 정지 버튼으로 주행 중단
- 비상 정지 버튼은 현재 탐사 정지 API와 연결되어 모터 정지를 수행

### 네트워크 안정성

- 노트북 핫스팟 연결용 Wi-Fi 인터페이스와 RSSI 스캔용 Wi-Fi 동글을 분리
- 연결 중인 Wi-Fi 인터페이스는 스캔 대상에서 제외
- USB 동글이 없어도 전체 웹 서버가 죽지 않고 스캔 불가 상태로 유지

### 전력 안정성

- LiDAR와 Wi-Fi 동글 동시 동작 방지
- RSSI 샘플링 전후 guard time 적용
- 샘플링 후 Wi-Fi 동글 비활성화

## 13. 보고서에 강조할 핵심 차별점

1. UWB 기반 위치 좌표와 Wi-Fi RSSI를 결합한 RF Fingerprinting DB 구축
2. AP 위치 추정이 아니라 로봇 측정 위치 기반 fingerprint 수집
3. LiDAR 기반 장애물 감지 및 회피
4. LiDAR와 Wi-Fi 동글 전력 배타 제어
5. 연결 인터페이스와 스캔 인터페이스 분리로 SSH/핫스팟 연결 안정성 확보
6. SQLite 기반 raw measurement 및 grid fingerprint 통계 저장
7. 웹 대시보드에서 실시간 히트맵, 카메라, 상태, CSV export 제공

