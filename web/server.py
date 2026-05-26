import asyncio
import base64
import csv
import datetime
import os
import sys
import threading
import time

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel

from config import APP_SETTINGS, COMM_SETTINGS, DRIVE_SETTINGS
from core.rf_localization import RFLocalizer, aps_to_rssi_vector


app = FastAPI(title="Realtime Radio Heatmap Mapper API")

current_dir = os.path.dirname(os.path.abspath(__file__))
templates = Jinja2Templates(directory=os.path.join(current_dir, "templates"))

CSV_OUTPUT_DIR = os.path.join(os.path.abspath(os.path.join(current_dir, os.pardir)), "saved_csv")
IMAGE_OUTPUT_DIR = os.path.join(os.path.abspath(os.path.join(current_dir, os.pardir)), "saved_heatmaps")
os.makedirs(CSV_OUTPUT_DIR, exist_ok=True)
os.makedirs(IMAGE_OUTPUT_DIR, exist_ok=True)

system_context = {
    "navigator": None,
    "scanner": None,
    "camera": None,
    "fingerprint_db": [],
    "rf_db": None,
}

connected_clients = []

def _number_or_none(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if np.isfinite(number) else None


def build_lidar_status(nav=None, lidar_data=None):
    nav = nav if nav is not None else system_context.get("navigator")
    lidar = getattr(nav, "lidar", None) if nav else None
    if lidar_data is None:
        try:
            lidar_data = lidar.snapshot() if lidar else {}
        except Exception as exc:
            lidar_data = {"lidar_error": str(exc)}

    lidar_data = lidar_data or {}
    obstacle = lidar_data.get("lidar_obstacle") or {}
    distance_m = _number_or_none(obstacle.get("distance_m"))
    angle_deg = _number_or_none(obstacle.get("angle_deg"))
    threshold_m = _number_or_none(obstacle.get("threshold_m"))
    connected = bool(lidar_data.get("lidar_connected"))
    obstacle_detected = bool(obstacle.get("obstacle"))
    nav_status = str(getattr(nav, "status", "") if nav else "")
    stopped_by_obstacle = obstacle_detected or nav_status.startswith(("LiDAR stop", "Diamond LiDAR stop"))

    configured_ports = COMM_SETTINGS.get("LIDAR_SERIAL_PORTS", [])
    configured_baudrates = COMM_SETTINGS.get("LIDAR_SERIAL_BAUDS", [])
    port = lidar_data.get("lidar_port") or (configured_ports[0] if configured_ports else None)
    baudrate = lidar_data.get("lidar_baud") or (configured_baudrates[0] if configured_baudrates else None)
    baudrate_number = _number_or_none(baudrate)

    return {
        "connected": connected,
        "connection_status": "연결됨" if connected else "연결 안 됨",
        "port": port,
        "baudrate": int(baudrate_number) if baudrate_number is not None else baudrate,
        "nearest_distance_cm": round(distance_m * 100.0, 1) if distance_m is not None else None,
        "nearest_angle_deg": round(angle_deg, 1) if angle_deg is not None else None,
        "obstacle_detected": obstacle_detected,
        "obstacle_status": "감지됨" if obstacle_detected else "감지 안 됨",
        "vehicle_status": "장애물 때문에 정지" if stopped_by_obstacle else "주행 가능",
        "stopped_by_obstacle": stopped_by_obstacle,
        "drive_enabled": not stopped_by_obstacle,
        "motor_action": "stop" if stopped_by_obstacle else "drive_enable",
        "error": lidar_data.get("lidar_error") or "",
        "last_seen": lidar_data.get("lidar_last_seen"),
        "point_count": lidar_data.get("lidar_point_count", 0),
        "fresh_points": obstacle.get("fresh_points", 0),
        "selected_points": obstacle.get("front_points", 0),
        "threshold_cm": round(threshold_m * 100.0, 1) if threshold_m is not None else None,
        "front_center_deg": obstacle.get(
            "center_deg",
            DRIVE_SETTINGS.get("LIDAR_OBSTACLE_FRONT_CENTER_DEG", 0.0),
        ),
        "front_width_deg": obstacle.get(
            "width_deg",
            DRIVE_SETTINGS.get("LIDAR_OBSTACLE_FRONT_WIDTH_DEG", 360.0),
        ),
        "raw": lidar_data,
    }


def build_ultrasonic_status(nav=None):
    nav = nav if nav is not None else system_context.get("navigator")
    ultrasonic = getattr(nav, "ultrasonic", None) if nav else None
    return {
        "enabled": DRIVE_SETTINGS.get("ULTRASONIC_ENABLE", False),
        "ready": bool(getattr(ultrasonic, "ready", False)),
        "threshold_cm": DRIVE_SETTINGS.get("ULTRASONIC_THRESHOLD_CM", 35),
        "distance_cm": getattr(nav, "_last_ultrasonic_distance_cm", None) if nav else None,
        "obstacle": bool(getattr(nav, "_ultrasonic_obstacle_active", False)) if nav else False,
        "error": getattr(ultrasonic, "error", "") if ultrasonic else "ultrasonic object missing",
    }


def _get_rf_db():
    rf_db = system_context.get("rf_db")
    if rf_db:
        return rf_db

    nav = system_context.get("navigator")
    rf_db = getattr(nav, "rf_db", None) if nav else None
    if rf_db:
        system_context["rf_db"] = rf_db
    return rf_db


def _rf_db_unavailable():
    return {"ok": False, "message": "RF Fingerprint DB is not available"}


def _waypoint_states_for_ui(nav):
    states = [dict(item) for item in getattr(nav, "waypoint_states", [])]
    if states:
        return states

    configured = DRIVE_SETTINGS.get("UNVISITED_WAYPOINTS") or []
    return [
        {
            "x": float(point[0]),
            "y": float(point[1]),
            "visited": False,
            "failed_count": 0,
            "state": "pending",
        }
        for point in configured
        if len(point) == 2
    ]


def build_state_payload(new_points=None):
    nav = system_context["navigator"]

    if not nav:
        return {
            "type": "state",
            "ready": False,
            "message": "System not ready",
        }

    drive_x, drive_y = nav.get_filtered_position()
    x, y = nav.get_display_position() if hasattr(nav, "get_display_position") else (drive_x, drive_y)

    uwb_data = {}
    if getattr(nav, "uwb", None):
        uwb_data = nav.uwb.snapshot()
        imu_seen = uwb_data.get("imu_last_seen_monotonic")
        if imu_seen is not None:
            try:
                uwb_data["imu_age_sec"] = round(max(0.0, time.monotonic() - float(imu_seen)), 2)
                uwb_data["imu_receiving"] = (
                    bool(uwb_data.get("imu_last_seen"))
                    and uwb_data["imu_age_sec"] <= DRIVE_SETTINGS.get(
                        "AUTO_ZIGZAG_PIVOT_IMU_STALE_SEC",
                        1.0,
                    )
                )
            except (TypeError, ValueError):
                uwb_data["imu_age_sec"] = None
                uwb_data["imu_receiving"] = False
        else:
            uwb_data["imu_age_sec"] = None
            uwb_data["imu_receiving"] = False

    # 화면은 부드러운 display 좌표를 우선 사용하고, 없으면 주행용 drive 좌표로 대체한다.
    if x is None:
        x = uwb_data.get("display_x", uwb_data.get("x", drive_x))

    if y is None:
        y = uwb_data.get("display_y", uwb_data.get("y", drive_y))

    left_speed, right_speed = None, None
    if hasattr(nav.robot, 'get_motor_speeds'):
        left_speed, right_speed = nav.robot.get_motor_speeds()

    motion_data = nav._motion_snapshot() if hasattr(nav, "_motion_snapshot") else {}
    rssi_power = nav._rssi_power_snapshot() if hasattr(nav, "_rssi_power_snapshot") else {}
    current_target = getattr(nav, "current_target", None)
    waypoint_states = _waypoint_states_for_ui(nav)
    lidar_data = nav.lidar.snapshot() if getattr(nav, "lidar", None) else {}
    lidar_status = build_lidar_status(nav, lidar_data)
    ultrasonic_status = build_ultrasonic_status(nav)

    return {
        "type": "state",
        "ready": True,
        "status": "running" if nav.is_running else "stopped",
        "nav_status": getattr(nav, "status", ""),
        "path_mode": getattr(nav, "path_mode", DRIVE_SETTINGS.get("AUTO_PATH_MODE", "")),
        "current_position": [round(x, 3), round(y, 3)] if x is not None and y is not None else None,
        "drive_position": [round(drive_x, 3), round(drive_y, 3)] if drive_x is not None and drive_y is not None else None,
        "current_target": current_target,
        "current_waypoint_index": getattr(nav, "current_waypoint_index", 0),
        "total_waypoints": getattr(nav, "total_waypoints", 0) or len(waypoint_states),
        "visited_count": getattr(nav, "visited_count", 0),
        "waypoint_states": waypoint_states,
        "target_distance": getattr(nav, "distance_to_target", None),
        "distance_to_target": getattr(nav, "distance_to_target", None),
        "target_bearing": getattr(nav, "target_bearing", None),
        "current_heading": getattr(nav, "current_heading", None),
        "heading_error": getattr(nav, "heading_error", None),
        "auto_timed_waypoint_enable": DRIVE_SETTINGS.get("AUTO_TIMED_WAYPOINT_ENABLE", False),
        "waypoint_arrived": getattr(nav, "last_waypoint_arrived", False),
        "waypoint_arrive_count": getattr(nav, "waypoint_arrive_count", 0),
        "position_clamped": getattr(nav, "last_position_clamped", False),
        "robot_ready": bool(getattr(nav, "robot", None)),

        "x": round(x, 3) if x is not None else None,
        "y": round(y, 3) if y is not None else None,

        "left_speed": round(left_speed, 1) if left_speed is not None else None,
        "right_speed": round(right_speed, 1) if right_speed is not None else None,

        "uwb_valid": bool(uwb_data.get("uwb_valid")),
        "uwb_receiving": bool(uwb_data.get("uwb_connected") or uwb_data.get("uwb_last_seen") or uwb_data.get("uwb_raw")),
        "uwb_connected": bool(uwb_data.get("uwb_connected")),
        "uwb_parse_ok": bool(uwb_data.get("uwb_parse_ok")),
        "uwb_error": uwb_data.get("uwb_error", ""),
        "uwb_anchor_count": uwb_data.get("uwb_anchor_count", 0),
        "uwb_last_seen": uwb_data.get("uwb_last_seen"),

        "d1": uwb_data.get("d1"),
        "d2": uwb_data.get("d2"),
        "d3": uwb_data.get("d3"),
        "d4": uwb_data.get("d4"),
        "rssi1": uwb_data.get("rssi1"),
        "rssi2": uwb_data.get("rssi2"),
        "rssi3": uwb_data.get("rssi3"),
        "rssi4": uwb_data.get("rssi4"),
        "uwb_rssi": uwb_data.get("uwb_rssi"),
        "uwb_rssi_count": uwb_data.get("uwb_rssi_count", 0),
        "uwb_distance_error_m": uwb_data.get("uwb_distance_error_m"),
        "uwb_distance_error_max_m": uwb_data.get("uwb_distance_error_max_m"),
        "uwb_position_outlier_count": uwb_data.get("uwb_position_outlier_count", 0),
        "uwb_position_outlier_reason": uwb_data.get("uwb_position_outlier_reason", ""),
        "esp32_wifi_count": uwb_data.get("esp32_wifi_count", 0),
        "esp32_wifi_last_seen": uwb_data.get("esp32_wifi_last_seen"),

        "uwb_raw": uwb_data.get("uwb_raw", ""),
        "motion": motion_data,
        "rssi_power": rssi_power,
        "ultrasonic": ultrasonic_status,
        "lidar": lidar_data,
        "lidar_status": lidar_status,
        # HTML의 updateUwbDebug()가 WebSocket과 /api/state 양쪽에서 같은 구조를 받도록 추가
        "uwb": uwb_data,
        "new_points": new_points or [],
        "points": list(system_context["fingerprint_db"]),
    }

class AutoMappingRequest(BaseModel):
    width: float = APP_SETTINGS["ROOM_WIDTH_M"]
    height: float = APP_SETTINGS["ROOM_HEIGHT_M"]


class ImuPivotTestRequest(BaseModel):
    direction: str = "right"
    degrees: float = 90.0


class CompactArcTestRequest(BaseModel):
    direction: str = "right"
    degrees: float = 165.0


def _calibrate_origin_with_retry(uwb, timeout_sec=1.0, map_x=0.0, map_y=0.0):
    deadline = time.monotonic() + max(0.0, timeout_sec)
    while time.monotonic() <= deadline:
        if uwb.calibrate_origin(map_x=map_x, map_y=map_y):
            return True
        time.sleep(0.05)
    return False


def _create_placeholder_frame(width=320, height=240, text="Camera unavailable"):
    image = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(
        image,
        text,
        (12, height // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    ret, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return buffer.tobytes() if ret else b""

async def generate_video_stream():
    placeholder_frame = _create_placeholder_frame()
    while True:
        camera = system_context["camera"]
        frame = None
        if camera:
            frame = camera.get_latest_frame()

        if frame is None:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + placeholder_frame + b"\r\n"
            )
        else:
            ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ret:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"
                )

        await asyncio.sleep(0.04)


@app.get("/api/video_feed")
async def video_feed():
    return StreamingResponse(
        generate_video_stream(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/api/video_frame")
async def video_frame():
    camera = system_context.get("camera")
    frame = None
    if camera:
        frame = camera.get_latest_frame()

    if frame is None:
        jpeg = _create_placeholder_frame()
        return Response(content=jpeg, media_type="image/jpeg")

    ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
    if not ret:
        return Response(content=_create_placeholder_frame(), media_type="image/jpeg")

    return Response(content=buffer.tobytes(), media_type="image/jpeg")


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.append(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        if websocket in connected_clients:
            connected_clients.remove(websocket)


async def broadcast_realtime_state():
    last_db_len = 0
    while True:
        if connected_clients:
            nav = system_context["navigator"]
            if nav:
                drive_x, drive_y = nav.get_filtered_position()
                x, y = nav.get_display_position() if hasattr(nav, "get_display_position") else (drive_x, drive_y)
                status = "running" if nav.is_running else "stopped"
                uwb_debug = {}
                if getattr(nav, "uwb", None):
                    uwb_debug = nav.uwb.snapshot()
                    imu_seen = uwb_debug.get("imu_last_seen_monotonic")
                    if imu_seen is not None:
                        try:
                            uwb_debug["imu_age_sec"] = round(max(0.0, time.monotonic() - float(imu_seen)), 2)
                            uwb_debug["imu_receiving"] = (
                                bool(uwb_debug.get("imu_last_seen"))
                                and uwb_debug["imu_age_sec"] <= DRIVE_SETTINGS.get(
                                    "AUTO_ZIGZAG_PIVOT_IMU_STALE_SEC",
                                    1.0,
                                )
                            )
                        except (TypeError, ValueError):
                            uwb_debug["imu_age_sec"] = None
                            uwb_debug["imu_receiving"] = False
                    else:
                        uwb_debug["imu_age_sec"] = None
                        uwb_debug["imu_receiving"] = False

                if x is None:
                    x = uwb_debug.get("display_x", uwb_debug.get("x", drive_x))
                if y is None:
                    y = uwb_debug.get("display_y", uwb_debug.get("y", drive_y))

                left_speed, right_speed = None, None
                if hasattr(nav.robot, 'get_motor_speeds'):
                    left_speed, right_speed = nav.robot.get_motor_speeds()
                motion_debug = nav._motion_snapshot() if hasattr(nav, "_motion_snapshot") else {}
                lidar_debug = nav.lidar.snapshot() if getattr(nav, "lidar", None) else {}
                lidar_status = build_lidar_status(nav, lidar_debug)
                ultrasonic_status = build_ultrasonic_status(nav)
                waypoint_states = _waypoint_states_for_ui(nav)

                current_db = system_context["fingerprint_db"]
                current_db_len = len(current_db)
                if current_db_len < last_db_len:
                    last_db_len = 0
                new_points = current_db[last_db_len:] if current_db_len > last_db_len else []
                last_db_len = current_db_len

                msg = {
                    "type": "state",
                    "status": status,
                    "nav_status": getattr(nav, "status", ""),
                    "path_mode": getattr(nav, "path_mode", DRIVE_SETTINGS.get("AUTO_PATH_MODE", "")),
                    "current_position": [round(x, 3), round(y, 3)] if x is not None and y is not None else None,
                    "drive_position": [round(drive_x, 3), round(drive_y, 3)] if drive_x is not None and drive_y is not None else None,
                    "current_target": getattr(nav, "current_target", None),
                    "current_waypoint_index": getattr(nav, "current_waypoint_index", 0),
                    "total_waypoints": getattr(nav, "total_waypoints", 0) or len(waypoint_states),
                    "visited_count": getattr(nav, "visited_count", 0),
                    "waypoint_states": waypoint_states,
                    "target_distance": getattr(nav, "distance_to_target", None),
                    "distance_to_target": getattr(nav, "distance_to_target", None),
                    "target_bearing": getattr(nav, "target_bearing", None),
                    "current_heading": getattr(nav, "current_heading", None),
                    "heading_error": getattr(nav, "heading_error", None),
                    "auto_timed_waypoint_enable": DRIVE_SETTINGS.get("AUTO_TIMED_WAYPOINT_ENABLE", False),
                    "waypoint_arrived": getattr(nav, "last_waypoint_arrived", False),
                    "waypoint_arrive_count": getattr(nav, "waypoint_arrive_count", 0),
                    "robot_ready": bool(getattr(nav, "robot", None)),
                    "x": round(x, 3) if x is not None else None,
                    "y": round(y, 3) if y is not None else None,
                    "left_speed": round(left_speed, 1) if left_speed is not None else None,
                    "right_speed": round(right_speed, 1) if right_speed is not None else None,
                    "uwb_valid": bool(uwb_debug.get("uwb_valid")),
                    "uwb_receiving": bool(uwb_debug.get("uwb_connected") or uwb_debug.get("uwb_last_seen") or uwb_debug.get("uwb_raw")),
                    "uwb_connected": bool(uwb_debug.get("uwb_connected")),
                    "uwb_parse_ok": bool(uwb_debug.get("uwb_parse_ok")),
                    "uwb_error": uwb_debug.get("uwb_error", ""),
                    "uwb_anchor_count": uwb_debug.get("uwb_anchor_count", 0),
                    "uwb_last_seen": uwb_debug.get("uwb_last_seen"),
                    "uwb_position_outlier_count": uwb_debug.get("uwb_position_outlier_count", 0),
                    "uwb_position_outlier_reason": uwb_debug.get("uwb_position_outlier_reason", ""),
                    "new_points": new_points,
                    "points": list(current_db),
                    "uwb": uwb_debug,
                    "motion": motion_debug,
                    "ultrasonic": ultrasonic_status,
                    "lidar": lidar_debug,
                    "lidar_status": lidar_status,
                }

                stale_clients = []
                for client in connected_clients:
                    try:
                        await client.send_json(msg)
                    except Exception:
                        stale_clients.append(client)

                for client in stale_clients:
                    if client in connected_clients:
                        connected_clients.remove(client)
        await asyncio.sleep(APP_SETTINGS.get("STATE_BROADCAST_INTERVAL_SEC", 0.05))


@app.on_event("startup")
async def startup_event():
    asyncio.create_task(broadcast_realtime_state())



@app.get("/api/state")
async def api_state():
    return build_state_payload()

@app.get("/api/lidar_status")
async def api_lidar_status():
    return build_lidar_status()


@app.get("/api/rf_db/summary")
async def rf_db_summary():
    rf_db = _get_rf_db()
    if not rf_db:
        return _rf_db_unavailable()

    try:
        rf_db.create_tables()
        conn = rf_db.connect()
        scan_count = conn.execute("SELECT COUNT(*) FROM scans").fetchone()[0]
        ap_count = conn.execute("SELECT COUNT(*) FROM access_points").fetchone()[0]
        measurement_count = conn.execute("SELECT COUNT(*) FROM measurements").fetchone()[0]
        fingerprint_grid_count = conn.execute(
            "SELECT COUNT(DISTINCT grid_id) FROM fingerprints"
        ).fetchone()[0]
        fingerprint_row_count = conn.execute("SELECT COUNT(*) FROM fingerprints").fetchone()[0]
        last_scan = conn.execute(
            """
            SELECT s.scan_id, s.timestamp, s.x, s.y, s.grid_id, COUNT(m.measurement_id) AS ap_count
            FROM scans s
            LEFT JOIN measurements m ON m.scan_id = s.scan_id
            GROUP BY s.scan_id
            ORDER BY s.scan_id DESC
            LIMIT 1
            """
        ).fetchone()
    except Exception as exc:
        return {"ok": False, "message": f"RF Fingerprint DB summary failed: {exc}"}

    return {
        "scan_count": scan_count,
        "ap_count": ap_count,
        "measurement_count": measurement_count,
        "fingerprint_grid_count": fingerprint_grid_count,
        "fingerprint_row_count": fingerprint_row_count,
        "last_scan": {
            "scan_id": last_scan[0],
            "timestamp": last_scan[1],
            "x": last_scan[2],
            "y": last_scan[3],
            "grid_id": last_scan[4],
            "ap_count": last_scan[5],
        } if last_scan else None,
    }


@app.get("/api/rf_db/fingerprints")
async def rf_db_fingerprints(limit: int = 100):
    rf_db = _get_rf_db()
    if not rf_db:
        return _rf_db_unavailable()

    try:
        limit = max(1, min(int(limit), 5000))
        rf_db.create_tables()
        conn = rf_db.connect()
        rows = conn.execute(
            """
            SELECT
                fingerprint_id, grid_id, floor, x_center, y_center,
                bssid, avg_rssi, min_rssi, max_rssi, std_rssi,
                sample_count, updated_at
            FROM fingerprints
            ORDER BY updated_at DESC, grid_id, bssid
            LIMIT ?
            """,
            (limit,),
        ).fetchall()
    except Exception as exc:
        return {"ok": False, "message": f"RF Fingerprint DB query failed: {exc}"}

    fingerprints = [
        {
            "fingerprint_id": row[0],
            "grid_id": row[1],
            "floor": row[2],
            "x_center": row[3],
            "y_center": row[4],
            "bssid": row[5],
            "avg_rssi": row[6],
            "min_rssi": row[7],
            "max_rssi": row[8],
            "std_rssi": row[9],
            "sample_count": row[10],
            "updated_at": row[11],
        }
        for row in rows
    ]
    return {"ok": True, "limit": limit, "count": len(fingerprints), "fingerprints": fingerprints}


@app.get("/api/rf_db/heatmap_data")
async def rf_db_heatmap_data(type: str = "strongest", bssid: str = ""):
    rf_db = _get_rf_db()
    if not rf_db:
        return _rf_db_unavailable()

    heatmap_type = str(type or "strongest").strip().lower()
    normalized_bssid = str(bssid or "").strip().upper()

    try:
        rf_db.create_tables()
        conn = rf_db.connect()

        if heatmap_type == "strongest":
            rows = conn.execute(
                """
                SELECT grid_id, x_center, y_center, MAX(avg_rssi) AS value
                FROM fingerprints
                WHERE avg_rssi IS NOT NULL
                GROUP BY grid_id, x_center, y_center
                ORDER BY grid_id
                """
            ).fetchall()
            points = [
                {
                    "grid_id": row[0],
                    "x": row[1],
                    "y": row[2],
                    "value": row[3],
                    "label": "strongest_rssi",
                }
                for row in rows
            ]

        elif heatmap_type == "ap":
            if not normalized_bssid:
                return {"ok": False, "message": "bssid is required for type=ap"}
            rows = conn.execute(
                """
                SELECT grid_id, x_center, y_center, avg_rssi
                FROM fingerprints
                WHERE UPPER(bssid) = ? AND avg_rssi IS NOT NULL
                ORDER BY grid_id
                """,
                (normalized_bssid,),
            ).fetchall()
            points = [
                {
                    "grid_id": row[0],
                    "x": row[1],
                    "y": row[2],
                    "value": row[3],
                    "label": normalized_bssid,
                }
                for row in rows
            ]

        elif heatmap_type == "ap_count":
            rows = conn.execute(
                """
                SELECT grid_id, x_center, y_center, COUNT(DISTINCT bssid) AS value
                FROM fingerprints
                WHERE bssid IS NOT NULL AND bssid != ''
                GROUP BY grid_id, x_center, y_center
                ORDER BY grid_id
                """
            ).fetchall()
            points = [
                {
                    "grid_id": row[0],
                    "x": row[1],
                    "y": row[2],
                    "value": row[3],
                    "label": "ap_count",
                }
                for row in rows
            ]

        elif heatmap_type == "weak":
            rows = conn.execute(
                """
                SELECT grid_id, x_center, y_center, MAX(avg_rssi) AS strongest_rssi
                FROM fingerprints
                WHERE avg_rssi IS NOT NULL
                GROUP BY grid_id, x_center, y_center
                ORDER BY grid_id
                """
            ).fetchall()
            points = [
                {
                    "grid_id": row[0],
                    "x": row[1],
                    "y": row[2],
                    "value": 1 if row[3] is not None and row[3] < -85.0 else 0,
                    "label": "weak_area",
                    "strongest_rssi": row[3],
                }
                for row in rows
            ]

        else:
            return {
                "ok": False,
                "message": "type must be one of: strongest, ap, ap_count, weak",
            }
    except Exception as exc:
        return {"ok": False, "message": f"RF heatmap data query failed: {exc}"}

    response = {"ok": True, "type": heatmap_type, "points": points}
    if heatmap_type == "ap":
        response["bssid"] = normalized_bssid
    return response


@app.get("/api/rf_db/locate")
async def rf_db_locate(k: int = 3):
    rf_db = _get_rf_db()
    scanner = system_context["scanner"]
    nav = system_context["navigator"]

    if not rf_db:
        return _rf_db_unavailable()
    if not scanner:
        return {"ok": False, "message": "Wi-Fi scanner is not available"}
    if nav and nav.is_running:
        return {
            "ok": False,
            "message": "주행 중 Wi-Fi 위치 추정 스캔은 차단됩니다. 정지 상태에서 실행하세요.",
        }

    try:
        rf_db.create_tables()
        grid_vectors = rf_db.get_all_grid_vectors()
    except Exception as exc:
        return {"ok": False, "message": f"RF fingerprint DB read failed: {exc}"}

    if not grid_vectors:
        return {"ok": False, "message": "DB fingerprint가 충분하지 않습니다. 먼저 RSSI 샘플을 저장하세요."}

    try:
        if nav and hasattr(nav, "capture_fingerprint_with_uwb"):
            fingerprint = nav.capture_fingerprint_with_uwb(scanner)
            aps = fingerprint.get("aps", [])
        else:
            aps = scanner.scan()
            fingerprint = {
                "aps": aps,
                "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            }
        current_vector = aps_to_rssi_vector(aps)
    except Exception as exc:
        return {"ok": False, "message": f"Current Wi-Fi scan failed: {exc}"}

    if not current_vector:
        return {"ok": False, "message": "현재 Wi-Fi scan에서 사용할 BSSID-RSSI 값이 없습니다."}

    try:
        localizer = RFLocalizer()
        result = localizer.estimate_location(current_vector, grid_vectors, k=k)
    except Exception as exc:
        return {"ok": False, "message": f"RF localization failed: {exc}"}

    return {
        "ok": True,
        "k": max(1, int(k or 1)),
        "current_vector": current_vector,
        "ap_count": len(current_vector),
        "grid_count": len(grid_vectors),
        "fingerprint": fingerprint,
        "result": result,
    }


@app.post("/api/rf_db/rebuild")
async def rf_db_rebuild():
    rf_db = _get_rf_db()
    if not rf_db:
        return _rf_db_unavailable()

    try:
        results = rf_db.rebuild_all_fingerprints()
    except Exception as exc:
        return {"ok": False, "message": f"RF Fingerprint DB rebuild failed: {exc}"}

    return {
        "ok": True,
        "message": "RF Fingerprint DB rebuilt",
        "grid_count": len(results),
        "updated_rows_by_grid": results,
        "updated_row_count": sum(results.values()),
    }


@app.get("/")
async def index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})


@app.get("/download_project")
async def download_project():
    project_root = os.path.abspath(os.path.join(current_dir, os.pardir))
    zip_path = os.path.join(project_root, "autonomous_heatmap_project_ready.zip")
    return FileResponse(
        zip_path,
        media_type="application/zip",
        filename="autonomous_heatmap_project_ready.zip",
    )


@app.get("/ws_backup")
async def ws_backup():
    nav = system_context["navigator"]
    if not nav:
        return {"status": "error", "message": "System not ready"}
    return nav.snapshot()


@app.post("/api/scan")
async def trigger_scan():
    scanner = system_context["scanner"]
    nav = system_context["navigator"]
    if not scanner or not nav:
        return {"status": "error", "message": "System not ready"}
    if nav.is_running:
        return {"status": "error", "message": "주행 중 수동 RSSI 스캔은 차단됩니다. 샘플링 지점에서만 동글을 켭니다."}

    current_x, current_y = nav.get_filtered_position()
    if hasattr(nav, "capture_fingerprint_with_uwb"):
        fingerprint = nav.capture_fingerprint_with_uwb(scanner)
    else:
        fingerprint = scanner.capture_fingerprint(current_x, current_y)
    system_context["fingerprint_db"].append(fingerprint)
    rf_db = _get_rf_db()
    scan_id = None
    if rf_db:
        try:
            scan_id = rf_db.save_fingerprint_record(fingerprint)
        except Exception as exc:
            print(f"Manual RF SQLite save failed: {exc}", flush=True)

    return {"status": "success", "data": fingerprint, "scan_id": scan_id}


@app.post("/api/calibrate_origin")
async def calibrate_origin():
    nav = system_context["navigator"]
    if not nav or not getattr(nav, "uwb", None):
        return {"status": "error", "message": "UWB is not ready"}

    if not hasattr(nav.uwb, "calibrate_origin"):
        return {"status": "error", "message": "UWB origin calibration is not supported"}

    if nav.uwb.calibrate_origin():
        return {"status": "success", "data": nav.uwb.snapshot()}
    return {"status": "error", "message": "No raw UWB coordinate available"}


@app.post("/api/imu_pivot_test")
async def imu_pivot_test(req_data: ImuPivotTestRequest):
    nav = system_context["navigator"]
    if not nav or not getattr(nav, "robot", None):
        return {"status": "error", "message": "Robot controller is not ready"}
    if nav.is_running:
        return {"status": "error", "message": "Robot is already running"}
    worker = threading.Thread(
        target=nav.run_imu_pivot_test,
        args=(req_data.direction, req_data.degrees),
        daemon=True,
    )
    worker.start()
    return {"status": "success", "message": "IMU pivot test started"}


@app.post("/api/compact_arc_test")
async def compact_arc_test(req_data: CompactArcTestRequest):
    nav = system_context["navigator"]
    if not nav or not getattr(nav, "robot", None):
        return {"status": "error", "message": "Robot controller is not ready"}
    if nav.is_running:
        return {"status": "error", "message": "Robot is already running"}
    worker = threading.Thread(
        target=nav.run_compact_arc_test,
        args=(req_data.direction, req_data.degrees),
        daemon=True,
    )
    worker.start()
    return {"status": "success", "message": "Compact arc test started"}


@app.post("/api/start_auto")
async def start_auto(req_data: AutoMappingRequest):
    nav = system_context["navigator"]
    scanner = system_context["scanner"]

    if nav and not getattr(nav, "robot", None):
        return {"status": "error", "message": "Robot controller is not ready"}

    if nav and not nav.is_running:
        if getattr(nav, "uwb", None):
            if hasattr(nav.uwb, "reset_position_status"):
                nav.uwb.reset_position_status()
            if DRIVE_SETTINGS.get("UWB_CALIBRATE_ORIGIN_ON_AUTO_START", True) and hasattr(nav.uwb, "calibrate_origin"):
                _calibrate_origin_with_retry(
                    nav.uwb,
                    map_x=DRIVE_SETTINGS.get("AUTO_START_POSE_X_M", 0.0),
                    map_y=DRIVE_SETTINGS.get("AUTO_START_POSE_Y_M", 0.0),
                )
            elif hasattr(nav.uwb, "clear_origin_calibration"):
                nav.uwb.clear_origin_calibration()

        worker = threading.Thread(
            target=nav.autonomous_drive_grid,
            args=(req_data.width, req_data.height, scanner, system_context["fingerprint_db"]),
            daemon=True,
        )
        worker.start()
        return {"status": "success", "message": "Autonomous mapping started"}

    return {"status": "error", "message": "Already running or system error"}


@app.get("/api/heatmap")
async def get_heatmap(band: str = "2.4"):
    filtered_db = []
    for record in system_context["fingerprint_db"]:
        filtered_aps = [ap for ap in record["aps"] if band in str(ap.get("band", ""))]
        if filtered_aps:
            filtered_db.append(
                {
                    "x": record["x"],
                    "y": record["y"],
                    "aps": filtered_aps,
                    "timestamp": record["timestamp"],
                }
            )
    return {"band": band, "data": filtered_db}


@app.post("/api/stop_auto")
async def stop_auto():
    nav = system_context["navigator"]
    if not nav:
        return {"status": "error", "message": "System not ready"}
    nav.stop()
    return {"status": "success", "message": "탐사를 중지합니다."}


@app.post("/api/emergency_stop")
async def emergency_stop():
    nav = system_context["navigator"]
    if not nav:
        return {"status": "error", "message": "System not ready"}

    errors = []
    try:
        nav.stop_requested = True
        nav.running = False
        nav.status = "Emergency stopped"
    except Exception as exc:
        errors.append(f"state: {exc}")

    try:
        robot = getattr(nav, "robot", None)
        if robot:
            robot.stop()
    except Exception as exc:
        errors.append(f"motor: {exc}")

    try:
        if hasattr(nav, "_force_rssi_dongle_off"):
            nav._force_rssi_dongle_off("emergency_stop")
    except Exception as exc:
        errors.append(f"wifi_power: {exc}")

    try:
        nav.stop()
        nav.status = "Emergency stopped"
    except Exception as exc:
        errors.append(f"nav_stop: {exc}")

    if errors:
        return {
            "status": "warning",
            "message": "비상 정지를 요청했지만 일부 처리에서 경고가 있습니다.",
            "errors": errors,
        }
    return {"status": "success", "message": "비상 정지: 모터를 즉시 정지했습니다."}


@app.post("/api/reset_data")
async def reset_data():
    system_context["fingerprint_db"].clear()
    nav = system_context["navigator"]
    if nav:
        with nav.lock:
            nav.points.clear()
        nav.current_target = None
        nav.current_waypoint_index = 0
        nav.total_waypoints = 0
        nav.visited_count = 0
        nav.waypoint_states = []
        nav.distance_to_target = None
        nav.target_bearing = None
        nav.heading_error = None
        nav.last_waypoint_arrived = False
        nav.waypoint_arrive_count = 0
        nav.last_position_clamped = False
    rf_db_cleared = False
    rf_db_message = ""
    rf_db = _get_rf_db()
    if rf_db:
        try:
            rf_db.clear_all()
            rf_db_cleared = True
        except Exception as exc:
            rf_db_message = str(exc)
            print(f"RF DB reset failed: {exc}", flush=True)

    message = "수집 데이터가 초기화되었습니다."
    if rf_db:
        message += " RF DB도 초기화되었습니다." if rf_db_cleared else " RF DB 초기화는 실패했습니다."
    return {
        "status": "success",
        "message": message,
        "rf_db_cleared": rf_db_cleared,
        "rf_db_message": rf_db_message,
    }


def _current_fingerprint_rows():
    rows = []
    for record in system_context["fingerprint_db"]:
        timestamp = record.get("timestamp")
        x = record.get("x")
        y = record.get("y")
        uwb = record.get("uwb") or {}
        motion = record.get("motion") or {}
        aps = record.get("aps", [])
        if not aps:
            aps = [{}]
        for ap in aps:
            interface = ap.get("interface", "")
            band = ap.get("band", "")
            wifi_type = "ESP32" if interface == "esp32" else "Dongle" if interface.startswith(("wlan", "wlx")) else "Current AP" if band else "Unknown"
            rows.append(
                {
                    "timestamp": timestamp,
                    "x": x,
                    "y": y,
                    "ssid": ap.get("ssid", ""),
                    "bssid": ap.get("bssid", ""),
                    "rssi": ap.get("rssi", ""),
                    "band": band,
                    "interface": interface,
                    "wifi_type": wifi_type,
                    "uwb_d1": uwb.get("d1", ""),
                    "uwb_d2": uwb.get("d2", ""),
                    "uwb_d3": uwb.get("d3", ""),
                    "uwb_d4": uwb.get("d4", ""),
                    "uwb_rssi1": uwb.get("rssi1", ""),
                    "uwb_rssi2": uwb.get("rssi2", ""),
                    "uwb_rssi3": uwb.get("rssi3", ""),
                    "uwb_rssi4": uwb.get("rssi4", ""),
                    "uwb_rssi_avg": uwb.get("rssi_avg", ""),
                    "uwb_anchor_count": uwb.get("anchor_count", ""),
                    "uwb_rssi_count": uwb.get("rssi_count", ""),
                    "uwb_distance_error_m": uwb.get("distance_error_m", ""),
                    "uwb_distance_error_max_m": uwb.get("distance_error_max_m", ""),
                    "uwb_raw": uwb.get("raw", ""),
                    "odometry_distance_m": motion.get("odometry_distance_m", ""),
                    "uwb_distance_m": motion.get("uwb_distance_m", ""),
                    "wheel_distance_error_m": motion.get("wheel_distance_error_m", ""),
                    "heading_drift_deg": motion.get("heading_drift_deg", ""),
                    "last_turn_error_deg": motion.get("last_turn_error_deg", ""),
                }
            )
    return rows


@app.get("/api/export_csv")
async def export_csv():
    if not system_context["fingerprint_db"]:
        return {"status": "error", "message": "저장할 지문 데이터가 없습니다."}

    filename = datetime.datetime.now().strftime("wifi_fingerprint_%Y%m%d_%H%M%S.csv")
    filepath = os.path.join(CSV_OUTPUT_DIR, filename)
    rows = _current_fingerprint_rows()

    with open(filepath, "w", newline="", encoding="utf-8-sig") as csvfile:
        writer = csv.DictWriter(
            csvfile,
            fieldnames=[
                "timestamp", "x", "y", "ssid", "bssid", "rssi", "band", "interface", "wifi_type",
                "uwb_d1", "uwb_d2", "uwb_d3", "uwb_d4",
                "uwb_rssi1", "uwb_rssi2", "uwb_rssi3", "uwb_rssi4", "uwb_rssi_avg",
                "uwb_anchor_count", "uwb_rssi_count",
                "uwb_distance_error_m", "uwb_distance_error_max_m", "uwb_raw",
                "odometry_distance_m", "uwb_distance_m", "wheel_distance_error_m",
                "heading_drift_deg", "last_turn_error_deg",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    return FileResponse(filepath, media_type="text/csv", filename=filename)


@app.get("/api/rf_db/export_measurements_csv")
async def rf_db_export_measurements_csv():
    rf_db = _get_rf_db()
    if not rf_db:
        return _rf_db_unavailable()

    filename = datetime.datetime.now().strftime("rf_measurements_%Y%m%d_%H%M%S.csv")
    filepath = os.path.join(CSV_OUTPUT_DIR, filename)
    fieldnames = [
        "timestamp", "x", "y", "floor", "grid_id",
        "bssid", "ssid", "rssi", "frequency", "channel", "band", "interface",
    ]

    try:
        rf_db.create_tables()
        rf_db.rebuild_all_fingerprints()
        conn = rf_db.connect()
        rows = conn.execute(
            """
            SELECT
                s.timestamp, s.x, s.y, s.floor, s.grid_id,
                m.bssid, ap.ssid, m.rssi, m.frequency, m.channel, m.band, m.interface
            FROM measurements m
            JOIN scans s ON s.scan_id = m.scan_id
            LEFT JOIN access_points ap ON ap.bssid = m.bssid
            ORDER BY s.scan_id, m.measurement_id
            """
        ).fetchall()

        with open(filepath, "w", newline="", encoding="utf-8-sig") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(dict(zip(fieldnames, row)))
    except Exception as exc:
        return {"ok": False, "message": f"RF measurements CSV export failed: {exc}"}

    return FileResponse(filepath, media_type="text/csv", filename=filename)


@app.get("/api/rf_db/export_fingerprints_csv")
async def rf_db_export_fingerprints_csv():
    rf_db = _get_rf_db()
    if not rf_db:
        return _rf_db_unavailable()

    filename = datetime.datetime.now().strftime("rf_fingerprints_%Y%m%d_%H%M%S.csv")
    filepath = os.path.join(CSV_OUTPUT_DIR, filename)
    fieldnames = [
        "timestamp", "x", "y", "ssid", "bssid", "avg_rssi", "std_rssi", "band",
    ]

    try:
        rf_db.create_tables()
        conn = rf_db.connect()
        rows = conn.execute(
            """
            SELECT
                f.updated_at AS timestamp,
                f.x_center AS x,
                f.y_center AS y,
                COALESCE(ap.ssid, '') AS ssid,
                f.bssid,
                f.avg_rssi,
                f.std_rssi,
                CASE
                    WHEN ap.last_frequency >= 5000 THEN '5GHz'
                    WHEN ap.last_frequency > 0 THEN '2.4GHz'
                    ELSE ''
                END AS band
            FROM fingerprints f
            LEFT JOIN access_points ap ON ap.bssid = f.bssid
            ORDER BY f.grid_id, f.bssid
            """
        ).fetchall()

        with open(filepath, "w", newline="", encoding="utf-8-sig") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(dict(zip(fieldnames, row)))
    except Exception as exc:
        return {"ok": False, "message": f"RF fingerprints CSV export failed: {exc}"}

    return FileResponse(filepath, media_type="text/csv", filename=filename)


@app.get("/api/csv_files")
async def csv_files():
    files = []
    for filename in sorted(os.listdir(CSV_OUTPUT_DIR), reverse=True):
        if filename.lower().endswith(".csv"):
            files.append(filename)
    return {"files": files, "folder": "saved_csv"}


@app.get("/saved_csv/{filename}")
async def download_saved_csv(filename: str):
    safe_name = os.path.basename(filename)
    filepath = os.path.join(CSV_OUTPUT_DIR, safe_name)
    if not os.path.exists(filepath):
        return {"status": "error", "message": "파일을 찾을 수 없습니다."}
    return FileResponse(filepath, media_type="text/csv", filename=safe_name)


@app.post("/api/save_heatmap_image")
async def save_heatmap_image(request: Request):
    payload = await request.json()
    image_data = payload.get("image_data")
    if not image_data or not image_data.startswith("data:image/png;base64,"):
        return {"status": "error", "message": "Invalid image data."}

    image_payload = image_data.split(",", 1)[1]
    filename = datetime.datetime.now().strftime("heatmap_%Y%m%d_%H%M%S.png")
    filepath = os.path.join(IMAGE_OUTPUT_DIR, filename)

    try:
        with open(filepath, "wb") as f:
            f.write(base64.b64decode(image_payload))
    except Exception as exc:
        return {"status": "error", "message": f"이미지 저장 중 오류: {exc}"}

    return {"status": "success", "filename": filename}


@app.get("/api/heatmap_images")
async def heatmap_images():
    files = []
    for filename in sorted(os.listdir(IMAGE_OUTPUT_DIR), reverse=True):
        if filename.lower().endswith(".png"):
            files.append(filename)
    return {"files": files, "folder": "saved_heatmaps"}


@app.get("/heatmap_images/{filename}")
async def download_heatmap_image(filename: str):
    safe_name = os.path.basename(filename)
    filepath = os.path.join(IMAGE_OUTPUT_DIR, safe_name)
    if not os.path.exists(filepath):
        return {"status": "error", "message": "파일을 찾을 수 없습니다."}
    return FileResponse(filepath, media_type="image/png", filename=safe_name)


@app.get("/api/wifi_scan")
async def wifi_scan():
    scanner = system_context["scanner"]
    nav = system_context["navigator"]
    if not scanner or not nav:
        return {"ok": False, "aps": []}
    if nav.is_running:
        return {
            "ok": False,
            "message": "주행 중 Wi-Fi 동글 스캔은 차단됩니다. RSSI 샘플링 단계에서만 동글을 켭니다.",
        }

    x, y = nav.get_filtered_position()
    return {"ok": True, "fingerprint": nav.capture_fingerprint_with_uwb(scanner, x=x, y=y)}


@app.get("/api/net_status")
async def net_status():
    scanner = system_context["scanner"]
    if not scanner:
        return {"ok": False, "message": "Wi-Fi scanner is not available"}

    try:
        status = scanner.get_network_status(log=True) if hasattr(scanner, "get_network_status") else {}
        power = scanner.get_power_status() if hasattr(scanner, "get_power_status") else {}
    except Exception as exc:
        return {"ok": False, "message": f"Network status unavailable: {exc}"}

    return {
        "ok": True,
        "network": status,
        "power": power,
    }


def start_web_server(navigator, scanner, camera=None):
    system_context["navigator"] = navigator
    system_context["scanner"] = scanner
    system_context["camera"] = camera
    system_context["rf_db"] = getattr(navigator, "rf_db", None) if navigator else None
    if scanner and hasattr(scanner, "get_network_status"):
        try:
            net = scanner.get_network_status(log=True)
            print(f"[WiFi] web startup all interfaces: {net.get('all_wifi_interfaces', [])}", flush=True)
            print(f"[WiFi] web startup connected: {net.get('connected_interfaces', [])}", flush=True)
            print(f"[WiFi] web startup scan interfaces: {net.get('scan_interfaces', [])}", flush=True)
            print(f"[WiFi] web startup excluded: {net.get('excluded_interfaces', [])}", flush=True)
        except Exception as exc:
            print(f"[WiFi] web startup net status unavailable: {exc}", flush=True)

    host = APP_SETTINGS["WEB_HOST"]
    port = APP_SETTINGS["WEB_PORT"]
    access_host = "127.0.0.1" if host == "0.0.0.0" else host
    print(f"Web server: http://{access_host}:{port}", flush=True)
    if host == "0.0.0.0":
        print(f"Web server bind: http://{host}:{port}", flush=True)
    try:
        uvicorn.run(app, host=host, port=port, log_level="info")
    except OSError as exc:
        print(f"Failed to start web server: {exc}", flush=True)
        sys.exit(1)
