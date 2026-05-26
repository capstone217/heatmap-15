import math
import os
import sqlite3
import threading
import time

try:
    from config import APP_SETTINGS, DRIVE_SETTINGS
except Exception:
    APP_SETTINGS = {}
    DRIVE_SETTINGS = {}

try:
    from core.rf_fingerprint import grid_center_from_id, xy_to_grid_id
except ModuleNotFoundError:
    from rf_fingerprint import grid_center_from_id, xy_to_grid_id


DEFAULT_DB_PATH = "saved_csv/rf_fingerprint.db"
DEFAULT_GRID_SIZE_M = 0.3


class RFFingerprintDB:
    def __init__(self, db_path=None, grid_size=None):
        self.db_path = db_path or APP_SETTINGS.get("RF_FINGERPRINT_DB_PATH", DEFAULT_DB_PATH)
        self.grid_size = self._resolve_grid_size(grid_size)
        self.conn = None
        self.lock = threading.RLock()
        self.last_grid_id = None

    def connect(self):
        with self.lock:
            if self.conn is not None:
                return self.conn

            db_dir = os.path.dirname(os.path.abspath(self.db_path))
            if db_dir:
                os.makedirs(db_dir, exist_ok=True)

            self.conn = sqlite3.connect(self.db_path, timeout=10.0, check_same_thread=False)
            self.conn.execute("PRAGMA foreign_keys = ON")
            self.conn.execute("PRAGMA busy_timeout = 10000")
            return self.conn

    def create_tables(self):
        with self.lock:
            conn = self.connect()
            conn.executescript(
                """
                CREATE TABLE IF NOT EXISTS scans (
                    scan_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT,
                    x REAL,
                    y REAL,
                    floor TEXT,
                    grid_id TEXT
                );

                CREATE TABLE IF NOT EXISTS access_points (
                    bssid TEXT PRIMARY KEY,
                    ssid TEXT,
                    last_frequency INTEGER,
                    last_channel INTEGER,
                    first_seen TEXT,
                    last_seen TEXT
                );

                CREATE TABLE IF NOT EXISTS measurements (
                    measurement_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    scan_id INTEGER,
                    bssid TEXT,
                    rssi REAL,
                    frequency INTEGER,
                    channel INTEGER,
                    band TEXT,
                    interface TEXT,
                    FOREIGN KEY(scan_id) REFERENCES scans(scan_id),
                    FOREIGN KEY(bssid) REFERENCES access_points(bssid)
                );

                CREATE TABLE IF NOT EXISTS fingerprints (
                    fingerprint_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    grid_id TEXT,
                    floor TEXT,
                    x_center REAL,
                    y_center REAL,
                    bssid TEXT,
                    avg_rssi REAL,
                    min_rssi REAL,
                    max_rssi REAL,
                    std_rssi REAL,
                    sample_count INTEGER,
                    updated_at TEXT,
                    UNIQUE(grid_id, bssid)
                );
                """
            )
            conn.commit()

    def insert_scan(self, timestamp, x, y, floor, grid_id):
        with self.lock:
            conn = self.connect()
            cursor = conn.execute(
                """
                INSERT INTO scans (timestamp, x, y, floor, grid_id)
                VALUES (?, ?, ?, ?, ?)
                """,
                (timestamp, self._float_or_none(x), self._float_or_none(y), floor, grid_id),
            )
            conn.commit()
            return cursor.lastrowid

    def upsert_access_point(self, ap):
        with self.lock:
            conn = self.connect()
            timestamp = self._ap_timestamp(ap)
            bssid = self._normalize_bssid(ap.get("bssid") or ap.get("mac"))
            if not bssid:
                return None

            conn.execute(
                """
                INSERT INTO access_points (
                    bssid, ssid, last_frequency, last_channel, first_seen, last_seen
                )
                VALUES (?, ?, ?, ?, ?, ?)
                ON CONFLICT(bssid) DO UPDATE SET
                    ssid = excluded.ssid,
                    last_frequency = excluded.last_frequency,
                    last_channel = excluded.last_channel,
                    last_seen = excluded.last_seen
                """,
                (
                    bssid,
                    str(ap.get("ssid", "") or ""),
                    self._int_or_none(ap.get("frequency", ap.get("frequency_mhz"))),
                    self._int_or_none(ap.get("channel")),
                    timestamp,
                    timestamp,
                ),
            )
            conn.commit()
            return bssid

    def insert_measurement(self, scan_id, ap):
        with self.lock:
            bssid = self.upsert_access_point(ap)
            if not bssid:
                return None

            conn = self.connect()
            cursor = conn.execute(
                """
                INSERT INTO measurements (
                    scan_id, bssid, rssi, frequency, channel, band, interface
                )
                VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    scan_id,
                    bssid,
                    self._float_or_none(ap.get("rssi")),
                    self._int_or_none(ap.get("frequency", ap.get("frequency_mhz"))),
                    self._int_or_none(ap.get("channel")),
                    str(ap.get("band", "") or ""),
                    str(ap.get("interface", "") or ""),
                ),
            )
            conn.commit()
            return cursor.lastrowid

    def save_fingerprint_record(self, record, floor="F1"):
        with self.lock:
            self.create_tables()

            x = record.get("x")
            y = record.get("y")
            timestamp = record.get("timestamp") or time.strftime("%Y-%m-%d %H:%M:%S")
            grid_id = record.get("grid_id")
            if not grid_id:
                try:
                    grid_id = xy_to_grid_id(x, y, grid_size=self.grid_size, floor=floor)
                except Exception:
                    grid_id = None
            self.last_grid_id = grid_id

            scan_id = self.insert_scan(timestamp, x, y, floor, grid_id)
            for ap in record.get("aps") or []:
                self.insert_measurement(scan_id, ap)

            if grid_id:
                self.update_grid_fingerprints(grid_id)
            return scan_id

    def clear_all(self):
        with self.lock:
            self.create_tables()
            conn = self.connect()
            conn.executescript(
                """
                DELETE FROM measurements;
                DELETE FROM fingerprints;
                DELETE FROM scans;
                DELETE FROM access_points;
                DELETE FROM sqlite_sequence
                WHERE name IN ('measurements', 'fingerprints', 'scans');
                """
            )
            conn.commit()
            self.last_grid_id = None

    def update_grid_fingerprints(self, grid_id):
        self.create_tables()
        conn = self.connect()
        rows = conn.execute(
            """
            SELECT s.floor, m.bssid, m.rssi
            FROM scans s
            JOIN measurements m ON m.scan_id = s.scan_id
            WHERE s.grid_id = ?
              AND m.bssid IS NOT NULL
              AND m.bssid != ''
              AND m.rssi IS NOT NULL
            """,
            (grid_id,),
        ).fetchall()
        if not rows:
            return 0

        grouped = {}
        for _floor, bssid, rssi in rows:
            normalized_bssid = self._normalize_bssid(bssid)
            rssi_value = self._float_or_none(rssi)
            if not normalized_bssid or rssi_value is None:
                continue
            grouped.setdefault(normalized_bssid, []).append(rssi_value)

        if not grouped:
            return 0

        grid_center = grid_center_from_id(grid_id, self.grid_size)
        floor = grid_center["floor"]
        updated_at = time.strftime("%Y-%m-%d %H:%M:%S")

        for bssid, rssis in grouped.items():
            sample_count = len(rssis)
            avg_rssi = sum(rssis) / sample_count
            min_rssi = min(rssis)
            max_rssi = max(rssis)
            variance = sum((value - avg_rssi) ** 2 for value in rssis) / sample_count
            std_rssi = math.sqrt(variance)

            conn.execute(
                """
                INSERT INTO fingerprints (
                    grid_id, floor, x_center, y_center, bssid,
                    avg_rssi, min_rssi, max_rssi, std_rssi,
                    sample_count, updated_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(grid_id, bssid) DO UPDATE SET
                    floor = excluded.floor,
                    x_center = excluded.x_center,
                    y_center = excluded.y_center,
                    avg_rssi = excluded.avg_rssi,
                    min_rssi = excluded.min_rssi,
                    max_rssi = excluded.max_rssi,
                    std_rssi = excluded.std_rssi,
                    sample_count = excluded.sample_count,
                    updated_at = excluded.updated_at
                """,
                (
                    grid_id,
                    floor,
                    grid_center["x_center"],
                    grid_center["y_center"],
                    bssid,
                    avg_rssi,
                    min_rssi,
                    max_rssi,
                    std_rssi,
                    sample_count,
                    updated_at,
                ),
            )

        conn.commit()
        return len(grouped)

    def rebuild_all_fingerprints(self):
        self.create_tables()
        conn = self.connect()
        scan_rows = conn.execute(
            """
            SELECT scan_id, x, y, floor
            FROM scans
            WHERE x IS NOT NULL AND y IS NOT NULL
            """
        ).fetchall()
        for scan_id, x, y, floor in scan_rows:
            try:
                grid_id = xy_to_grid_id(x, y, grid_size=self.grid_size, floor=floor or "F1")
            except Exception:
                grid_id = None
            conn.execute(
                "UPDATE scans SET grid_id = ? WHERE scan_id = ?",
                (grid_id, scan_id),
            )
        conn.execute("DELETE FROM fingerprints")
        conn.commit()

        rows = conn.execute(
            """
            SELECT DISTINCT grid_id
            FROM scans
            WHERE grid_id IS NOT NULL AND grid_id != ''
            ORDER BY grid_id
            """
        ).fetchall()

        results = {}
        for (grid_id,) in rows:
            results[grid_id] = self.update_grid_fingerprints(grid_id)
        return results

    def get_fingerprint_vector(self, grid_id):
        self.create_tables()
        conn = self.connect()
        rows = conn.execute(
            """
            SELECT bssid, avg_rssi
            FROM fingerprints
            WHERE grid_id = ?
            ORDER BY bssid
            """,
            (grid_id,),
        ).fetchall()
        return {
            self._normalize_bssid(bssid): avg_rssi
            for bssid, avg_rssi in rows
            if bssid and avg_rssi is not None
        }

    def get_all_grid_vectors(self):
        self.create_tables()
        conn = self.connect()
        rows = conn.execute(
            """
            SELECT grid_id, floor, x_center, y_center, bssid, avg_rssi
            FROM fingerprints
            WHERE grid_id IS NOT NULL AND grid_id != ''
            ORDER BY grid_id, bssid
            """
        ).fetchall()

        grids = {}
        for grid_id, floor, x_center, y_center, bssid, avg_rssi in rows:
            grid = grids.setdefault(
                grid_id,
                {
                    "x_center": x_center,
                    "y_center": y_center,
                    "floor": floor,
                    "vector": {},
                },
            )
            if bssid and avg_rssi is not None:
                grid["vector"][self._normalize_bssid(bssid)] = avg_rssi
        return grids

    def close(self):
        if self.conn is not None:
            self.conn.close()
            self.conn = None

    def _resolve_grid_size(self, grid_size):
        if grid_size is None:
            grid_size = DRIVE_SETTINGS.get("AUTO_GRID_SPACING_M", DEFAULT_GRID_SIZE_M)
        try:
            grid_size = float(grid_size)
        except (TypeError, ValueError) as exc:
            raise ValueError("grid_size must be a positive number") from exc
        if grid_size <= 0:
            raise ValueError("grid_size must be greater than 0")
        return grid_size

    @staticmethod
    def _normalize_bssid(value):
        return str(value or "").strip().upper()

    @staticmethod
    def _float_or_none(value):
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    @staticmethod
    def _int_or_none(value):
        try:
            number = int(float(value))
        except (TypeError, ValueError):
            return None
        return number

    @staticmethod
    def _ap_timestamp(ap):
        return (
            ap.get("timestamp")
            or ap.get("scanned_at")
            or time.strftime("%Y-%m-%d %H:%M:%S")
        )


if __name__ == "__main__":
    example = {
        "x": 1.25,
        "y": 0.82,
        "timestamp": "2026-05-20 12:00:00",
        "aps": [
            {
                "bssid": "AA:BB:CC:DD:EE:01",
                "ssid": "Example_AP",
                "rssi": -45.0,
                "frequency": 2412,
                "channel": 1,
                "band": "2.4GHz",
                "interface": "wlan1",
            }
        ],
    }
    db = RFFingerprintDB(":memory:")
    db.create_tables()
    scan_id = db.save_fingerprint_record(example)
    print({"scan_id": scan_id, "grid": grid_center_from_id("F1_4_2", 0.3)})
