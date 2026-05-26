import threading
import time

from config import DRIVE_SETTINGS


class USBCamera:
    def __init__(self, device_id=0, width=320, height=240):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.cap = None
        self.running = False
        self.ready = False
        self.lock = threading.Lock()
        self.latest_frame = None
        self.obstacle = False
        self.turn_direction = "right"
        self.scores = {"left": 0.0, "center": 0.0, "right": 0.0}

    def connect(self):
        try:
            import cv2
        except ImportError as exc:
            print(f"OpenCV import failed: {exc}", flush=True)
            return False

        self.cv2 = cv2

        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.cap = None
            time.sleep(0.1)

        candidates = [self.device_id, self.device_id + 1, self.device_id + 2, "/dev/video0", "/dev/video1", "/dev/video2"]
        for index in candidates:
            for backend in [cv2.CAP_ANY, cv2.CAP_V4L2] if hasattr(cv2, 'CAP_V4L2') else [cv2.CAP_ANY]:
                try:
                    self.cap = cv2.VideoCapture(index, backend)
                except Exception as exc:
                    print(f"Camera connect attempt {index} backend {backend} failed: {exc}", flush=True)
                    self.cap = None
                    continue

                time.sleep(0.2)
                if not self.cap or not self.cap.isOpened():
                    if self.cap:
                        self.cap.release()
                        self.cap = None
                    continue

                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                # Read one frame to verify the camera is truly working
                ok = False
                frame = None
                for _ in range(10):
                    ok, frame = self.cap.read()
                    if ok and frame is not None:
                        break
                    time.sleep(0.05)
                if not ok or frame is None:
                    if self.cap:
                        self.cap.release()
                        self.cap = None
                    print(f"Camera device {index} opened but no frame read.", flush=True)
                    continue

                self.device_id = index
                self.ready = True
                with self.lock:
                    self.latest_frame = frame.copy()
                print(f"Camera device {index} opened successfully with backend {backend}.", flush=True)
                return True

        print(f"Camera device {self.device_id} not available. Camera obstacle detection is disabled.", flush=True)
        return False

    def start_capture_thread(self):
        if not self.ready:
            return
        self.running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()

    def release(self):
        self.running = False
        if self.cap:
            self.cap.release()

    def snapshot(self):
        with self.lock:
            return {
                "camera_ready": self.ready,
                "camera_obstacle": self.obstacle,
                "camera_turn": self.turn_direction,
                "camera_left_score": self.scores["left"],
                "camera_center_score": self.scores["center"],
                "camera_right_score": self.scores["right"],
            }

    def get_latest_frame(self):
        with self.lock:
            return None if self.latest_frame is None else self.latest_frame.copy()

    def jpeg_frames(self):
        while self.running:
            with self.lock:
                frame = None if self.latest_frame is None else self.latest_frame.copy()

            if frame is None:
                time.sleep(0.05)
                continue

            ok, buffer = self.cv2.imencode(".jpg", frame)
            if ok:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"

    def _capture_loop(self):
        failed_reads = 0
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                failed_reads += 1
                if failed_reads >= 30:
                    print("Camera frame read failed repeatedly. Reconnecting camera.", flush=True)
                    self.ready = False
                    if self.cap:
                        self.cap.release()
                        self.cap = None
                    if self.connect():
                        failed_reads = 0
                    else:
                        time.sleep(0.5)
                    continue
                time.sleep(0.05)
                continue

            failed_reads = 0
            obstacle, turn_direction, scores = self._analyze(frame)

            with self.lock:
                self.obstacle = obstacle
                self.turn_direction = turn_direction
                self.scores = scores
                self.latest_frame = frame

            time.sleep(0.03)

    def _analyze(self, frame):
        h, w = frame.shape[:2]
        roi = frame[int(h * 0.50):h, :]
        left = roi[:, 0:int(w * 0.33)]
        center = roi[:, int(w * 0.33):int(w * 0.67)]
        right = roi[:, int(w * 0.67):w]

        scores = {
            "left": self._region_score(left),
            "center": self._region_score(center),
            "right": self._region_score(right),
        }
        obstacle = scores["center"] > DRIVE_SETTINGS["CENTER_OBSTACLE_SCORE"]
        turn_direction = "left" if scores["left"] <= scores["right"] else "right"
        return obstacle, turn_direction, scores

    def _region_score(self, region):
        gray = self.cv2.cvtColor(region, self.cv2.COLOR_BGR2GRAY)
        blur = self.cv2.GaussianBlur(gray, (7, 7), 0)
        edges = self.cv2.Canny(blur, 60, 140)
        edge_score = self.cv2.countNonZero(edges) / edges.size
        dark_mask = self.cv2.inRange(blur, 0, 55)
        dark_score = self.cv2.countNonZero(dark_mask) / dark_mask.size
        return edge_score * 0.7 + dark_score * 0.3

    def _draw_overlay(self, frame, turn_direction, scores):
        if not DRIVE_SETTINGS.get("CAMERA_DEBUG_OVERLAY_ENABLE", False):
            return frame
        h, w = frame.shape[:2]
        y0 = int(h * 0.50)
        x1 = int(w * 0.33)
        x2 = int(w * 0.67)
        self.cv2.line(frame, (x1, y0), (x1, h), (0, 255, 255), 1)
        self.cv2.line(frame, (x2, y0), (x2, h), (0, 255, 255), 1)
        self.cv2.line(frame, (0, y0), (w, y0), (0, 255, 255), 1)
        self.cv2.putText(frame, f"turn {turn_direction}", (8, 22), self.cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        self.cv2.putText(frame, f"C {scores['center']:.2f}", (8, 46), self.cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        return frame
