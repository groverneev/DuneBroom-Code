#!/usr/bin/env python3
"""Live camera view in a browser, with the checkerboard tilt readout (motors never move).

    python3 tools/live_view.py            then open http://neev-rpi5-green.local:8000

Camera runs on auto (exposure/focus/white balance). When the printed board
(docs/checkerboard_25mm.pdf) is fully in view, height and tilt are measured
about twice a second and drawn on the picture: green when within 1.5 deg.
Ctrl+C to stop (frees the camera for other tools).
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rover.camera import Camera  # noqa: E402
from rover.camera_model import (  # noqa: E402
    CALIBRATION_PATH, PATTERN, SQUARE_MM, CameraModel, approx_model, find_board, ground_from_board,
)

OK_DEG = 1.5
STREAM_WIDTH = 960

PAGE = b"""<!doctype html><html><head><meta name=viewport content="width=device-width">
<title>DuneBroom camera</title></head>
<body style="margin:0;background:#111"><img src="/stream" style="width:100%;display:block"></body></html>"""


class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None        # latest BGR frame
        self.jpeg = b""          # latest annotated stream JPEG
        self.result = None       # (corners, Ground) or None
        self.result_t = 0.0


def lean_text(v: float, pos: str, neg: str) -> str:
    return "level" if abs(v) < 0.5 else f"{abs(v):.1f} deg toward {pos if v > 0 else neg}"


def capture_loop(cam: Camera, shared: Shared, stop: threading.Event) -> None:
    while not stop.is_set():
        _, frame = cam.read()
        with shared.lock:
            shared.frame = frame
            result, age = shared.result, time.monotonic() - shared.result_t
        out = frame.copy()
        h, w = out.shape[:2]
        cv2.line(out, (w // 2, 0), (w // 2, h), (0, 255, 255), 2)
        cv2.line(out, (0, h // 2), (w, h // 2), (0, 255, 255), 2)
        if result and age < 2.0:
            corners, g = result
            cv2.drawChessboardCorners(out, PATTERN, corners, True)
            worst = max(abs(g.lean_top_deg), abs(g.lean_right_deg))
            colour = (0, 200, 0) if worst <= OK_DEG else (0, 0, 255)
            lines = [f"height {g.height_mm:.0f} mm   view {g.footprint_w_mm:.0f} x {g.footprint_h_mm:.0f} mm",
                     f"top/bottom: {lean_text(g.lean_top_deg, 'TOP', 'BOTTOM')}",
                     f"left/right: {lean_text(g.lean_right_deg, 'RIGHT', 'LEFT')}"]
        else:
            colour, lines = (0, 165, 255), ["board not found (whole board in view?)"]
        for i, text in enumerate(lines):
            y = 60 + i * 60
            cv2.putText(out, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 0, 0), 8)
            cv2.putText(out, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 1.6, colour, 3)
        small = cv2.resize(out, (STREAM_WIDTH, round(h * STREAM_WIDTH / w)))
        ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with shared.lock:
                shared.jpeg = buf.tobytes()


def detect_loop(shared: Shared, model: CameraModel | None, square: float, stop: threading.Event) -> None:
    while not stop.is_set():
        with shared.lock:
            frame = shared.frame
        if frame is None:
            time.sleep(0.1)
            continue
        m = model or approx_model(frame.shape[1], frame.shape[0])
        corners = find_board(frame)
        result = (corners, ground_from_board(m, corners, square)) if corners is not None else None
        with shared.lock:
            shared.result, shared.result_t = result, time.monotonic()


def make_handler(shared: Shared):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(PAGE)
                return
            if self.path != "/stream":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last = b""
            try:
                while True:
                    with shared.lock:
                        jpeg = shared.jpeg
                    if jpeg and jpeg is not last:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
                        last = jpeg
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--square-mm", type=float, default=SQUARE_MM)
    args = parser.parse_args()

    model = CameraModel.load() if CALIBRATION_PATH.exists() else None
    shared, stop = Shared(), threading.Event()
    cam = Camera().open(lock=False)
    threads = [threading.Thread(target=capture_loop, args=(cam, shared, stop), daemon=True),
               threading.Thread(target=detect_loop, args=(shared, model, args.square_mm, stop), daemon=True)]
    for t in threads:
        t.start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(shared))
    server.daemon_threads = True
    print(f"live view: http://neev-rpi5-green.local:{args.port}  (Ctrl+C to stop)"
          + ("" if model else "  [tilt is rough: lens not calibrated yet]"))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        time.sleep(0.3)
        cam.close()


if __name__ == "__main__":
    main()
