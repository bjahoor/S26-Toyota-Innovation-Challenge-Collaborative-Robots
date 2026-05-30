"""
Web-based viewer for ArUco GridBoard camera calibration.

This is a thin wrapper around calibrateCamera.py: it reuses that module's board
configuration and calibration math (`build_board`, `_run_calibration`, and all the
constants) but replaces the desktop cv2.imshow window with a browser UI served over
HTTP. The live feed is streamed as MJPEG and capture/calibrate/quit are driven by
on-page buttons or the Space / C / Q keys.

Usage:
    python calibrateCameraWeb.py                 # calibrate viewer on :8000
    python calibrateCameraWeb.py --preview       # undistort preview on :8000
    python calibrateCameraWeb.py --port 9000     # different port
    python calibrateCameraWeb.py --camera 0      # different camera index

The original desktop tool (calibrateCamera.py) is untouched and still works.
"""

import cv2
import numpy as np
import argparse
import os
import sys
import time
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# Reuse all configuration + the calibration math from the original script.
import calibrateCamera as cc

# Make console output UTF-8 safe on Windows (avoids UnicodeEncodeError on the
# box-drawing characters printed by the imported module on a legacy codepage).
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

WEB_PORT = 8000


def build_detector(aruco_dict):
    """Same detector settings as calibrateCamera.calibrate() (sub-pixel refine)."""
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    p.cornerRefinementMaxIterations = 30
    p.cornerRefinementMinAccuracy = 0.001
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 23
    p.adaptiveThreshWinSizeStep = 10
    p.minMarkerPerimeterRate = 0.03
    return cv2.aruco.ArucoDetector(aruco_dict, p)


# ──────────────────────────────────────────────────────────────────────────────
# Shared state between the camera worker thread and the HTTP handlers
# ──────────────────────────────────────────────────────────────────────────────
class CalibState:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None              # latest encoded frame (bytes)
        self.captured = 0
        self.detected = False
        self.n_markers = 0
        self.message = "Starting camera…"
        self.phase = "capturing"      # capturing | calibrating | done | quit
        self.rms = None
        # one-shot command flags set by the browser, consumed by the worker
        self.cmd_capture = False
        self.cmd_calibrate = False
        self.cmd_quit = False

    def snapshot(self):
        with self.lock:
            return {
                "captured": self.captured,
                "detected": self.detected,
                "n_markers": self.n_markers,
                "message": self.message,
                "phase": self.phase,
                "rms": self.rms,
                "min_frames": cc.MIN_VALID_FRAMES,
                "min_markers": cc.MIN_MARKERS,
            }


_PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>Camera Calibration</title>
<style>
 body{{font-family:system-ui,Segoe UI,sans-serif;background:#111;color:#eee;margin:0;padding:16px;text-align:center}}
 h1{{font-size:18px;font-weight:600;margin:4px 0 12px}}
 #feed{{max-width:96vw;border:2px solid #333;border-radius:6px}}
 #status{{margin:12px auto;max-width:720px;font-size:15px;min-height:22px}}
 .pill{{display:inline-block;padding:2px 10px;border-radius:12px;font-weight:600}}
 .ok{{background:#0a5;color:#fff}} .bad{{background:#a33;color:#fff}}
 button{{font-size:16px;padding:10px 18px;margin:6px;border:0;border-radius:6px;cursor:pointer;color:#fff}}
 #cap{{background:#2563eb}} #cal{{background:#059669}} #quit{{background:#b91c1c}}
 button:disabled{{opacity:.4;cursor:not-allowed}}
 small{{color:#888}}
</style></head><body>
 <h1>ArUco GridBoard Calibration — live viewer</h1>
 <img id="feed" src="/stream"><br>
 <div id="status">…</div>
 <button id="cap"  onclick="cmd('capture')">Capture &nbsp;(Space)</button>
 <button id="cal"  onclick="cmd('calibrate')">Calibrate &nbsp;(C)</button>
 <button id="quit" onclick="cmd('quit')">Quit &nbsp;(Q)</button>
 <div><small>Hold the board steady when capturing. Collect {min_frames}+ frames from many angles, then Calibrate.</small></div>
<script>
 function cmd(c){{fetch('/cmd/'+c,{{method:'POST'}});}}
 document.addEventListener('keydown',e=>{{
   if(e.code==='Space'){{e.preventDefault();cmd('capture');}}
   else if(e.key==='c'||e.key==='C')cmd('calibrate');
   else if(e.key==='q'||e.key==='Q')cmd('quit');
 }});
 async function poll(){{
   try{{
     const s=await (await fetch('/status')).json();
     let det=s.detected?`<span class="pill ok">${{s.n_markers}} markers</span>`
                       :`<span class="pill bad">need ${{s.min_markers}}+ (saw ${{s.n_markers}})</span>`;
     let extra=s.rms!=null?` | <b>RMS ${{s.rms.toFixed(4)}}</b>`:'';
     document.getElementById('status').innerHTML=
        `${{det}} &nbsp; Captured: <b>${{s.captured}}</b>/${{s.min_frames}}${{extra}}<br>${{s.message}}`;
     const calBtn=document.getElementById('cal');
     calBtn.disabled=(s.captured<s.min_frames)||s.phase!=='capturing';
     document.getElementById('cap').disabled=s.phase!=='capturing';
   }}catch(e){{}}
 }}
 setInterval(poll,400); poll();
</script>
</body></html>"""


def _make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass  # silence per-request console spam

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = _PAGE.format(min_frames=cc.MIN_VALID_FRAMES).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/status":
                body = json.dumps(state.snapshot()).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while state.phase != "quit":
                        with state.lock:
                            jpg = state.jpeg
                        if jpg is not None:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(
                                ("Content-Length: %d\r\n\r\n" % len(jpg)).encode())
                            self.wfile.write(jpg)
                            self.wfile.write(b"\r\n")
                        time.sleep(0.03)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path.startswith("/cmd/"):
                cmd = self.path.rsplit("/", 1)[-1]
                with state.lock:
                    if cmd == "capture":
                        state.cmd_capture = True
                    elif cmd == "calibrate":
                        state.cmd_calibrate = True
                    elif cmd == "quit":
                        state.cmd_quit = True
                self.send_response(204)
                self.end_headers()
            else:
                self.send_error(404)

    return Handler


# ──────────────────────────────────────────────────────────────────────────────
# Camera worker — mirrors calibrateCamera.calibrate() but headless (no imshow)
# ──────────────────────────────────────────────────────────────────────────────
def _camera_worker(state, camera_index):
    board, aruco_dict = cc.build_board()
    detector = build_detector(aruco_dict)

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        with state.lock:
            state.message = f"ERROR: cannot open camera index {camera_index}"
            state.phase = "quit"
        return

    all_obj_points, all_img_points = [], []
    image_size = None

    with state.lock:
        state.message = "Hold the ArUco board in view, then Capture."

    while True:
        with state.lock:
            if state.cmd_quit:
                state.phase = "quit"
                break
            take = state.cmd_capture
            calib = state.cmd_calibrate
            state.cmd_capture = False
            state.cmd_calibrate = False

        ret, frame = cap.read()
        if not ret:
            with state.lock:
                state.message = "Failed to grab frame."
            time.sleep(0.05)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]
        marker_corners, marker_ids, _ = detector.detectMarkers(gray)
        display = frame.copy()

        detected = marker_ids is not None and len(marker_ids) >= cc.MIN_MARKERS
        n = 0 if marker_ids is None else len(marker_ids)
        if detected:
            cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)

        # ── handle a capture request ──
        if take:
            if detected:
                obj_pts, img_pts = board.matchImagePoints(marker_corners, marker_ids)
                if (obj_pts is not None and
                        len(obj_pts) >= cc.MIN_POINTS_PER_FRAME and
                        len(img_pts) == len(obj_pts)):
                    all_obj_points.append(obj_pts)
                    all_img_points.append(img_pts)
                    with state.lock:
                        state.captured = len(all_obj_points)
                        state.message = (f"Frame {state.captured} captured "
                                         f"({n} markers, {len(obj_pts)} pts)")
                    print(f" Frame {len(all_obj_points):3d} captured "
                          f"({n} markers, {len(obj_pts)} points)")
                else:
                    with state.lock:
                        state.message = "Not enough valid points — hold steadier."
            else:
                with state.lock:
                    state.message = f"Need {cc.MIN_MARKERS}+ markers to capture (saw {n})."

        # ── handle a calibrate request ──
        if calib:
            if len(all_obj_points) < cc.MIN_VALID_FRAMES:
                with state.lock:
                    state.message = (f"Need at least {cc.MIN_VALID_FRAMES} frames "
                                     f"(have {len(all_obj_points)}).")
            else:
                with state.lock:
                    state.phase = "calibrating"
                    state.message = f"Calibrating on {len(all_obj_points)} frames…"
                print(f"\nRunning calibration + outlier removal on "
                      f"{len(all_obj_points)} frames …")
                # Reuse the original module's calibration + outlier-removal + save.
                rms = _run_calibration_get_rms(all_obj_points, all_img_points, image_size)
                with state.lock:
                    state.rms = float(rms)
                    state.phase = "done"
                    state.message = (f"Done. Saved {cc.OUTPUT_FILE}. "
                                     f"RMS={rms:.4f} (target < 0.5). You may close this tab.")

        # ── overlay status + publish frame ──
        with state.lock:
            captured = state.captured
            phase = state.phase
            rms_val = state.rms
        if phase == "done":
            txt = f"DONE  RMS={rms_val:.4f}  saved {cc.OUTPUT_FILE}"
            color = (0, 200, 0)
        elif detected:
            txt = f"Detected {n} markers | Captured: {captured} | Space=capture C=calibrate Q=quit"
            color = (0, 200, 0)
        else:
            txt = f"Need {cc.MIN_MARKERS}+ markers (saw {n}) | Captured: {captured}"
            color = (0, 0, 220)
        cv2.putText(display, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        ok, buf = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with state.lock:
                state.jpeg = buf.tobytes()
                state.detected = detected
                state.n_markers = n

    cap.release()


def _run_calibration_get_rms(all_obj_points, all_img_points, image_size):
    """Call the original module's _run_calibration and recover the saved RMS.

    calibrateCamera._run_calibration() saves camera_params.npz (including
    rms_error) but returns None, so we read the RMS back from the file it wrote.
    """
    cc._run_calibration(all_obj_points, all_img_points, image_size)
    data = np.load(cc.OUTPUT_FILE)
    return float(data["rms_error"])


def calibrate_web(camera_index, port):
    state = CalibState()
    worker = threading.Thread(target=_camera_worker,
                              args=(state, camera_index), daemon=True)
    worker.start()

    server = ThreadingHTTPServer(("0.0.0.0", port), _make_handler(state))
    url = f"http://localhost:{port}/"
    print("\n──────────────────────────────────────────────")
    print(" CALIBRATION WEB VIEWER (ArUco GridBoard 4×4)")
    print(f" • Open in your browser:  {url}")
    print(" • Buttons (or keys): Space=capture  C=calibrate  Q=quit")
    print(f" • Capture {cc.MIN_VALID_FRAMES}+ frames from many angles, then Calibrate.")
    print(" • Ctrl+C here also stops the server.")
    print("──────────────────────────────────────────────\n")

    def _watch():
        while state.phase != "quit":
            time.sleep(0.25)
        server.shutdown()
    threading.Thread(target=_watch, daemon=True).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nInterrupted — shutting down.")
    finally:
        with state.lock:
            state.phase = "quit"
        server.server_close()


# ──────────────────────────────────────────────────────────────────────────────
# Undistort preview — web stream (view-only, no controls)
# ──────────────────────────────────────────────────────────────────────────────
def preview_web(camera_index, port):
    if not os.path.exists(cc.OUTPUT_FILE):
        print(f"No calibration file found: {cc.OUTPUT_FILE}\nRun calibration first.")
        return

    data = np.load(cc.OUTPUT_FILE)
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]
    rms = float(data["rms_error"])
    print(f"\n Loaded calibration | FINAL RMS error: {rms:.4f}")

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {camera_index}")

    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Failed to grab a frame from the camera.")
    h, w = frame.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), alpha=1)
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_16SC2)

    shared = {"jpeg": None, "run": True, "lock": threading.Lock()}

    def worker():
        while shared["run"]:
            ok, f = cap.read()
            if not ok:
                time.sleep(0.03)
                continue
            undistorted = cv2.remap(f, map1, map2, cv2.INTER_LINEAR)
            x, y, rw, rh = roi
            crop = undistorted[y:y+rh, x:x+rw]
            if crop.size == 0:
                crop = undistorted
            crop = cv2.resize(crop, (w, h))
            combined = np.hstack([f, crop])
            cv2.putText(combined, "Original", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 80, 255), 2)
            cv2.putText(combined, f"Undistorted (RMS={rms:.3f})", (w + 10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 0), 2)
            ok2, buf = cv2.imencode(".jpg", combined, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok2:
                with shared["lock"]:
                    shared["jpeg"] = buf.tobytes()
            time.sleep(0.03)
        cap.release()

    threading.Thread(target=worker, daemon=True).start()

    page = ("<!doctype html><html><head><meta charset='utf-8'>"
            "<title>Undistort Preview</title>"
            "<style>body{background:#111;text-align:center;margin:0;padding:12px}"
            "img{max-width:98vw;border:2px solid #333;border-radius:6px}</style>"
            "</head><body><img src='/stream'></body></html>").encode()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while shared["run"]:
                        with shared["lock"]:
                            jpg = shared["jpeg"]
                        if jpg is not None:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                            self.wfile.write(
                                ("Content-Length: %d\r\n\r\n" % len(jpg)).encode())
                            self.wfile.write(jpg + b"\r\n")
                        time.sleep(0.03)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
            else:
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(page)

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print(f"\n UNDISTORT PREVIEW | Open http://localhost:{port}/ | Ctrl+C to quit\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nInterrupted — shutting down.")
    finally:
        shared["run"] = False
        server.server_close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Web viewer for ArUco GridBoard camera calibration")
    parser.add_argument("--preview", action="store_true",
                        help="Web undistort preview instead of calibration")
    parser.add_argument("--camera", type=int, default=cc.CAMERA_INDEX,
                        help="Webcam index (default: %(default)s)")
    parser.add_argument("--port", type=int, default=WEB_PORT,
                        help="Port for the web viewer (default: %(default)s)")
    args = parser.parse_args()

    if args.preview:
        preview_web(args.camera, args.port)
    else:
        calibrate_web(args.camera, args.port)
