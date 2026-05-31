r"""
pick_web.py - browser control panel for the pick-and-place runtime.

Serves a web UI (HTTP) on a port that:
  * STREAMS the camera live (MJPEG) with the safety overlay drawn on top,
  * VISUALIZES the runtime flow (IDLE -> INIT -> SCAN PLATE -> SCAN TARGET ->
    PICK & PLACE -> DONE) and highlights the current state/step,
  * gives INTERACTIVE BUTTONS for control, confirmation, looping and stepping:
        Start | Pause/Resume | Confirm-Next | Stop | Home | E-STOP
        + toggles: Step mode, Loop, Confirm-at-danger.

It reuses the real abstraction modules -- dobotArm / grasp_orientation /
hand_detect / safety_supervisor -- and the detectors in pick_engine.py (whose
constants mirror pickCVBlock.py, the source of truth). pickCVBlock.py is left
untouched; this is an additional front end for the same flow.

Design notes
------------
* ONE worker thread owns the camera and runs the state machine. Robot moves are
  issued NON-BLOCKING (queued PTP + polling), so the stream keeps flowing during a
  move and E-STOP can abort mid-motion (force-stop clears the queue, the poll then
  completes). This is the watchdog behaviour from the plan doc.
* GRACEFUL DEGRADATION: if no Dobot is found it runs in SIMULATION mode (moves are
  timed no-ops) so the whole flow is demoable without the arm. If the camera fails
  it shows a placeholder and keeps retrying. If MediaPipe/the hand model is missing
  the safety overlay is simply absent (gates pass through).

Run (UTF-8 needed for the Dobot DLL; the script also self-reconfigures stdout):
    $env:PYTHONUTF8 = "1"
    cd Collaborative_Robotics
    ..\.venv\Scripts\python.exe pick_web.py                 # Orbbec (index 1), port 8080
    ..\.venv\Scripts\python.exe pick_web.py --camera 0      # laptop webcam
    ..\.venv\Scripts\python.exe pick_web.py --port 9000
Then open http://localhost:8080/ in a browser.
"""

import os
import sys
import json
import time
import threading
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# The vendored Dobot DLL prints a non-ASCII char ('：') the moment it loads, which
# crashes under Windows' default cp1252 console. Force UTF-8 BEFORE importing
# dobotArm / DobotDllType (their module-level dType.load() triggers that print).
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

# Resolve paths relative to THIS file and put this folder on sys.path so the
# `import lib.DobotDllType` / `import dobotArm` resolve from any working directory.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import numpy as np
import cv2

import lib.DobotDllType as dType
import dobotArm
import pick_engine as engine
import safety_supervisor as safety

# Hand detection is optional -- degrade to "no safety overlay" if it can't load.
try:
    from hand_detect import HandDetector
    _HAND_OK = True
except Exception as _e:  # pragma: no cover - depends on local install
    HandDetector = None
    _HAND_OK = False
    print(f"[safety] hand detection unavailable ({_e}); running WITHOUT the safety overlay.")


# --------------------------------------------------------------------------- #
# Config
# --------------------------------------------------------------------------- #
WEB_PORT = 8080
CAMERA_INDEX = 1                # Orbbec (calibrated). Override with --camera.
SAFETY_ZONE_MARGIN = 0.15       # central 70% of the frame is the danger zone
SAFETY_CLEAR_FRAMES = 20        # resume only after the zone is clear this many frames

# Simulated timings (frames @ ~30 fps) when no robot is connected, so the flow is
# still watchable without the arm.
SIM_MOVE_FRAMES = 14
SIM_HOME_FRAMES = 36
GRIP_SETTLE_FRAMES = 15         # matches dobotArm's dSleep(500) settle, non-blocking

# UI phase order (HALTED/ERROR are shown specially, not in this ladder).
PHASES = ["IDLE", "INIT", "SCAN_PLATE", "SCAN_TARGET", "PICK_PLACE", "DONE"]

# The per-part pick/place program, as data so the UI can list every step and
# highlight the active one. `target` is resolved to (x,y,z,r) at run time.
#   danger : a contact step -> gated by the "Confirm-at-danger" toggle
#   safety : a move into the work area -> blocked while a hand is in the danger zone
STEP_DEFS = [
    {"id": "pick_approach",  "label": "Pick · approach (Z_SAFE)",      "kind": "move",          "target": "part_safe",  "danger": False, "safety": True},
    {"id": "pick_descend",   "label": "Pick · descend (Z_PICK)",       "kind": "move",          "target": "part_pick",  "danger": True,  "safety": True},
    {"id": "pick_grip",      "label": "Pick · close gripper",          "kind": "grip_close",    "target": None,         "danger": True,  "safety": False},
    {"id": "pick_lift",      "label": "Pick · lift (Z_SAFE)",          "kind": "move",          "target": "part_safe",  "danger": False, "safety": False},
    {"id": "home_clear",     "label": "Home · clear camera view",      "kind": "move",          "target": "home",       "danger": False, "safety": False},
    {"id": "retray",         "label": "Re-detect tray (drop zone)",    "kind": "detect_tray",   "target": None,         "danger": False, "safety": False},
    {"id": "place_approach", "label": "Place · approach (Z_SAFE)",     "kind": "move",          "target": "drop_safe",  "danger": False, "safety": True},
    {"id": "place_descend",  "label": "Place · descend (Z_PLACE)",     "kind": "move",          "target": "drop_place", "danger": True,  "safety": True},
    {"id": "place_release",  "label": "Place · open gripper + pump off", "kind": "grip_release", "target": None,        "danger": True,  "safety": False},
    {"id": "place_lift",     "label": "Place · lift (Z_SAFE)",         "kind": "move",          "target": "drop_safe",  "danger": False, "safety": False},
]


# --------------------------------------------------------------------------- #
# Shared state between the worker thread and the HTTP handlers
# --------------------------------------------------------------------------- #
class RunState:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.quit = False
        self._cmds = []
        self.status = {
            "phase": "IDLE", "message": "Idle — press Start.", "progress": 0,
            "step_index": -1, "robot_mode": "?", "safety": "—", "hand": False,
            "step_mode": False, "loop_mode": False, "confirm_danger": True,
            "paused": False, "awaiting": False, "prompt": "",
            "part_index": 0, "total_parts": 0, "tray": None, "log": [],
        }

    def push_cmd(self, c):
        with self.lock:
            self._cmds.append(c)

    def pop_cmds(self):
        with self.lock:
            c, self._cmds = self._cmds, []
            return c

    def set_status(self, **kw):
        with self.lock:
            self.status.update(kw)

    def get_status(self):
        with self.lock:
            return dict(self.status)

    def set_jpeg(self, b):
        with self.lock:
            self.jpeg = b

    def get_jpeg(self):
        with self.lock:
            return self.jpeg


# --------------------------------------------------------------------------- #
# The controllable runtime (worker thread)
# --------------------------------------------------------------------------- #
class PickRuntime:
    def __init__(self, state, camera_index):
        self.state = state
        self.camera_index = camera_index

        # camera / calibration
        self.cap = None
        self.H = None
        self.map1 = self.map2 = None
        self.frame_w = self.frame_h = 0

        # safety
        self.hand_detector = None
        self.supervisor = None
        self._safety_state = "—"
        self._hand = False

        # robot
        self.api = dType.load()
        self.sim = True
        self.robot_mode = "?"
        self._connected_attempted = False

        # in-flight non-blocking move ('real' = queue index, 'sim' = frame countdown)
        self._move = None
        self._on_done = None

        # phase / flow
        self._phase = "IDLE"
        self._message = "Idle — press Start."
        self._progress = 0
        self._awaiting = False
        self._prompt = ""
        self._log_dq = deque(maxlen=40)

        # detectors (built once calibration is loaded)
        self.plate_det = None
        self.target_det = None

        # modes / flags
        self.step_mode = False
        self.loop_mode = False
        self.confirm_danger = True
        self.paused = False

        # pick/place sub-state
        self.tray_xy = None
        self.targets = []
        self.total_parts = 0
        self.part_index = 0
        self.step_index = 0
        self.step_acked = False
        self.step_issued = False
        self.drop_xy = None

    # ---- small helpers ----------------------------------------------------- #
    def _log(self, msg):
        self._message = msg
        self._log_dq.append(msg)
        print(f"[pick_web] {msg}")

    def _set_phase(self, p):
        self._phase = p

    def _set_awaiting(self, flag, prompt=""):
        self._awaiting = flag
        self._prompt = prompt

    def _scan_confirm_required(self):
        # A locked detection waits for the user when stepping OR confirming dangers.
        return self.step_mode or self.confirm_danger

    # ---- setup ------------------------------------------------------------- #
    def _open_camera(self):
        try:
            cap = cv2.VideoCapture(self.camera_index)
        except Exception:
            cap = None
        if not cap or not cap.isOpened():
            self.cap = None
            return False
        frame = None
        for _ in range(10):  # first reads off a fresh camera are often empty
            ok, frame = cap.read()
            if ok and frame is not None:
                break
        if frame is None:
            cap.release()
            self.cap = None
            return False
        self.cap = cap
        self.frame_h, self.frame_w = frame.shape[:2]
        return True

    def _load_calibration(self):
        try:
            self.H = np.load(os.path.join(_HERE, "HomographyMatrix.npy"))
            data = np.load(os.path.join(_HERE, "camera_params.npz"))
            cm, dc = data["camera_matrix"], data["dist_coeffs"]
            if self.frame_h and self.frame_w:
                self.map1, self.map2 = engine.build_undistort_maps(
                    (self.frame_h, self.frame_w), cm, dc)
            self.plate_det = engine.PlateDetector(self.H)
            self.target_det = engine.TargetDetector(self.H)
            self._log("calibration loaded.")
        except Exception as e:
            self._log(f"calibration load failed ({e}); detection will be unavailable.")

    def _build_safety(self):
        if not _HAND_OK or not self.frame_w:
            return
        try:
            self.hand_detector = HandDetector()
            zone = safety.zone_from_frame(self.frame_w, self.frame_h,
                                          margin_frac=SAFETY_ZONE_MARGIN)
            self.supervisor = safety.SafetySupervisor(zone, clear_frames=SAFETY_CLEAR_FRAMES)
            self._log(f"safety active. Danger zone {zone} px.")
        except Exception as e:
            self.hand_detector = None
            self.supervisor = None
            self._log(f"could not start hand detector ({e}); no safety overlay.")

    # ---- robot ------------------------------------------------------------- #
    def _connect_robot(self):
        try:
            ports = dType.SearchDobot(self.api)
        except Exception:
            ports = []
        if not ports or "COM" not in str(ports[0]):
            self.sim = True
            self.robot_mode = "SIMULATION (no Dobot found)"
            self._log("no Dobot found → SIMULATION mode.")
            return
        try:
            st = dType.ConnectDobot(self.api, ports[0], 115200)[0]
        except Exception as e:
            self.sim = True
            self.robot_mode = "SIMULATION (connect error)"
            self._log(f"connect error ({e}) → SIMULATION mode.")
            return
        if st != dType.DobotConnect.DobotConnect_NoError:
            self.sim = True
            self.robot_mode = "SIMULATION (connect failed)"
            self._log("connect failed → SIMULATION mode.")
            return
        self.sim = False
        self.robot_mode = f"connected {ports[0]}"
        self._log(f"connected to Dobot on {ports[0]}.")

    def _start_homing(self):
        if self.sim:
            self._start_settle(SIM_HOME_FRAMES, self._after_init)
            return
        try:
            dType.SetQueuedCmdStopExec(self.api)
            dType.SetQueuedCmdClear(self.api)
            dType.SetPTPCommonParams(self.api, 50, 50, isQueued=1)
            h = dobotArm.home_pos
            dType.SetHOMEParams(self.api, h[0], h[1], h[2], 0, isQueued=1)
            idx = dType.SetHOMECmd(self.api, temp=0, isQueued=1)[0]
            dType.SetQueuedCmdStartExec(self.api)
            self._move = {"kind": "real", "exec": idx}
            self._on_done = self._after_init
            self._log("homing the arm…")
        except Exception as e:
            self._log(f"homing failed ({e}); aborting to IDLE.")
            self._set_phase("IDLE")

    def _after_init(self):
        # Open the gripper + pump off so a pick starts from a known state.
        if not self.sim:
            try:
                dType.SetEndEffectorGripper(self.api, 1, 0, 0)
                dType.SetEndEffectorSuctionCup(self.api, 1, 0, 0)
            except Exception:
                pass
        if self.plate_det:
            self.plate_det.reset()
        if self.target_det:
            self.target_det.reset()
        self.tray_xy = None
        self.targets = []
        self.total_parts = 0
        self.part_index = 0
        self.step_index = 0
        self.step_acked = False
        self.step_issued = False
        self.drop_xy = None
        if self.plate_det is None:
            self._log("no calibration → cannot scan; back to IDLE.")
            self._set_phase("IDLE")
            return
        self._set_phase("SCAN_PLATE")
        self._log("ready — scanning for the tray.")

    # ---- motion primitives (non-blocking) ---------------------------------- #
    def _issue_xyz(self, x, y, z, r=0.0):
        if self.sim:
            self._move = {"kind": "sim", "frames": SIM_MOVE_FRAMES}
            return
        try:
            idx = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode,
                                  x, y, z, r, isQueued=0)[0]
            self._move = {"kind": "real", "exec": idx}
        except Exception as e:
            self._log(f"move command failed ({e}).")
            self._move = {"kind": "sim", "frames": 1}

    def _start_move(self, x, y, z, r, on_done):
        self._on_done = on_done
        self._issue_xyz(x, y, z, r)

    def _start_settle(self, frames, on_done):
        """A non-blocking 'wait N frames' used for gripper settle and sim homing."""
        self._on_done = on_done
        self._move = {"kind": "sim", "frames": frames}

    def _poll_move(self):
        """Advance/clear the in-flight move. Returns True when it has finished."""
        m = self._move
        if m is None:
            return True
        if m["kind"] == "sim":
            m["frames"] -= 1
            if m["frames"] <= 0:
                self._move = None
                return True
            return False
        try:
            done = dType.GetQueuedCmdCurrentIndex(self.api)[0] >= m["exec"]
        except Exception:
            done = True
        if done:
            self._move = None
            return True
        return False

    # ---- global commands --------------------------------------------------- #
    def _handle_global(self, cmds):
        for c in cmds:
            if c == "estop":
                self._estop()
            elif c == "stop":
                self._stop()
            elif c == "home":
                self._home()
            elif c == "pause":
                self.paused = not self.paused
                self._log("paused." if self.paused else "resumed.")
            elif c == "mode_step":
                self.step_mode = not self.step_mode
            elif c == "mode_loop":
                self.loop_mode = not self.loop_mode
            elif c == "mode_confirm":
                self.confirm_danger = not self.confirm_danger
            elif c == "start":
                self._start()

    def _start(self):
        if self._phase in ("IDLE", "DONE", "HALTED", "ERROR"):
            self.paused = False
            self._connected_attempted = False
            self._set_awaiting(False, "")
            self._set_phase("INIT")
            self._log("starting…")

    def _estop(self):
        if not self.sim:
            try:
                dobotArm.stop_motion(self.api)  # force-stop + clear the queue
            except Exception:
                pass
        self._move = None
        self._on_done = None
        self._set_awaiting(False, "")
        self._set_phase("HALTED")
        self._log("E-STOP — arm halted, queue cleared.")

    def _stop(self):
        if not self.sim and self._move is not None:
            try:
                dobotArm.stop_motion(self.api)
            except Exception:
                pass
        self._move = None
        self._on_done = None
        self.part_index = 0
        self.step_index = 0
        self.step_issued = False
        self.step_acked = False
        self._set_awaiting(False, "")
        self._set_phase("IDLE")
        self._log("stopped — back to IDLE.")

    def _home(self):
        if self._phase not in ("IDLE", "DONE", "HALTED", "ERROR") or self._move is not None:
            self._log("Home is available only when idle/halted and not moving.")
            return
        if not self._connected_attempted:
            self._log("press Start first to connect the arm.")
            return
        if self.sim:
            self._start_settle(SIM_HOME_FRAMES, lambda: self._log("home reached (sim)."))
            self._log("homing (sim)…")
            return
        h = dobotArm.home_pos
        self._start_move(h[0], h[1], h[2], 0, lambda: self._log("home reached."))
        self._log("moving home…")

    # ---- phase dispatch ---------------------------------------------------- #
    def _dispatch(self, frame, display, cmds, safe):
        p = self._phase
        if p == "INIT":
            self._tick_init()
        elif p == "SCAN_PLATE":
            self._tick_scan_plate(frame, display, cmds)
        elif p == "SCAN_TARGET":
            self._tick_scan_target(frame, display, cmds)
        elif p == "PICK_PLACE":
            self._tick_pick_place(frame, display, cmds, safe)
        elif p == "DONE":
            pass  # waits for Start (or loop is handled when the batch finishes)

    def _tick_init(self):
        if not self._connected_attempted:
            self._connected_attempted = True
            self._connect_robot()
            self._start_homing()  # completion (_after_init) advances to SCAN_PLATE

    def _tick_scan_plate(self, frame, display, cmds):
        if self.plate_det is None:
            return
        self._progress, locked = self.plate_det.process(frame, display)
        if locked is None:
            return
        if self._scan_confirm_required() and not self._awaiting:
            self._set_awaiting(True, f"Tray locked at x={locked[0]:.0f} y={locked[1]:.0f} — Confirm")
        if self._awaiting:
            if "confirm" not in cmds:
                return
            self._set_awaiting(False, "")
        self.tray_xy = locked
        self.target_det.reset()
        self._progress = 0
        self._set_phase("SCAN_TARGET")
        self._log(f"tray locked at {locked[0]:.0f},{locked[1]:.0f} — scanning parts.")

    def _tick_scan_target(self, frame, display, cmds):
        if self.target_det is None:
            return
        self._progress, locked = self.target_det.process(frame, display)
        if locked is None:
            return
        if self._scan_confirm_required() and not self._awaiting:
            self._set_awaiting(True, f"{len(locked)} part(s) locked — Confirm to pick")
        if self._awaiting:
            if "confirm" not in cmds:
                return
            self._set_awaiting(False, "")
        self.targets = locked
        self.total_parts = len(locked)
        self.part_index = 0
        self.step_index = 0
        self.step_acked = False
        self.step_issued = False
        self.drop_xy = None
        self._progress = 0
        self._set_phase("PICK_PLACE")
        self._log(f"{self.total_parts} part(s) locked — executing pick/place.")

    def _resolve_target(self, target):
        part = self.targets[self.part_index]
        px, py, pr = part[0], part[1], part[2]
        if target == "part_safe":
            return (px, py, engine.Z_SAFE, pr)
        if target == "part_pick":
            return (px, py, engine.Z_PICK, pr)
        if target == "home":
            h = dobotArm.home_pos
            return (h[0], h[1], h[2], 0.0)
        if target in ("drop_safe", "drop_place"):
            if self.drop_xy is None:
                return None
            dx, dy = self.drop_xy
            z = engine.Z_SAFE if target == "drop_safe" else engine.Z_PLACE
            return (dx, dy, z, 0.0)
        return None

    def _complete_step(self):
        self.step_index += 1
        self.step_acked = False
        self.step_issued = False

    def _do_gripper(self, kind):
        if not self.sim:
            try:
                if kind == "grip_close":
                    dType.SetEndEffectorGripper(self.api, 1, 1, 0)
                else:  # grip_release
                    dType.SetEndEffectorGripper(self.api, 1, 0, 0)
                    dType.SetEndEffectorSuctionCup(self.api, 1, 0, 0)
            except Exception as e:
                self._log(f"gripper command failed ({e}).")
        self._log("gripper close." if kind == "grip_close" else "gripper open + pump off.")
        self._start_settle(GRIP_SETTLE_FRAMES, self._complete_step)

    def _tick_pick_place(self, frame, display, cmds, safe):
        # Finished all parts?
        if self.part_index >= self.total_parts:
            if self.loop_mode:
                self._log("batch complete → looping.")
                self.plate_det.reset()
                self.target_det.reset()
                self.tray_xy = None
                self._progress = 0
                self._set_phase("SCAN_PLATE")
            else:
                self._log("batch complete.")
                self._set_phase("DONE")
            return

        # Finished all steps for this part?
        if self.step_index >= len(STEP_DEFS):
            self.part_index += 1
            self.step_index = 0
            self.step_acked = False
            self.step_issued = False
            self.drop_xy = None
            self._log(f"part {self.part_index}/{self.total_parts} placed.")
            return

        step = STEP_DEFS[self.step_index]

        # Confirmation / stepping gate.
        must_wait = self.step_mode or (self.confirm_danger and step["danger"])
        if must_wait and not self.step_acked:
            if not self._awaiting:
                self._set_awaiting(True, f"{step['label']} — Confirm to execute")
            if "confirm" not in cmds:
                return
            self.step_acked = True
            self._set_awaiting(False, "")

        # Safety gate: don't move into the work area while a hand is in the zone.
        if step.get("safety") and not safe:
            self._message = f"HELD (hand in zone) — {step['label']}"
            return

        kind = step["kind"]
        if kind == "move":
            if not self.step_issued:
                xyzr = self._resolve_target(step["target"])
                if xyzr is None:
                    self._log("could not resolve target; skipping part.")
                    self.step_index = len(STEP_DEFS)
                    return
                self.step_issued = True
                self._start_move(*xyzr, on_done=self._complete_step)
            # while moving, the loop top polls and then calls _complete_step
        elif kind in ("grip_close", "grip_release"):
            if not self.step_issued:
                self.step_issued = True
                self._do_gripper(kind)
        elif kind == "detect_tray":
            self._progress, locked = self.plate_det.process(frame, display)
            if locked is not None:
                self.drop_xy = locked
                self._progress = 0
                self._log(f"tray re-locked at {locked[0]:.0f},{locked[1]:.0f}.")
                self._complete_step()

    # ---- frame output ------------------------------------------------------ #
    def _draw_banner(self, display):
        h, w = display.shape[:2]
        cv2.rectangle(display, (0, 0), (w, 26), (0, 0, 0), -1)
        cv2.putText(display, f"{self._phase}  |  {self._message}", (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        if self._awaiting:
            cv2.rectangle(display, (0, 27), (w, 50), (0, 140, 200), -1)
            cv2.putText(display, f"CONFIRM: {self._prompt}", (8, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        if self._safety_state == "STOP":
            cv2.rectangle(display, (2, 2), (w - 3, h - 3), (0, 0, 255), 4)
        if self.paused:
            cv2.putText(display, "PAUSED", (w - 110, h - 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

    def _publish(self, display):
        self._sync_status()
        self._draw_banner(display)
        ok, buf = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.state.set_jpeg(buf.tobytes())

    def _publish_placeholder(self):
        img = np.zeros((480, 640, 3), np.uint8)
        cv2.putText(img, f"NO CAMERA (index {self.camera_index})", (60, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(img, "Reseat the Orbbec USB cable, or pass --camera 0.", (40, 260),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        self._message = "no camera — retrying"
        self._sync_status()
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.state.set_jpeg(buf.tobytes())

    def _sync_status(self):
        step_index = self.step_index if self._phase == "PICK_PLACE" else -1
        self.state.set_status(
            phase=self._phase, message=self._message, progress=self._progress,
            step_index=step_index, robot_mode=self.robot_mode,
            safety=self._safety_state, hand=self._hand,
            step_mode=self.step_mode, loop_mode=self.loop_mode,
            confirm_danger=self.confirm_danger, paused=self.paused,
            awaiting=self._awaiting, prompt=self._prompt,
            part_index=self.part_index, total_parts=self.total_parts,
            tray=([round(self.tray_xy[0], 1), round(self.tray_xy[1], 1)] if self.tray_xy else None),
            log=list(self._log_dq),
        )

    # ---- main loop --------------------------------------------------------- #
    def run(self):
        if not self._open_camera():
            self._log(f"camera index {self.camera_index} unavailable at start.")
        self._load_calibration()
        self._build_safety()

        miss = 0
        while not self.state.quit:
            if self.cap is None:
                self._publish_placeholder()
                time.sleep(0.1)
                miss += 1
                if miss % 30 == 0 and self._open_camera():  # ~3s retry
                    self._load_calibration()
                    self._build_safety()
                # still honour global commands (so Stop/quit work without a camera)
                self._handle_global(self.state.pop_cmds())
                continue
            miss = 0

            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.cap.release()
                self.cap = None
                self._log("camera dropped a frame — will retry.")
                continue

            if self.map1 is not None:
                frame = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
            display = frame.copy()

            # --- safety perception every frame (keeps the overlay + gate live) ---
            safe = True
            centroids = []
            if self.hand_detector and self.supervisor:
                try:
                    res = self.hand_detector.process(frame)
                    centroids = [hd.centroid for hd in res.hands]
                    self.supervisor.update(centroids)
                    safety.draw_overlay(display, centroids, self.supervisor)
                    self._safety_state = self.supervisor.state
                    self._hand = len(centroids) > 0
                    safe = self.supervisor.is_safe
                except Exception:
                    pass

            # --- consume commands ---
            cmds = self.state.pop_cmds()
            self._handle_global(cmds)

            # --- advance an in-flight move; block phase progress until it finishes ---
            if self._move is not None:
                if not self._poll_move():
                    self._publish(display)
                    continue
                cb, self._on_done = self._on_done, None
                if cb:
                    cb()

            # --- phase state machine (skipped while paused) ---
            if not self.paused:
                self._dispatch(frame, display, cmds, safe)

            self._publish(display)

        self._cleanup()

    def _cleanup(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        try:
            if not self.sim:
                dType.DisconnectDobot(self.api)
        except Exception:
            pass
        try:
            if self.hand_detector is not None:
                self.hand_detector.close()
        except Exception:
            pass
        print("[pick_web] worker stopped.")


# --------------------------------------------------------------------------- #
# Web page  (no .format() -> CSS braces stay literal; tokens replaced below)
# --------------------------------------------------------------------------- #
_PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pick &amp; Place — Control Panel</title>
<style>
 :root{--bg:#0e0f12;--panel:#181a1f;--line:#2a2d34;--txt:#e8e8ea;--muted:#8a8f98;}
 *{box-sizing:border-box}
 body{font-family:system-ui,Segoe UI,sans-serif;background:var(--bg);color:var(--txt);margin:0;padding:14px}
 h1{font-size:17px;margin:0 0 12px}
 .wrap{display:flex;gap:14px;flex-wrap:wrap;align-items:flex-start}
 .col{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px}
 #feed{display:block;width:640px;max-width:96vw;border-radius:8px;background:#000}
 .right{flex:1;min-width:320px}
 .badges{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px}
 .badge{font-size:12px;padding:3px 10px;border-radius:12px;background:#23262d;border:1px solid var(--line)}
 .badge b{color:#fff}
 .ok{background:#0a5a33;border-color:#0a5a33}
 .stop{background:#7a1020;border-color:#7a1020}
 #awaiting{display:none;background:#1d3a52;border:1px solid #2f6da3;border-radius:8px;padding:8px 10px;margin-bottom:10px;font-weight:600;animation:pulse 1.1s infinite}
 @keyframes pulse{50%{background:#27557d}}
 .flow{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:8px}
 .node{font-size:12px;padding:4px 9px;border-radius:6px;background:#23262d;border:1px solid var(--line);color:var(--muted)}
 .node.active{background:#2563eb;border-color:#2563eb;color:#fff;font-weight:700}
 .node.halt{background:#7a1020;border-color:#7a1020;color:#fff}
 .bar{height:8px;background:#23262d;border-radius:5px;overflow:hidden;margin:8px 0}
 .bar>i{display:block;height:100%;width:0;background:#3b82f6;transition:width .15s}
 .steps{margin:8px 0;border-top:1px solid var(--line);padding-top:8px}
 .step{font-size:12px;padding:3px 8px;border-radius:5px;color:var(--muted);display:flex;gap:8px}
 .step.active{background:#16351f;color:#9be9a8;font-weight:700}
 .step.done{color:#5f6671;text-decoration:line-through}
 .controls{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:10px 0}
 button{font-size:14px;padding:10px;border:0;border-radius:7px;cursor:pointer;color:#fff;background:#33373f}
 button:hover{filter:brightness(1.12)}
 #start{background:#059669}#confirm{background:#2563eb}#estop{background:#dc2626;grid-column:1/3;font-weight:800;font-size:16px}
 .toggles{display:flex;gap:8px;flex-wrap:wrap;margin:8px 0}
 .tg{font-size:12px;padding:7px 11px;border-radius:7px;background:#23262d;border:1px solid var(--line);cursor:pointer;color:var(--muted)}
 .tg.on{background:#3b3f0e;border-color:#8a8f1e;color:#f2f3b0}
 #log{font-family:ui-monospace,Consolas,monospace;font-size:11px;color:var(--muted);background:#101216;border:1px solid var(--line);border-radius:7px;padding:8px;height:130px;overflow:auto;white-space:pre-wrap}
 small{color:var(--muted)}
</style></head><body>
 <h1>Pick &amp; Place — Runtime Control Panel</h1>
 <div class="wrap">
   <div class="col"><img id="feed" src="/stream"></div>
   <div class="col right">
     <div class="badges">
       <span class="badge">Robot: <b id="robot">?</b></span>
       <span class="badge" id="safetyb">Safety: <b id="safety">—</b></span>
       <span class="badge">Parts: <b id="parts">0/0</b></span>
       <span class="badge">Tray: <b id="tray">—</b></span>
     </div>
     <div id="awaiting"></div>
     <div class="flow" id="flow"></div>
     <div class="bar"><i id="barfill"></i></div>
     <div class="steps" id="steps"></div>
     <div class="controls">
       <button id="start" onclick="cmd('start')">▶ Start</button>
       <button id="pause" onclick="cmd('pause')">⏸ Pause</button>
       <button id="confirm" onclick="cmd('confirm')">✔ Confirm / Next</button>
       <button id="home" onclick="cmd('home')">⌂ Home</button>
       <button id="stop" onclick="cmd('stop')">■ Stop</button>
       <button id="estop" onclick="cmd('estop')">⛔ E-STOP</button>
     </div>
     <div class="toggles">
       <span class="tg" id="tg_step"    onclick="cmd('mode_step')">Step mode</span>
       <span class="tg" id="tg_loop"    onclick="cmd('mode_loop')">Loop</span>
       <span class="tg" id="tg_confirm" onclick="cmd('mode_confirm')">Confirm @ danger</span>
     </div>
     <small>Keys: <b>Enter</b>=Confirm/Next · <b>p</b>=Pause · <b>s</b>=Stop · <b>Esc</b>=E-STOP</small>
     <div id="log" style="margin-top:10px"></div>
   </div>
 </div>
<script>
 const PHASES=__PHASES__, STEPS=__STEPS__;
 function cmd(c){fetch('/cmd/'+c,{method:'POST'});}
 document.addEventListener('keydown',e=>{
   if(e.key==='Enter'){e.preventDefault();cmd('confirm');}
   else if(e.key==='p')cmd('pause');
   else if(e.key==='s')cmd('stop');
   else if(e.key==='Escape')cmd('estop');
 });
 function setTg(id,on){const el=document.getElementById(id);el.classList.toggle('on',!!on);}
 async function poll(){
   let s; try{ s=await (await fetch('/status')).json(); }catch(e){ return; }
   document.getElementById('robot').textContent=s.robot_mode;
   document.getElementById('safety').textContent=s.safety+(s.hand?' (hand)':'');
   const sb=document.getElementById('safetyb');
   sb.className='badge'+(s.safety==='STOP'?' stop':(s.safety==='NORMAL'?' ok':''));
   document.getElementById('parts').textContent=s.part_index+'/'+s.total_parts;
   document.getElementById('tray').textContent=s.tray?('x'+s.tray[0]+' y'+s.tray[1]):'—';

   const aw=document.getElementById('awaiting');
   aw.style.display=s.awaiting?'block':'none';
   aw.textContent='⏳ '+s.prompt;

   // flow ladder
   const flow=document.getElementById('flow'); flow.innerHTML='';
   const halted=(s.phase==='HALTED'||s.phase==='ERROR');
   PHASES.forEach(p=>{
     const d=document.createElement('div'); d.className='node'+(p===s.phase?' active':'');
     d.textContent=p.replace('_',' '); flow.appendChild(d);
   });
   if(halted){const d=document.createElement('div');d.className='node halt';d.textContent=s.phase;flow.appendChild(d);}

   // progress bar (scan phases)
   const scan=(s.phase==='SCAN_PLATE'||s.phase==='SCAN_TARGET');
   document.getElementById('barfill').style.width=(scan?s.progress:0)+'%';

   // pick/place step list
   const steps=document.getElementById('steps'); steps.innerHTML='';
   if(s.phase==='PICK_PLACE'){
     STEPS.forEach((lbl,i)=>{
       const d=document.createElement('div');
       d.className='step'+(i===s.step_index?' active':(i<s.step_index?' done':''));
       d.textContent=(i===s.step_index?'▶ ':'   ')+lbl; steps.appendChild(d);
     });
   }
   setTg('tg_step',s.step_mode); setTg('tg_loop',s.loop_mode); setTg('tg_confirm',s.confirm_danger);
   document.getElementById('pause').textContent=s.paused?'▶ Resume':'⏸ Pause';
   document.getElementById('log').textContent=(s.log||[]).slice().reverse().join('\n');
 }
 setInterval(poll,300); poll();
</script>
</body></html>"""


def _render_page():
    return (_PAGE
            .replace("__PHASES__", json.dumps(PHASES))
            .replace("__STEPS__", json.dumps([s["label"] for s in STEP_DEFS])))


# --------------------------------------------------------------------------- #
# HTTP handlers
# --------------------------------------------------------------------------- #
def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass  # silence per-request console spam

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", _render_page().encode("utf-8"))
            elif self.path == "/status":
                self._send(200, "application/json", json.dumps(state.get_status()).encode("utf-8"))
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while not state.quit:
                        jpg = state.get_jpeg()
                        if jpg is not None:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                            self.wfile.write(("Content-Length: %d\r\n\r\n" % len(jpg)).encode())
                            self.wfile.write(jpg + b"\r\n")
                        time.sleep(0.04)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path.startswith("/cmd/"):
                c = self.path.rsplit("/", 1)[-1]
                if c == "quit":
                    state.quit = True
                else:
                    state.push_cmd(c)
                self.send_response(204)
                self.end_headers()
            else:
                self.send_error(404)

    return Handler


def main():
    ap = argparse.ArgumentParser(description="Web control panel for the pick-and-place runtime.")
    ap.add_argument("--port", type=int, default=WEB_PORT, help=f"web port (default {WEB_PORT})")
    ap.add_argument("--camera", type=int, default=CAMERA_INDEX,
                    help=f"camera index (default {CAMERA_INDEX} = Orbbec; 0 = laptop webcam)")
    args = ap.parse_args()

    state = RunState()
    runtime = PickRuntime(state, args.camera)
    worker = threading.Thread(target=runtime.run, daemon=True)
    worker.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(state))
    print("\n──────────────────────────────────────────────")
    print(" PICK & PLACE — WEB CONTROL PANEL")
    print(f" • Open in your browser:  http://localhost:{args.port}/")
    print(f" • Camera index {args.camera} (1 = Orbbec, 0 = laptop webcam)")
    print(" • No Dobot found → runs in SIMULATION so the flow is still demoable.")
    print(" • Ctrl+C here stops the server.")
    print("──────────────────────────────────────────────\n")

    def _watch():
        while not state.quit:
            time.sleep(0.25)
        server.shutdown()
    threading.Thread(target=_watch, daemon=True).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nInterrupted — shutting down.")
    finally:
        state.quit = True
        server.server_close()


if __name__ == "__main__":
    main()
