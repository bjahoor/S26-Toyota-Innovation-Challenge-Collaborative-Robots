r"""
goto_caliper.py - calibration sanity check: move the arm to the red caliper.

Purpose: you don't want to run the whole pick/place state machine just to find
out whether the pixel -> robot mapping is dialed in. This tool does the minimum:
detect the red caliper, map its centroid to robot mm via the SAME homography
pickCVBlock.py uses, and -- only when you press a key -- send the arm to hover
directly over it at a safe height. If the gripper lines up over the part, the
calibration is good. If it's off by some consistent offset, you'll see exactly
how much and in which direction.

It deliberately does NOT pick anything up. Hover (Z_SAFE) checks the X/Y mapping;
an optional descend (Z_PICK) checks the Z. Nothing moves until you ask it to.

Detection + constants mirror pickCVBlock.py.phase_detect_targets() so what this
sees is what the real pick script sees -- do not re-tune them here; if detection
misbehaves, fix lighting/placement (see the team notes). pickCVBlock.py is the
source of truth for these values.

Controls (focus the "goto_caliper" window):
    SPACE  move the arm to hover over the detected caliper at Z_SAFE
    d      descend to Z_PICK at the last hover point (check the pick height)
    u      lift back up to Z_SAFE at the last point
    h      send the arm home (out of the camera's view) so you can re-detect
    q/Esc  go home, then quit

Run (UTF-8 needed for the Dobot DLL; the script also self-reconfigures stdout):
    $env:PYTHONUTF8 = "1"
    cd Collaborative_Robotics
    ..\.venv\Scripts\python.exe goto_caliper.py        # Orbbec (index 1, calibrated camera)
    ..\.venv\Scripts\python.exe goto_caliper.py 1      # explicit camera index

NOTE: the homography is only valid for the camera it was calibrated on (the
Orbbec, index 1). Running on any other camera makes the robot coordinates
meaningless.
"""

import os
import sys

# The vendored Dobot DLL prints a non-ASCII char ('：') the moment it loads, which
# crashes under Windows' default cp1252 console. Force UTF-8 BEFORE importing
# dobotArm (its module-level dType.load() triggers that print). Same fix as
# pickCVBlock.py / controlArm.py.
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

# Resolve paths relative to THIS file so it runs from any working directory, and
# put this folder on sys.path so `import lib.DobotDllType` / `import dobotArm`
# resolve no matter where you launch from (same pattern as controlArm.py).
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import numpy as np
import cv2

import lib.DobotDllType as dType
import dobotArm


# --- Heights (mm). Mirror pickCVBlock.py. ---
Z_SAFE = 40    # hover height: clears the table/part, used for the X/Y check
Z_PICK = -25   # pick height: where the claw would actually grab (the Z check)

# --- Red caliper detection. Mirror pickCVBlock.py.phase_detect_targets(). ---
TARGET_MIN_AREA = 30      # smallest red blob that counts as a part
TARGET_MAX_AREA = 500     # largest red blob (rejects big things like a hand)
TARGET_MIN_SAT = 150      # vivid-red floor (drops duller skin tones)
TARGET_MIN_VAL = 100      # brightness floor

# --- Reach guard (mm). The Magician's workspace is limited; a bad detection
#     could map to a point that flings the arm into a hard limit. The calibration
#     grid (getTransformationMatrix.py) covered x in [200,260], y in [-80,40], so
#     anything well outside this generous box is almost certainly a bad mapping,
#     not a real part -> refuse the move and say so. Widen if you intentionally
#     work outside it. ---
REACH_X_MIN, REACH_X_MAX = 120.0, 330.0
REACH_Y_MIN, REACH_Y_MAX = -200.0, 200.0

CAMERA_INDEX = 1  # the Orbbec (calibrated). Override via argv[1] if you must.

# Stability: a detection must hold within this many pixels for this many frames
# before it's shown as "STABLE" (green). You can still command a move on an
# unstable point, but stable means the mapping isn't jittering.
PIXEL_TOLERANCE = 10
STABLE_FRAMES = 5


def pixel_to_robot(u, v, H):
    """Map an image pixel (u, v) to robot-frame (x, y) mm via the homography.
    Identical to pickCVBlock.pixel_to_robot()."""
    p = np.array([u, v, 1.0])
    xy = H @ p
    xy /= xy[2]
    return xy[0], xy[1]


def in_reach(x, y):
    return (REACH_X_MIN <= x <= REACH_X_MAX) and (REACH_Y_MIN <= y <= REACH_Y_MAX)


def detect_calipers(frame, H):
    """Return a list of detections [(cx, cy, rx, ry, area), ...] in this frame,
    using the exact color+size gate from pickCVBlock.phase_detect_targets()."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (3, 3), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, TARGET_MIN_SAT, TARGET_MIN_VAL]),
                            np.array([10, 255, 255])) + \
           cv2.inRange(hsv, np.array([170, TARGET_MIN_SAT, TARGET_MIN_VAL]),
                            np.array([180, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dets = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if TARGET_MIN_AREA < area < TARGET_MAX_AREA:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                rx, ry = pixel_to_robot(cx, cy, H)
                dets.append((cx, cy, rx, ry, area))
    return dets, mask


def open_camera(index):
    """Open a camera and confirm it actually DELIVERS a frame (the Orbbec can open
    but hand back nothing -- always verify a real frame, not just isOpened())."""
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(
            f"Cannot open camera index {index}. The calibrated camera is the "
            "Orbbec (index 1). If it isn't enumerating, reseat its USB cable."
        )
    frame = None
    for _ in range(10):  # first reads off a fresh camera are often empty
        ret, frame = cap.read()
        if ret and frame is not None:
            break
    if frame is None:
        cap.release()
        raise RuntimeError(
            f"Camera index {index} opened but returned no frames. "
            "Reseat the Orbbec USB cable and retry."
        )
    return cap, frame


def _shutdown(api, cap, robot_ready, interrupted):
    """Release the camera and bring the robot to a safe, disconnected state.

    Order matters: free the camera FIRST (instant, non-blocking, and the Orbbec's
    USB handle is the resource most likely to wedge), so it's released even if the
    robot cleanup below stalls or eats a second Ctrl+C. Then, only if the robot
    actually came up:
      * clean exit (q/Esc/other exception) -> park home; the arm is idle because
        every move is blocking and finishes before the next keypress is read.
      * interrupted (Ctrl+C) -> a move may be in flight, so force-stop + clear the
        queue and do NOT issue a fresh blocking move (after a clear it could
        busy-wait forever).
    Always disconnect so the serial port is freed (prevents 'DobotConnect_Occupied'
    on the next launch). Every robot call is guarded individually.
    """
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()

    if api is None:
        return
    try:
        if robot_ready and interrupted:
            dobotArm.stop_motion(api)          # abort the in-flight move
        elif robot_ready:
            print("Returning the arm home...")
            dobotArm.move_to_home(api)
    except Exception:
        pass
    try:
        dType.DisconnectDobot(api)             # free the serial port for next run
    except Exception:
        pass
    print("Disconnected. Done.")


def main():
    index = CAMERA_INDEX
    if len(sys.argv) > 1:
        index = int(sys.argv[1])
        if index != 1:
            print(f"[warn] camera index {index} is not the calibrated Orbbec (1); "
                  "the robot coordinates will be meaningless.")

    # --- camera + undistort maps (same setup as pickCVBlock.py) ---
    H = np.load(os.path.join(_HERE, "HomographyMatrix.npy"))
    data = np.load(os.path.join(_HERE, "camera_params.npz"))
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]

    # Everything that opens a resource (camera, robot) lives INSIDE the try so the
    # finally always cleans up. cap/api start None; robot_ready gates motion in the
    # shutdown path; interrupted picks force-stop vs. park-home on the way out.
    cap = None
    api = None
    robot_ready = False
    interrupted = False
    try:
        cap, frame = open_camera(index)
        h, w = frame.shape[:2]
        new_K, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1)
        map1, map2 = cv2.initUndistortRectifyMap(
            camera_matrix, dist_coeffs, None, new_K, (w, h), cv2.CV_16SC2)

        # robot: connect + home (homing moves the arm OUT of the camera view)
        print("Initializing robot (it will home -- keep the workspace clear)...")
        api = dType.load()
        dobotArm.initialize_robot(api)   # exits the process if it can't connect
        robot_ready = True
        dobotArm.open_gripper(api)   # open so a hover/descend visibly straddles the part
        dobotArm.stop_pump(api)
        print("Robot homed. Hold the caliper in view; press SPACE to send the arm over it.")

        last_xy = None                 # last commanded (x, y); used by d / u
        stable_pt = None               # last pixel centroid, for the stability counter
        stable_count = 0

        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue
            frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            display = frame.copy()

            dets, _ = detect_calipers(frame, H)

            # Choose the strongest detection (largest area) as THE target.
            target = max(dets, key=lambda d: d[4]) if dets else None

            # Stability: is the target sitting still?
            if target is not None:
                cx, cy = target[0], target[1]
                if stable_pt is not None and \
                   abs(cx - stable_pt[0]) < PIXEL_TOLERANCE and \
                   abs(cy - stable_pt[1]) < PIXEL_TOLERANCE:
                    stable_count += 1
                else:
                    stable_count = 0
                stable_pt = (cx, cy)
            else:
                stable_count = 0
                stable_pt = None
            is_stable = stable_count >= STABLE_FRAMES

            # Draw every detection; highlight the chosen target.
            for (dx, dy, rx, ry, area) in dets:
                is_target = target is not None and (dx, dy) == (target[0], target[1])
                color = (0, 255, 255) if is_target else (0, 180, 0)
                cv2.circle(display, (dx, dy), 6, color, -1)
                if is_target:
                    reach_ok = in_reach(rx, ry)
                    tag = "" if reach_ok else "  OUT OF REACH"
                    cv2.putText(display, f"robot x={rx:6.1f} y={ry:6.1f}{tag}",
                                (dx + 10, dy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (0, 255, 0) if reach_ok else (0, 0, 255), 2)

            # Status banner.
            if target is None:
                banner = "no caliper detected"
                bcolor = (0, 0, 255)
            else:
                banner = ("STABLE" if is_stable else "detecting...") + \
                         "   SPACE=hover  d=descend  u=up  h=home  q=quit"
                bcolor = (0, 255, 0) if is_stable else (0, 255, 255)
            cv2.putText(display, banner, (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, bcolor, 2)
            if last_xy is not None:
                cv2.putText(display, f"last move: x={last_xy[0]:.1f} y={last_xy[1]:.1f}",
                            (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            cv2.imshow("goto_caliper", display)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):       # q or Esc
                break

            elif key == ord(" "):           # hover over the detected caliper
                if target is None:
                    print("[skip] no caliper detected.")
                    continue
                rx, ry = target[2], target[3]
                if not in_reach(rx, ry):
                    print(f"[refused] target ({rx:.1f}, {ry:.1f}) is outside the reach "
                          f"box x[{REACH_X_MIN},{REACH_X_MAX}] y[{REACH_Y_MIN},"
                          f"{REACH_Y_MAX}] -- almost certainly a bad detection. "
                          "Not moving.")
                    continue
                print(f"-> hovering over caliper at robot x={rx:.1f} y={ry:.1f}, z={Z_SAFE}")
                dobotArm.move_to_xyz(api, rx, ry, Z_SAFE)
                last_xy = (rx, ry)
                print("   arrived. Eyeball: is the gripper centered over the part?")

            elif key == ord("d"):           # descend to the pick height
                if last_xy is None:
                    print("[skip] hover first (SPACE), then descend.")
                    continue
                print(f"-> descending to z={Z_PICK} at x={last_xy[0]:.1f} y={last_xy[1]:.1f}")
                dobotArm.move_to_xyz(api, last_xy[0], last_xy[1], Z_PICK)
                print("   at pick height. 'u' to lift back up.")

            elif key == ord("u"):           # lift back to safe height
                if last_xy is None:
                    print("[skip] nothing to lift from yet.")
                    continue
                dobotArm.move_to_xyz(api, last_xy[0], last_xy[1], Z_SAFE)
                print("   lifted to Z_SAFE.")

            elif key == ord("h"):           # park out of the camera's view
                print("-> homing (out of view) so you can re-detect.")
                dobotArm.move_to_home(api)
    except KeyboardInterrupt:
        # Ctrl+C: a blocking move may be in flight. Flag it so shutdown force-stops
        # the arm rather than issuing another (possibly-hanging) blocking move.
        interrupted = True
        print("\n[Ctrl+C] interrupting -- stopping the arm.")
    finally:
        _shutdown(api, cap, robot_ready, interrupted)


if __name__ == "__main__":
    main()
