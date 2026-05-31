import sys

# The vendored Dobot DLL prints a non-ASCII char ('：') the moment it loads, which
# crashes under Windows' default cp1252 console encoding. Force UTF-8 on our streams
# BEFORE importing dobotArm (its module-level dType.load() triggers that print).
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

import lib.DobotDllType as dType
import dobotArm
import time
import numpy as np
import cv2
import os

# Useful Global Variables
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

cam = cv2.VideoCapture(1)   # Orbbec is index 1 (index 0 is the laptop webcam). The
                            # homography MUST be built from the same camera it will be
                            # used with, so this has to match pickCVBlock's CAMERA_INDEX.

if not cam.isOpened():
    print("Camera failed to open")
    exit()

# Loads from the current working directory (run this script from Collaborative_Robotics/).
data = np.load("camera_params.npz")
camera_matrix = data["camera_matrix"]
dist_coeffs   = data["dist_coeffs"]

# compute undistort maps once. Warm up first: the initial reads off a freshly-opened
# camera are often empty, so loop until we get a real frame instead of crashing on None.
frame = None
for _ in range(15):
    ret, frame = cam.read()
    if ret and frame is not None:
        break
if frame is None:
    raise RuntimeError("Camera index 1 opened but returned no frames "
                       "(Orbbec dropped off USB? reseat the cable and retry).")
h,w = frame.shape[:2]

new_K, roi = cv2.getOptimalNewCameraMatrix(
    camera_matrix,
    dist_coeffs,
    (w,h),
    1
)

map1, map2 = cv2.initUndistortRectifyMap(
    camera_matrix,
    dist_coeffs,
    None,
    new_K,
    (w,h),
    cv2.CV_16SC2
)

api = dType.load()

# robot coordinates in mm
robot_points = np.array([
    [200,-80],
    [230,-80],
    [260,-80],

    [200,-40],
    [230,-40],
    [260,-40],

    [200,0],
    [230,0],
    [260,0],

    [200,40],
    [230,40],
    [260,40]
], dtype=np.float32)


# --- Marker detection tuning ---
# Only red blobs in this pixel-area window count as the marker. This rejects tiny
# specks (noise/glints) and large red things (a sleeve, the red part if it's bigger).
# Widen if your marker isn't detected; narrow if it locks onto the wrong red object.
MARKER_MIN_AREA = 20
MARKER_MAX_AREA = 2000
# Two calibration points whose pixels are closer than this are treated as the SAME
# point — the usual sign the marker didn't move (or the detector locked a fixed red
# object). The original bug: 3 points 60 mm apart all read ~the same pixel, which
# makes findHomography degenerate. We refuse those.
MIN_POINT_SEPARATION_PX = 15


def detect_red_blobs(frame):
    """Return all qualifying red blobs as a list of (cx, cy, area), largest first."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower1 = np.array([0,120,70]);   upper1 = np.array([10,255,255])
    lower2 = np.array([170,120,70]); upper2 = np.array([180,255,255])
    mask = cv2.inRange(hsv,lower1,upper1) + cv2.inRange(hsv,lower2,upper2)
    mask = cv2.medianBlur(mask,5)

    contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blobs = []
    for c in contours:
        a = cv2.contourArea(c)
        if not (MARKER_MIN_AREA < a < MARKER_MAX_AREA):
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"]/M["m00"]); cy = int(M["m01"]/M["m00"])
        blobs.append((cx, cy, a))
    blobs.sort(key=lambda b: b[2], reverse=True)
    return blobs


def detect_red_center(frame):
    """Largest in-range red blob, or None. (Thin wrapper kept for compatibility.)"""
    blobs = detect_red_blobs(frame)
    if not blobs:
        return None
    cx, cy, _ = blobs[0]
    return cx, cy


# ------------------------------------------------
# CALIBRATION
# ------------------------------------------------

def _too_close(candidate, saved):
    """True if `candidate` pixel is within MIN_POINT_SEPARATION_PX of any saved point."""
    for (su, sv) in saved:
        if ((candidate[0]-su)**2 + (candidate[1]-sv)**2) ** 0.5 < MIN_POINT_SEPARATION_PX:
            return (su, sv)
    return None


def collect_calibration():

    pixel_points = []
    total = len(robot_points)

    for idx, pt in enumerate(robot_points):

        x, y = pt

        print("\n----------------------------------")
        print(f"Point {idx+1}/{total}  ->  robot {pt}")

        # move to pick height
        dobotArm.move_to_xyz(api, x, y, -24)

        print("Press SPACE when robot is in position (or Q to abort)")

        # wait for space
        aborted = False
        while True:
            ret, frame = cam.read()
            if not ret or frame is None:
                continue
            frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

            cv2.putText(frame, f"Point {idx+1}/{total} robot{tuple(pt)}: tip in position? SPACE",
                        (20,35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.imshow("Calibration", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 32:        # space
                break
            if key in (ord('q'), 27):  # q / Esc
                aborted = True
                break
        if aborted:
            print("Aborted by user.")
            break

        # move robot away so camera can see the spot
        print("Moving robot away")
        dobotArm.move_to_xyz(api, 200, 0, 80)

        print("Place the RED marker exactly where the tip was. SPACE=save, R=skip-retry, Q=abort")

        while True:
            ret, frame = cam.read()
            if not ret or frame is None:
                continue
            frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            disp = frame.copy()

            blobs = detect_red_blobs(frame)
            detected = (blobs[0][0], blobs[0][1]) if blobs else None

            # Draw every red candidate; highlight the chosen (largest) one.
            for i,(bx,by,ba) in enumerate(blobs):
                col = (0,0,255) if i==0 else (0,180,255)
                cv2.circle(disp,(bx,by),6,col,-1)
                cv2.putText(disp,f"a={int(ba)}",(bx+8,by),cv2.FONT_HERSHEY_SIMPLEX,0.5,col,1)
            # Show already-saved points so the user can see they must pick a NEW spot.
            for (su,sv) in pixel_points:
                cv2.drawMarker(disp,(int(su),int(sv)),(255,0,0),cv2.MARKER_TILTED_CROSS,12,2)

            dup = _too_close(detected, pixel_points) if detected else None
            if detected is None:
                status = "no marker in size range - place it / adjust lighting"; scol=(0,165,255)
            elif dup is not None:
                status = f"SAME as a saved point {dup} - MOVE the marker"; scol=(0,0,255)
            else:
                status = f"marker at {detected}  SPACE to save"; scol=(0,255,0)

            cv2.putText(disp, f"Point {idx+1}/{total} robot{tuple(pt)}  captured={len(pixel_points)}",
                        (20,35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.putText(disp, status, (20,65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, scol, 2)
            cv2.imshow("Calibration", disp)

            key = cv2.waitKey(1) & 0xFF
            if key == 32 and detected is not None and dup is None:
                print(f"  saved pixel {detected} for robot {pt}")
                pixel_points.append(detected)
                break
            if key == 32 and dup is not None:
                print(f"  REJECTED {detected}: too close to {dup}. Move the marker and retry.")
            if key in (ord('q'), 27):
                aborted = True
                break
        if aborted:
            print("Aborted by user.")
            break

    return np.array(pixel_points, dtype=np.float32)


def compute_homography(pixel_points):
    # Use only the robot points we actually captured pixels for (collection may have
    # been aborted early). findHomography needs the two arrays to correspond 1:1.
    used_robot = robot_points[:len(pixel_points)]

    H, status = cv2.findHomography(pixel_points, used_robot)
    if H is None:
        print("findHomography FAILED (points likely collinear/degenerate). NOT saving.")
        return None

    # Reprojection error: push each captured pixel through H and compare to the known
    # robot mm. This tells us the calibration quality WITHOUT moving the arm.
    def p2r(u, v):
        p = np.array([u, v, 1.0]); xy = H @ p; xy /= xy[2]; return xy[0], xy[1]
    errs = []
    for (u, v), (rx, ry) in zip(pixel_points, used_robot):
        ex, ey = p2r(u, v)
        errs.append(((ex-rx)**2 + (ey-ry)**2) ** 0.5)
    errs = np.array(errs)
    inliers = int(status.sum()) if status is not None else len(pixel_points)

    print("\nHomography Matrix\n", H)
    print(f"\nReprojection error: mean={errs.mean():.2f} mm  max={errs.max():.2f} mm  "
          f"(inliers {inliers}/{len(pixel_points)})")

    # A good hand-eye homography here should be well under a few mm mean. If it's huge,
    # the point set was bad (duplicates / mislabeled) — don't clobber the saved file.
    if errs.mean() > 10.0:
        print("** Mean error > 10 mm — calibration looks BAD. NOT saving HomographyMatrix.npy. **")
        print("   (Re-run; make sure each marker placement is a distinct, correct spot.)")
        return None

    np.save("HomographyMatrix.npy", H)
    print("Matrix saved -> HomographyMatrix.npy")
    return H


# ------------------------------------------------
# MAIN
# ------------------------------------------------

def run():
    dobotArm.initialize_robot(api)

    pixel_points = collect_calibration()

    if len(pixel_points) < 4:
        print(f"Only {len(pixel_points)} points captured — need >= 4. Nothing saved.")
        cam.release(); cv2.destroyAllWindows()
        return

    compute_homography(pixel_points)

    cam.release()
    cv2.destroyAllWindows()


# Good Luck!
run()