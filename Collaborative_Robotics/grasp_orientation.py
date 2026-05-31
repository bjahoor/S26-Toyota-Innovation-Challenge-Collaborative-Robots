r"""
grasp_orientation.py - figure out which way the gripper must rotate to grab an
odd-shaped part by its SHORT side.

THE IDEA (what the user asked for)
----------------------------------
The red brake caliper is roughly rectangular. A two-finger gripper grabs most
reliably when its jaws close across the part's *narrow* dimension -- i.e. the
fingers press the two long faces, closing along the SHORT axis. So:

    1. find the part's LONG axis in the camera image (PCA / minAreaRect),
    2. the jaws must close PERPENDICULAR to it (across the short side),
    3. rotate the gripper (the `rHead` arg of dobotArm.move_to_xyz) to match.

Because a rectangle is identical under a 180 deg flip and a parallel grip is
symmetric, every possible part orientation folds into a 180 deg span -- which
fits exactly inside the gripper's +/-90 deg R range. So a valid grip angle
always exists (see wrap_to_pm90).

THE ONE SUBTLETY -- image angle != gripper angle
------------------------------------------------
OpenCV gives the angle in PIXEL space; the gripper rotates in ROBOT space, and
the camera->robot homography is roughly a 180 deg flip with a few degrees of
skew (see HomographyMatrix.npy / H_matrix.json). So we do NOT use the image
angle directly: we step a short segment along the long axis, map BOTH endpoints
through the homography with pixel_to_robot(), and take atan2 of the robot-space
delta. That folds the camera->robot rotation in automatically.

After that there is a single fixed offset between "robot-space long-axis angle"
and the gripper's R=0 reference (plus any few-degree mounting error). That is
GRIP_R_OFFSET_DEG -- set it ONCE with a quick test on the arm (see below).

This module is PURE GEOMETRY: you pass in a contour (the caller keeps its own
tuned color mask) and it returns a GraspPose. It never imports the Dobot DLL and
never moves the arm, so it is safe to import and to run on any machine.

HOW TO WIRE INTO pickCVBlock.py (no re-tuning of TARGET_*; one extra call)
-------------------------------------------------------------------------
In phase_detect_targets(), where you already have the contour `cnt` and compute
the centroid, also compute the grip angle:

    import grasp_orientation
    ...
    grasp = grasp_orientation.grasp_from_contour(cnt, H_matrix)
    if grasp is not None:
        rx, ry = grasp.robot_xy            # same pick point you already use
        rhead  = grasp.grip_r_deg          # NEW: how far to rotate the gripper
        current_list.append((rx, ry, rhead))

Then in the pick sequence pass it as the rotation (pre-rotate at safe height so
the jaws are already aligned before descending):

    dobotArm.move_to_xyz(api, pick_x, pick_y, Z_SAFE, rhead)
    dobotArm.move_to_xyz(api, pick_x, pick_y, Z_PICK, rhead)
    dobotArm.close_gripper(api)

CALIBRATING GRIP_R_OFFSET_DEG (one time, needs the arm)
-------------------------------------------------------
1. Lay the caliper at a known angle in view; run the preview to read grip_r_deg.
2. Command that grip_r as rHead and lower onto the part.
3. If the jaws line up to clamp the SHORT side -> offset is correct (leave 0).
   If they clamp the LONG side instead -> set GRIP_R_OFFSET_DEG = 90.
   If they are off by a constant few degrees -> add that residual here.

STANDALONE PREVIEW (no robot)
-----------------------------
Mirrors pickCVBlock's red mask just to feed contours in for eyeballing; loads
HomographyMatrix.npy + camera_params.npz (files only, no arm) so the grip angle
shown is the REAL robot-space value.

    ..\.venv\Scripts\python.exe grasp_orientation.py        # Orbbec (index 1)
    ..\.venv\Scripts\python.exe grasp_orientation.py 0      # laptop webcam
"""

import os
import sys
import math
from collections import namedtuple

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# CALIBRATION CONSTANT -- set once on the arm (see module docstring).
# Pure offset between the robot-space SHORT-axis angle and the gripper's R=0
# reference. Expected near 0; use 90 if picks clamp the long side instead.
# ---------------------------------------------------------------------------
GRIP_R_OFFSET_DEG = 0.0

# How far (in pixels) to step along the long axis when converting the image
# angle to a robot angle through the homography. Big enough to beat pixel
# rounding, small enough that the local-linear approximation of H holds.
_ANGLE_STEP_PX = 40.0


# One detected part's grasp solution. Pixel fields are always present; the
# robot-space fields are None if no homography H was supplied.
#   centroid_px        : (u, v) int      -- grip point in pixels (moments centroid)
#   long_axis_deg_img  : float           -- long-axis angle in IMAGE space (deg)
#   long_axis_deg_robot: float | None    -- long-axis angle in ROBOT space (deg)
#   robot_xy           : (x, y) | None   -- grip point in robot mm (pixel_to_robot)
#   grip_r_deg         : float | None    -- gripper R (rHead) to clamp the SHORT
#                                           side, wrapped into (-90, 90]
#   box_px             : np.ndarray 4x2  -- minAreaRect corners (for drawing)
#   long_len_px        : float           -- longer side length in pixels
#   short_len_px       : float           -- shorter side length in pixels
#   axis_ratio         : float           -- long/short extent; ~1 => orientation
#                                           is ill-defined (near-square)
GraspPose = namedtuple("GraspPose", [
    "centroid_px", "long_axis_deg_img", "long_axis_deg_robot", "robot_xy",
    "grip_r_deg", "box_px", "long_len_px", "short_len_px", "axis_ratio",
])


def pixel_to_robot(u, v, H):
    """Map a pixel (u, v) to robot-frame (x, y) mm via the 3x3 homography H.

    Same math as pickCVBlock.pixel_to_robot, duplicated here so this module
    stays standalone (importing pickCVBlock would run its top-level code and
    load the Dobot DLL)."""
    p = np.array([u, v, 1.0])
    xy = H @ p
    xy /= xy[2]
    return float(xy[0]), float(xy[1])


def wrap_to_pm90(deg):
    """Fold an angle onto (-90, 90].

    A part's long axis is a LINE, so theta and theta+180 mean the same
    orientation; a parallel grip is likewise symmetric. Folding mod 180 into
    (-90, 90] therefore loses nothing and keeps the command inside the
    gripper's +/-90 deg R range (dobotArm.rotate_end_effector clamps to that)."""
    return (deg + 90.0) % 180.0 - 90.0


def _long_axis_image(cnt, method="pca"):
    """Return (angle_img_deg, box_px, long_len_px, short_len_px, axis_ratio).

    angle_img_deg is the LONG-axis direction in image space, derived
    unambiguously from the box edge vectors (avoids the OpenCV (w,h)/angle
    convention footgun). `method` chooses how the angle is estimated:
      "pca"     -- principal axis of all contour points. More robust for an
                   irregular 3D-printed part whose red blob isn't a clean rect.
      "minrect" -- the longer edge of cv2.minAreaRect.
    The box + side lengths (used for drawing / the square-ambiguity check) always
    come from minAreaRect regardless of method."""
    rect = cv2.minAreaRect(cnt)                 # ((cx,cy),(w,h),angle)
    box = cv2.boxPoints(rect).astype(np.float32)  # 4x2 corners, ordered

    e0 = box[1] - box[0]
    e1 = box[2] - box[1]
    len0 = float(np.hypot(*e0))
    len1 = float(np.hypot(*e1))
    if len0 >= len1:
        long_vec, long_len, short_len = e0, len0, len1
    else:
        long_vec, long_len, short_len = e1, len1, len0

    axis_ratio = (long_len / short_len) if short_len > 1e-6 else float("inf")

    if method == "pca":
        pts = cnt.reshape(-1, 2).astype(np.float64)
        if len(pts) >= 2:
            _mean, eigvecs, _eigvals = cv2.PCACompute2(pts, mean=None)
            v = eigvecs[0]                      # principal (long) axis vector
            angle_img = math.degrees(math.atan2(v[1], v[0]))
        else:
            angle_img = math.degrees(math.atan2(long_vec[1], long_vec[0]))
    else:  # "minrect"
        angle_img = math.degrees(math.atan2(long_vec[1], long_vec[0]))

    return angle_img, box.astype(np.int32), long_len, short_len, axis_ratio


def image_angle_to_robot_angle(u, v, angle_img_deg, H, step_px=_ANGLE_STEP_PX):
    """Convert a LINE angle at pixel (u, v) from image space to robot space by
    mapping a short segment along it through the homography. This folds in the
    camera->robot rotation/flip/skew so the gripper rotates the right way."""
    a = math.radians(angle_img_deg)
    rx0, ry0 = pixel_to_robot(u, v, H)
    rx1, ry1 = pixel_to_robot(u + step_px * math.cos(a),
                              v + step_px * math.sin(a), H)
    return math.degrees(math.atan2(ry1 - ry0, rx1 - rx0))


def grasp_from_contour(cnt, H=None, method="pca"):
    """Compute the grasp (grip point + gripper rotation) for one part contour.

    cnt    : an OpenCV contour (the caller's own tuned color mask produced it).
    H      : 3x3 pixel->robot homography (HomographyMatrix.npy). If None, only
             the pixel-space fields are filled and grip_r_deg is None.
    method : "pca" (default, robust for irregular parts) or "minrect".

    Returns a GraspPose, or None if the contour has no area (degenerate)."""
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return None
    cu = int(M["m10"] / M["m00"])
    cv_ = int(M["m01"] / M["m00"])

    angle_img, box, long_len, short_len, axis_ratio = _long_axis_image(cnt, method)

    long_axis_robot = robot_xy = grip_r = None
    if H is not None:
        long_axis_robot = image_angle_to_robot_angle(cu, cv_, angle_img, H)
        robot_xy = pixel_to_robot(cu, cv_, H)
        # Jaws close across the SHORT side => perpendicular to the long axis
        # (+90), then apply the one-time R=0 calibration offset.
        grip_r = wrap_to_pm90(long_axis_robot + 90.0 + GRIP_R_OFFSET_DEG)

    return GraspPose(
        centroid_px=(cu, cv_),
        long_axis_deg_img=angle_img,
        long_axis_deg_robot=long_axis_robot,
        robot_xy=robot_xy,
        grip_r_deg=grip_r,
        box_px=box,
        long_len_px=long_len,
        short_len_px=short_len,
        axis_ratio=axis_ratio,
    )


def grip_angle(cnt, H, method="pca"):
    """Convenience: just the gripper rotation (rHead) for a contour, or None.

    Lets a caller do `rhead = grasp_orientation.grip_angle(cnt, H_matrix)`
    without unpacking the full GraspPose."""
    g = grasp_from_contour(cnt, H, method)
    return None if g is None else g.grip_r_deg


def draw_grasp(frame, grasp, locked=False):
    """Overlay the grasp on `frame` in place: rotated box, long axis (yellow),
    jaw-closing direction (magenta = how the fingers will press), centroid, and
    the numeric grip angle. Cyan box once `locked`."""
    box_color = (255, 255, 0) if locked else (0, 200, 0)
    cv2.drawContours(frame, [grasp.box_px], -1, box_color, 2)

    cu, cv_ = grasp.centroid_px
    L = max(30.0, grasp.long_len_px * 0.5)

    # Long axis (image space) -- yellow.
    a = math.radians(grasp.long_axis_deg_img)
    p1 = (int(cu - L * math.cos(a)), int(cv_ - L * math.sin(a)))
    p2 = (int(cu + L * math.cos(a)), int(cv_ + L * math.sin(a)))
    cv2.line(frame, p1, p2, (0, 255, 255), 2)

    # Jaw-closing direction = perpendicular to the long axis -- magenta.
    s = grasp.short_len_px * 0.5 + 12.0
    q1 = (int(cu - s * math.sin(a)), int(cv_ + s * math.cos(a)))
    q2 = (int(cu + s * math.sin(a)), int(cv_ - s * math.cos(a)))
    cv2.line(frame, q1, q2, (255, 0, 255), 2)

    cv2.circle(frame, (cu, cv_), 5, (0, 0, 255), -1)

    txt = f"img {grasp.long_axis_deg_img:+.0f}"
    if grasp.grip_r_deg is not None:
        txt += f" | gripR {grasp.grip_r_deg:+.0f}"
    if grasp.axis_ratio < 1.15:
        txt += "  (near-square: angle weak)"
    cv2.putText(frame, txt, (cu + 12, cv_ - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)
    return frame


# ===========================================================================
# STANDALONE PREVIEW (no robot). The red detection below is ONLY to feed
# contours into the preview -- the production path keeps pickCVBlock's own
# tuned mask and just calls grasp_from_contour(). Constants mirror pickCVBlock
# so what you see matches what it would detect.
# ===========================================================================
_HERE = os.path.dirname(os.path.abspath(__file__))

# mirror of pickCVBlock TARGET_* (preview only -- do not tune here)
_TARGET_MIN_AREA = 30
_TARGET_MAX_AREA = 500
_TARGET_MIN_SAT = 150
_TARGET_MIN_VAL = 100


def _load_calibration():
    """Load H and the undistort maps from files (no robot needed). Returns
    (H, map1, map2) with maps None if camera_params.npz is absent."""
    H = None
    h_path = os.path.join(_HERE, "HomographyMatrix.npy")
    if os.path.exists(h_path):
        H = np.load(h_path)
    else:
        print("[warn] HomographyMatrix.npy not found -> grip angle will be blank.")

    map1 = map2 = None
    cam_path = os.path.join(_HERE, "camera_params.npz")
    if os.path.exists(cam_path):
        data = np.load(cam_path)
        return H, data["camera_matrix"], data["dist_coeffs"]
    print("[warn] camera_params.npz not found -> showing raw (undistorted) frame.")
    return H, None, None


def _detect_red_contours(frame):
    """Same red HSV gate as pickCVBlock.phase_detect_targets (preview only)."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (3, 3), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, _TARGET_MIN_SAT, _TARGET_MIN_VAL]),
                       np.array([10, 255, 255])) + \
           cv2.inRange(hsv, np.array([170, _TARGET_MIN_SAT, _TARGET_MIN_VAL]),
                       np.array([180, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [c for c in contours if _TARGET_MIN_AREA < cv2.contourArea(c) < _TARGET_MAX_AREA]


def _run_preview(camera_index):
    print(f"[grasp_orientation] camera index {camera_index} "
          f"(0 = laptop webcam, 1 = Orbbec). Press 'q' or Esc to quit.")
    H, camera_matrix, dist_coeffs = _load_calibration()

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {camera_index}. "
                           "Try the other index: `grasp_orientation.py 0`.")

    # Warm up: first reads off a freshly-opened camera are often empty.
    frame = None
    for _ in range(10):
        ret, frame = cap.read()
        if ret and frame is not None:
            break
    if frame is None:
        cap.release()
        raise RuntimeError(f"Camera index {camera_index} opened but returned no frames.")

    map1 = map2 = None
    if camera_matrix is not None:
        h, w = frame.shape[:2]
        new_K, _roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1)
        map1, map2 = cv2.initUndistortRectifyMap(
            camera_matrix, dist_coeffs, None, new_K, (w, h), cv2.CV_16SC2)

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue
            if map1 is not None:
                frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

            for cnt in _detect_red_contours(frame):
                grasp = grasp_from_contour(cnt, H)
                if grasp is not None:
                    draw_grasp(frame, grasp)
                    if grasp.grip_r_deg is not None:
                        print(f"centroid={grasp.centroid_px} "
                              f"img={grasp.long_axis_deg_img:+.1f} "
                              f"gripR={grasp.grip_r_deg:+.1f} "
                              f"ratio={grasp.axis_ratio:.2f}")

            cv2.putText(frame, "yellow=long axis  magenta=jaws close here",
                        (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
            cv2.imshow("grasp_orientation (no robot)", frame)
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

    idx = 1  # Orbbec by project convention; pass 0 for the laptop webcam.
    if len(sys.argv) > 1:
        idx = int(sys.argv[1])
    _run_preview(idx)
