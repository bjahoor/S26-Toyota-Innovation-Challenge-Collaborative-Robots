r"""
pick_engine.py - import-safe core of the pick-and-place runtime.

Pulls the pure, reusable pieces out of pickCVBlock.py so the flow can be driven by
something other than that script's single-threaded blocking main loop -- e.g. the
web controller in pick_web.py: the tuned detection parameters, the pixel->robot
mapping, undistort-map construction, and stateful per-frame detectors that
encapsulate the "stable for N frames -> lock" logic (without pickCVBlock's blocking
`while True` loop, so a server can advance them one frame at a time).

This module opens NO camera and connects to NO robot at import time, so it is safe
to import from a server thread or a unit test.

Detection constants MIRROR pickCVBlock.py and follow the same repo convention as
grasp_orientation.py / goto_caliper.py: **pickCVBlock.py is the SOURCE OF TRUTH**
and these values are deliberately calibrated -- do NOT re-tune them here. If
detection misbehaves, fix lighting/placement/timing (see the team notes), not these.
"""

import numpy as np
import cv2

import grasp_orientation  # pure geometry; safe to import (no camera/robot)


# --- Heights (mm). Mirror pickCVBlock.py. ---
Z_SAFE = 40
Z_PICK = -25
Z_PLACE = Z_PICK

# --- Locking. Mirror pickCVBlock.py. ---
STABILITY_LIMIT = 30
PIXEL_TOLERANCE = 10

# --- Drop-zone (metal tray) detection. Mirror pickCVBlock.py. ---
PLATE_MIN_RADIUS = 25
PLATE_MAX_RADIUS = 50
PLATE_PARAM1 = 1000
PLATE_PARAM2 = 1
PLATE_SAT_MAX = 100
PLATE_VAL_MIN = 0
PLATE_VAL_MAX = 255

# --- Red part (pick target) detection. Mirror pickCVBlock.py. ---
TARGET_MIN_AREA = 30
TARGET_MAX_AREA = 500
TARGET_MIN_SAT = 150
TARGET_MIN_VAL = 100


def pixel_to_robot(u, v, H):
    """Map an image pixel (u, v) to robot-frame (x, y) mm via the homography H.
    Identical math to pickCVBlock.pixel_to_robot()."""
    p = np.array([u, v, 1.0])
    xy = H @ p
    xy /= xy[2]
    return float(xy[0]), float(xy[1])


def build_undistort_maps(frame_shape, camera_matrix, dist_coeffs):
    """Build the (map1, map2) remap tables once, the same way pickCVBlock does."""
    h, w = frame_shape[:2]
    new_K, _roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1)
    return cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None, new_K, (w, h), cv2.CV_16SC2)


class PlateDetector:
    """Per-frame metal-tray detector with stability locking.

    Mirrors pickCVBlock.phase_detect_plates() but WITHOUT the blocking while-loop:
    call process(frame) once per camera frame. It carries the stability counter
    across calls and reports a 0..100 lock progress; when locked it returns the
    tray's robot-frame (x, y). Call reset() to start a fresh lock (e.g. each loop).
    """

    def __init__(self, H, stability_limit=STABILITY_LIMIT):
        self.H = H
        self.stability_limit = stability_limit
        self.reset()

    def reset(self):
        self._counter = 0
        self._last_count = 0
        self.locked = None

    def process(self, frame, display=None):
        """Returns (progress_0_100, locked_xy_or_None). Draws the ring on `display`
        (the frame you intend to show) if one is given."""
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (3, 3), 0), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 0, PLATE_VAL_MIN]),
                                np.array([180, PLATE_SAT_MAX, PLATE_VAL_MAX]))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        blurred = cv2.medianBlur(mask, 7)

        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, 150,
                                   param1=PLATE_PARAM1, param2=PLATE_PARAM2,
                                   minRadius=PLATE_MIN_RADIUS, maxRadius=PLATE_MAX_RADIUS)
        found = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            i = circles[0, 0]   # strongest circle -> there is exactly one tray
            if display is not None:
                cv2.circle(display, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(display, (i[0], i[1]), 3, (0, 0, 255), -1)
            found.append(pixel_to_robot(i[0], i[1], self.H))

        # --- AUTO-LOCK LOGIC (mirror of pickCVBlock) ---
        if len(found) > 0 and len(found) == self._last_count:
            self._counter += 1
        else:
            self._counter = 0
            self._last_count = len(found)

        if self._counter >= self.stability_limit:
            self.locked = found[0]
            return 100, self.locked
        return int(self._counter / self.stability_limit * 100), None


class TargetDetector:
    """Per-frame red-part detector with stability locking + grip orientation.

    Mirrors pickCVBlock.phase_detect_targets(): the same red HSV + area gate, and
    each accepted contour goes through grasp_orientation.grasp_from_contour() so a
    locked target carries (robot_x, robot_y, grip_r_deg). Returns the full list when
    locked. WITHOUT the blocking while-loop -- call once per frame.
    """

    def __init__(self, H, stability_limit=STABILITY_LIMIT):
        self.H = H
        self.stability_limit = stability_limit
        self.reset()

    def reset(self):
        self._counter = 0
        self._last_count = 0
        self.locked = None

    def process(self, frame, display=None):
        """Returns (progress_0_100, locked_list_or_None) where each list item is
        (robot_x, robot_y, grip_r_deg). Draws grasp overlays on `display` if given."""
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (3, 3), 0), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, TARGET_MIN_SAT, TARGET_MIN_VAL]),
                                np.array([10, 255, 255])) + \
               cv2.inRange(hsv, np.array([170, TARGET_MIN_SAT, TARGET_MIN_VAL]),
                                np.array([180, 255, 255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found = []
        for cnt in contours:
            if TARGET_MIN_AREA < cv2.contourArea(cnt) < TARGET_MAX_AREA:
                grasp = grasp_orientation.grasp_from_contour(cnt, self.H)
                if grasp is not None and grasp.robot_xy is not None:
                    rx, ry = grasp.robot_xy
                    grip = grasp.grip_r_deg if grasp.grip_r_deg is not None else 0.0
                    found.append((rx, ry, grip))
                    if display is not None:
                        grasp_orientation.draw_grasp(display, grasp)

        # --- STABILITY LOGIC (mirror of pickCVBlock: only count when something seen) ---
        if len(found) != 0:
            if len(found) == self._last_count:
                self._counter += 1
            else:
                self._counter = 0
                self._last_count = len(found)

        if self._counter >= self.stability_limit:
            self.locked = found
            return 100, self.locked
        return int(self._counter / self.stability_limit * 100), None
