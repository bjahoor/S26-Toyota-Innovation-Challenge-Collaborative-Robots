r"""
hand_detect.py - Step 1 of the Reactive Safety Supervisor.

Wraps MediaPipe Tasks HandLandmarker and reports, per camera frame, whether a
hand is present and where it is in PIXEL space.

Why pixel space? The STOP demo only needs "is a hand near the robot's work zone
in the image?". Staying in pixels dodges the single biggest risk -- hand-eye
calibration + raised-hand parallax (see demo-roadmap-2026-05-31.md). Mapping the
centroid to robot mm via pixel_to_robot() / HomographyMatrix.npy is a later
(M4+) upgrade, not needed to score the safety milestone.

MediaPipe 0.10.35 is Tasks-only (there is NO legacy mp.solutions.hands), so this
uses mediapipe.tasks.python.vision.HandLandmarker plus a downloaded
`hand_landmarker.task` model file that must sit next to this script. Download it
once (needs internet) from:
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task

Run standalone for the M1 checkpoint (live preview: draws every hand + its
centroid, prints the centroid, shows FPS). Press 'q' or Esc to quit:
    ..\.venv\Scripts\python.exe hand_detect.py        # Orbbec  (index 1, project default)
    ..\.venv\Scripts\python.exe hand_detect.py 0      # laptop webcam (index 0)

Reused by safety_supervisor.py (M2/M3): construct a HandDetector once, then call
.process(frame) each loop; check result.present and result.hands.
"""

import os
import sys
from collections import namedtuple

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision


# Resolve the model relative to THIS file so the script runs from any working
# directory (the supervisor may import it from elsewhere).
_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(_HERE, "hand_landmarker.task")

# MediaPipe's wrist landmark index (0). The full hand is 21 landmarks; we report
# the mean of all of them as the "centroid" (more stable than any single point).
_WRIST = 0


# One detected hand. `centroid` and `landmarks_px` are in PIXEL coordinates
# (origin top-left, +x right, +y down) of the frame passed to process().
#   centroid      : (u, v) int        -- mean of the 21 landmarks (hand center)
#   landmarks_px  : [(u, v), ...] x21 -- every landmark, for future fingertip /
#                                        nearest-point logic in the supervisor
#   handedness    : "Left" | "Right" | "" -- label from the camera's POV
Hand = namedtuple("Hand", ["centroid", "landmarks_px", "handedness"])

# Result of one frame. `present` is True iff at least one hand was found.
# `hands` is the list of Hand (length 0..num_hands). The supervisor decides which
# hand matters (e.g. the one nearest the work zone); M1 just needs `present`.
HandResult = namedtuple("HandResult", ["present", "hands"])


class HandDetector:
    """
    Thin, reusable wrapper around MediaPipe Tasks HandLandmarker.

    Defaults to VIDEO running mode: synchronous (returns the result directly, no
    threading/callbacks) and uses MediaPipe's frame-to-frame tracking for steadier
    output -- both good fits for the single-threaded cooperative-hop supervisor.
    A monotonically increasing frame counter supplies the required timestamps, so
    correctness never depends on wall-clock timing.

    Lower confidences bias toward *seeing* a hand, which is the fail-safe
    direction for a safety stop (a false "hand present" only stops the arm early).
    """

    def __init__(self, model_path=DEFAULT_MODEL, num_hands=2,
                 min_detection_confidence=0.5, min_presence_confidence=0.5,
                 min_tracking_confidence=0.5, running_mode="VIDEO"):
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f"Hand model not found: {model_path}\n"
                "Download hand_landmarker.task (see the module docstring) and place "
                "it next to hand_detect.py."
            )

        mode = running_mode.upper()
        if mode == "VIDEO":
            self._mode = mp_vision.RunningMode.VIDEO
        elif mode == "IMAGE":
            self._mode = mp_vision.RunningMode.IMAGE
        else:
            raise ValueError("running_mode must be 'VIDEO' or 'IMAGE' "
                             "(LIVE_STREAM is async and intentionally unsupported)")

        options = mp_vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model_path),
            running_mode=self._mode,
            num_hands=num_hands,
            min_hand_detection_confidence=min_detection_confidence,
            min_hand_presence_confidence=min_presence_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._landmarker = mp_vision.HandLandmarker.create_from_options(options)
        self._frame_idx = 0

    def process(self, frame_bgr):
        """
        Detect hands in one OpenCV BGR frame. Returns a HandResult with all
        coordinates already converted to pixels for this frame's size.
        """
        h, w = frame_bgr.shape[:2]

        # OpenCV is BGR; MediaPipe expects SRGB.
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        if self._mode == mp_vision.RunningMode.VIDEO:
            # Timestamps only need to strictly increase; a frame counter does that
            # regardless of how fast process() is actually called.
            self._frame_idx += 1
            result = self._landmarker.detect_for_video(mp_image, self._frame_idx * 33)
        else:
            result = self._landmarker.detect(mp_image)

        hands = []
        for i, landmarks in enumerate(result.hand_landmarks):
            pts = [(int(np.clip(lm.x, 0.0, 1.0) * w),
                    int(np.clip(lm.y, 0.0, 1.0) * h)) for lm in landmarks]
            cx = int(sum(p[0] for p in pts) / len(pts))
            cy = int(sum(p[1] for p in pts) / len(pts))

            label = ""
            if i < len(result.handedness) and result.handedness[i]:
                label = result.handedness[i][0].category_name

            hands.append(Hand(centroid=(cx, cy), landmarks_px=pts, handedness=label))

        return HandResult(present=len(hands) > 0, hands=hands)

    def close(self):
        self._landmarker.close()

    # Context-manager sugar so callers can `with HandDetector() as det:`.
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def draw_overlay(frame, result):
    """Draw every landmark + a bold centroid dot for each detected hand (in place)."""
    for hand in result.hands:
        for (u, v) in hand.landmarks_px:
            cv2.circle(frame, (u, v), 2, (0, 200, 0), -1)
        cu, cv_ = hand.centroid
        cv2.circle(frame, (cu, cv_), 8, (0, 0, 255), -1)
        label = hand.handedness or "hand"
        cv2.putText(frame, f"{label} ({cu},{cv_})", (cu + 12, cv_),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    return frame


def _run_preview(camera_index):
    """M1 checkpoint: live webcam preview proving the hand is tracked reliably."""
    print(f"[hand_detect] opening camera index {camera_index} "
          f"(0 = laptop webcam, 1 = Orbbec). Press 'q' or Esc to quit.")
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(
            f"Cannot open camera index {camera_index}. "
            "Try the other index (pass it as an argument: `hand_detect.py 0`)."
        )

    # Warm up: the first reads off a freshly-opened camera are often empty.
    frame = None
    for _ in range(10):
        ret, frame = cap.read()
        if ret and frame is not None:
            break
    if frame is None:
        cap.release()
        raise RuntimeError(f"Camera index {camera_index} opened but returned no frames.")

    prev_t = cv2.getTickCount()
    fps = 0.0
    try:
        with HandDetector() as detector:
            while True:
                ret, frame = cap.read()
                if not ret or frame is None:
                    continue

                result = detector.process(frame)
                draw_overlay(frame, result)

                # Smoothed FPS readout (confirms the loop is fast enough to react).
                now = cv2.getTickCount()
                dt = (now - prev_t) / cv2.getTickFrequency()
                prev_t = now
                if dt > 0:
                    fps = 0.9 * fps + 0.1 * (1.0 / dt)

                status = f"hands: {len(result.hands)}   {fps:4.1f} FPS"
                cv2.putText(frame, status, (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (255, 255, 0), 2)
                if result.present:
                    print("hand centroids:", [h.centroid for h in result.hands])

                cv2.imshow("hand_detect (M1)", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):  # q or Esc
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    # This module doesn't import the Dobot DLL, so it doesn't hit the cp1252 crash,
    # but reconfigure to UTF-8 anyway so prints are safe from any terminal.
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

    # Default to the Orbbec (index 1, the project convention); allow an override
    # like `hand_detect.py 0` to test on the laptop webcam.
    idx = 1
    if len(sys.argv) > 1:
        idx = int(sys.argv[1])
    _run_preview(idx)
