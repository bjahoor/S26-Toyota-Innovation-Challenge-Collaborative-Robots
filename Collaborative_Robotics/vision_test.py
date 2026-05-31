# Standalone vision test — NO robot needed (runs on Linux).
# Mirrors the red detection in pickCVBlock.py so you can see what the
# current detector picks up when you hold the caliper in frame.
#
# Run:  python3 vision_test.py --camera 4
#   - left window  = live camera
#   - right window = red mask (what passes the COLOR gate)
#   - green outline + area number = blobs that also pass the SIZE gate (>800)
# Press q to quit.

import cv2
import numpy as np
import argparse

# --- same parameters as pickCVBlock.py phase_detect_targets() ---
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])
AREA_GATE = 800  # min contour area to count as a target

parser = argparse.ArgumentParser()
parser.add_argument("--camera", type=int, default=4, help="camera index (USB cam is 4 on this machine)")
args = parser.parse_args()

cap = cv2.VideoCapture(args.camera)
if not cap.isOpened():
    print(f"Could not open camera {args.camera}")
    raise SystemExit

print("Hold the caliper in frame. Drag the 'area gate' slider to tune. Press q to quit.")

# live sliders to tune detection without restarting
#   min area = lower size gate (caliper is small)
#   max area = upper size cap   (a hand is big -> reject)
#   min sat  = color strictness (caliper is vivid red; skin is dull -> reject)
cv2.namedWindow("camera")
cv2.createTrackbar("min area", "camera", 90, 2000, lambda v: None)
cv2.createTrackbar("max area", "camera", 500, 8000, lambda v: None)
cv2.createTrackbar("min sat", "camera", 150, 255, lambda v: None)
cv2.createTrackbar("min val", "camera", 100, 255, lambda v: None)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    MIN_AREA = cv2.getTrackbarPos("min area", "camera")
    MAX_AREA = cv2.getTrackbarPos("max area", "camera")
    MIN_SAT = cv2.getTrackbarPos("min sat", "camera")
    MIN_VAL = cv2.getTrackbarPos("min val", "camera")

    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (3, 3), 0), cv2.COLOR_BGR2HSV)
    lower1 = np.array([0, MIN_SAT, MIN_VAL])
    lower2 = np.array([170, MIN_SAT, MIN_VAL])
    mask = cv2.inRange(hsv, lower1, UPPER_RED_1) + \
           cv2.inRange(hsv, lower2, UPPER_RED_2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largest_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        largest_area = max(largest_area, area)
        if MIN_AREA < area < MAX_AREA:             # passes BOTH size gates
            cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

    cv2.putText(frame, f"largest red blob: {int(largest_area)} px   "
                f"min={MIN_AREA} max={MAX_AREA} sat={MIN_SAT} val={MIN_VAL}",
                (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    cv2.imshow("camera", frame)
    cv2.imshow("red mask", mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s') or key == ord('q'):        # s = snapshot values, q = quit (also prints)
        print("\n--- vision_test current values ---")
        print(f"min area = {MIN_AREA}")
        print(f"max area = {MAX_AREA}")
        print(f"min sat = {MIN_SAT}")
        print(f"min val = {MIN_VAL}")
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
