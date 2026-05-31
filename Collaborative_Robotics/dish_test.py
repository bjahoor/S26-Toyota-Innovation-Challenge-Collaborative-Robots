# Standalone DISH (drop-zone) test — NO robot needed (runs on Linux).
# Mirrors phase_detect_plates() in pickCVBlock.py: finds the round metal tray by
# SHAPE (HoughCircles) + a color pre-mask, so other round objects are ignored.
#
# Run:  python3 dish_test.py --camera 4
#   - camera window = detected circles (green ring + red center)
#   - edges window  = the Canny edges HoughCircles works from
#   - sliders tune the detector live
# Press q to quit.
#
# There is always exactly ONE dish, so we keep only the single strongest
# circle HoughCircles returns and ignore the rest -> nothing else gets confused.
#
# The knobs that matter:
#   min/max diameter = the dish's apparent size in PIXELS across (must bracket it)
#   sensitivity      = HoughCircles param2: lower -> more circles (and false ones),
#                                           higher -> only strong, clean circles
#   edge thresh      = HoughCircles param1: Canny high threshold (edge strength)
#   sat max / val min/max = the tray is colorless METAL, so keep only low-saturation
#                       pixels within a brightness window (metal reflects, so range
#                       it), and look for the circle only there -> others ignored

import cv2
import numpy as np
import argparse

MIN_DIST = 150  # min pixels between two circle centers (raise if dishes merge)

# --- stability lock ---
STABILITY_LIMIT = 30   # frames the dish must stay put before it "locks" (~1s at 30fps)
PIXEL_TOLERANCE = 15   # dish center may drift at most this many px and still count as stable

parser = argparse.ArgumentParser()
parser.add_argument("--camera", type=int, default=4, help="camera index (USB cam is 4 on this machine)")
args = parser.parse_args()

cap = cv2.VideoCapture(args.camera)
if not cap.isOpened():
    print(f"Could not open camera {args.camera}")
    raise SystemExit

cv2.namedWindow("camera")
cv2.createTrackbar("min diameter", "camera", 50,  800, lambda v: None)
cv2.createTrackbar("max diameter", "camera", 100, 800, lambda v: None)
cv2.createTrackbar("sensitivity",  "camera", 1,   100, lambda v: None)  # param2
cv2.createTrackbar("edge thresh",  "camera", 1000, 1000, lambda v: None)  # param1
cv2.createTrackbar("sat max",      "camera", 100, 255, lambda v: None)  # keep pixels BELOW this saturation (metal is grayish)
cv2.createTrackbar("val min",      "camera", 0,   255, lambda v: None)  # brightness floor (accept darker metal)
cv2.createTrackbar("val max",      "camera", 255, 255, lambda v: None)  # brightness ceiling (cap bright glints if needed)
cv2.createTrackbar("brightness",   "camera", 34,  128, lambda v: None)  # CAMERA brightness; slider 0..128 maps to v4l2 -64..+64 (slider-64), so 34 = -30

print("Point the camera at the metal tray. Tune diameter first, then the sat/val mask, then sensitivity. Press q to quit.")

# stability-lock state, carried across frames
stable_x, stable_y = 0, 0
stability_counter = 0
last_brightness = None   # only push brightness to the camera when it changes

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    min_d = cv2.getTrackbarPos("min diameter", "camera")
    max_d = cv2.getTrackbarPos("max diameter", "camera")
    param2 = max(1, cv2.getTrackbarPos("sensitivity", "camera"))
    param1 = max(1, cv2.getTrackbarPos("edge thresh", "camera"))
    sat_max = cv2.getTrackbarPos("sat max", "camera")
    val_min = cv2.getTrackbarPos("val min", "camera")
    val_max = max(val_min + 1, cv2.getTrackbarPos("val max", "camera"))

    bright = cv2.getTrackbarPos("brightness", "camera") - 64   # map 0..128 -> -64..+64
    if bright != last_brightness:
        cap.set(cv2.CAP_PROP_BRIGHTNESS, bright)
        last_brightness = bright

    max_d = max(max_d, min_d + 2)             # keep max above min
    min_r, max_r = min_d // 2, max_d // 2     # HoughCircles works in radius
    max_r = max(max_r, min_r + 1)

    # --- METAL PRE-MASK: keep only low-saturation pixels inside a brightness
    #     window, then find the circle on that mask so other round things are
    #     ignored (hue is irrelevant for colorless metal) ---
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (3, 3), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 0, val_min]), np.array([180, sat_max, val_max]))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)    # drop specks
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)   # fill gaps
    blurred = cv2.medianBlur(mask, 7)

    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, MIN_DIST,
                               param1=param1, param2=param2,
                               minRadius=min_r, maxRadius=max_r)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        c = circles[0, 0]                       # strongest circle = the one dish
        cx, cy, d = int(c[0]), int(c[1]), int(c[2]) * 2

        # --- STABILITY LOCK: count consecutive frames the dish barely moves ---
        if abs(cx - stable_x) < PIXEL_TOLERANCE and abs(cy - stable_y) < PIXEL_TOLERANCE:
            stability_counter += 1
        else:
            stability_counter = 0               # jumped -> start over
        stable_x, stable_y = cx, cy

        locked = stability_counter >= STABILITY_LIMIT
        progress = min(100, int(stability_counter / STABILITY_LIMIT * 100))
        ring_color = (255, 255, 0) if locked else (0, 255, 0)   # cyan once locked

        cv2.circle(frame, (cx, cy), c[2], ring_color, 2)        # ring
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)         # center
        status = f"LOCKED  d={d} at ({cx},{cy})" if locked else f"locking {progress}%   d={d}"
    else:
        stability_counter = 0                   # lost the dish -> reset
        status = "no dish"

    cv2.putText(frame, status, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(frame, f"D[{min_d},{max_d}] sens={param2} edge={param1} sat<{sat_max} val[{val_min},{val_max}]",
                (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    cv2.imshow("camera", frame)
    cv2.imshow("mask", mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s') or key == ord('q'):        # s = snapshot values, q = quit (also prints)
        print("\n--- dish_test current values ---")
        print(f"min diameter = {min_d}")
        print(f"max diameter = {max_d}")
        print(f"sensitivity (param2) = {param2}")
        print(f"edge thresh (param1) = {param1}")
        print(f"sat max = {sat_max}")
        print(f"val min = {val_min}")
        print(f"val max = {val_max}")
        print(f"brightness = {bright}")
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
