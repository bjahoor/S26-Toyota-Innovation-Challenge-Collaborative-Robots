#This code is a simplified implementation of a collaborative robotics system that detects plates and targets using computer vision, 
#and then commands a Dobot robotic arm to pick and place objects accordingly. The system operates in three phases: scanning for plates, 
#scanning for targets, and executing the pick/place operations. 
#Stability checks are implemented to ensure reliable detection before proceeding to the next phase.

# Note: there are parameters that are useful to the successful operation of the robot arm. Read through the code before running the program.

# How to use: 
# 1. Ensure you have the Dobot robotic arm set up and connected to your computer.
# 2. Place the plates (drop zones) and targets (red blocks) within the camera's
# field of view.
# 3. Run the script. The system will first scan for plates, then targets, and finally execute the pick/place operations based on the detected positions.
# 4. Monitor the console output and the video feed for feedback on the system's status and operations

#Other Useful Codes you can use:
#dobotArm.move_to_xyz(api, pick_x, pick_y, Z_SAFE, rHead): moves the robot to the specified (x, y, z) coordinates with a specified rotation for the end effector (rHead). Z_SAFE is a predefined constant that ensures the robot maintains a safe height to avoid collisions when moving horizontally.



import sys

# The vendored Dobot DLL prints a non-ASCII character ('：') the moment it loads,
# which crashes under Windows' default cp1252 console encoding. Force UTF-8 on our
# streams BEFORE importing dobotArm (its module-level dType.load() triggers that
# print) so this runs from any terminal, not only one with PYTHONUTF8=1 set.
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

import dobotArm
import grasp_orientation   # computes the gripper rotation to grab a part by its short side
import lib.DobotDllType as dType
import numpy as np
import cv2
import time

# Reactive safety layer. Logic lives in its own modules (hand_detect.py /
# safety_supervisor.py) so this script stays a thin orchestrator and the safety
# logic is reusable + unit-tested there. If MediaPipe or the hand model is missing,
# we degrade gracefully to the plain demo instead of crashing.
import safety_supervisor as safety
try:
    from hand_detect import HandDetector
    _HAND_OK = True
except Exception as _e:
    HandDetector = None
    _HAND_OK = False
    print(f"[safety] hand detection unavailable ({_e}); running WITHOUT the safety layer.")

# How many consecutive hand-free frames before the supervisor resumes the task after
# a STOP (~0.7 s at 30 fps). Larger = more conservative resume.
SAFETY_CLEAR_FRAMES = 20
# Danger-zone size as a fraction margin around the frame (0.15 -> central 70%).
SAFETY_ZONE_MARGIN = 0.15


"""CONSTANTS"""

CAMERA_INDEX = 1 #the Orbbec Astra Pro is camera index 1; index 0 is the laptop webcam. The calibration files (camera_params.npz / HomographyMatrix.npy) were made with the Orbbec, so we MUST read from index 1 or the undistort + pixel->robot mapping are applied to the wrong camera.

Z_SAFE = 40 #what is the clearance distance for the robot arm to avoid collisions when moving horizontally?
Z_PICK = -25 #what is the  height for the robot claw to successfully pick up the target?
Z_PLACE = Z_PICK #height to descend to before releasing over the tray. Defaults to Z_PICK (the tray bottom sits on the same table as the pick surface). Raise this a few mm if the gripper jams into the tray rim; lower it slightly if the part still drops from a height.
STABILITY_LIMIT = 30  #how many consecutive frames of stable detection before we "lock in" the positions and move to the next phase? (at 30fps, 30 frames is about 1 second)
PIXEL_TOLERANCE = 10  #object can move at most this # of pixels to be considered stationary

# Drop-zone (metal tray) detection — tuned in dish_test.py, rounded to nearest 10
PLATE_MIN_RADIUS = 25     # half of min diameter (50 px)
PLATE_MAX_RADIUS = 50     # half of max diameter (100 px)
PLATE_PARAM1 = 1000       # HoughCircles edge threshold (Canny high)
PLATE_PARAM2 = 1          # HoughCircles sensitivity (low = lenient; relies on mask + diameter bracket)
PLATE_SAT_MAX = 100       # metal is grayish -> keep only pixels BELOW this saturation
PLATE_VAL_MIN = 0         # brightness window floor
PLATE_VAL_MAX = 255       # brightness window ceiling (caps bright glints)

# Red part (pick target) detection — tuned in vision_test.py
TARGET_MIN_AREA = 30      # smallest red blob that counts as a part
TARGET_MAX_AREA = 500     # largest red blob (rejects big things like a hand)
TARGET_MIN_SAT = 150      # vivid-red floor (drops duller skin tones)
TARGET_MIN_VAL = 100      # brightness floor

machine_state = "scanning plate" 

# --- INITIALIZATION FOR CAMERA TRANSFORMATION ---
# MAKE SURE THAT YOU HAVE RAN calibrateCamera.py FIRST TO GENERATE THE camera_params.npz FILE
api = dType.load()
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    raise RuntimeError(f"Cannot open camera index {CAMERA_INDEX} (the Orbbec). "
                       "Is it plugged in? (index 0 is the laptop webcam.)")
H_matrix = np.load("HomographyMatrix.npy")
data = np.load("./camera_params.npz")
camera_matrix = data["camera_matrix"]
dist_coeffs   = data["dist_coeffs"]

# Compute undistort maps once. The first reads off a freshly-opened camera are
# often empty, so warm up until we get a real frame instead of crashing on
# frame.shape when the first read returns None.
frame = None
for _ in range(10):
    ret, frame = cap.read()
    if ret and frame is not None:
        break
if frame is None:
    raise RuntimeError(f"Camera index {CAMERA_INDEX} opened but returned no frames.")
h, w = frame.shape[:2]
new_K, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w,h), 1)
map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, new_K, (w,h), cv2.CV_16SC2)

def pixel_to_robot(u, v, H):
    p = np.array([u, v, 1])
    xy = H @ p
    xy /= xy[2]
    return xy[0], xy[1]


# --- SAFETY LAYER SETUP (built once; reused every frame) ---
# hand_detector is None when MediaPipe/the model is missing -> safety becomes a no-op.
hand_detector = None
supervisor = None
if _HAND_OK:
    try:
        hand_detector = HandDetector()
        danger_zone = safety.zone_from_frame(w, h, margin_frac=SAFETY_ZONE_MARGIN)
        supervisor = safety.SafetySupervisor(danger_zone, clear_frames=SAFETY_CLEAR_FRAMES)
        print(f"[safety] active. Danger zone {danger_zone} (px), "
              f"resume after {SAFETY_CLEAR_FRAMES} clear frames.")
    except Exception as e:
        print(f"[safety] could not start hand detector ({e}); running WITHOUT safety.")
        hand_detector = None
        supervisor = None


def safety_checkpoint(context=""):
    """
    Cooperative safety gate (the MVP reactive behaviour). Call this BETWEEN robot
    motions: it blocks here as long as a hand is inside the danger zone, and returns
    only once the zone has been clear long enough (supervisor hysteresis). Because
    move_to_xyz is blocking, checking between hops gives a reaction latency of about
    one hop — reach in and the arm freezes before the next move; withdraw and it
    resumes.

    No-op when the safety layer is disabled, so the plain demo is unaffected.
    """
    if hand_detector is None or supervisor is None:
        return

    announced = False
    fail_count = 0
    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            # Camera dropped frames (the Orbbec does this intermittently). We KEEP
            # HOLDING the arm while blind — that's the fail-safe choice — but warn so
            # it isn't mistaken for a hang.
            fail_count += 1
            if fail_count % 60 == 0:
                print("[safety] no camera frames — holding the arm (cannot see the zone).")
            cv2.waitKey(5)
            continue
        fail_count = 0
        frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

        result = hand_detector.process(frame)
        centroids = [hand.centroid for hand in result.hands]
        supervisor.update(centroids)

        display_frame = frame.copy()
        safety.draw_overlay(display_frame, centroids, supervisor)
        if context:
            cv2.putText(display_frame, context, (20, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("Detection", display_frame)
        cv2.waitKey(1)

        if supervisor.is_safe:
            if announced:
                print("[safety] zone clear — resuming.")
            return
        if not announced:
            print(f"[safety] HAND IN DANGER ZONE {context}— holding until clear.")
            announced = True


# State machine logic to control the flow of the program through the three phases: scanning for plates, scanning for targets, and executing pick/place operations.
# THIS STATE MACHINE IS TOO SIMPLE. Can you think of logics that should change the robot's sequnece of actions?
# Ex: what if the robot fails to pick up a target? should it retry? should it go back to scanning for targets in case the target was moved? what if a new plate is added during the pick/place phase?
# What if a human's hand is in sight during pick/place phase? (safety first!)

def next_state():
    global machine_state
    if machine_state == "scanning plate":
        machine_state = "scanning target"
    elif machine_state == "scanning target":
        machine_state = "pick place"
    elif machine_state == "pick place":
        machine_state = "scanning plate"
    else:
        machine_state = "scanning plate"



# ---------------------------------------------------------
# PHASE 1: DETECT Part Drop Zones (Plates)
# this script assumes a metallic circular plate as the drop zone, but you can modify the detection logic to fit your specific use case.
# ---------------------------------------------------------
def phase_detect_plates():
    print("\n[PHASE 1] Scanning for drop zones. Waiting for stability...")
    stability_counter = 0
    last_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue
        frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
        display_frame = frame.copy()
        
        # --- METAL PRE-MASK: keep only low-saturation pixels inside a brightness
        #     window, then find the circle on that mask so other round things are
        #     ignored (hue is irrelevant for colorless metal) ---
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

        current_list = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            i = circles[0, 0]   # only the strongest circle — there is exactly one tray
            cv2.circle(display_frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            rx, ry = pixel_to_robot(i[0], i[1], H_matrix)
            current_list.append((rx, ry))

        # --- AUTO-LOCK LOGIC ---
        if len(current_list) > 0 and len(current_list) == last_count:
            stability_counter += 1
        else:
            stability_counter = 0
            last_count = len(current_list)

        progress = int((stability_counter / STABILITY_LIMIT) * 100)
        cv2.putText(display_frame, f"LOCKING PLATES: {progress}%", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow("Detection", display_frame)
        cv2.waitKey(1)

        if stability_counter >= STABILITY_LIMIT:
            print(f"Locked {len(current_list)} plates.")
            return current_list
  
 

# ---------------------------------------------------------
# PHASE 2: DETECT Red velcros to pick up (Red Blocks)
# this script assumes the targets to be picked up are red blocks
# be aware your target maynot be red, and they may not be rectangular! You will need to modify the detection logic to fit your specific use case.
# ---------------------------------------------------------
def phase_detect_targets():
    print("\n[PHASE 2] Scanning for targets. Waiting for stability...")
    stability_counter = 0
    last_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret: continue
        
        frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
        # Create a display copy so drawings don't affect next frame's HSV detection
        display_frame = frame.copy()
        
        # Red Tag Logic
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (3,3), 0), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0,TARGET_MIN_SAT,TARGET_MIN_VAL]), np.array([10,255,255])) + \
               cv2.inRange(hsv, np.array([170,TARGET_MIN_SAT,TARGET_MIN_VAL]), np.array([180,255,255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        current_list = []
        for cnt in contours:
            if TARGET_MIN_AREA < cv2.contourArea(cnt) < TARGET_MAX_AREA:
                # Pick point AND gripper rotation (grab across the short side).
                grasp = grasp_orientation.grasp_from_contour(cnt, H_matrix)
                if grasp is not None:
                    rx, ry = grasp.robot_xy
                    current_list.append((rx, ry, grasp.grip_r_deg))
                    # Draw detection + grip orientation on display_frame only
                    grasp_orientation.draw_grasp(display_frame, grasp)
                    
        cv2.waitKey(1)

        # --- STABILITY LOGIC ---
        if len(current_list) != 0:
            if len(current_list) > 0 and len(current_list) == last_count:
                stability_counter += 1
            else:
                stability_counter = 0
                last_count = len(current_list)

        # Visual Feedback
        progress = int((stability_counter / STABILITY_LIMIT) * 100)
        color = (0, 255, 0) if progress < 100 else (255, 255, 0)
        
        cv2.putText(display_frame, f"LOCKING TARGETS: {progress}%", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.imshow("Detection", display_frame)
        
        # --- EXIT CONDITION ---
        if stability_counter >= STABILITY_LIMIT:
            print(f"[SUCCESS] Locked {len(current_list)} targets.")
            #cv2.waitKey(500) # Brief pause so you can see the 100%
    
            return current_list


# ---------------------------------------------------------
# PHASE 3: PICK/PLACE LOOP
# This function assumes 1 drop zone only has 1 part, and executes the pick/place operations in batches.
# if you are picking up rigid car parts, would you still be able to move directly to the object and to the drop zone? 
# Do you need collision avoidance? Think about if the robot gripper accidentally hits the plate or other parts on the way to the target, what would happen? How would you modify the robot's movement logic to avoid collisions?
# ---------------------------------------------------------
def phase_execute_batch(api, pick_list, drop_list):
    # (Removed a stray `cv2.VideoCapture(0)` that used to be here: it opened a
    #  second, unused camera handle on the wrong index and leaked it every batch.)
    time.sleep(0.5)
    
    if len(pick_list) == 0 or len(drop_list) == 0:
        print("missing targets, aborting")
        return False
    
    # There is always exactly ONE tray -> place EVERY picked part into it
    batch_size = len(pick_list)
    print(f"\n[PHASE 3] Executing batch of {batch_size} operations.")

    for i in range(batch_size):
        pick_x, pick_y, pick_r = pick_list[i]
        print(f"Task {i+1}: picking part at {pick_x, pick_y}, grip angle {pick_r:.1f} deg")

        # --- PICK SEQUENCE ---
        # Safety gate before each descent into the work area (cooperative check
        # between hops): if a hand is in the danger zone, hold here until it clears.
        # Pre-rotate the gripper at safe height so the jaws are already aligned to
        # the part's short side, then descend straight down and grab.
        safety_checkpoint("[pick: approach]")
        dobotArm.move_to_xyz(api, pick_x, pick_y, Z_SAFE, pick_r)
        safety_checkpoint("[pick: descending]")
        dobotArm.move_to_xyz(api, pick_x, pick_y, Z_PICK, pick_r)

        dobotArm.close_gripper(api)
        dobotArm.move_to_xyz(api, pick_x, pick_y, Z_SAFE, pick_r)

        # --- RE-DETECT THE TRAY for THIS piece (in case it moved) ---
        # move the arm out of the camera's view first, then lock the tray fresh
        dobotArm.move_to_home(api)
        drop_x, drop_y = phase_detect_plates()[0]
        print(f"Task {i+1}: tray locked at {drop_x, drop_y}, placing")

        # --- PLACE SEQUENCE ---
        safety_checkpoint("[place: approach]")
        dobotArm.move_to_xyz(api, drop_x, drop_y, Z_SAFE)   # arrive over the tray at safe height
        safety_checkpoint("[place: descending]")
        dobotArm.move_to_xyz(api, drop_x, drop_y, Z_PLACE)  # descend before releasing (previously released at Z_SAFE -> part dropped from height)
        dobotArm.open_gripper(api)                           # release the part on/into the tray
        dobotArm.stop_pump(api)
        dobotArm.move_to_xyz(api, drop_x, drop_y, Z_SAFE)   # lift clear

    # irl, it is ok for 1 dish to contain multiple parts
    # if len(pick_list) > len(drop_list):
    #     for i in range(len(pick_list)):
    #         pick_x, pick_y = pick_list[i]
    #         drop_x, drop_y = drop_list[0]
    #         # --- PICK SEQUENCE ---
    #         dobotArm.move_to_xyz(api, pick_x, pick_y, Z_SAFE)
    #         dobotArm.move_to_xyz(api, pick_x, pick_y, Z_PICK)
    #         dobotArm.close_gripper(api)
    #         dobotArm.move_to_xyz(api, pick_x, pick_y, Z_SAFE)

    #     # --- PLACE SEQUENCE ---
    #         dobotArm.move_to_xyz(api, drop_x, drop_y, Z_SAFE)
    #         dobotArm.open_gripper(api)
    #         dobotArm.stop_pump(api)
    #         dobotArm.move_to_xyz(api, drop_x, drop_y, Z_SAFE)

    print("\nBatch Complete.")
    return True
 

# ---------------------------------------------------------
# MAIN EXECUTION
# contains an oversimplified state machine that runs the three phases sequentially. You can modify the logic to fit your specific use case.
# ---------------------------------------------------------
errored = False
try:
    dobotArm.initialize_robot(api)
    dobotArm.open_gripper(api)
    dobotArm.stop_pump(api)

    while machine_state == "scanning plate":
        drop_zone = phase_detect_plates()
        if drop_zone is not None:
            next_state()


    while machine_state == "scanning target":
        pick_target = phase_detect_targets()
        if pick_target is not None:
            next_state()


    while machine_state == "pick place":
        completed = phase_execute_batch(api, pick_target, drop_zone)
        if completed:
            next_state()
        else: break

except KeyboardInterrupt:
    errored = True
    print("\nInterrupted (Ctrl+C) — stopping the arm.")
except Exception as e:
    errored = True
    print(f"\nError during run: {e} — stopping the arm.")

finally:
    # --- GRACEFUL SHUTDOWN (runs on normal finish, crash, or Ctrl+C) ---
    # Each step is guarded so one failure (e.g. the arm already gone) can't
    # block the rest of the cleanup.
    try:
        if errored:
            # Mid-motion: force-stop and clear the queue. Do NOT then issue a
            # move — with the queue cleared/stopped it would busy-wait forever.
            dobotArm.stop_motion(api)
        else:
            # Clean finish: the queue has drained, so park at the safe home pose.
            dobotArm.move_to_home(api)
    except Exception:
        pass
    try:
        dType.DisconnectDobot(api)   # close the serial link (won't drop the arm)
    except Exception:
        pass
    try:
        if hand_detector is not None:
            hand_detector.close()    # release the MediaPipe HandLandmarker
    except Exception:
        pass
    cap.release()
    cv2.destroyAllWindows()
    print("Shutdown complete.")