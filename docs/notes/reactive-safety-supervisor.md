# Reactive Safety Supervisor — Plan & Implementation Steps

A human-machine-interface (HMI) safety layer for the Dobot Magician: the camera
watches the workspace, and when a human hand gets too close, the supervisor
overrides the robot's task to **slow, stop, or retreat**.

This is the chosen approach for the Collaborative Robotics "Human-Machine
Interface" milestone, selected over NVIDIA cuRobo. cuRobo's real-time MPC is
*unexecutable* on the Dobot (the arm only accepts queued serial commands, with no
high-rate joint servo stream), and its CUDA/Linux stack fights this Win11 / no-WSL2
machine. The supervisor instead reasons in Cartesian space and drives the arm the
only way it listens — through its own PTP/CP command queue.

> File-path note: this doc lives in `docs/notes/`; code links point up to
> `../../Collaborative_Robotics/`.

---

## 1. Mental model

A referee with a whistle. The arm does its normal pick-and-place job, while the
supervisor continuously asks *"where is the hand, and is it too close?"* When the
answer is "too close," it overrides the task.

**Guiding principle — fail-safe degradation:** every error (bad calibration,
dropped frame, jitter) must push the arm toward stopping *earlier*, never toward
missing the hand. `STOP`-with-a-conservative-margin is the primary behavior;
graceful steering is a bonus layered on top.

---

## 2. Architecture & data flow

```
   Orbbec camera (index 1)
          | frames
          v
  +------------------+     hand present?  +  hand position (robot frame)
  |  PERCEPTION      | ------------------------------------------+
  |  MediaPipe Hands |                                           |
  |  -> pixel centroid                                           |
  |  -> pixel_to_robot                                           v
  +------------------+                              +----------------------+
                                                   |  SUPERVISOR          |
   arm pose (GetPose, optional) ------------------ |  (state machine)     |
                                                   |  zones + hysteresis  |
                                                   +----------+-----------+
                                                              | decision
                       +---------------------------------------+----------------+
                       v               v                       v                v
                    NORMAL           SLOW                    STOP            RETREAT
                  (run task)   (reduce speed)         (force-stop queue)  (go to home)
                                                              |
                                                              v
                                                   dobotArm.py -> Dobot queue
```

Three logical pieces:

1. **Perception** — MediaPipe finds the hand per frame -> centroid pixel ->
   robot-frame XY via the existing homography.
2. **Supervisor** — a small state machine converting "hand distance" into a
   behavior, with hysteresis so it doesn't flicker at zone boundaries.
3. **Actuation** — issues motion (or stop) through
   [dobotArm.py](../../Collaborative_Robotics/dobotArm.py).

---

## 3. Safety behavior layers (zones)

Concentric zones, distances in robot millimetres:

| Zone    | Condition            | Behavior                              | State     |
|---------|----------------------|---------------------------------------|-----------|
| Clear   | hand far / absent    | run the task at full speed            | `NORMAL`  |
| Warning | hand approaching     | scale speed down (`SetPTPCommonParams`) | `SLOW`  |
| Danger  | hand close           | halt immediately (force-stop + clear) | `STOP`    |
| (opt.)  | hand persists        | retreat to home pose                  | `RETREAT` |

**Hysteresis / resume rule:** only return to `NORMAL` after the hand has been
clear for *N consecutive frames* (~1 second). Without this the arm oscillates
start/stop at the boundary.

Suggested starting thresholds (tune conservatively on the real setup):
- Danger radius: generous — the human can out-reach the arm, so bias large.
- Warning radius: a band outside danger for the speed ramp.

---

## 4. The one subtlety that defines this project

[`move_to_xyz()`](../../Collaborative_Robotics/dobotArm.py) is **blocking** — it
enqueues a point then sits in
`while execCmd > GetQueuedCmdCurrentIndex: dSleep(25)` until the arm arrives.
While that loop runs, Python can't react to anything. So "reactive" requires
solving this. Two approaches:

- **MVP (cooperative, single-threaded):** break each long move into short hops
  and check the hand *between* hops. Reaction latency ~= one hop (~100-200 ms).
  Simple, no threading bugs. **Start here.**
- **Upgrade (watchdog thread):** a perception thread sets a shared `EMERGENCY`
  flag; a watchdog calls `SetQueuedCmdForceStopExec` + `SetQueuedCmdClear` to
  abort mid-motion. True reactivity, but you must handle the blocking poll exiting
  cleanly when the queue is cleared.

---

## 5. Implementation steps (build order)

### Step 0 — Environment (do first, ~30 min)
- Create a **Python 3.12** venv (NOT 3.13 — MediaPipe has no 3.13 Windows wheel;
  this machine has both, so point the venv at 3.12 explicitly).
- `pip install mediapipe opencv-python numpy` (the Dobot DLL is already vendored
  in [Collaborative_Robotics/lib/](../../Collaborative_Robotics/lib/)).
- Smoke-test the Orbbec opens on **index 1** (the existing pick scripts use index
  0, which is the laptop webcam — wrong camera).

### Step 1 — Hand detection module (`hand_detect.py`)
- Wrap MediaPipe Hands; return `(hand_present: bool, centroid_pixel: (u, v))` per
  frame (use the wrist/palm landmark for the centroid).
- Test standalone with a live preview window drawing a dot on your hand.
- **Checkpoint:** it tracks your hand reliably.

### Step 2 — Map hand -> robot frame
- Reuse [`pixel_to_robot()`](../../Collaborative_Robotics/pickCVBlock.py) +
  `HomographyMatrix.npy` (produced by
  [getTransformationMatrix.py](../../Collaborative_Robotics/getTransformationMatrix.py)).
- Put your hand at a known table spot, print robot XY, sanity-check against a ruler.
- **Checkpoint:** pixel -> mm is roughly correct.
- WARNING: this homography is a **table-plane** map; a raised hand has parallax
  error. Fine for a conservative danger zone now; add Orbbec depth later for true 3D.

### Step 3 — Supervisor state machine (`safety_supervisor.py`)
- Pure logic: input = hand distance, output = state
  (`NORMAL` / `SLOW` / `STOP` / `RETREAT`), with the zone thresholds + hysteresis
  counter.
- Unit-test with fake distances (no hardware needed).
- **Checkpoint:** feeding distances flips states correctly.

### Step 4 — Wire STOP to the Dobot
- Add `stop_motion(api)` to
  [dobotArm.py](../../Collaborative_Robotics/dobotArm.py):
  `SetQueuedCmdForceStopExec` then `SetQueuedCmdClear`.
- Run the arm shuttling between two points; wave your hand -> it halts.
- This is the **core safety behavior**.

### Step 5 — Integrate (the demo-able checkpoint)
- Main loop: a trivial task (shuttle A<->B in short hops) gated by the supervisor —
  check hand before each hop, `STOP` if danger.
- **Checkpoint:** arm works normally, freezes when you reach in, resumes when you
  withdraw. **This alone scores the Safety milestone.**

### Step 6 — Add the polish layers
- `SLOW` zone via `SetPTPCommonParams` speed scaling.
- `RETREAT` to home on persistent intrusion.
- Tune margins conservatively.
- On-screen state/distance readout for the demo.

### Step 7 — Optional upgrades (only if time)
- Orbbec depth -> true 3D hand position (removes parallax error).
- Distance to the **actual TCP** via `GetPose` (more genuinely "collaborative").
- Watchdog-thread true mid-motion abort (Step 4 upgrade).
- A PyBullet / Swift (Robotics Toolbox) viz overlay for judging.

---

## 6. Reuse vs. write new

**Reuse:**
- [dobotArm.py](../../Collaborative_Robotics/dobotArm.py) — motion primitives.
- `HomographyMatrix.npy` +
  [`pixel_to_robot()`](../../Collaborative_Robotics/pickCVBlock.py) — frame mapping.
- The camera / undistort setup pattern from
  [pickCVBlock.py](../../Collaborative_Robotics/pickCVBlock.py).

**Write new (~200 lines total):**
- `hand_detect.py`
- `safety_supervisor.py`
- a `stop_motion()` helper in
  [dobotArm.py](../../Collaborative_Robotics/dobotArm.py)
- a thin main loop.

---

## 7. Key risks (verified)

1. **Calibration is the real risk, not the library.**
   [calibrateCamera.py](../../Collaborative_Robotics/calibrateCamera.py) does
   **intrinsics only**. You need the camera -> robot-base **extrinsic** (hand-eye)
   transform — that's what
   [getTransformationMatrix.py](../../Collaborative_Robotics/getTransformationMatrix.py)
   is for. Verify it early; it's the dominant time sink. Mitigation: make
   `STOP`-with-margin primary so calibration error degrades to "stops too early"
   (safe), never "misses the hand."

2. **Pin Python 3.12.** MediaPipe Windows wheels stop at cp312; 3.13+ forces a
   Bazel source build. This machine has 3.13 present — point the project venv at 3.12.

3. **Limited steering authority.** With ~3 positional DOF + 1 wrist and no null
   space, the arm can't gracefully reorient to dodge. Repulsion should move
   end-effector *position* only; `STOP`/retreat must remain primary.

4. **The blocking-move problem** (see Section 4) — solve with cooperative hops
   before attempting a watchdog thread.

---

## 8. Golden rule

Get the **Step 5 STOP checkpoint** working before touching depth or calibration
upgrades. It guarantees a scoreable demo independent of the riskiest piece
(calibration accuracy).

> **Bottom line:** build the reactive safety supervisor (MediaPipe + Orbbec +
> pydobot/dobotArm, `STOP`-first). Keep Robotics Toolbox / PyBullet in reserve
> purely for planned-motion visualization once the safety demo is live.
