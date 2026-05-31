# Project Status & Resume Point

_Last updated: 2026-05-31_

A living "where are we / what's next" doc for the Collaborative Robotics work.
For the detailed design of the chosen milestone, see
[reactive-safety-supervisor.md](reactive-safety-supervisor.md).

---

## At a glance

- **Milestone:** Human-Machine Interface → building a **Reactive Safety Supervisor**
  (camera watches the workspace; when a hand gets too close, the Dobot is
  overridden to slow / stop / retreat). cuRobo was evaluated and ruled out.
- **Existing demo:** `pickCVBlock.py` — a CV-driven pick-and-place (detect tray →
  detect red part → pick & place) that the safety layer will eventually wrap.
- **State today:** environment is ready; the pick/place vision is tuned and
  committed; the safety supervisor itself is **not started yet** (still at Step 0→1).
- **⚠️ Do this first:** commit the uncommitted camera-index / UTF-8 / frame-guard
  fixes in `pickCVBlock.py` (committed `HEAD` still opens the wrong camera).

---

## Environment & hardware (the setup that must be right)

| Thing | Value / note |
|---|---|
| OS | Windows 11 Home, no WSL2 |
| GPU | RTX 3060 6 GB (cuRobo-capable, but cuRobo not used) |
| Python | **3.12** — pinned because MediaPipe has no 3.13 Windows wheel. Machine also has 3.13; use 3.12 explicitly. |
| venv | `.venv/` at repo root (rebuilt on 3.12). VSCode interpreter + `launch.json` point here. |
| Packages | `mediapipe 0.10.35`, `opencv-python 4.13.0`, `numpy 2.4.6` (+ pydobot/serial via vendored DLL) |
| Camera | **Orbbec Astra Pro = index 1**; laptop webcam = index 0. Calibration was done on index 1, so all scripts must read index 1. |
| Dobot | Magician on **COM3** (auto-detected via `SearchDobot`); serial command queue only (no high-rate servo stream) |
| UTF-8 | The vendored Dobot DLL prints a non-ASCII char on load and **crashes under cp1252**. Need `PYTHONUTF8=1` (VSCode terminal sets it; `pickCVBlock.py` now also self-reconfigures stdout). |
| MediaPipe API | 0.10.35 is **Tasks-only** — no legacy `mp.solutions.hands`. Use `mediapipe.tasks.python.vision.HandLandmarker` + a downloaded `hand_landmarker.task` model. |

---

## ✅ Done

### Step 0 — Environment (reactive-safety-supervisor plan)
- Rebuilt `.venv` on **Python 3.12.10** (the old venv was 3.13 → MediaPipe-incompatible).
- Installed `mediapipe`, `opencv-python`, `numpy`.
- Verified the **Orbbec opens on camera index 1** and delivers 640×480 frames.
- Verified Dobot is reachable on **COM3** (64-bit DLL loads; `SearchDobot` → `['COM3']`).
- Confirmed the `pickCVBlock` startup path: calibration files load
  (`camera_params.npz` 3×3 + 1×5, `HomographyMatrix.npy` 3×3), undistort maps build,
  pixel→robot mapping returns sane mm.

### `pickCVBlock.py` — bug fixes (functional)
- Camera index **0 → 1** (was reading the laptop webcam; calibration is for the Orbbec).
- UTF-8 stdout/stderr reconfigure so the Dobot DLL import doesn't crash outside the VSCode terminal.
- Frame-read guards + camera-open check (no more crash on an empty first frame).
- Removed a stray `cv2.VideoCapture(0)` that leaked a camera handle each batch.
- **Verified live:** arm homes, Phase 1 locks the metal tray, advances to Phase 2.

### Vision tuning — committed (later sessions, 2026-05-30)
- `24c4d62` — tuned the **metal-tray drop-zone** detection; re-detect tray per pick.
- `68dbde8` — tuned the **red-part (pick target)** detection (`TARGET_*`: area 300–500,
  sat ≥150, val ≥100 — catches the caliper, rejects a hand). Tuning tools added:
  `dish_test.py` (tray) and `vision_test.py` (red part, `s` = snapshot values).
- **The tuning is authoritative — do not re-tune it without an explicit go-ahead.**

---

## 🔄 In progress / not yet committed

- **Uncommitted in working tree:** the camera-index→1, UTF-8, and frame-guard fixes
  in `pickCVBlock.py` (~32 lines). Committed `HEAD` (`68dbde8`) **still has
  `cv2.VideoCapture(0)`**, so the working copy is the only thing making it use the
  right camera. Commit these so they aren't lost.
- **`pickCVBlock.py` full run not yet validated end-to-end** with the committed
  tuning — confirmed through Phase 1; Phase 2 → Phase 3 (actual pick & place) still
  needs a clean run-through on hardware.

---

## 📋 To do

### Immediate
- [ ] **Commit** the camera-index / UTF-8 / frame-guard fixes in `pickCVBlock.py`.

### Track A — finish the pick/place demo
- [ ] Re-run `pickCVBlock.py` end-to-end and confirm a full pick **and** place.
- [ ] **Fix the place height bug:** the place sequence opens the gripper at
      `Z_SAFE = 40` without descending, so the part is dropped from height. It should
      descend (≈ `Z_PICK`) before releasing. See `phase_execute_batch`.
- [ ] Review the gripper-vs-suction mix in pick/place (`close_gripper` + `stop_pump`)
      against the actual end-effector.
- [ ] Note: the main loop runs **one** plate→target→pick/place cycle then exits.

### Track B — Reactive Safety Supervisor (the milestone) — see the plan doc
- [ ] **Step 1** — `hand_detect.py`: wrap MediaPipe **Tasks `HandLandmarker`**
      (download `hand_landmarker.task` first); return `(hand_present, centroid_pixel)`.
      Standalone live-preview test.
- [ ] **Step 2** — map hand pixel → robot frame by reusing `pixel_to_robot()` +
      `HomographyMatrix.npy`. (Table-plane homography → parallax error for a raised
      hand; fine for a conservative danger zone now.)
- [ ] **Step 3** — `safety_supervisor.py`: state machine
      `NORMAL/SLOW/STOP/RETREAT` with zone thresholds + hysteresis. Unit-test with fake distances.
- [ ] **Step 4** — add `stop_motion(api)` to `dobotArm.py`
      (`SetQueuedCmdForceStopExec` + `SetQueuedCmdClear`); wave-to-halt test. **Core safety behavior.**
- [ ] **Step 5** — integrate: trivial A↔B shuttle task gated by the supervisor;
      reach in → freezes, withdraw → resumes. **This alone scores the Safety milestone.**
- [ ] **Step 6** — polish: `SLOW` speed-scaling, `RETREAT` to home, on-screen readout.
- [ ] **Step 7** (optional) — Orbbec depth for true 3D; distance to actual TCP via
      `GetPose`; watchdog-thread mid-motion abort; PyBullet/Swift viz.

> **Golden rule (from the plan):** get the Step 5 STOP checkpoint working before
> touching depth/calibration upgrades — it guarantees a scoreable demo.

---

## Known bugs / risks

- **Place-from-height** (Track A bug above) — most likely to misbehave on a demo.
- **Calibration is the real risk** for the supervisor, not the libraries:
  `calibrateCamera.py` does **intrinsics only**; the camera→robot-base **extrinsic**
  (hand-eye) is `getTransformationMatrix.py` → `HomographyMatrix.npy`. Make
  `STOP`-with-margin primary so calibration error degrades to "stops early" (safe).
- **The blocking-move problem:** `move_to_xyz()` busy-waits until the arm arrives, so
  nothing else runs mid-move. For reactivity, do short cooperative hops (MVP) before
  attempting a watchdog thread.
- **Orbbec auto-exposure drifts** frame-to-frame, which shifts a marginal target's
  apparent saturation. Keep targets vividly colored; keep detection-threshold
  experiments in throwaway scratch files.
- **venv was rebuilt mid-lock** (a running `jointListener.py` held the old
  `python.exe`); it works but isn't pristine — a clean `py -3.12 -m venv .venv` +
  reinstall is a cheap reset if anything looks off.

---

## Script map (`Collaborative_Robotics/`)

| File | Purpose |
|---|---|
| `pickCVBlock.py` | Main pick-and-place state machine (detect tray → red part → pick/place). |
| `dobotArm.py` | Motion primitives: `initialize_robot`, `move_to_xyz`, `move_to_home`, gripper/pump. |
| `controlArm.py` | CLI to drive the arm (`xyz/joint/home/rotate/grip/suction/pose`), `--live` UDP readout. |
| `jointListener.py` | UDP listener that prints live joint angles broadcast by `controlArm.py --live --udp`. |
| `calibrateCamera.py` | Camera **intrinsics** calibration (ArUco). |
| `calibrateCameraWeb.py` | Web viewer wrapper for calibration on `:8000`. |
| `getTransformationMatrix.py` | Produces `HomographyMatrix.npy` (pixel→robot, table plane). |
| `dish_test.py` | Tuning tool for the metal-tray drop-zone detection. |
| `vision_test.py` | Tuning tool for the red-part detection (`s` snapshots HSV/area values). |
| `testDobot.py` | Misc Dobot test. |
| Data | `camera_params.npz` (intrinsics), `HomographyMatrix.npy` (pixel→robot). |

---

## Handy commands

```powershell
# Run the pick/place demo (UTF-8 is required for the Dobot DLL)
$env:PYTHONUTF8 = "1"
cd Collaborative_Robotics
..\.venv\Scripts\python.exe pickCVBlock.py

# Drive the arm directly
..\.venv\Scripts\python.exe controlArm.py home
..\.venv\Scripts\python.exe controlArm.py xyz 200 50 50

# Tune detection (scratch tools)
..\.venv\Scripts\python.exe vision_test.py     # red part
..\.venv\Scripts\python.exe dish_test.py       # tray
```
