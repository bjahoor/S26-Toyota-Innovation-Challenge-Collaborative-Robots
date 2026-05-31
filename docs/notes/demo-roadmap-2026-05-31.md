# Demo Roadmap — 2026-05-31, 2:30 PM

_Authored 10:26 AM. Demo at **14:30**. Plan to be **demo-ready by 14:00** (30-min buffer for
setup, lighting, and a dry run). That leaves ~3.5 hours of working time._

**The milestone being demoed:** Human-Machine Interface → **Reactive Safety Supervisor**
(reach toward the arm → it stops; withdraw → it resumes). The pick-and-place demo is the
**safety net** we lock down first so we always have *something* live.

> **Strategy: build a fallback ladder.** Each milestone below leaves a self-contained,
> demoable state. If we run out of time at any rung, we demo the last green gate — never
> nothing. Lock the safety net first, then climb toward the headline.

---

## 🔖 Status — updated 2026-05-31 (LATEST; robot currently unavailable again)

The original same-day countdown below (10:30 → 14:30) is **superseded** — treat this block
as the live status. The Dobot/Orbbec come and go; what's been verified on hardware vs.
written-but-untested is tracked per row.

Legend: 🟢 verified on hardware · 🟡 code done, needs hardware test · 🔵 done & verified
without hardware (unit tests / imports) · 🔴 not started / still to do.

| Gate | State | Notes |
|---|---|---|
| **M0** full pick-and-place | 🟡 **works once, NOT repeatable** | Ran clean **once** on hardware (pick→place→Batch Complete). A **second run failed**: arm moved, gripper opened/closed twice without grabbing, then "complete" — i.e. it grabbed air. Likely pick-coordinate drift between runs or `Z_PICK` height. **Needs the repeatability fix below before it's a reliable fallback.** |
| **M0** place-from-height fix | 🟢 verified | `Z_PLACE` descend ran on hardware in the clean run. |
| **M0** hand-eye recalibration | 🟢 redone this session | Old homography was stale (camera had moved → systematic miss). Recalibrated; `getTransformationMatrix.py` hardened (index 1, UTF-8, frame warm-up, marker **size filter + duplicate rejection + mm-error gate**). One clean pick proved it; second-run miss hints at residual drift — **lock the camera down.** |
| **M0** commit all fixes | 🔴 still to do | Nothing committed yet — working tree is the only copy of everything below. |
| **M1** hand detection | 🟢 **GREEN** | `hand_detect.py` live-verified (1190+ frames, clean 1- & 2-hand tracking, no false positives). Re-confirm on the Orbbec at demo time. |
| **M2** `stop_motion()` | 🟢 verified | `dobotArm.py`; ran on the arm in the smoke test (force-stop + clear, no fault). |
| **M2** `safety_supervisor.py` state machine | 🔵 **done (16/16 unit tests pass)** | New module: pure-logic `NORMAL`/`STOP` + hysteresis. No hardware needed; `python safety_supervisor.py` runs the tests. |
| **M3** safety integrated into pickCVBlock | 🔵 code done, imports verified | `safety_checkpoint()` gates each descent in the pick & place sequences (cooperative hops). Degrades to plain demo if MediaPipe/model missing; holds arm if camera drops frames. **Compiles + imports clean. Wave-to-halt NOT yet hardware-tested.** |
| **M3** standalone A↔B shuttle demo | 🔴 optional | The plan's trivial shuttle. Integrating into the real pick-and-place (above) is the path we took instead; a bare shuttle is only needed if we want the simplest possible headline. |
| **M4** on-screen readout / SLOW / RETREAT | 🟡 partial | `safety_supervisor.draw_overlay()` already draws the zone + state (the readout). `SLOW`/`RETREAT` not done. |

> **Hardware notes (2026-05-31):** Dobot enumerates on **COM4** (was COM3; `SearchDobot`
> auto-detects, so no code change). The **Orbbec is flaky** — dropped off USB several times,
> and once opened but delivered no frames (MSMF error `-1072875772`); **reseating the USB
> cable fixed it each time.** Always confirm index 1 actually *delivers a frame* (not just
> `isOpened()`) before running. `TARGET_MIN_AREA` was lowered 300→50, which let Phase 2 lock
> the under-saturated (S≈147) red part.

### What's left to do (in priority order)
1. 🔴 **Make M0 repeatable** — diagnose the second-run grab failure. On the arm, watch run 2's
   pick: jaws close **beside** the part → coordinate drift (recalibrate + physically secure the
   camera); jaws close **above** it → lower `Z_PICK` / raise the part. **This is the top fallback risk.**
2. 🟡 **Wave-to-halt hardware test** of the integrated safety layer — reach into the danger zone
   mid-run → arm holds at the next `safety_checkpoint()`; withdraw → resumes. This is the M2/M3
   gate that **scores the milestone**; the code is in, only the live test remains.
3. 🔴 **Deferred robustness (not yet done, owner said "not now"):** wire the unused
   `PIXEL_TOLERANCE` into a real motion-based stability lock (Phase 1/2 currently lock on blob
   *count* only, not on the part having stopped moving) — a likely contributor to wrong-coord locks.
4. 🔴 **Clear the target after a place** so run 2 doesn't re-detect the red part sitting in the tray.
5. 🔴 **Multi-cycle loop** — the main loop runs exactly one pick/place pass then exits (a "second run"
   means relaunching, which re-homes + re-opens the flaky Orbbec).
6. 🔴 **Commit everything** — `dobotArm.stop_motion`, the pickCVBlock fixes + safety wiring,
   `hand_detect.py` + model, `safety_supervisor.py`, the hardened calibration script, and these notes.
7. 🟡 **Tune `GRIP_R_OFFSET_DEG`** in `grasp_orientation.py` once on the arm (one-time grip-angle zero).

---

## The fallback ladder (what we can demo at each gate)

| If we get to… | We can demo… | Scores the milestone? |
|---|---|---|
| **M0** | Clean pick-and-place | No — but a working live demo |
| **M2** | Arm shuttles A↔B, **halts on hand wave** | **Yes** (core safety behavior) |
| **M3** | Full integrated supervisor: freeze + resume | **Yes**, cleanly |
| **M4** | + on-screen readout / SLOW / RETREAT polish | Yes, polished |

**Golden rule (from the plan doc):** get the **STOP checkpoint (M2)** working before
touching any depth/calibration upgrade. STOP-with-margin is what scores the demo.

---

## ⏱️ Timeline

### M0 — Lock the safety net (10:30 → 11:15, 45 min)
Get the existing pick-and-place to a clean, repeatable run. This is the guaranteed fallback.

- [ ] **Commit** the uncommitted camera-index→1 / UTF-8 / frame-guard fixes in
      `pickCVBlock.py` (HEAD still opens the wrong camera — don't lose these).
- [x] **Fix the place-from-height bug** (code done 2026-05-31; hardware run pending).
      `phase_execute_batch` now descends to a new `Z_PLACE` constant (defaults to `Z_PICK`)
      **before** `open_gripper`, then lifts back to `Z_SAFE`:
      ```python
      dobotArm.move_to_xyz(api, drop_x, drop_y, Z_SAFE)   # over the drop
      dobotArm.move_to_xyz(api, drop_x, drop_y, Z_PLACE)  # descend first (was releasing at Z_SAFE)
      dobotArm.open_gripper(api)                           # release on the surface
      dobotArm.stop_pump(api)
      dobotArm.move_to_xyz(api, drop_x, drop_y, Z_SAFE)   # lift clear
      ```
      (Tune `Z_PLACE` up a few mm if the part jams; don't crash the gripper into the tray.)
- [ ] **Run end-to-end** and confirm a full pick **and** a clean place. Repeat 2–3×.
- 🟢 **GATE M0:** pick-and-place works repeatably → fallback demo secured.

> Do **not** re-tune the `PLATE_*` / `TARGET_*` detection constants — they're calibrated.
> If detection misbehaves, fix lighting/placement/timing, not the thresholds.

---

### M1 — Hand detection, pixel-space (11:15 → 12:00, 45 min)
New file `hand_detect.py`. Steps 1–2 of the plan, but **stay in pixel space** to dodge the
calibration risk (see note below).

- [x] Download `hand_landmarker.task` (float16, ~7.8 MB) — done, sits next to the script.
- [x] Wrap MediaPipe **Tasks** `HandLandmarker` (0.10.35 has no legacy `mp.solutions.hands`)
      in a `HandDetector` class returning `HandResult(present, hands)` with pixel-space
      centroids (+ all 21 landmarks per hand for later fingertip logic).
- [x] Live preview: draws landmarks + centroid, prints the centroid, shows FPS.
- 🟢 **GATE M1 — GREEN (2026-05-31).** Live-verified on the **laptop webcam (index 0)**:
      1190+ frames of clean single- and two-hand tracking, ~1–2 px jitter when still, no
      false positives. The Orbbec (index 1) wasn't enumerating this session, but detection
      doesn't need the calibrated camera — **re-confirm on the Orbbec at demo time.**

> **Key time-saver — work in pixel space, not robot mm.** The STOP demo only needs *"is a
> hand near the robot's work zone in the image?"* Define the danger zone as a pixel box (or
> radius around the work area) and skip `pixel_to_robot` / `HomographyMatrix.npy` entirely.
> This removes the single biggest risk (hand-eye calibration + raised-hand parallax). Add
> robot-frame mapping later only as an M4 upgrade if there's time.

---

### M2 — Supervisor + STOP (12:00 → 13:00, 60 min) ← **the milestone gate**
New file `safety_supervisor.py` + a `stop_motion` helper. Steps 3–4.

- [x] `stop_motion(api)` in
      [dobotArm.py](../../Collaborative_Robotics/dobotArm.py):
      `SetQueuedCmdForceStopExec(api)` then `SetQueuedCmdClear(api)` — **verified on the arm**
      (smoke test, no fault).
- [x] State machine in [safety_supervisor.py](../../Collaborative_Robotics/safety_supervisor.py):
      input = hand centroids (pixels) → output `NORMAL / STOP`, with **hysteresis** (resume only
      after the zone is clear for `clear_frames` ≈ 0.7 s). **16/16 unit tests pass, no hardware.**
      `SLOW`/`RETREAT` deferred to M4.
- [ ] **Wave-to-halt hardware test** (the only piece left for this gate): integrated into
      `pickCVBlock` via `safety_checkpoint()` — code in & imports verified, but the live
      reach-in-→-freeze test still needs the arm.
- 🟡 **GATE M2: code complete, hardware test pending.** This is what scores the milestone.

---

### M3 — Integrated, demo-shaped (13:00 → 13:45, 45 min)
Step 5 — the clean headline demo. Solve the blocking-move problem with **cooperative hops**.

- [x] **Cooperative-hop gating implemented in the real pick-and-place** (chose this over a
      bare A↔B shuttle): `safety_checkpoint()` is called before each descent in
      `phase_execute_batch`'s pick and place sequences. It reads a frame, runs `HandDetector`,
      updates the `SafetySupervisor`, and **blocks while a hand is in the danger zone**,
      resuming once clear (hysteresis). No threading. Compiles + imports verified.
- [ ] Behavior on hardware: arm runs normally → reach in → **freezes** at the next checkpoint
      → withdraw → **resumes**. (Code in; needs the live test — same item as the M2 gate.)
- 🟡 **GATE M3:** integrated and code-complete; freeze-and-resume **not yet shown on the arm**.

> Don't attempt the watchdog-thread mid-motion abort under time pressure — cooperative hops
> are the MVP and are reliable. Watchdog is M5/optional only.
>
> Note: reaction latency is **one hop**, and a hop here is a full blocking `move_to_xyz`
> (it busy-waits to completion). So a hand entering *mid-move* isn't caught until that move
> finishes and the next `safety_checkpoint()` runs. Good enough to score; the watchdog-thread
> upgrade is what would make it instant.

---

### M4 — Polish, only if green by ~13:45 (13:45 → 14:00, 15 min)
Pick the highest-visual-impact item; skip the rest.

- [ ] **On-screen readout** (biggest demo payoff): draw the current state
      (`NORMAL`/`STOP`) and the danger zone on the camera window so judges *see* it react.
- [ ] (opt.) `SLOW` zone — scale speed via `SetPTPCommonParams` in a band outside danger.
- [ ] (opt.) `RETREAT` to home on a persistent intrusion.

---

### 14:00 → 14:30 — Freeze & rehearse
- [ ] **Code freeze.** No new features after 14:00.
- [ ] Full dry-run of the exact demo sequence, twice. Fix lighting/placement, not code.
- [ ] Stage the workspace: camera on **index 1** (verify it *delivers a frame*),
      `$env:PYTHONUTF8="1"` set, Dobot on **COM4** homed, hand-landmarker model present,
      part + tray placed, **camera physically secured** (so the homography stays valid).
- [ ] Have the **M0 pick-and-place** ready as the instant fallback if the supervisor flakes.

---

## Pre-flight checklist (the things that silently break the demo)
- [ ] `$env:PYTHONUTF8 = "1"` in the demo terminal (Dobot DLL crashes under cp1252).
- [ ] Camera **index 1** (Orbbec), not 0 (laptop webcam) — and confirm it **delivers a frame**,
      not just `isOpened()`. If it won't stream, **reseat the USB cable** (the known fix).
- [ ] `hand_landmarker.task` downloaded and pathed (no internet at demo = no model = no demo).
- [ ] Dobot powered, homed, **COM4** detected; clear physical reach envelope.
- [ ] **Camera physically locked down** — if it shifts after calibration, picks miss (this bit us).
- [ ] Conservative danger zone — bias it **large**; a too-early stop is safe, a missed hand isn't.
- [ ] Good, steady lighting (Orbbec auto-exposure drifts; vivid part color).
- [ ] Sanity-check: `python safety_supervisor.py` → "ALL SAFETY SUPERVISOR TESTS PASSED".

## Cut-list (drop these the moment we're behind)
1. Robot-frame (mm) mapping for the **safety zone** → it already stays in pixel space.
2. `SLOW` and `RETREAT` → STOP-only is enough to score.
3. Watchdog-thread abort → cooperative hops only.
4. Orbbec depth / true 3D → table-plane is fine.
5. Any re-tuning of the (already-calibrated) vision thresholds.
6. `grasp_orientation` rotation → if grip-angle misbehaves, pass `rHead=0` (grab straight-down)
   and place the part aligned to the jaws; the safety demo doesn't depend on it.

## Run commands
```powershell
$env:PYTHONUTF8 = "1"
cd Collaborative_Robotics
# The integrated demo IS pickCVBlock now — it runs the pick-and-place with the
# reactive safety layer (reach into the zone → it holds → withdraw → resumes).
..\.venv\Scripts\python.exe pickCVBlock.py          # M0 fallback + M2/M3 safety headline
..\.venv\Scripts\python.exe safety_supervisor.py    # M2 unit tests (no hardware) — should all pass
..\.venv\Scripts\python.exe hand_detect.py          # M1 standalone test (Orbbec, index 1)
..\.venv\Scripts\python.exe hand_detect.py 0        # M1 on the laptop webcam if the Orbbec isn't enumerating
..\.venv\Scripts\python.exe getTransformationMatrix.py   # redo hand-eye calibration (moves arm)
```
