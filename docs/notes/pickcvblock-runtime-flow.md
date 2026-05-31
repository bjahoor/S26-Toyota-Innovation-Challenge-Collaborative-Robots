# pickCVBlock.py — Runtime Flow

A high-level map of how [pickCVBlock.py](../../Collaborative_Robotics/pickCVBlock.py)
runs: a CV-driven pick-and-place state machine for the Dobot Magician, with a
reactive safety gate layered on top. See also
[reactive-safety-supervisor.md](reactive-safety-supervisor.md) (the safety design)
and [project-status.md](project-status.md) (where the code currently stands).

---

## Architecture diagram

```
                              HARDWARE
   ┌────────────────────┐                      ┌──────────────────────┐
   │  Orbbec camera      │                      │  Dobot Magician arm   │
   │  (cv2 index 1)      │                      │  (serial queue, COM*) │
   └─────────┬───────────┘                      └───────────▲──────────┘
             │ frames                            cmds via    │
             │                                   dobotArm /   │
             ▼                                   DobotDllType │
 ════════════════════════════ pickCVBlock.py ══════════════════════════
                                                              │
  ┌─────────────────────────── STARTUP (module level) ───────┴───────┐
  │  • sys.stdout → UTF-8  (so Dobot DLL load doesn't crash)          │
  │  • api = dType.load()            • cap = VideoCapture(1)          │
  │  • load HomographyMatrix.npy (pixel→robot mm)                     │
  │  • load camera_params.npz → build undistort map1/map2            │
  │  • build safety layer:  HandDetector + SafetySupervisor          │
  │       └─ if MediaPipe/model missing → safety = no-op (graceful)  │
  └──────────────────────────────────┬───────────────────────────────┘
                                      │
                          initialize_robot · open_gripper · stop_pump
                                      │
                                      ▼
  ┌──────────────────── MAIN STATE MACHINE (try:) ───────────────────┐
  │                                                                   │
  │  state="scanning plate"  ─────►  PHASE 1: phase_detect_plates()   │
  │                                  ┌──────── frame loop ─────────┐  │
  │                                  │ read→undistort→metal mask   │  │
  │                                  │ →HoughCircles→pixel_to_robot│◄─┼─ stability
  │                                  │ stable 30 frames? ──no──────┘  │  counter
  │                                  └──── yes → return tray (x,y) ───┘
  │                                      │ next_state()                │
  │  state="scanning target" ────►  PHASE 2: phase_detect_targets()   │
  │                                  ┌──────── frame loop ─────────┐  │
  │                                  │ read→undistort→red HSV mask │  │
  │                                  │ →contours (area gate)       │◄─┼─ stability
  │                                  │ →grasp_orientation: grip ∠  │  │  counter
  │                                  │ stable 30 frames? ──no──────┘  │
  │                                  └─ yes → return [(x,y,grip_r)]──┘
  │                                      │ next_state()                │
  │  state="pick place"  ────────►  PHASE 3: phase_execute_batch()    │
  │                                                                   │
  │     for each part in pick_list:                                   │
  │       ┌─────────────────────────────────────────────────────┐    │
  │       │ safety_checkpoint("pick approach") ──┐               │    │
  │       │ move_to_xyz(part, Z_SAFE, grip_r)    │               │    │
  │       │ safety_checkpoint("pick descend")  ──┤               │    │
  │       │ move_to_xyz(part, Z_PICK, grip_r)    │  each gate    │    │
  │       │ close_gripper · lift to Z_SAFE       │  blocks until │    │
  │       │ move_to_home  (clear camera view)    │  zone clear   │    │
  │       │ drop=phase_detect_plates()  ◄── re-lock tray         │    │
  │       │ safety_checkpoint("place approach")──┤               │    │
  │       │ move_to_xyz(drop, Z_SAFE)            │               │    │
  │       │ safety_checkpoint("place descend") ──┘               │    │
  │       │ move_to_xyz(drop, Z_PLACE) · open_gripper · stop_pump│    │
  │       │ lift to Z_SAFE                                       │    │
  │       └─────────────────────────────────────────────────────┘    │
  │                              │ "Batch Complete" → next_state()    │
  └──────────────────────────────┼───────────────────────────────────┘
                                  ▼
  ┌──────────────────────── finally: SHUTDOWN ───────────────────────┐
  │  errored?  ── yes ─► stop_motion (force-stop + clear queue)       │
  │            ── no  ─► move_to_home                                 │
  │  DisconnectDobot · hand_detector.close · cap.release · destroyAll │
  └───────────────────────────────────────────────────────────────────┘


   ╔══════════ safety_checkpoint(context)  — the reactive gate ════════╗
   ║  (called BETWEEN blocking moves; no-op if safety disabled)         ║
   ║                                                                    ║
   ║   ┌──────────────── loop ────────────────┐                        ║
   ║   │ cap.read → undistort                  │                        ║
   ║   │ HandDetector.process(frame)           │  hand centroids         ║
   ║   │ supervisor.update(centroids)          │──────────┐             ║
   ║   │ draw overlay → imshow("Detection")    │          ▼             ║
   ║   │ supervisor.is_safe? ── no ── HOLD ─────┘    NORMAL / STOP       ║
   ║   └──────────── yes → return (resume task) ──┘  (+ hysteresis:     ║
   ║                                                  N clear frames)   ║
   ╚════════════════════════════════════════════════════════════════════╝
```

---

## The key idea

There are **two kinds of feedback loop**:

- **Stability counters (Phases 1–2)** gate *perception* — don't "lock in" a
  detection until it holds for ~30 frames (~1 s), so a flickering or moving target
  can't be committed.
- **`safety_checkpoint` (Phase 3)** gates *actuation* — because `move_to_xyz` is
  **blocking** (busy-waits until the arm arrives), the supervisor can only react
  *between* hops. So the gate sits before each approach/descent and holds the arm
  there while a hand is in the danger zone. This is the "reach in → it freezes,
  withdraw → it resumes" behavior, with hysteresis so it doesn't chatter at the
  zone boundary.

One full pass = **one** plate→target→pick/place cycle, then the program exits. The
`finally` block always runs (clean park-home on success, force-stop on error/Ctrl+C).

## Module dependencies (who does what)

| Module | Role in this flow |
|---|---|
| `dobotArm.py` | Motion primitives (`move_to_xyz`, gripper, `stop_motion`) → Dobot queue. |
| `grasp_orientation.py` | Phase 2: gripper rotation to grab the part by its short side. |
| `hand_detect.py` | `safety_checkpoint`: MediaPipe hand centroids per frame. |
| `safety_supervisor.py` | `safety_checkpoint`: NORMAL/STOP decision + hysteresis + overlay. |
| `HomographyMatrix.npy` / `camera_params.npz` | pixel→robot mapping + lens undistort. |
