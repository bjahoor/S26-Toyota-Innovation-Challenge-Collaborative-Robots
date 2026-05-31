"""
safety_supervisor.py - reactive safety state machine for the pick-and-place demo.

This is the "referee with a whistle": each frame it's told whether a human hand is
in the robot's danger zone, and it decides whether the task may run (NORMAL) or must
halt (STOP). It is the milestone-scoring core of the Reactive Safety Supervisor.

Two deliberate design choices (see docs/notes/demo-roadmap-2026-05-31.md):

  * PIXEL SPACE, not robot mm. The danger zone is a pixel box over the work area in
    the camera image. This dodges the biggest risk (hand-eye calibration + raised-
    hand parallax) - we only need "is a hand near the work area in the image?".

  * HYSTERESIS. Once STOP is triggered, we only return to NORMAL after the hand has
    been clear for N consecutive frames (~1 s). Without this the arm chatters
    start/stop at the zone boundary.

Fail-safe principle: every uncertainty should bias toward STOP. A false "hand
present" only stops the arm early (safe); a missed hand is the dangerous case.

This module is PURE LOGIC for the decision (no camera, no robot, no OpenCV needed)
so it can be unit-tested without hardware - run `python safety_supervisor.py` to
execute the built-in self-tests. The optional draw_overlay() helper uses OpenCV but
is only imported lazily so the core stays dependency-light.

It is consumed by pickCVBlock.py through a thin hook (build a SafetySupervisor +
hand_detect.HandDetector once, call .update() each frame). It does NOT open a
camera itself - pickCVBlock already owns the one camera and passes frames in.
"""

NORMAL = "NORMAL"
STOP = "STOP"


def zone_from_frame(width, height, margin_frac=0.15):
    """
    Build a default danger-zone pixel box as a centered rectangle covering the
    middle (1 - 2*margin_frac) of the frame. With margin_frac=0.15 that's the
    central 70% in each axis - generous on purpose ("bias it large": a too-big zone
    stops early, which is safe). Returns (x1, y1, x2, y2).

    Tune per setup by passing an explicit box to SafetySupervisor instead; the
    overlay draws the zone so you can see what to adjust.
    """
    mx = int(width * margin_frac)
    my = int(height * margin_frac)
    return (mx, my, width - mx, height - my)


class SafetySupervisor:
    """
    State machine: input = hand centroids (pixels) per frame; output = NORMAL/STOP.

    zone:          (x1, y1, x2, y2) inclusive pixel box. Any hand centroid inside it
                   is an intrusion.
    clear_frames:  how many consecutive hand-free frames are required to recover from
                   STOP back to NORMAL (the hysteresis). At ~30 fps, 15 ~= 0.5 s; the
                   pickCVBlock hook can raise it for a more conservative resume.
    """

    def __init__(self, zone, clear_frames=15):
        x1, y1, x2, y2 = zone
        # Normalize so callers can pass corners in any order.
        self.zone = (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))
        if clear_frames < 1:
            clear_frames = 1
        self.clear_frames = clear_frames
        self.state = NORMAL
        # Start "already clear" so we don't spuriously STOP on frame 1.
        self._clear_count = clear_frames

    def point_in_zone(self, u, v):
        x1, y1, x2, y2 = self.zone
        return x1 <= u <= x2 and y1 <= v <= y2

    def hand_in_zone(self, centroids):
        """centroids: iterable of (u, v) pixel points. True if any is in the zone."""
        for c in centroids:
            if c is None:
                continue
            u, v = c[0], c[1]
            if self.point_in_zone(u, v):
                return True
        return False

    def update(self, centroids):
        """
        Advance the state machine by one frame. Pass the hand centroids detected this
        frame (empty list if none). Returns the new state.

        - Any hand in the zone -> STOP immediately, reset the clear counter.
        - No hand in the zone   -> count clear frames; once we've seen `clear_frames`
                                   in a row, recover to NORMAL (hysteresis).
        """
        if self.hand_in_zone(centroids):
            self.state = STOP
            self._clear_count = 0
        else:
            self._clear_count += 1
            if self._clear_count >= self.clear_frames:
                self.state = NORMAL
        return self.state

    @property
    def is_safe(self):
        """True when the task is allowed to run."""
        return self.state == NORMAL

    def reset(self):
        self.state = NORMAL
        self._clear_count = self.clear_frames


def draw_overlay(frame, centroids, supervisor):
    """
    Draw the danger zone and current state onto a BGR frame (in place) for the demo.
    Green box = NORMAL, red box = STOP. Imports OpenCV lazily so the decision core
    has no hard OpenCV dependency.
    """
    import cv2

    x1, y1, x2, y2 = supervisor.zone
    stopped = supervisor.state == STOP
    color = (0, 0, 255) if stopped else (0, 200, 0)

    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    cv2.putText(frame, f"SAFETY: {supervisor.state}", (x1, max(0, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    for c in centroids:
        if c is None:
            continue
        u, v = int(c[0]), int(c[1])
        inzone = supervisor.point_in_zone(u, v)
        cv2.circle(frame, (u, v), 8, (0, 0, 255) if inzone else (0, 200, 255), -1)
    return frame


# ---------------------------------------------------------------------------
# Self-tests (no hardware). Run: python safety_supervisor.py
# ---------------------------------------------------------------------------
def _run_self_tests():
    failures = []

    def check(name, cond):
        if cond:
            print(f"  PASS  {name}")
        else:
            print(f"  FAIL  {name}")
            failures.append(name)

    zone = (100, 100, 200, 200)

    # Zone membership
    s = SafetySupervisor(zone, clear_frames=3)
    check("point inside zone detected", s.point_in_zone(150, 150))
    check("point outside zone rejected", not s.point_in_zone(50, 50))
    check("point on zone edge counts", s.point_in_zone(100, 100))

    # Starts NORMAL, no hand keeps NORMAL
    s = SafetySupervisor(zone, clear_frames=3)
    check("starts NORMAL", s.state == NORMAL)
    s.update([])
    check("no hand stays NORMAL", s.state == NORMAL)

    # Hand in zone -> immediate STOP
    s.update([(150, 150)])
    check("hand in zone -> STOP", s.state == STOP)

    # Hand outside zone does NOT clear instantly (hysteresis)
    s.update([(10, 10)])
    check("1 clear frame still STOP (hysteresis)", s.state == STOP)
    s.update([])
    check("2 clear frames still STOP", s.state == STOP)
    s.update([])
    check("3 clear frames -> NORMAL (recovered)", s.state == NORMAL)

    # Re-intrusion mid-recovery resets the counter
    s = SafetySupervisor(zone, clear_frames=3)
    s.update([(150, 150)])          # STOP
    s.update([])                    # clear 1
    s.update([(150, 150)])          # intrude again -> STOP, reset
    check("re-intrusion resets to STOP", s.state == STOP)
    s.update([]); s.update([])
    check("only 2 clear frames after reset still STOP", s.state == STOP)
    s.update([])
    check("3rd clear frame after reset -> NORMAL", s.state == NORMAL)

    # Multiple hands: one in, one out -> STOP
    s = SafetySupervisor(zone, clear_frames=2)
    s.update([(10, 10), (150, 150)])
    check("any hand in zone (of several) -> STOP", s.state == STOP)

    # is_safe reflects state
    check("is_safe True when NORMAL", SafetySupervisor(zone).is_safe)

    # zone_from_frame geometry
    z = zone_from_frame(640, 480, 0.15)
    check("zone_from_frame centered box", z == (96, 72, 544, 408))

    # corner order normalization
    s = SafetySupervisor((200, 200, 100, 100))
    check("zone corners normalized", s.zone == (100, 100, 200, 200))

    print()
    if failures:
        print(f"{len(failures)} TEST(S) FAILED: {failures}")
        return 1
    print("ALL SAFETY SUPERVISOR TESTS PASSED")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(_run_self_tests())
