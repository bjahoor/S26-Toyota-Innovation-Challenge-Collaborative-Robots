#controlArm.py
#A small, reusable command-line tool for driving the Dobot arm to specific
#coordinates and controlling each of its components (gripper, pump, joints,
#end-effector rotation). Built on top of the helpers in dobotArm.py so it
#behaves exactly like the rest of the codebase.
#
#---------------------------------------------------------------------------
#HOW TO USE
#---------------------------------------------------------------------------
#Move to a Cartesian coordinate (x, y, z in mm):
#    python controlArm.py xyz 200 50 50
#    python controlArm.py xyz 200 50 50 --r 45      # with end-effector rotation
#
#Move by joint angles (degrees):
#    python controlArm.py joint 0 45 45
#    python controlArm.py joint 0 45 45 --j4 30
#
#Go to the safe home position:
#    python controlArm.py home
#
#Rotate just the end effector (-90..90 deg):
#    python controlArm.py rotate 90
#
#Gripper / pump:
#    python controlArm.py grip open
#    python controlArm.py grip close
#    python controlArm.py suction on
#    python controlArm.py suction off
#
#Print the current joint angles J1, J2, J3 (does NOT re-home):
#    python controlArm.py pose
#
#WATCH THE ARM MOVE LIVE (--live):
#    Add --live to any motion command to print the live joint angles (J1, J2, J3)
#    RIGHT IN THIS SAME TERMINAL, on a single line that refreshes every
#    --interval seconds (default 0.25s = 4 Hz) until the move finishes. No second
#    script or terminal needed:
#        python controlArm.py --live xyz 250 0 30
#
#    (Optional) --udp ALSO broadcasts the angles over UDP to jointListener.py for
#    a separate dashboard, but you don't need it for the in-terminal readout:
#        python controlArm.py --live --udp xyz 250 0 30
#
#Flags (put them before the command):
#    --no-home    connect WITHOUT running the homing routine. Faster for repeated
#                 calls once the robot has already been homed this power cycle.
#    --speed N    velocity/acceleration ratio, 1-100 (default 50).
#    --live       print live joint angles in this terminal while the arm moves.
#    --interval S seconds between live updates (default 0.25 = 4 Hz).
#    --udp        ALSO push the angles over UDP (for jointListener.py).
#    --host H     UDP destination host for --udp (default 127.0.0.1).
#    --port N     UDP destination port for --udp (default 9999).
#
#You can also import this module and reuse goto()/connect() from your own code:
#    from controlArm import connect, goto
#    api = connect()
#    goto(api, 200, 50, 50)
#---------------------------------------------------------------------------

import os
import sys
import json
import time
import socket
import argparse

# Make the imports work no matter what directory you call this from.
# (dobotArm imports "lib.DobotDllType", which needs this folder on the path.)
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# The vendored Dobot DLL prints a non-ASCII character ('：', U+FF1A) the moment it
# loads, which crashes under Windows' default cp1252 console encoding. Force UTF-8
# on our streams BEFORE importing dobotArm/DobotDllType (their module-level
# dType.load() triggers that print) so this runs from any terminal, not only one
# launched with PYTHONUTF8=1 set. (Same fix as pickCVBlock.py.)
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

import lib.DobotDllType as dType
import dobotArm


def _open_connection(api):
    """Find and connect to the Dobot, clear the queue and any alarm state.

    Exits with a clear, specific message instead of hanging on failure.
    """
    com_port = dType.SearchDobot(api)[0]
    if "COM" not in com_port:
        print("Error: no Dobot COM port found. Is the USB cable plugged in and the "
              "arm powered on? Exiting.")
        sys.exit(1)

    print(f"Found Dobot on {com_port}, connecting...")
    state = dType.ConnectDobot(api, com_port, 115200)[0]
    if state != dType.DobotConnect.DobotConnect_NoError:
        msg = dobotArm.CON_STR.get(state, state)
        print(f"Failed to connect to Dobot ({msg}).")
        if msg == "DobotConnect_Occupied":
            print("The port is busy -- close DobotStudio or any other still-running "
                  "controlArm/python process that's holding it, then retry.")
        sys.exit(1)

    # Stop/clear anything left in the queue so stale commands can't fire.
    dType.SetQueuedCmdStopExec(api)
    dType.SetQueuedCmdClear(api)

    # Clear any alarm state. A Magician that booted into an alarm (e.g. after a
    # limit hit or hard power-off) will SILENTLY refuse to move until cleared --
    # this is a very common "nothing happens" cause.
    clear_alarms = getattr(dType, "ClearAllAlarmsState", None)
    if clear_alarms:
        clear_alarms(api)
    return com_port


def _wait_cmd(api, target_index, timeout, label):
    """Wait until the queue reaches target_index, printing live elapsed time.

    Returns True if it completed, False if it exceeded `timeout` seconds (so the
    caller can report the problem instead of the terminal hanging forever).
    """
    start = time.monotonic()
    last_secs = -1
    while target_index > dType.GetQueuedCmdCurrentIndex(api)[0]:
        elapsed = time.monotonic() - start
        if int(elapsed) != last_secs:
            last_secs = int(elapsed)
            sys.stdout.write(f"\r  {label}... {int(elapsed)}s ")
            sys.stdout.flush()
        if elapsed > timeout:
            sys.stdout.write("\n")
            sys.stdout.flush()
            return False
        dType.dSleep(50)
    sys.stdout.write(f"\r  {label} done ({time.monotonic() - start:.1f}s).        \n")
    sys.stdout.flush()
    return True


def connect(home=True, speed=50, home_timeout=60.0):
    """Connect to the Dobot and get it ready to move, returning the api handle.

    home=True  -> runs the homing routine (recommended after each power-on),
                  showing progress and timing out with a diagnosis if it stalls.
    home=False -> just connects, clears the queue/alarms and sets the speed.
                  Faster for repeated calls when the robot is already homed.
    """
    api = dType.load()
    _open_connection(api)

    if not home:
        dType.SetPTPCommonParams(api, speed, speed, isQueued=0)
        dType.SetQueuedCmdStartExec(api)
        print("Connected (no homing).")
        return api

    # --- homing: enqueue speed + home params + the home command, then run them ---
    dType.SetPTPCommonParams(api, speed, speed, isQueued=1)
    dType.SetHOMEParams(api, dobotArm.home_pos[0], dobotArm.home_pos[1],
                        dobotArm.home_pos[2], 0, isQueued=1)
    home_index = dType.SetHOMECmd(api, temp=0, isQueued=1)[0]
    dType.SetQueuedCmdStartExec(api)

    print("Homing the arm (it moves to its init position and beeps; ~15-30s)...")
    if not _wait_cmd(api, home_index, home_timeout, "homing"):
        print(f"The arm connected but never finished homing within {int(home_timeout)}s.\n"
              "Most likely one of:\n"
              "  * the 12V power adapter isn't plugged in or base power is off\n"
              "    (USB alone detects the arm but cannot move it),\n"
              "  * the arm is in an alarm/limit state, or\n"
              "  * something is physically blocking the arm.\n"
              "Check the power adapter + base LED, clear any obstruction, then retry.\n"
              "If it is already homed, you can skip homing with:  --no-home")
        sys.exit(1)
    return api


def goto(api, x, y, z, r=0):
    """Convenience wrapper so other scripts can move the arm in one call."""
    dobotArm.move_to_xyz(api, x, y, z, r)


def get_joints(api):
    """Return the current joint angles (J1, J2, J3) in degrees.

    GetPose returns [x, y, z, r, J1, J2, J3, J4]; the joints are indices 4-6.
    """
    pose = dType.GetPose(api)
    return pose[4], pose[5], pose[6]


def print_joints(api):
    j1, j2, j3 = get_joints(api)
    print("Joint angles [J1, J2, J3] (deg):")
    print("  ", [round(j1, 2), round(j2, 2), round(j3, 2)])
    return j1, j2, j3


# ---------------------------------------------------------------------------
# LIVE JOINT-ANGLE OUTPUT (--live)
#
# A "sink" is any function with the signature sink(joints, moving, t, command,
# target). The move loop calls it every `interval` seconds. The default sink
# prints to THIS terminal on a single updating line; --udp adds a second sink
# that broadcasts the same data for jointListener.py. All Dobot API calls stay
# on this (the only) thread.
# ---------------------------------------------------------------------------
def make_inline_printer():
    """Return a sink that prints the live joint angles in this terminal on one
    overwriting line, so the readout stays a single tidy line, not a flood."""
    def show(joints, moving, t, command, target):
        status = "MOVING" if moving else "idle  "
        line = (f"[{status}] t={t:6.2f}s   "
                f"J1={joints[0]:8.2f}  J2={joints[1]:8.2f}  J3={joints[2]:8.2f}")
        # While moving, overwrite the same line (\r, no newline). When the move
        # finishes (moving=False) end with a newline so the final value stays.
        sys.stdout.write("\r" + line + "   " + ("" if moving else "\n"))
        sys.stdout.flush()
    return show


def fan_out(*sinks):
    """Combine several sinks into one that forwards to all of them."""
    def call(*a):
        for s in sinks:
            s(*a)
    return call


def make_udp_sender(host, port):
    """Return a send(joints, moving, t, command, target) function that pushes a
    JSON datagram to host:port. Returns a no-op if the socket can't be created."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except OSError as e:
        print(f"Could not open UDP socket ({e}); --live disabled.")
        return lambda *a, **k: None

    addr = (host, port)

    def send(joints, moving, t, command, target):
        msg = {
            "j1": round(float(joints[0]), 3),
            "j2": round(float(joints[1]), 3),
            "j3": round(float(joints[2]), 3),
            "moving": moving,
            "t": round(t, 3),
            "command": command,
            "target": target,
        }
        try:
            sock.sendto(json.dumps(msg).encode("utf-8"), addr)
        except OSError:
            pass  # nothing listening / transient error -> just skip this sample

    return send


def _run_published_move(api, issue_cmd, interval, command, target, send):
    """Issue a PTP command and push the live joint angles every `interval`
    seconds until the motion completes. Runs entirely on the calling thread."""
    start = time.monotonic()
    send(get_joints(api), True, 0.0, command, target)
    last_index = issue_cmd()
    last_pub = -1e9
    while last_index > dType.GetQueuedCmdCurrentIndex(api)[0]:
        now = time.monotonic()
        if now - last_pub >= interval:
            send(get_joints(api), True, now - start, command, target)
            last_pub = now
        dType.dSleep(20)
    # Final resting snapshot (moving=False so the listener knows we're done).
    send(get_joints(api), False, time.monotonic() - start, command, target)


def move_xyz_published(api, x, y, z, r, interval, send):
    _run_published_move(
        api,
        lambda: dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, x, y, z, r, isQueued=0)[0],
        interval, "xyz", [x, y, z, r], send)


def move_joint_published(api, j1, j2, j3, j4, interval, send):
    _run_published_move(
        api,
        lambda: dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJANGLEMode, j1, j2, j3, j4, isQueued=0)[0],
        interval, "joint", [j1, j2, j3, j4], send)


def rotate_published(api, angle, interval, send):
    pose = dType.GetPose(api)
    _run_published_move(
        api,
        lambda: dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, pose[0], pose[1], pose[2], angle, isQueued=0)[0],
        interval, "rotate", [angle], send)


def build_parser():
    p = argparse.ArgumentParser(
        description="Drive the Dobot arm to coordinates / control its components."
    )
    p.add_argument("--no-home", action="store_true",
                   help="connect without running the homing routine")
    p.add_argument("--speed", type=int, default=50,
                   help="velocity/acceleration ratio 1-100 (default 50)")
    p.add_argument("--live", action="store_true",
                   help="print live joint angles in this terminal while the arm moves")
    p.add_argument("--interval", type=float, default=0.25,
                   help="seconds between live updates (default 0.25 = 4 Hz)")
    p.add_argument("--udp", action="store_true",
                   help="ALSO push the live angles over UDP (for jointListener.py)")
    p.add_argument("--host", default="127.0.0.1",
                   help="UDP destination host for --udp (default 127.0.0.1)")
    p.add_argument("--port", type=int, default=9999,
                   help="UDP destination port for --udp (default 9999)")
    p.add_argument("--home-timeout", type=float, default=60.0,
                   help="max seconds to wait for homing before reporting a stall (default 60)")

    sub = p.add_subparsers(dest="command", required=True)

    s = sub.add_parser("xyz", help="move to a Cartesian coordinate (mm)")
    s.add_argument("x", type=float)
    s.add_argument("y", type=float)
    s.add_argument("z", type=float)
    s.add_argument("--r", type=float, default=0.0,
                   help="end-effector rotation in degrees")

    s = sub.add_parser("joint", help="move by joint angles (degrees)")
    s.add_argument("j1", type=float)
    s.add_argument("j2", type=float)
    s.add_argument("j3", type=float)
    s.add_argument("--j4", type=float, default=0.0)

    sub.add_parser("home", help="move to the safe home position")

    s = sub.add_parser("rotate", help="rotate the end effector (-90..90 deg)")
    s.add_argument("angle", type=float)

    s = sub.add_parser("grip", help="open/close the gripper")
    s.add_argument("state", choices=["open", "close"])

    s = sub.add_parser("suction", help="turn the pneumatic pump on/off")
    s.add_argument("state", choices=["on", "off"])

    sub.add_parser("pose", help="print the current joint angles J1, J2, J3 (does not move/home)")

    return p


# Commands that actually produce sustained motion (where --live makes sense).
_MOTION_COMMANDS = {"xyz", "joint", "home", "rotate"}


def main(argv=None):
    args = build_parser().parse_args(argv)

    # "pose" is read-only, so never home for it (homing would move the arm).
    needs_home = (not args.no_home) and args.command != "pose"
    api = connect(home=needs_home, speed=args.speed, home_timeout=args.home_timeout)

    # Set up live joint-angle output only for motion commands.
    live = args.live and args.command in _MOTION_COMMANDS
    if args.live and not live:
        print(f"(--live ignored: '{args.command}' isn't a motion command)")
    send = None
    if live:
        sinks = [make_inline_printer()]          # default: print in this terminal
        if args.udp:                             # optional: also broadcast over UDP
            sinks.append(make_udp_sender(args.host, args.port))
            print(f"Also pushing live angles to {args.host}:{args.port} "
                  f"(run jointListener.py to watch)")
        send = fan_out(*sinks)
        print(f"Live joint angles (updating every {args.interval}s):")

    cmd = args.command
    if cmd == "xyz":
        print(f"Moving to x={args.x}, y={args.y}, z={args.z}, r={args.r}")
        if live:
            move_xyz_published(api, args.x, args.y, args.z, args.r, args.interval, send)
        else:
            dobotArm.move_to_xyz(api, args.x, args.y, args.z, args.r)

    elif cmd == "joint":
        print(f"Moving to joints J1={args.j1}, J2={args.j2}, J3={args.j3}, J4={args.j4}")
        if live:
            move_joint_published(api, args.j1, args.j2, args.j3, args.j4, args.interval, send)
        else:
            dobotArm.move_joint_angles(api, args.j1, args.j2, args.j3, args.j4)

    elif cmd == "home":
        print("Moving to home position", dobotArm.home_pos)
        if live:
            hp = dobotArm.home_pos
            move_xyz_published(api, hp[0], hp[1], hp[2], 0, args.interval, send)
        else:
            dobotArm.move_to_home(api)

    elif cmd == "rotate":
        if not -90 <= args.angle <= 90:
            print("Angle must be between -90 and 90 degrees.")
            sys.exit(1)
        print(f"Rotating end effector to {args.angle} deg")
        if live:
            rotate_published(api, args.angle, args.interval, send)
        else:
            dobotArm.rotate_end_effector(api, args.angle)

    elif cmd == "grip":
        if args.state == "open":
            dobotArm.open_gripper(api)
        else:
            dobotArm.close_gripper(api)
        print(f"Gripper {args.state}.")

    elif cmd == "suction":
        if args.state == "on":
            # api, enableCtrl=1, on=1, isQueued=0  -> pump ON
            dType.SetEndEffectorSuctionCup(api, 1, 1, 0)
            dType.dSleep(100)
        else:
            dobotArm.stop_pump(api)
        print(f"Suction {args.state}.")

    # Always report the final joint angles.
    print_joints(api)


if __name__ == "__main__":
    main()
