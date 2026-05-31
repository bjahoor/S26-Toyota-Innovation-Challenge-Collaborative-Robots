#jointListener.py
#Listens for the UDP joint-angle datagrams that controlArm.py --live pushes,
#and prints them live as they arrive. Run this in a SECOND terminal, then run a
#--live move in the first one.
#
#    Terminal 1:  python jointListener.py
#    Terminal 2:  python controlArm.py --live xyz 250 0 30
#
#Options:
#    --host H   address to bind (default 0.0.0.0 = listen on all interfaces)
#    --port N   UDP port to listen on (default 9999, must match controlArm --port)

import json
import socket
import argparse


def main():
    ap = argparse.ArgumentParser(
        description="Print the joint angles broadcast by controlArm.py --live")
    ap.add_argument("--host", default="0.0.0.0",
                    help="address to bind (default 0.0.0.0)")
    ap.add_argument("--port", type=int, default=9999,
                    help="UDP port to listen on (default 9999)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # NOTE: deliberately NOT setting SO_REUSEADDR. On Windows that flag lets a
    # second listener bind the SAME UDP port, after which incoming datagrams are
    # delivered to only one of the listeners at random -- so a stale/forgotten
    # listener silently "steals" the live data and the new one shows nothing.
    # Without it, a second launch fails loudly here instead (see below).
    try:
        sock.bind((args.host, args.port))
    except OSError as e:
        print(f"Could not bind {args.host}:{args.port} -- {e}\n"
              "A listener is probably already running on this port. Close the other "
              "one (or pick a different --port, matching controlArm.py --port).")
        return

    print(f"Listening for joint angles on {args.host}:{args.port}  (Ctrl+C to quit)")
    try:
        while True:
            data, _ = sock.recvfrom(4096)
            try:
                d = json.loads(data.decode("utf-8"))
            except ValueError:
                continue  # ignore anything that isn't our JSON

            status = "MOVING" if d.get("moving") else "idle  "
            line = (f"[{status}] t={d.get('t', 0):6.2f}s   "
                    f"J1={d.get('j1', 0):8.2f}  "
                    f"J2={d.get('j2', 0):8.2f}  "
                    f"J3={d.get('j3', 0):8.2f}")

            # While moving, keep overwriting one line; print a fresh line when the
            # move finishes so each completed move leaves a record.
            if d.get("moving"):
                print("\r" + line + "   ", end="", flush=True)
            else:
                print("\r" + line + "   ")
    except KeyboardInterrupt:
        print("\nDone.")


if __name__ == "__main__":
    main()
