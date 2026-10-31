"""Stream the two leader arms on this Mac to the robot's teleop page (pi4/web/teleop_web.py).

    python leader_bridge.py            # opens the SSH tunnel itself, then streams at 50 Hz
    python leader_bridge.py --swap     # if the follower arms come out mirrored (left<->right)

It opens one SSH tunnel to the Pi that carries both the leader stream
(127.0.0.1:8766) and the dashboard (http://127.0.0.1:8080), so the page link
works for as long as this runs. Reads Present_Position only: it never enables
torque on the leaders.
"""
import argparse
import json
import select
import socket
import subprocess
import sys
import time

import scservo_sdk as scs

LEFT = "/dev/cu.usbmodem5AE60529891"    # leader 5AE6052989 drives the LEFT follower (confirmed 2026-10-03)
RIGHT = "/dev/cu.usbmodem5AE60833451"   # leader 5AE6083345 drives the RIGHT follower
HOST = "alohamini-ts"
POS, MIN_LIM, MAX_LIM = 56, 9, 11
IDS = (1, 2, 3, 4, 5, 6)


class Leader:
    def __init__(self, dev):
        self.dev, self.ok, self.last, self.gripper = dev, False, None, None
        self.ph = scs.PortHandler(dev)
        try:
            if not self.ph.openPort() or not self.ph.setBaudRate(1_000_000):
                raise OSError("open failed")
        except Exception:
            print(f"leader {dev} not found: that follower arm will stay limp")
            return
        self.ok = True
        self.pk = scs.PacketHandler(0)
        self.sync = scs.GroupSyncRead(self.ph, self.pk, POS, 2)
        for i in IDS:
            self.sync.addParam(i)
        lo = self.pk.read2ByteTxRx(self.ph, 6, MIN_LIM)[0]
        hi = self.pk.read2ByteTxRx(self.ph, 6, MAX_LIM)[0]
        self.gripper = [lo, hi]

    def read(self):
        if not self.ok:
            return None
        if self.sync.txRxPacket() == scs.COMM_SUCCESS:
            vals = [self.sync.getData(i, POS, 2) for i in IDS]
            if all(v is not None for v in vals):
                self.last = vals
        return self.last


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--swap", action="store_true", help="swap which leader drives which follower")
    ap.add_argument("--hz", type=float, default=50)
    args = ap.parse_args()

    left, right = (RIGHT, LEFT) if args.swap else (LEFT, RIGHT)
    L, R = Leader(left), Leader(right)
    if not (L.ok or R.ok):
        print("no leader arms found; dashboard only (base, lift, cameras)")
    lim = {k: {"gripper": v.gripper} for k, v in (("left", L), ("right", R)) if v.ok}
    print(f"leaders: left={left}  right={right}")

    def open_tunnel():
        return subprocess.Popen(
            ["ssh", "-N", "-o", "BatchMode=yes", "-o", "ExitOnForwardFailure=yes",
             "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=2",
             "-L", "8766:127.0.0.1:8766", "-L", "8080:127.0.0.1:8080", HOST])
    tunnel = open_tunnel()
    time.sleep(2.5)
    print("dashboard: http://127.0.0.1:8080")

    period = 1.0 / args.hz
    try:
        while True:
            try:
                s = socket.create_connection(("127.0.0.1", 8766), timeout=3)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print("streaming leaders to the robot ... (Ctrl-C to stop)")
                n, rtt, rxbuf = 0, None, b""
                while True:
                    t0 = time.perf_counter()
                    msg = {"left": L.read(), "right": R.read()}
                    if n % 50 == 0:
                        msg["lim"] = lim
                    if n % 25 == 0:
                        msg["ping"] = time.perf_counter()      # robot echoes it: measures the link
                    if rtt is not None:
                        msg["rtt_ms"] = rtt
                    while select.select([s], [], [], 0)[0]:
                        chunk = s.recv(4096)
                        if not chunk:
                            raise OSError("robot closed the link")
                        rxbuf += chunk
                        while b"\n" in rxbuf:
                            line, rxbuf = rxbuf.split(b"\n", 1)
                            try:
                                rtt = round((time.perf_counter() - json.loads(line)["pong"]) * 1000, 1)
                            except (ValueError, KeyError):
                                pass
                    s.sendall((json.dumps(msg) + "\n").encode())
                    n += 1
                    time.sleep(max(0.0, period - (time.perf_counter() - t0)))
            except OSError as e:
                # Robot rebooted or lost power: the SSH tunnel dies with it, so reopen it.
                if tunnel.poll() is not None:
                    tunnel = open_tunnel()
                print(f"robot not reachable ({e}); retrying in 3 s")
                time.sleep(3)
    except KeyboardInterrupt:
        print("\nstopped; follower arms hold their last pose")
    finally:
        tunnel.terminate()


if __name__ == "__main__":
    main()
