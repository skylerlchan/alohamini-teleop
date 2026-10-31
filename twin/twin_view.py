"""Kinematic MuJoCo twin of the AlohaMini. Poses the model from the Pi's /twin/stream.

The Pi reads the real servos. This process only displays them. It does not send
motor commands.

    pip install "mujoco>=3.1.3"
    set ALOHAMINI_PASSCODE=the-passcode
    python twin/twin_view.py --url https://<pi-hostname>.ts.net/twin/stream

--url may be the site origin (https://<host>) or the full /twin/stream URL.
An SSH tunnel works too: --url http://127.0.0.1:8080/twin/stream

Press R in the viewer to put the base back at the origin and re-anchor the lift.
Joint direction flips live in twin_map.json.
"""
import argparse
import json
import math
import os
import ssl
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
WHEEL_JOINTS = ("wheel_left_spin", "wheel_back_spin", "wheel_right_spin")
HERE = Path(__file__).resolve().parent


def unwrap_delta(prev, tick, ticks_per_rev=4096):
    """Shortest step from prev to tick on a wrapping encoder."""
    delta = int(tick) - int(prev)
    half = ticks_per_rev // 2
    if delta > half:
        delta -= ticks_per_rev
    elif delta < -half:
        delta += ticks_per_rev
    return delta


def arm_angle(tick, lo, hi, qlo, qhi, sign):
    """Map a servo tick through its own limits onto the MJCF joint range."""
    span = hi - lo
    if span == 0:
        return None
    frac = (float(tick) - lo) / span
    if sign < 0:
        frac = 1.0 - frac
    frac = min(1.0, max(0.0, frac))
    return qlo + frac * (qhi - qlo)


def stream_url(url):
    url = url.strip().rstrip("/")
    if not url.endswith("/twin/stream"):
        url += "/twin/stream"
    return url


class JointIndex:
    def __init__(self, model, mujoco):
        self.qadr = {}
        self.range = {}
        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if not name:
                continue
            self.qadr[name] = int(model.jnt_qposadr[i])
            self.range[name] = (float(model.jnt_range[i][0]), float(model.jnt_range[i][1]))
        needed = ["base_x", "base_y", "base_yaw", "lift", *WHEEL_JOINTS]
        needed += [f"{side}_{j}" for side in ("left", "right") for j in JOINTS]
        missing = [n for n in needed if n not in self.qadr]
        if missing:
            raise SystemExit("model is missing joints: " + ", ".join(missing))


class Mirror:
    """Integrated base and lift, plus the latest arm angles. Writes qpos; never steps."""

    def __init__(self, joints, mapping):
        self.joints = joints
        self.mapping = mapping
        self.arm_q = {}
        self.reset()

    def reset(self):
        self.x = self.y = self.yaw = 0.0
        self.wheels = [0.0, 0.0, 0.0]
        self.lift_m = float(self.mapping.get("lift_start_m", 0.2))
        self.lift_tick = None
        self.last_t = None
        self.last_seq = -1
        self._clamp_lift()

    def _clamp_lift(self):
        lo, hi = self.joints.range["lift"]
        if hi > lo:
            self.lift_m = min(hi, max(lo, self.lift_m))

    def consume(self, snap):
        """Fold one snapshot into the pose. The same seq is ignored."""
        seq = snap.get("seq")
        if seq is None or seq == self.last_seq:
            return
        t = float(snap.get("t") or 0.0)
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t = t
        self.last_seq = seq
        if 0.0 < dt < 0.25:
            base = snap.get("base") or {}
            vx = float(base.get("vx") or 0.0)
            vy = float(base.get("vy") or 0.0)
            wz = float(base.get("wz") or 0.0)
            self.yaw += wz * dt
            cy, sy = math.cos(self.yaw), math.sin(self.yaw)
            self.x += (vx * cy - vy * sy) * dt
            self.y += (vx * sy + vy * cy) * dt
            vel = (snap.get("wheels") or {}).get("vel")
            signs = self.mapping.get("wheel_sign") or [1, 1, 1]
            scale = float(self.mapping.get("wheel_rad_per_raw", 5.215 / 3400))
            if vel:
                for i, raw in enumerate(vel[:3]):
                    self.wheels[i] += float(signs[i]) * float(raw) * scale * dt
        self._follow_lift(snap)
        signs = self.mapping.get("joint_sign") or {}
        for side in ("left", "right"):
            arm = snap.get(side) or {}
            ticks, lims = arm.get("tick"), arm.get("lim")
            if not arm.get("present") or not ticks or not lims:
                continue
            if len(ticks) != 6 or len(lims) != 6:
                continue
            for i, joint in enumerate(JOINTS):
                name = f"{side}_{joint}"
                lo, hi = lims[i]
                qlo, qhi = self.joints.range[name]
                q = arm_angle(ticks[i], lo, hi, qlo, qhi, signs.get(name, 1))
                if q is not None:
                    self.arm_q[name] = q

    def _follow_lift(self, snap):
        lift = snap.get("lift") or {}
        tick = lift.get("tick")
        if tick is None or not lift.get("present"):
            return
        if self.lift_tick is None:
            self.lift_tick = int(tick)
            return
        n = int(self.mapping.get("ticks_per_rev", 4096))
        delta = unwrap_delta(self.lift_tick, tick, n)
        self.lift_tick = int(tick)
        if abs(delta) > int(n * 0.75):
            return
        mm = float(self.mapping.get("lift_mm_per_rev", 84.0))
        sign = float(self.mapping.get("lift_sign", 1))
        self.lift_m += sign * delta / n * (mm / 1000.0)
        self._clamp_lift()

    def write(self, data):
        def put(name, value):
            data.qpos[self.joints.qadr[name]] = value
        put("base_x", self.x)
        put("base_y", self.y)
        put("base_yaw", self.yaw)
        put("lift", self.lift_m)
        for i, name in enumerate(WHEEL_JOINTS):
            put(name, self.wheels[i])
        for name, q in self.arm_q.items():
            put(name, q)


def reader(url, token, insecure, box, stop):
    ctx = ssl._create_unverified_context() if insecure else None
    headers = {"Authorization": f"Bearer {token}", "Accept": "text/event-stream",
               "Cache-Control": "no-cache"}
    while not stop.is_set():
        try:
            req = urllib.request.Request(url, headers=headers)
            kwargs = {"timeout": 60}
            if url.startswith("https"):
                kwargs["context"] = ctx if insecure else None
            with urllib.request.urlopen(req, **kwargs) as resp:
                print(f"connected {url}", flush=True)
                with box["lock"]:
                    box["up"] = True
                for raw in resp:
                    if stop.is_set():
                        break
                    line = raw.decode("utf-8", "replace").strip()
                    if not line.startswith("data:"):
                        continue
                    try:
                        snap = json.loads(line[5:].strip())
                    except ValueError:
                        continue
                    with box["lock"]:
                        box["snap"] = snap
                        box["recv"] = time.time()
        except urllib.error.HTTPError as exc:
            print(f"twin stream HTTP {exc.code}"
                  + (" (passcode rejected)" if exc.code in (401, 403) else ""), flush=True)
            if exc.code in (401, 403):
                stop.set()
                return
        except Exception as exc:
            print(f"twin stream lost ({exc}); retrying", flush=True)
        with box["lock"]:
            box["up"] = False
        if stop.wait(2):
            return


def main():
    ap = argparse.ArgumentParser(description="MuJoCo kinematic twin of the live AlohaMini")
    ap.add_argument("--url", required=True, help="tailnet origin or full /twin/stream URL")
    ap.add_argument("--token", default=os.environ.get("ALOHAMINI_PASSCODE", ""),
                    help="passcode; defaults to ALOHAMINI_PASSCODE")
    ap.add_argument("--xml", default=str(HERE / "alohamini_twin.xml"))
    ap.add_argument("--map", default=str(HERE / "twin_map.json"))
    ap.add_argument("--insecure", action="store_true", help="skip TLS verification")
    args = ap.parse_args()
    if not args.token.strip():
        sys.exit("set ALOHAMINI_PASSCODE or pass --token")

    try:
        import mujoco
        import mujoco.viewer
    except ImportError:
        sys.exit('MuJoCo is not installed. Run: pip install "mujoco>=3.1.3"')

    mapping = json.loads(Path(args.map).read_text(encoding="utf-8"))
    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    joints = JointIndex(model, mujoco)
    mirror = Mirror(joints, mapping)
    mirror.write(data)
    mujoco.mj_forward(model, data)

    url = stream_url(args.url)
    box = {"lock": threading.Lock(), "snap": None, "recv": 0.0, "up": False}
    stop = threading.Event()
    reset = threading.Event()
    threading.Thread(target=reader, args=(url, args.token.strip(), args.insecure, box, stop),
                     daemon=True).start()

    def on_key(keycode):
        try:
            ch = chr(keycode)
        except ValueError:
            return
        if ch in ("r", "R"):
            reset.set()

    print("viewer open. R zeros the base and re-anchors the lift. Close the window to quit.", flush=True)
    printed = 0.0
    try:
        with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
            viewer.cam.lookat[:] = [0.0, 0.0, 0.45]
            viewer.cam.distance = 1.8
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
            while viewer.is_running() and not stop.is_set():
                if reset.is_set():
                    reset.clear()
                    with viewer.lock():
                        mirror.reset()
                        mirror.write(data)
                        mujoco.mj_forward(model, data)
                with box["lock"]:
                    snap = box["snap"]
                    up = box["up"]
                    age = None if not box["recv"] else time.time() - box["recv"]
                with viewer.lock():
                    if snap is not None:
                        mirror.consume(snap)
                        mirror.write(data)
                    mujoco.mj_forward(model, data)
                viewer.sync()
                now = time.time()
                if now - printed > 1.0:
                    printed = now
                    if snap is None:
                        print("waiting for the robot…", flush=True)
                    else:
                        age_s = "–" if age is None else f"{age * 1000:.0f} ms"
                        batt = snap.get("battery_v")
                        batt_s = "–" if batt is None else f"{batt:.1f} V"
                        print(f"twin #{snap.get('seq')}  age {age_s}  "
                              f"{'STALE ' if snap.get('stale') else ''}"
                              f"battery {batt_s}  lift {mirror.lift_m:.3f} m  "
                              f"{'up' if up else 'reconnecting'}", flush=True)
                time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()


if __name__ == "__main__":
    main()
