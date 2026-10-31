#!/usr/bin/env python3
"""Full AlohaMini teleop from one Pi process: wheels, lift and both follower arms.

Grown from base_web.py (kept unchanged as the base-only fallback). The page still
drives the base (WASD/QE) and lift (U/J). New: both follower arms copy the two
leader arms plugged into the Mac. The Mac runs so101/leader_bridge.py, which
reads the leaders at 50 Hz and streams raw ticks over an SSH tunnel to
127.0.0.1:8766 here. Arms stay limp until "Arms on" is pressed on the page; then
torque comes on one joint at a time and each arm eases to its leader over 4 s.

Wiring on this robot (2026-10-03): the board with wheels 8/9/10 carries the
LEFT arm; the other board carries the RIGHT arm and the lift (ID 11). Ports are
found by probing, never hardcoded.

HOW PEOPLE CONNECT
Binds to 127.0.0.1 only. Tailscale publishes it:
    tailscale serve  --bg 8080   -> tailnet only, needs Tailscale on the client
    tailscale funnel --bg 8080   -> public HTTPS URL, nothing to install
Either way the passcode is the gate. Humans type it on the page; agents send
    Authorization: Bearer <passcode>

WHY THE STOP PATH IS THE WAY IT IS
Feetech servos LATCH the last velocity command: they keep spinning until told
otherwise. An early version zeroed the wheels only in a `finally` block, so a
SIGTERM mid-motion left the robot driving across the floor. Now every exit path
-- signal, exception, atexit -- runs the same hard stop, which both zeroes the
velocity and drops torque.

Threading: only ONE thread ever touches the serial bus, the control loop. HTTP
handlers only update a shared target under a lock. Serial is not thread safe.
"""

import atexit
import glob
import hashlib
import hmac
import json
import math
import logging
import os
import secrets
import signal
import threading
import time
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import Motor, MotorNormMode
from lerobot.robots.alohamini.alohamini import AlohaMini
from lerobot.robots.alohamini.config_alohamini import AlohaMiniConfig

# ---- absolute caps = the MEASURED hardware ceiling -------------------------
# Measured by commanding rising raw velocities and reading Present_Velocity:
#   commanded 3000 -> 2950;  3500 -> 3400 (peak);  4000/4500/5000 -> 3200-3300
# So 3400 raw is the real limit, not lerobot's default of 3000.
#   3400 / (4096/360) = 298.8 deg/s = 5.215 rad/s of wheel
#   linear   = 5.215 * 0.05        = 0.261 m/s
#   rotation = 5.215 * 0.05 / 0.125 = 2.086 rad/s = 120 deg/s
MAX_RAW = 3400
MAX_VX = 0.26
MAX_VY = 0.26
MAX_WZ = 2.08
MAX_LIFT_VEL = 2400         # raw units (~49 mm/s at 84 mm/rev), 4x the old 600; 3400 maxed the motor load going up
LIFT_STALL_GUARD = False    # user turned it off 2026-10-03 ("hate the stopping"); the servo's own overload cut-out still applies
LIFT_STALL_LOAD = 700       # |Present_Load| (0-1000) that counts as hitting an end of travel
LIFT_STALL_READS = 3        # consecutive over-limit reads (checked every 5th cycle) before stopping

# Rotation ramps hard, translation stays gentle: spinning in place barely moves
# the centre of mass, but driving forward is what tips a tall robot.
# Gentle ("friendly") ramps, 2026-10-03: the old ones felt abrupt.
ACC_LIN, DEC_LIN = 0.40, 0.70          # 0.65 s to full drive speed, 0.4 s to stop
ACC_ROT, DEC_ROT = 3.50, 5.00          # 0.6 s to full spin, 0.4 s to stop
ACC_LIFT, DEC_LIFT = 2400.0, 4800.0    # 1 s to full lift speed, 0.5 s to stop
# On top of the ramps, an S-curve: a short low-pass rounds off the corners where
# acceleration starts and ends, which is what makes motion feel jerky.
SMOOTH_TAU_S = 0.15

# ---- battery: read off the servos' own voltage sensors (no extra hardware) ----
BATT_EVERY = 25             # cycles between reads (2 Hz at 50 Hz)
BATT_FULL, BATT_EMPTY = 12.6, 10.5   # assumes a 3S lithium pack; the % is an estimate
BATT_LOW = 10.9             # page turns red and warns below this

WATCHDOG_S = 0.4
LOOP_HZ = 50

# ---- arms ----
ARM_IDS = (1, 2, 3, 4, 5, 6)
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
LEADER_PORT = 8766          # leader_bridge.py on the Mac connects here via ssh -L
LEADER_STALE_S = 0.5        # no leader packet for this long -> arms hold their pose
# Lowest-lag tuning (2026-10-03, "input on the dot"): no servo speed or accel cap,
# stiffer P. Measured on 2026-09-23: P16 ~113 ms lag, P32 ~77 ms, P64 ~58 ms.
ARM_ACCEL = 60              # servo accel limit (~520 deg/s^2): rounds off snaps, barely adds lag
ARM_SPEED = 0               # 0 = no speed cap in position mode
ARM_P_GAIN = 32             # stiffness; lerobot ships 16 (laggy), 48 felt harsh. ~77 ms lag measured at 32.
ARM_MAX_STEP = 400          # per-cycle safety clamp only (~35 deg in 20 ms): never limits a human hand
ARM_RAMP_S = 4.0            # soft start from the follower's pose to the leader's
ARM_TORQUE_GAP = 8          # cycles between per-joint torque-ons (160 ms); avoids brown-out
LIMIT_MARGIN = 30           # stay this many ticks inside each servo's own limits
BIND = "127.0.0.1"          # never 0.0.0.0; Tailscale publishes it
PORT = 8080
# The serial device is NOT stable. /dev/ttyACM0 and ttyACM1 swap between
# replugs and reboots, and sometimes only one board is connected. So never
# hardcode it: find the bus by asking which one answers on the wheel IDs.
WHEEL_IDS = (8, 9, 10)
LIFT_ID = 11

SESSION_TTL_S = 12 * 3600
MAX_FAILS = 10              # failed codes before a lockout
FAIL_WINDOW_S = 300
LOCKOUT_S = 300
STATE_DIR = Path.home() / ".alohamini"

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("base_web")
logging.getLogger().setLevel(logging.INFO)   # lerobot lowers this to WARNING

_lock = threading.Lock()
_target = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "lift": 0.0}
_last_cmd = 0.0
_stats = {"sent": 0, "watchdog_stops": 0, "last_error": None,
          "connected": False, "lift_present": False,
          "actual": {"vx": 0.0, "vy": 0.0, "wz": 0.0, "lift": 0.0}}
_robot = None               # lerobot AlohaMini, used ONLY for wheel kinematics (never connected)
_busL = None                # wheels 8/9/10 + left arm 1-6
_busR = None                # right arm 1-6 + lift 11
_lift_bus = None            # alias of _busR when the lift answers
_flim = {"left": {}, "right": {}}     # follower (lo, hi) per joint
_leader = {"left": None, "right": None, "lim": {}, "t": 0.0, "client": None, "packets": 0}
_arms = {s: {"state": "off", "goal": None, "start": None, "t0": 0.0, "queue": [], "err": 0}
         for s in ("left", "right")}
_arm_req = {"on": False}
_shutdown = threading.Event()
# Positive lift command should raise the carriage. lerobot's LiftAxisConfig uses
# dir_sign = -1, so match it. Flip if up and down come out swapped.
LIFT_DIR = -1

_auth_lock = threading.Lock()
_fails: list[float] = []
_locked_until = 0.0

# ---- one driver at a time ----
# The "seat": the browser that holds it drives; every other browser watches.
# A watcher's heartbeat must NOT feed the watchdog, or a dead driver tab
# would leave the robot moving. STOP works for everyone.
SEAT_GONE_S = 2.0           # driver's page went quiet this long -> seat frees
SEAT_IDLE_S = 60.0          # driver pressed nothing this long -> seat frees
_seat = {"id": None, "name": "", "t_seen": 0.0, "t_active": 0.0}


def _seat_check(cid: str, name: str, moving: bool, want: str | None) -> bool:
    """Update the seat for one request from client `cid`. Returns True if it drives.
    want: "take" claims a free seat, "leave" gives it up, None just reports.
    Caller holds _lock."""
    now = time.time()
    if _seat["id"] and (now - _seat["t_seen"] > SEAT_GONE_S
                        or now - _seat["t_active"] > SEAT_IDLE_S):
        log.info("seat freed (%s went quiet)", _seat["name"] or "driver")
        _seat.update(id=None, name="")
    if want == "take" and cid and _seat["id"] in (None, cid):
        if _seat["id"] != cid:
            log.info("seat taken by %s", name or cid[:6])
            _seat.update(t_active=now)
        _seat.update(id=cid, name=name[:24])
    if want == "leave" and _seat["id"] == cid:
        _seat.update(id=None, name="")
        return False
    if cid and _seat["id"] == cid:
        _seat["t_seen"] = now
        if moving:
            _seat["t_active"] = now
        return True
    return False


# ---------------------------------------------------------------- passcode --
# Unambiguous alphabet: no O/0, I/1, L, S/5, so a code read aloud survives.
_ALPHABET = "ABCDEFGHJKMNPQRTUVWXY2346789"


def _load_secrets() -> tuple[str, bytes]:
    """Return (passcode, hmac_secret), creating and persisting them once."""
    STATE_DIR.mkdir(mode=0o700, exist_ok=True)
    pc_file, sk_file = STATE_DIR / "passcode", STATE_DIR / "session_key"

    env = os.environ.get("ALOHAMINI_PASSCODE", "").strip()
    if env:
        passcode = env
        pc_file.write_text(passcode + "\n")
        pc_file.chmod(0o600)
    elif pc_file.exists() and pc_file.read_text().strip():
        passcode = pc_file.read_text().strip()
    else:
        passcode = "-".join(
            "".join(secrets.choice(_ALPHABET) for _ in range(3)) for _ in range(2)
        )
        pc_file.write_text(passcode + "\n")
        pc_file.chmod(0o600)

    if sk_file.exists() and len(sk_file.read_bytes()) >= 32:
        key = sk_file.read_bytes()
    else:
        key = secrets.token_bytes(32)
        sk_file.write_bytes(key)
        sk_file.chmod(0o600)
    return passcode, key


PASSCODE, _SESSION_KEY = _load_secrets()


def _make_token(ttl=SESSION_TTL_S) -> str:
    exp = str(int(time.time()) + ttl)
    mac = hmac.new(_SESSION_KEY, exp.encode(), hashlib.sha256).hexdigest()[:32]
    return f"{exp}.{mac}"


def _token_ok(tok: str) -> bool:
    try:
        exp_s, mac = tok.split(".", 1)
    except ValueError:
        return False
    if not exp_s.isdigit() or int(exp_s) < time.time():
        return False
    good = hmac.new(_SESSION_KEY, exp_s.encode(), hashlib.sha256).hexdigest()[:32]
    return hmac.compare_digest(mac, good)


def _locked() -> tuple[bool, int]:
    with _auth_lock:
        left = int(_locked_until - time.time())
        return (left > 0), max(0, left)


def _note_fail() -> None:
    global _locked_until
    now = time.time()
    with _auth_lock:
        _fails[:] = [t for t in _fails if now - t < FAIL_WINDOW_S]
        _fails.append(now)
        if len(_fails) >= MAX_FAILS:
            _locked_until = now + LOCKOUT_S
            _fails.clear()
            log.warning("too many wrong passcodes; locked for %ds", LOCKOUT_S)


def _code_ok(code: str) -> bool:
    return hmac.compare_digest(code.strip().upper().replace(" ", ""),
                               PASSCODE.upper())


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# -------------------------------------------------------------- the robot --
def hard_stop(reason="unknown"):
    """Zero every velocity and drop torque. Safe to call repeatedly."""
    if _robot is None or _busL is None:
        return
    try:
        wheels = _robot._body_to_wheel_raw(0.0, 0.0, 0.0)
        for _ in range(3):   # a dropped packet must not leave the robot driving
            _busL.sync_write("Goal_Velocity", wheels, normalize=False)
        _busL.disable_torque()
        log.warning("HARD STOP done (%s): wheels zeroed, torque off", reason)
    except Exception as exc:
        log.error("HARD STOP FAILED (%s): %s -- CUT THE 12V SERVO POWER", reason, exc)
    # The lift is a separate bus, so stop it separately and never let a wheel
    # failure skip it.
    if _lift_bus is not None:
        try:
            for _ in range(3):
                _lift_bus.write("Goal_Velocity", "lift_axis", 0, normalize=False)
            _lift_bus.disable_torque()
        except Exception as exc:
            log.error("LIFT HARD STOP FAILED (%s): %s -- CUT THE 12V SERVO POWER", reason, exc)
    if _busR is not None:
        try:
            _busR.disable_torque()   # right arm (lift already handled above)
        except Exception as exc:
            log.error("RIGHT BUS TORQUE-OFF FAILED (%s): %s", reason, exc)
    for s in _arms.values():
        s.update(state="off", goal=None, queue=[])


def _on_signal(signum, _frame):
    log.warning("signal %s received", signum)
    _shutdown.set()
    hard_stop(f"signal {signum}")
    raise SystemExit(0)


AXES = (("vx", ACC_LIN, DEC_LIN), ("vy", ACC_LIN, DEC_LIN),
        ("wz", ACC_ROT, DEC_ROT), ("lift", ACC_LIFT, DEC_LIFT))


def _slew(cur, tgt, dt):
    """Move cur towards tgt under acceleration limits.

    The acceleration rate applies only while the magnitude grows in the same
    direction; slowing or reversing uses the faster deceleration rate, so
    stopping is never delayed by a gentle ramp.
    """
    out = {}
    for name, acc, dec in AXES:
        c, t = cur[name], tgt[name]
        growing = (abs(t) > abs(c)) and (t * c >= 0)
        step = (acc if growing else dec) * dt
        d = t - c
        out[name] = t if abs(d) <= step else c + (step if d > 0 else -step)
    return out


def control_loop():
    """The only thread allowed to write to the serial bus."""
    period = 1.0 / LOOP_HZ
    cur = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "lift": 0.0}
    sm = dict(cur)
    last = time.perf_counter()
    while not _shutdown.is_set():
        start = time.perf_counter()
        dt = min(0.25, max(1e-3, start - last))
        _stats["loop_hz"] = round(0.9 * _stats.get("loop_hz", LOOP_HZ) + 0.1 / dt, 1)
        last = start
        with _lock:
            tgt = {k: _target[k] for k in ("vx", "vy", "wz", "lift")}
            if (time.time() - _last_cmd) > WATCHDOG_S and any(tgt.values()):
                _target.update(vx=0.0, vy=0.0, wz=0.0, lift=0.0)
                tgt = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "lift": 0.0}
                _stats["watchdog_stops"] += 1
        cur = _slew(cur, tgt, dt)
        k = 1.0 - math.exp(-dt / SMOOTH_TAU_S)
        sm = {a: sm[a] + (cur[a] - sm[a]) * k for a in cur}
        out = {a: (0.0 if abs(v) < 1e-4 else v) for a, v in sm.items()}
        try:
            _busL.sync_write(
                "Goal_Velocity",
                # lerobot takes spin in DEG/s; cur["wz"] is rad/s. Passing rad/s made Q/E 57x too slow.
                _robot._body_to_wheel_raw(out["vx"], out["vy"], math.degrees(out["wz"]), max_raw=MAX_RAW),
                normalize=False,
            )
            if _lift_bus is not None and _stats.get("lift_ok", True):
                try:
                    _lift_bus.write("Goal_Velocity", "lift_axis",
                                    int(LIFT_DIR * out["lift"]), normalize=False, num_retry=0)
                except Exception as exc:
                    # The lift motor went silent (overload/overheat cut-out or a loose cable).
                    # Stop trying every cycle -- each failed write costs ~20 ms of bus time
                    # and starves the arms -- and probe it every 2 s instead.
                    _stats["lift_ok"] = False
                    _stats["lift_error"] = f"lift motor not answering: {exc}"
                    _stats["lift_retry_at"] = time.time() + 2.0
                    log.error("lift motor stopped answering: %s", exc)
                # The lift has no end switches and no position limits in velocity
                # mode, so at full speed it must notice a hard stop by itself.
                _stats["sent_mod"] = (_stats.get("sent_mod", 0) + 1) % 5
                if abs(cur["lift"]) > 200 and _stats["sent_mod"] == 0 and _stats.get("lift_ok", True):
                    raw = _lift_bus.read("Present_Load", "lift_axis", normalize=False, num_retry=0)
                    spd = _lift_bus.read("Present_Velocity", "lift_axis", normalize=False, num_retry=0)
                    _stats["lift_speed"] = spd
                    load = abs(raw & 0x3FF)
                    _stats["lift_load"] = load
                    _stats["stall_n"] = _stats.get("stall_n", 0) + 1 if load > LIFT_STALL_LOAD else 0
                    if LIFT_STALL_GUARD and _stats["stall_n"] >= LIFT_STALL_READS:
                        _stats["lift_block"] = 1 if cur["lift"] > 0 else -1   # refuse this direction until reversed
                        cur["lift"] = 0.0
                        with _lock:
                            _target["lift"] = 0.0
                        _lift_bus.write("Goal_Velocity", "lift_axis", 0, normalize=False)
                        _stats["lift_stalls"] = _stats.get("lift_stalls", 0) + 1
                        _stats["stall_n"] = 0
                        log.warning("lift stopped: load %d at an end of travel", load)
            elif _lift_bus is not None and time.time() > _stats.get("lift_retry_at", 0):
                _stats["lift_retry_at"] = time.time() + 2.0
                cur["lift"] = sm["lift"] = 0.0
                if _lift_bus.ping("lift_axis", num_retry=0) is not None:
                    _lift_bus.write("Torque_Enable", "lift_axis", 0, normalize=False)
                    _lift_bus.write("Operating_Mode", "lift_axis", 1, normalize=False)
                    _lift_bus.write("Goal_Velocity", "lift_axis", 0, normalize=False)
                    _lift_bus.write("Torque_Enable", "lift_axis", 1, normalize=False)
                    _stats["lift_ok"], _stats["lift_error"] = True, None
                    log.info("lift motor answering again; re-enabled")
            _stats["sent"] += 1
            _stats["actual"] = {a: round(v, 3) for a, v in out.items()}
        except Exception as exc:   # a dead loop cannot stop the robot
            _stats["last_error"] = f"{type(exc).__name__}: {exc}"
            log.error("bus write failed: %s", exc)
        _stats["batt_tick"] = (_stats.get("batt_tick", 0) + 1) % BATT_EVERY
        if _stats["batt_tick"] == 0:
            read_battery()
        try:
            arms_step()
        except Exception as exc:
            _stats["last_error"] = f"arms: {type(exc).__name__}: {exc}"
            log.error("arm step failed: %s", exc)
        time.sleep(max(0.0, period - (time.perf_counter() - start)))


# ------------------------------------------------------------------- arms --
def _arm_bus(side):
    return _busL if side == "left" else _busR


def _arm_names(side):
    return [f"{side}_{j}" for j in JOINTS]


def _leader_target(side):
    """Leader ticks -> follower ticks for one arm, or None if no fresh leader data."""
    with _lock:
        ticks = _leader[side]
        fresh = (time.time() - _leader["t"]) < LEADER_STALE_S
        lim = _leader["lim"].get(side, {})
    if ticks is None or not fresh:
        return None
    out = {}
    for j, t in zip(JOINTS, ticks):
        lo, hi = _flim[side][j]
        if j == "gripper" and "gripper" in lim:
            # the two grippers were calibrated to different ranges: map range -> range
            llo, lhi = lim["gripper"]
            f = (t - llo) / max(1, lhi - llo)
            t = lo + f * (hi - lo)
        out[f"{side}_{j}"] = int(clamp(t, lo + LIMIT_MARGIN, hi - LIMIT_MARGIN))
    return out


def read_battery():
    """Lowest voltage any bus reports, in volts. The 12 V pack feeds every servo,
    so a sagging pack shows up here first. Runs in the control loop thread."""
    volts = []
    for bus, motor in ((_busL, "base_left_wheel"), (_busR, "lift_axis" if _lift_bus else "right_gripper")):
        if bus is None:
            continue
        try:
            volts.append(bus.read("Present_Voltage", motor, normalize=False) / 10.0)
        except Exception:
            pass
    if not volts:
        _stats["battery_v"] = None
        return
    raw = min(volts)
    prev = _stats.get("battery_raw")
    # ~5 s exponential average: hides the dip while driving, still tracks real drain
    v = raw if prev is None else prev + 0.1 * (raw - prev)
    _stats["battery_raw"] = v
    _stats["battery_v"] = round(v, 1)
    _stats["battery_pct"] = int(clamp((v - BATT_EMPTY) / (BATT_FULL - BATT_EMPTY), 0, 1) * 100)
    if v < BATT_LOW and not _stats.get("batt_warned"):
        log.warning("battery low: %.1f V", v)
        _stats["batt_warned"] = True


def arms_step():
    """Runs inside the control loop: the only place arm registers are written."""
    now = time.time()
    for side in ("left", "right"):
        bus, st = _arm_bus(side), _arms[side]
        if bus is None or not _flim[side]:
            continue
        names = _arm_names(side)
        want_on = _arm_req["on"]
        if not want_on and st["state"] != "off":
            bus.disable_torque(names)
            st.update(state="off", goal=None, queue=[])
            log.info("%s arm off (limp)", side)
            continue
        if not want_on:
            continue
        tgt = _leader_target(side)
        if st["state"] == "off":
            if tgt is None:
                continue            # wait for the leader before touching torque
            pos = bus.sync_read("Present_Position", names, normalize=False)
            st["start"] = {n: int(pos[n]) for n in names}
            st["goal"] = dict(st["start"])
            for n in names:
                bus.write("Acceleration", n, ARM_ACCEL, normalize=False)
                bus.write("Goal_Velocity", n, ARM_SPEED, normalize=False)
                bus.write("P_Coefficient", n, ARM_P_GAIN, normalize=False)
            bus.sync_write("Goal_Position", st["goal"], normalize=False)
            st.update(state="enabling", queue=list(names), tick=0)
            log.info("%s arm: enabling torque joint by joint", side)
            continue
        if st["state"] == "enabling":
            st["tick"] += 1
            if st["tick"] % ARM_TORQUE_GAP == 0 and st["queue"]:
                bus.enable_torque(st["queue"].pop(0))
            if not st["queue"]:
                st.update(state="ramping", t0=now)
            continue
        if tgt is None:
            st["stale"] = True      # leader lost: hold the last goal
            continue
        if st.get("stale"):
            st.update(state="ramping", start=dict(st["goal"]), t0=now, stale=False)
            log.info("%s arm: leader back, easing in again", side)
        if st["state"] == "ramping":
            a = min(1.0, (now - st["t0"]) / ARM_RAMP_S)
            want = {n: st["start"][n] + (tgt[n] - st["start"][n]) * a for n in names}
            if a >= 1.0:
                st["state"] = "on"
                log.info("%s arm following its leader", side)
        else:
            want = tgt
        goal = {n: int(st["goal"][n] + clamp(want[n] - st["goal"][n], -ARM_MAX_STEP, ARM_MAX_STEP))
                for n in names}
        st["goal"] = goal
        st["err"] = max(abs(tgt[n] - goal[n]) for n in names)
        bus.sync_write("Goal_Position", goal, normalize=False)


def _arm_status():
    with _lock:
        age = time.time() - _leader["t"] if _leader["t"] else None
        client = _leader["client"]
    lag_deg = {s: round(_arms[s]["err"] * 360 / 4096, 1) for s in ("left", "right")}
    return {"leader_connected": client is not None,
            "link_ms": _leader.get("rtt_ms"),
            "lag_deg": lag_deg,
            "leader_age_ms": None if age is None else int(age * 1000),
            "req_on": _arm_req["on"],
            **{s: {"state": _arms[s]["state"], "lag_ticks": _arms[s]["err"],
                   "present": bool(_flim[s])} for s in ("left", "right")}}


def leader_server():
    """Accept leader_bridge.py on 127.0.0.1:LEADER_PORT; one JSON line per frame."""
    import socket
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((BIND, LEADER_PORT))
    srv.listen(1)
    log.info("waiting for leader bridge on %s:%d", BIND, LEADER_PORT)
    while not _shutdown.is_set():
        conn, addr = srv.accept()
        log.info("leader bridge connected from %s", addr)
        with _lock:
            _leader["client"] = f"{addr[0]}:{addr[1]}"
        try:
            f = conn.makefile("r")
            for line in f:
                try:
                    d = json.loads(line)
                except ValueError:
                    continue
                if "ping" in d:
                    try:
                        conn.sendall((json.dumps({"pong": d["ping"]}) + "\n").encode())
                    except OSError:
                        pass
                with _lock:
                    if "rtt_ms" in d:
                        _leader["rtt_ms"] = d["rtt_ms"]
                    for side in ("left", "right"):
                        v = d.get(side)
                        if isinstance(v, list) and len(v) == 6:
                            _leader[side] = [int(x) for x in v]
                    if "lim" in d:
                        _leader["lim"] = d["lim"]
                    _leader["t"] = time.time()
                    _leader["packets"] += 1
        except Exception as exc:
            log.warning("leader bridge dropped: %s", exc)
        finally:
            conn.close()
            with _lock:
                _leader["client"] = None
            log.info("leader bridge disconnected; arms hold their pose")


# ------------------------------------------------------------------ pages --
LOGIN = """<!doctype html>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AlohaMini</title>
<style>
 :root{color-scheme:dark;--bg:#14161a;--fg:#e9edf2;--dim:#9aa4b2;--accent:#4da3ff;--bad:#ff5c5c}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--fg);font:16px/1.5 -apple-system,system-ui,sans-serif;
   display:grid;place-items:center;min-height:100vh;padding:24px}
 .card{width:100%;max-width:360px;text-align:center}
 h1{font-size:20px;margin:0 0 6px}
 p{color:var(--dim);font-size:14px;margin:0 0 22px}
 input{width:100%;font:600 24px/1 ui-monospace,SFMono-Regular,Menlo,monospace;letter-spacing:.14em;
   text-align:center;text-transform:uppercase;padding:16px;border-radius:12px;
   border:1px solid #2a2f37;background:#1d2127;color:var(--fg)}
 input:focus{outline:none;border-color:var(--accent)}
 button{width:100%;margin-top:12px;font:600 16px/1 inherit;padding:15px;border:0;border-radius:12px;
   background:var(--accent);color:#06121f;cursor:pointer}
 .err{color:var(--bad);font-size:13px;margin-top:14px;min-height:18px}
</style>
<div class="card">
  <h1>AlohaMini</h1>
  <p>Enter the passcode to drive the robot.</p>
  <form id="f" autocomplete="off">
    <input id="c" name="code" placeholder="ABC-123" autofocus
           inputmode="latin" autocapitalize="characters" spellcheck="false">
    <button type="submit">Unlock</button>
  </form>
  <div class="err" id="e"></div>
</div>
<script>
f.addEventListener("submit", async ev=>{
  ev.preventDefault(); e.textContent="";
  const r = await fetch("/login",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({code:c.value})});
  const j = await r.json().catch(()=>({}));
  if(r.ok && j.ok){ location.reload(); }
  else { e.textContent = j.error || "Wrong passcode."; c.select(); }
});
</script>
"""

PAGE = """<!doctype html>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AlohaMini Teleop</title>
<style>
 :root{color-scheme:dark;--bg:#14161a;--fg:#e9edf2;--dim:#9aa4b2;--accent:#4da3ff;
   --live:#37d67a;--stop:#ff5c5c;--off:#2a2f37}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.45 -apple-system,system-ui,sans-serif;padding:20px}
 .wrap{max-width:560px;margin:0 auto}
 h1{font-size:18px;margin:0 0 4px}
 .sub{color:var(--dim);font-size:13px;margin-bottom:16px}
 .pads{display:flex;gap:26px;justify-content:center;flex-wrap:wrap;margin:20px auto}
 .grid{display:grid;grid-template-columns:repeat(3,70px);grid-gap:8px}
 .liftcol{display:grid;grid-template-columns:70px;grid-gap:8px;align-content:start}
 .cap{grid-column:1/-1;font-size:11px;color:var(--dim);text-align:center;margin-bottom:2px}
 button{font:600 17px/1 inherit;height:62px;border:1px solid var(--off);border-radius:12px;
   background:#1d2127;color:var(--fg);cursor:pointer;-webkit-user-select:none;user-select:none;touch-action:none}
 button:active,button.on{background:var(--accent);border-color:var(--accent);color:#06121f}
 button[disabled]{opacity:.35;cursor:not-allowed}
 button[disabled]:active{background:#1d2127;border-color:var(--off);color:var(--fg)}
 button.stop{background:#2a1a1c;border-color:#5a2a2e;color:var(--stop);height:46px;width:100%;margin-top:10px}
 button.stop:active{background:var(--stop);color:#1a0608}
 .k{display:block;font-size:11px;color:var(--dim);font-weight:400;margin-top:3px}
 button:active .k,button.on .k{color:#06121f}
 .speed{margin:18px auto 0}
 .speed label{display:flex;justify-content:space-between;font-size:13px;color:var(--dim);margin-bottom:6px}
 input[type=range]{width:100%;accent-color:var(--accent)}
 .row{display:flex;gap:16px;justify-content:center;flex-wrap:wrap;margin-top:14px;font-size:13px;color:var(--dim)}
 .val{color:var(--fg);font-variant-numeric:tabular-nums}
 .dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--stop);margin-right:5px}
 .dot.live{background:var(--live)}
 .note{margin:18px auto 0;color:var(--dim);font-size:12px;text-align:center}
 .warn{margin:14px auto 0;padding:10px 12px;border:1px solid #5a4a2a;background:#231f16;
   border-radius:10px;color:#e8d9a8;font-size:12px}
 .wrap{max-width:1100px}
 .camgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:8px;margin:0 0 6px}
 .camgrid figure{margin:0;position:relative}
 .camgrid img{width:100%;aspect-ratio:4/3;object-fit:cover;background:#0d0f12;border:1px solid var(--off);border-radius:10px;display:block}
 .camgrid figcaption{position:absolute;left:8px;top:6px;font-size:11px;color:#fff;background:#0009;padding:1px 6px;border-radius:6px}
 .camwrap{margin:0 0 6px}
 #cam{width:100%;aspect-ratio:4/3;object-fit:cover;background:#0d0f12;border:1px solid var(--off);
   border-radius:12px;display:block}
 .camrow{display:flex;gap:8px;justify-content:center;margin-top:8px;flex-wrap:wrap}
 .camrow button{height:34px;padding:0 14px;font:600 13px/1 inherit;border-radius:9px}
 .hud{display:flex;gap:6px 16px;flex-wrap:wrap;font:600 12px/1.4 ui-monospace,SFMono-Regular,Menlo,monospace;
   color:var(--dim);margin:0 0 8px;padding:6px 10px;border:1px solid var(--off);border-radius:10px;background:#0d0f12}
 .hud b{font-weight:700} .g{color:var(--live)} .y{color:#f0c674} .r{color:var(--stop)}
 .batt{margin-left:14px;padding:2px 8px;border-radius:999px;border:1px solid var(--off);font-variant-numeric:tabular-nums}
 .batt.ok{color:var(--live);border-color:#1f5a38} .batt.mid{color:#f0c674;border-color:#5a4a2a} .batt.low{color:var(--stop);border-color:#5a2a2e}
 .arms{margin:22px auto 0;padding-top:14px;border-top:1px solid var(--off)}
 .arms .cap{font-size:11px;color:var(--dim);text-align:center;margin-bottom:8px}
 .armrow{display:grid;grid-template-columns:1fr 1fr;gap:10px}
 button.limp{color:#f0c674}
 .seat{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:0 0 10px;padding:10px 12px;
   border:1px solid var(--off);border-radius:12px;background:#0d0f12}
 .seat.you{border-color:#1f5a38;background:#0f1d15}
 .seat #seatmsg{flex:1;min-width:180px;font-weight:600}
 .seat input{font:15px inherit;padding:8px 10px;border-radius:9px;border:1px solid var(--off);background:#1d2127;color:var(--fg);width:150px}
 .seat button{height:38px;padding:0 16px;font-size:14px}
 #seatbtn.take{background:var(--live);border-color:var(--live);color:#06121f}
 body.watch .pads button,body.watch .armrow button{opacity:.35;pointer-events:none}
</style>
<div class="wrap">
<h1>AlohaMini teleop</h1>
<div class="sub"><span id="dot" class="dot"></span><span id="state">connecting…</span>
  <span class="batt" id="batt">battery —</span></div>
<div class="warn" id="battwarn" hidden>Battery low. Park the arms and charge soon: when it drops out, the Pi and every motor lose power at once.</div>

<div class="seat" id="seatbar">
  <span id="seatmsg">checking who is driving…</span>
  <input id="nm" placeholder="your name" maxlength="24" autocomplete="off">
  <button id="seatbtn">Take the wheel</button>
</div>
<div class="hud" id="hud">
  <span>PING <b id="hping">–</b></span><span>ARM LINK <b id="hlink">–</b></span>
  <span>ARM LAG <b id="hlag">–</b></span><span>LOOP <b id="hloop">–</b></span>
  <span>CAM <b id="hcam">–</b></span>
</div>
<div class="warn" id="liftwarn2" hidden></div>
<div class="camgrid" id="camgrid" hidden></div>

<div class="pads">
  <div class="grid">
    <div class="cap">base</div>
    <button data-k="q">Q<span class="k">spin left</span></button>
    <button data-k="w">W<span class="k">forward</span></button>
    <button data-k="e">E<span class="k">spin right</span></button>
    <button data-k="a">A<span class="k">left</span></button>
    <button data-k="s">S<span class="k">back</span></button>
    <button data-k="d">D<span class="k">right</span></button>
  </div>
  <div class="liftcol">
    <div class="cap">lift</div>
    <button data-k="u" id="bu">U<span class="k">up</span></button>
    <button data-k="j" id="bj">J<span class="k">down</span></button>
  </div>
</div>

<button class="stop" id="stopb">STOP BASE + LIFT <span class="k" style="display:inline">space</span></button>

<div class="arms">
  <div class="cap">arms (follow the leader arms on the Mac)</div>
  <div class="armrow">
    <button id="armon">Arms on<span class="k">ease in over 4 s</span></button>
    <button id="armoff" class="limp">Arms off<span class="k">go limp: hold them first</span></button>
  </div>
  <div class="row"><div>leader <span class="val" id="ld">not connected</span></div>
    <div>left <span class="val" id="al">off</span></div>
    <div>right <span class="val" id="ar">off</span></div></div>
</div>

<div class="speed">
  <label><span>speed</span><span class="val"><span id="pct">100</span>%  ·  T faster / G slower</span></label>
  <input type="range" id="sp" min="10" max="100" step="5" value="100">
</div>

<div class="row">
  <div>asked vx <span class="val" id="vx">0.00</span></div>
  <div>vy <span class="val" id="vy">0.00</span></div>
  <div>wz <span class="val" id="wz">0.00</span></div>
</div>
<div class="row">
  <div>now vx <span class="val" id="avx">0.00</span></div>
  <div>vy <span class="val" id="avy">0.00</span></div>
  <div>wz <span class="val" id="awz">0.00</span></div>
</div>
<div class="note">Hold a key or button to move. Release to stop.
Speed ramps so the robot does not lurch. It stops by itself if this page goes
quiet for 0.4 s. Top speed is the measured limit: 0.26 m/s, 2.08 rad/s.</div>
<div class="warn" id="liftwarn" hidden></div>
</div>

<script>
const MAXV=%MAXV%, MAXW=%MAXW%, MAXL=%MAXL%, LIFT=%LIFT%, CAMS=%CAMS%;
const held=new Set();
const CID=sessionStorage.cid||(sessionStorage.cid=Math.random().toString(36).slice(2)+Date.now().toString(36));
nm.value=localStorage.am_name||""; nm.addEventListener("input",()=>localStorage.am_name=nm.value.trim());
let driving=false, seatWant=null;
let scale=1.0, curCam=null;

// Cameras: every camera streams at once, one <img> each. A stalled stream
// retries itself every few seconds.
function startCam(img,i){ img.src="/cam/"+i+"?t="+Date.now(); }
if(CAMS.length){
  camgrid.hidden=false;
  CAMS.forEach((i,n)=>{
    const f=document.createElement("figure");
    const img=document.createElement("img"); img.alt="camera "+(n+1);
    const cap=document.createElement("figcaption"); cap.textContent="cam "+(n+1);
    img.addEventListener("error",()=>setTimeout(()=>startCam(img,i),3000));
    f.append(img,cap); camgrid.appendChild(f);
    setTimeout(()=>startCam(img,i), 300*n);   // stagger so the USB hub opens one at a time
  });
}
if(!LIFT){
  bu.disabled=true; bj.disabled=true; liftwarn.hidden=false;
  liftwarn.textContent="Lift disabled: no motor answers on ID %LIFTID%. Check the 90 cm "+
    "cable from the left board to the lift servo, its 12 V power, and that the servo is "+
    "set to ID %LIFTID%. Up and down switch on by themselves once it answers.";
}
function vel(){
  const x=(held.has("w")?1:0)-(held.has("s")?1:0);
  const y=(held.has("a")?1:0)-(held.has("d")?1:0);
  const r=(held.has("q")?1:0)-(held.has("e")?1:0);
  const l=LIFT?((held.has("u")?1:0)-(held.has("j")?1:0)):0;
  return {vx:x*MAXV*scale, vy:y*MAXV*scale, wz:r*MAXW*scale, lift:l*MAXL*scale};
}
function paint(v){
  vx.textContent=v.vx.toFixed(2); vy.textContent=v.vy.toFixed(2); wz.textContent=v.wz.toFixed(2);
  document.querySelectorAll("button[data-k]").forEach(b=>b.classList.toggle("on", held.has(b.dataset.k)));
}
let pingEma=null;
function grade(el,val,good,ok,txt){ el.textContent=txt; el.className = val==null?"":(val<=good?"g":(val<=ok?"y":"r")); }
async function send(){
  const v=vel(); paint(v);
  const t0=performance.now();
  try{
    const want=seatWant; seatWant=null;
    const r=await fetch("/cmd",{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify({...(driving||want==="take"?v:{vx:0,vy:0,wz:0,lift:0}),client:CID,name:nm.value.trim(),seat:want})});
    if(r.status===401){ location.reload(); return; }
    const j=await r.json();
    if(j.seat) paintSeat(j.seat);
    const rtt=performance.now()-t0; pingEma = pingEma==null?rtt:pingEma*0.8+rtt*0.2;
    grade(hping,pingEma,60,150,Math.round(pingEma)+" ms");
    if(j.arms){ const L=j.arms.link_ms;
      grade(hlink,L,40,100,L==null?"–":Math.round(L)+" ms");
      const d=Math.max(j.arms.lag_deg.left,j.arms.lag_deg.right);
      grade(hlag,d,3,10,(j.arms.left.state==="on"||j.arms.right.state==="on")?d.toFixed(1)+"°":"arms off"); }
    if(j.loop_hz!=null) grade(hloop,-j.loop_hz,-45,-35,Math.round(j.loop_hz)+" Hz");
    if(j.cam_fps){ const f=Object.values(j.cam_fps); const mn=f.length?Math.min(...f):null;
      grade(hcam,mn==null?null:-mn,-10,-6,f.length?f.map(x=>Math.round(x)).join("/")+" fps":"–"); }
    liftwarn2.hidden=!j.lift_error; if(j.lift_error) liftwarn2.textContent="Lift: "+j.lift_error+" Retrying every 2 s; switching the 12 V off and on usually clears an overload cut-out.";
    if(j.actual){ avx.textContent=(+j.actual.vx).toFixed(2);
                  avy.textContent=(+j.actual.vy).toFixed(2);
                  awz.textContent=(+j.actual.wz).toFixed(2); }
    if(j.battery_v!=null){ const v=j.battery_v, p=j.battery_pct;
      batt.textContent="battery "+v.toFixed(1)+" V · ~"+p+"%";
      batt.className="batt "+(v<%BATTLOW%?"low":(p<40?"mid":"ok"));
      battwarn.hidden = !(v<%BATTLOW%); }
    else { batt.textContent="battery —"; batt.className="batt"; }
    if(j.arms){ const a=j.arms;
      ld.textContent = a.leader_connected ? ("live · "+a.leader_age_ms+" ms") : "not connected";
      al.textContent = a.left.present ? a.left.state+(a.left.state==="on"?" · lag "+a.left.lag_ticks:"") : "missing";
      ar.textContent = a.right.present ? a.right.state+(a.right.state==="on"?" · lag "+a.right.lag_ticks:"") : "missing";
      armon.classList.toggle("on", a.req_on); }
    dot.classList.add("live");
    state.textContent="connected · "+j.sent+" commands · "+j.watchdog_stops+" auto-stops";
  }catch(e){ dot.classList.remove("live"); state.textContent="lost connection to the robot"; }
}
function paintSeat(s){
  const was=driving; driving=s.you;
  if(was&&!driving) held.clear();
  document.body.classList.toggle("watch",!driving);
  seatbar.classList.toggle("you",driving);
  seatbtn.disabled=!driving&&!s.free;
  seatbtn.className=driving?"":(s.free?"take":"");
  seatbtn.textContent=driving?"Give up the wheel":"Take the wheel";
  seatmsg.textContent=driving?"You are driving. Others are watching."
    :(s.free?"Nobody is driving. Type your name and take the wheel."
            :(s.driver||"Someone")+" is driving. You can watch and press STOP.");
}
seatbtn.addEventListener("click",()=>{
  if(!driving&&!nm.value.trim()){nm.focus();nm.placeholder="type your name first";return;}
  seatWant=driving?"leave":"take"; held.clear(); send();
});
function press(k){ if(!driving||held.has(k))return; held.add(k); send(); }
function release(k){ if(!held.delete(k))return; send(); }
function allStop(){ held.clear(); send(); }
function setScale(p){ p=Math.min(100,Math.max(10,p)); scale=p/100; sp.value=p; pct.textContent=p; }
// Continuous heartbeat: feeds the watchdog while a key is held and keeps the
// "now" readout live during the ramp down. If the page dies, so does it.
setInterval(send,150);

addEventListener("keydown",ev=>{
  if(ev.target===nm) return;           // typing your name must not drive
  const k=ev.key.toLowerCase();
  if(k===" "){ev.preventDefault();stopb.click();return;}
  if(k==="t"){ev.preventDefault();setScale(+sp.value+5);return;}
  if(k==="g"){ev.preventDefault();setScale(+sp.value-5);return;}
  const keys = LIFT ? "wasdqeuj" : "wasdqe";
  if(keys.includes(k)&&!ev.repeat){ev.preventDefault();press(k);}
});
addEventListener("keyup",ev=>{if(ev.target===nm)return; const k=ev.key.toLowerCase(); if("wasdqeuj".includes(k)){ev.preventDefault();release(k);}});
addEventListener("blur",allStop);
addEventListener("pagehide",allStop);
document.addEventListener("visibilitychange",()=>{if(document.hidden)allStop();});
sp.addEventListener("input",()=>setScale(+sp.value));
document.querySelectorAll("button[data-k]").forEach(b=>{
  const k=b.dataset.k;
  const down=ev=>{if(b.disabled)return;ev.preventDefault();press(k);};
  const up=ev=>{ev.preventDefault();release(k);};
  b.addEventListener("pointerdown",down); b.addEventListener("pointerup",up);
  b.addEventListener("pointerleave",up); b.addEventListener("pointercancel",up);
});
stopb.addEventListener("click",()=>{allStop();
  fetch("/stop",{method:"POST",headers:{"Content-Type":"application/json"},body:"{}"});});
async function arms(on){ await fetch("/arms",{method:"POST",headers:{"Content-Type":"application/json"},
  body:JSON.stringify({on,client:CID})}); send(); }
armon.addEventListener("click",()=>arms(true));
armoff.addEventListener("click",()=>arms(false));
send();
</script>
"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    # ---- helpers ----
    def _send(self, body: bytes, ctype: str, code=200, cookie: str | None = None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        if cookie:
            self.send_header("Set-Cookie", cookie)
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200, cookie=None):
        self._send(json.dumps(obj).encode(), "application/json", code, cookie)

    def _authed(self) -> bool:
        auth = self.headers.get("Authorization", "")
        if auth.startswith("Bearer ") and _code_ok(auth[7:]):
            return True          # agents: one header, no cookies
        raw = self.headers.get("Cookie")
        if raw:
            try:
                tok = SimpleCookie(raw).get("am_session")
                if tok and _token_ok(tok.value):
                    return True
            except Exception:
                pass
        return False

    def _body(self) -> dict:
        n = int(self.headers.get("Content-Length") or 0)
        try:
            return json.loads(self.rfile.read(n) or b"{}") if n else {}
        except Exception:
            return {}

    # ---- routes ----
    def _stream_camera(self, idx: int):
        """MJPEG stream of the shared feed. Runs until the client disconnects.
        Any number of viewers share one capture per camera."""
        feed = _feeds.get(idx)
        if feed is None:
            self._json({"error": "no such camera"}, 404)
            return
        feed.join()
        try:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            seq = -1
            while not _shutdown.is_set():
                buf, seq = feed.next_frame(seq)
                if buf is None:
                    continue        # no new frame within 2 s; keep waiting
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                 b"Content-Length: " + str(len(buf)).encode() +
                                 b"\r\n\r\n" + buf + b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass          # the viewer closed the tab; normal
        except Exception as exc:
            log.warning("camera %d viewer ended: %s", idx, exc)
        finally:
            feed.leave()

    def do_GET(self):
        if self.path == "/healthz":
            self._json({"ok": True, "connected": _stats["connected"]})
            return
        if self.path.startswith("/cam/"):
            if not self._authed():
                self._json({"error": "unauthorized"}, 401)
                return
            try:
                # the page adds ?t=<ms> to bust the image cache; strip it before parsing
                idx = int(self.path.split("?", 1)[0].split("/")[2])
            except (IndexError, ValueError):
                self._json({"error": "bad camera"}, 400)
                return
            if idx not in _cams:
                self._json({"error": "no such camera"}, 404)
                return
            self._stream_camera(idx)
            return
        if self.path in ("/", "/index.html"):
            if not self._authed():
                self._send(LOGIN.encode(), "text/html; charset=utf-8")
                return
            page = (PAGE.replace("%MAXV%", str(MAX_VX)).replace("%MAXW%", str(MAX_WZ))
                        .replace("%MAXL%", str(MAX_LIFT_VEL)).replace("%BATTLOW%", str(BATT_LOW)).replace("%LIFTID%", str(LIFT_ID))
                        .replace("%LIFT%", "true" if _stats["lift_present"] else "false")
                        .replace("%CAMS%", json.dumps(_cams)))
            self._send(page.encode(), "text/html; charset=utf-8")
            return
        if not self._authed():
            self._json({"error": "unauthorized"}, 401)
            return
        if self.path == "/status":
            with _lock:
                body = {**_stats, **_target}
            self._json({**body, "arms": _arm_status()})
            return
        self._json({"error": "not found"}, 404)

    def do_POST(self):
        global _last_cmd

        if self.path == "/login":
            locked, left = _locked()
            if locked:
                self._json({"error": f"Too many tries. Wait {left}s."}, 429)
                return
            code = str(self._body().get("code", ""))
            if _code_ok(code):
                cookie = (f"am_session={_make_token()}; Path=/; Max-Age={SESSION_TTL_S}; "
                          "HttpOnly; SameSite=Lax")
                log.info("passcode accepted")
                self._json({"ok": True}, cookie=cookie)
            else:
                _note_fail()
                log.warning("wrong passcode")
                self._json({"error": "Wrong passcode."}, 403)
            return

        if not self._authed():
            self._json({"error": "unauthorized"}, 401)
            return

        if self.path == "/stop":
            with _lock:
                _target.update(vx=0.0, vy=0.0, wz=0.0, lift=0.0)
                _last_cmd = time.time()
            self._json({"ok": True, "stopped": True})
            return

        if self.path == "/arms":
            d = self._body()
            with _lock:
                driving = _seat_check(str(d.get("client", ""))[:40], "", False, None)
            if not driving:
                self._json({"error": "only the driver can switch the arms"}, 403)
                return
            on = bool(d.get("on"))
            _arm_req["on"] = on
            log.info("arms %s requested from the page", "ON" if on else "OFF")
            self._json({"ok": True, "arms": _arm_status()})
            return

        if self.path != "/cmd":
            self._json({"error": "not found"}, 404)
            return

        d = self._body()
        with _lock:
            cid, name = str(d.get("client", ""))[:40], str(d.get("name", "")).strip()
            moving = any(float(d.get(k, 0.0) or 0.0) for k in ("vx", "vy", "wz", "lift"))
            driving = _seat_check(cid, name, moving, d.get("seat"))
            seat = {"you": driving, "driver": _seat["name"] if _seat["id"] else None,
                    "free": _seat["id"] is None}
            if not driving:
                d = {}          # watchers send nothing to the motors
        if not driving:
            self._json({"ok": True, "seat": seat, "sent": _stats["sent"],
                        "battery_v": _stats.get("battery_v"), "battery_pct": _stats.get("battery_pct"),
                        "loop_hz": _stats.get("loop_hz"), "lift_error": _stats.get("lift_error"),
                        "cam_fps": {str(k): v["fps"] for k, v in _cam_fps.items()},
                        "watchdog_stops": _stats["watchdog_stops"],
                        "actual": _stats["actual"], "arms": _arm_status()})
            return
        with _lock:
            _target["vx"] = clamp(float(d.get("vx", 0.0) or 0.0), -MAX_VX, MAX_VX)
            _target["vy"] = clamp(float(d.get("vy", 0.0) or 0.0), -MAX_VY, MAX_VY)
            _target["wz"] = clamp(float(d.get("wz", 0.0) or 0.0), -MAX_WZ, MAX_WZ)
            lift = clamp(float(d.get("lift", 0.0) or 0.0), -MAX_LIFT_VEL, MAX_LIFT_VEL)
            blk = _stats.get("lift_block", 0)
            if blk and lift * blk < 0:
                _stats["lift_block"] = blk = 0          # moving away from the end clears the block
            if blk and lift * blk > 0:
                lift = 0.0
            _target["lift"] = lift if _stats["lift_present"] else 0.0
            _last_cmd = time.time()
            resp = {"ok": True, "sent": _stats["sent"],
                    "battery_v": _stats.get("battery_v"), "battery_pct": _stats.get("battery_pct"),
                    "loop_hz": _stats.get("loop_hz"), "lift_error": _stats.get("lift_error"),
                    "cam_fps": {str(k): v["fps"] for k, v in _cam_fps.items()},
                    "watchdog_stops": _stats["watchdog_stops"],
                    "actual": _stats["actual"], "seat": seat, **_target}
        self._json({**resp, "arms": _arm_status()})


# ----------------------------------------------------------------- cameras --
# One capture thread per camera, shared by every viewer: two readers on the
# same /dev/video node fight, so viewers never open the device themselves. The
# thread starts with the first viewer and stops 5 s after the last one leaves,
# so an unwatched camera costs no CPU on the Pi.
CAM_CANDIDATES = (0, 1, 2, 3, 4, 5)
CAM_W, CAM_H, CAM_FPS, CAM_QUALITY = 640, 480, 12, 65
CAM_LINGER_S = 5.0
_cam_fps: dict[int, dict] = {}
_cams: list[int] = []


class CamFeed:
    def __init__(self, idx: int):
        self.idx = idx
        self.cond = threading.Condition()
        self.jpg: bytes | None = None
        self.seq = 0
        self.viewers = 0
        self.t_left = 0.0
        self.running = False

    def join(self):
        with self.cond:
            self.viewers += 1
            if not self.running:
                self.running = True
                threading.Thread(target=self._run, daemon=True, name=f"cam{self.idx}").start()

    def leave(self):
        with self.cond:
            self.viewers -= 1
            self.t_left = time.time()

    def next_frame(self, last_seq: int):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq or _shutdown.is_set(), timeout=2.0)
            if self.seq == last_seq:
                return None, last_seq
            return self.jpg, self.seq

    def _run(self):
        import cv2
        while True:
            cap = None
            try:
                for _ in range(10):           # the previous capture may still be closing
                    cap = cv2.VideoCapture(self.idx)
                    if cap.isOpened():
                        break
                    cap.release()
                    time.sleep(0.2)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
                cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
                period = 1.0 / CAM_FPS
                while not _shutdown.is_set():
                    with self.cond:
                        if self.viewers <= 0 and time.time() - self.t_left > CAM_LINGER_S:
                            break
                    t0 = time.perf_counter()
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        time.sleep(0.05)
                        continue
                    ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), CAM_QUALITY])
                    if not ok:
                        continue
                    with self.cond:
                        self.jpg, self.seq = jpg.tobytes(), self.seq + 1
                        self.cond.notify_all()
                    now = time.time()
                    c = _cam_fps.setdefault(self.idx, {"n": 0, "t": now, "fps": 0.0})
                    c["n"] += 1
                    if now - c["t"] >= 1.0:
                        c["fps"], c["n"], c["t"] = round(c["n"] / (now - c["t"]), 1), 0, now
                    time.sleep(max(0.0, period - (time.perf_counter() - t0)))
            except Exception as exc:
                log.warning("camera %d capture error: %s", self.idx, exc)
                time.sleep(1.0)
            finally:
                if cap is not None:
                    cap.release()
            with self.cond:
                if _shutdown.is_set() or self.viewers <= 0:
                    self.running = False
                    _cam_fps.pop(self.idx, None)
                    return
            # a viewer arrived while we were closing: go round again


_feeds: dict[int, CamFeed] = {}


def find_cameras() -> list[int]:
    """Return the video indexes that actually hand over a frame.

    Each Arducam exposes two nodes (capture + metadata) and the Pi adds a dozen
    bcm2835 codec nodes, so the only reliable test is to read a frame.
    """
    import cv2
    found = []
    for idx in CAM_CANDIDATES:
        cap = cv2.VideoCapture(idx)
        try:
            if not cap.isOpened():
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
            ok, frame = cap.read()
            if ok and frame is not None:
                found.append(idx)
                _feeds[idx] = CamFeed(idx)
        except Exception:
            pass
        finally:
            cap.release()
    return found


def _ping_ids(port: str, ids) -> list[int]:
    """Return which of `ids` answer on `port`. Opens and closes its own bus."""
    motors = {f"m{i}": Motor(i, "sts3215", MotorNormMode.RANGE_M100_100) for i in ids}
    b = FeetechMotorsBus(port=port, motors=motors)
    try:
        b.connect(handshake=False)
        return [i for i in ids if b.ping(f"m{i}", num_retry=2) is not None]
    except Exception:
        return []
    finally:
        try:
            b.disconnect()
        except Exception:
            pass


def find_buses() -> tuple[str | None, str | None]:
    """Locate the wheel bus and the lift bus, which are NOT the same bus here.

    Two facts make hardcoding impossible:
      1. ttyACM numbering follows enumeration order, so it changes on every
         replug and reboot.
      2. On this robot the lift (ID 11) sits on the OTHER board from the wheels
         (8/9/10). lerobot's AlohaMini assumes the lift shares the left bus,
         which is why a one-bus search never found it.
    So probe every port for both, and allow them to land on different ports.
    """
    ports = sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        log.error("no /dev/ttyACM* devices: the servo boards are not plugged in")
        return None, None

    wheel_port = lift_port = None
    for p in ports:
        found = _ping_ids(p, tuple(WHEEL_IDS) + (LIFT_ID,))
        wheels = [i for i in found if i in WHEEL_IDS]
        if wheels and wheel_port is None:
            wheel_port = p
            if len(wheels) < len(WHEEL_IDS):
                log.warning("only %d of 3 wheels on %s; driving will be wrong", len(wheels), p)
        if LIFT_ID in found and lift_port is None:
            lift_port = p
        log.info("%s answers: %s", p, found or "nothing")

    log.info("wheels -> %s | lift -> %s", wheel_port or "NOT FOUND", lift_port or "NOT FOUND")
    return wheel_port, lift_port


def _open_bus(port, motors):
    b = FeetechMotorsBus(port=port, motors=motors)
    b.connect(handshake=False)
    return b


def main():
    global _robot, _busL, _busR, _lift_bus

    wheel_port, lift_port = find_buses()
    if wheel_port is None:
        # Exit non-zero so systemd retries in 5 s instead of sitting here
        # pretending to be healthy. The boards are often plugged in later.
        raise SystemExit(1)
    others = [p for p in sorted(glob.glob("/dev/ttyACM*")) if p != wheel_port]
    right_port = lift_port if lift_port and lift_port != wheel_port else (others[0] if others else None)
    _stats["port"], _stats["lift_port"], _stats["right_port"] = wheel_port, lift_port, right_port

    cfg = AlohaMiniConfig()
    cfg.id = "AlohaMiniRobot"
    cfg.robot_model = "alohamini1"
    cfg.no_follower = True
    cfg.no_lift = True
    cfg.left_port = wheel_port
    cfg.cameras = {}
    _robot = AlohaMini(cfg)            # kinematics only; we open our own buses below

    arm = lambda side: {f"{side}_{j}": Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
                        for j, i in zip(JOINTS, ARM_IDS)}
    wheels = {n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
              for n, i in zip(("base_left_wheel", "base_back_wheel", "base_right_wheel"), WHEEL_IDS)}
    left_found = _ping_ids(wheel_port, ARM_IDS)
    _busL = _open_bus(wheel_port, {**(arm("left") if len(left_found) == 6 else {}), **wheels})
    _stats["connected"] = True
    if len(left_found) == 6:
        _flim["left"] = {j: (_busL.read("Min_Position_Limit", f"left_{j}", normalize=False),
                             _busL.read("Max_Position_Limit", f"left_{j}", normalize=False)) for j in JOINTS}
    else:
        log.warning("left arm: only IDs %s answer on %s; left arm disabled", left_found, wheel_port)

    try:
        from lerobot.motors import OperatingMode
        vel_mode, pos_mode = OperatingMode.VELOCITY.value, OperatingMode.POSITION.value
    except Exception:
        vel_mode, pos_mode = 1, 0
    for n in wheels:
        _busL.write("Operating_Mode", n, vel_mode, normalize=False)
    _busL.enable_torque(list(wheels))
    log.info("wheels ready on %s", wheel_port)

    if right_port:
        right_found = _ping_ids(right_port, ARM_IDS)
        rmotors = arm("right") if len(right_found) == 6 else {}
        if lift_port == right_port:
            rmotors["lift_axis"] = Motor(LIFT_ID, "sts3215", MotorNormMode.RANGE_M100_100)
        if rmotors:
            _busR = _open_bus(right_port, rmotors)
        if len(right_found) == 6:
            _flim["right"] = {j: (_busR.read("Min_Position_Limit", f"right_{j}", normalize=False),
                                  _busR.read("Max_Position_Limit", f"right_{j}", normalize=False)) for j in JOINTS}
        else:
            log.warning("right arm: only IDs %s answer on %s; right arm disabled", right_found, right_port)
    if lift_port and lift_port == right_port and _busR is not None:
        try:
            _busR.write("Operating_Mode", "lift_axis", vel_mode, normalize=False)
            _busR.enable_torque("lift_axis")
            _lift_bus = _busR
            _stats["lift_present"] = True
            log.info("lift ready in velocity mode on %s", right_port)
        except Exception as exc:
            log.error("lift setup failed, disabling it: %s", exc)
    elif lift_port:
        log.warning("lift on %s, which is the wheel board; not supported in this build", lift_port)

    for side in ("left", "right"):
        b = _arm_bus(side)
        if b is None or not _flim[side]:
            continue
        for n in _arm_names(side):
            if b.read("Operating_Mode", n, normalize=False) != pos_mode:
                log.warning("%s is not in position mode; arm control may misbehave", n)
        b.disable_torque(_arm_names(side))     # start limp; the page turns them on
        log.info("%s arm ready (limp) limits %s", side, _flim[side])

    global _cams
    _cams = find_cameras()
    _stats["cameras"] = _cams
    log.info("cameras: %s", _cams or "none")

    atexit.register(hard_stop, "atexit")
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, _on_signal)

    threading.Thread(target=control_loop, daemon=True).start()
    threading.Thread(target=leader_server, daemon=True).start()

    srv = ThreadingHTTPServer((BIND, PORT), Handler)
    log.info("listening on http://%s:%d (loopback only)", BIND, PORT)
    log.info("PASSCODE: %s   (also at %s)", PASSCODE, STATE_DIR / "passcode")
    try:
        srv.serve_forever()
    except BaseException as exc:
        log.warning("server exiting: %r", exc)
        raise
    finally:
        _shutdown.set()
        hard_stop("shutdown")


if __name__ == "__main__":
    main()
