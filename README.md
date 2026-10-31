# AlohaMini web teleop

Drive an [AlohaMini](https://github.com/liyiteng/lerobot_alohamini) robot from any browser: wheels, lift, both follower arms and all cameras, from one process on the robot's Raspberry Pi.

- **Browser control.** Hold W/A/S/D/Q/E or the on-screen buttons to drive. U/J move the lift and Space stops. Works on phones.
- **One driver at a time.** Everyone can watch; one person holds the "seat" and drives. The seat frees itself after 2 s without contact or 60 s idle. Anyone can press STOP.
- **Shared cameras.** One capture thread per camera, sent to any number of viewers. A camera only runs while someone watches.
- **Leader arms.** `mac/leader_bridge.py` reads two SO-101 leader arms on a laptop and streams them to the robot at 50 Hz.
- **Passcode login.** It locks out for 5 minutes after 10 wrong tries.

## Layout

| Path | Runs on | What it is |
|---|---|---|
| `pi/teleop_web.py` | Pi | Web server, control loop, cameras, seat logic |
| `mac/leader_bridge.py` | Laptop | Streams leader-arm positions to the Pi over SSH |
| `deploy/alohamini-teleop.service` | Pi | systemd unit: start at boot, restart on failure |
| `patches/lerobot_alohamini-no-lift.patch` | Pi | Adds `no_lift` to lerobot_alohamini for robots without a lift motor |

## Setup on the Pi

1. Install [lerobot_alohamini](https://github.com/liyiteng/lerobot_alohamini) in a venv (Python 3.12+), then apply the patch:
   ```bash
   cd ~/lerobot_alohamini && git apply ~/alohamini-teleop/patches/lerobot_alohamini-no-lift.patch
   ```
2. Copy `pi/teleop_web.py` to the Pi and install the service. Edit the user and paths in the unit first:
   ```bash
   sudo cp deploy/alohamini-teleop.service /etc/systemd/system/
   sudo systemctl daemon-reload && sudo systemctl enable --now alohamini-teleop
   ```
3. Set the passcode. Otherwise one is generated and saved in `~/.alohamini/passcode`:
   ```bash
   echo 'your-passcode' > ~/.alohamini/passcode && sudo systemctl restart alohamini-teleop
   ```

The server binds to `127.0.0.1:8080` only. To reach it, use an SSH tunnel (`ssh -L 8080:127.0.0.1:8080 <pi>`), or publish it with Tailscale (`tailscale serve` / `tailscale funnel`) or a Cloudflare Tunnel. In our tests, a Cloudflare Tunnel was about 3x faster than Tailscale Funnel: ~100 ms vs ~320 ms per command, with full camera frame rate.

Agents can skip the login page by sending `Authorization: Bearer <passcode>`.

## Leader arms (optional)

On the laptop with the leader arms plugged in:
```bash
pip install feetech-servo-sdk
python mac/leader_bridge.py          # add --swap if left and right come out mirrored
```
Edit `LEFT`, `RIGHT` (serial ports) and `HOST` (your SSH host for the Pi) at the top of the file. The arms stay limp until someone presses **Arms on** on the page.

## Safety

Feetech servos **latch** the last velocity command: if a process dies mid-move, the wheels keep spinning. Every exit path (signal, exception, atexit) runs the same hard stop, which zeroes the wheels and drops torque. A watchdog also stops the base if the page goes quiet for 0.4 s. Keep the service on `KillSignal=SIGTERM`. Your last-resort stop is unplugging the 12 V servo battery.

Speed caps are measured hardware limits: 0.26 m/s driving, 2.08 rad/s spinning, with gentle acceleration so a tall robot doesn't tip.
