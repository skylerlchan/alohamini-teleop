#!/bin/bash
# Auto-deploy for the robot. A systemd timer runs this every 30 s on the Pi.
#
# Push to main -> within ~30 s the Pi pulls it, checks pi/teleop_web.py
# compiles, waits until nobody is driving, installs it, restarts the service
# and checks the page answers. If the new version does not come up within
# 60 s, it puts the previous version back. Log: ~/deploy.log
set -u
REPO="$HOME/alohamini-teleop"
LIVE="$HOME/teleop_web.py"
SVC=alohamini-base-web
PY="$HOME/lerobot_alohamini/.venv/bin/python"
STATE="$HOME/.alohamini/deployed_sha"
LOG="$HOME/deploy.log"
log() { echo "$(date '+%F %T') $*" >> "$LOG"; }

cd "$REPO" || { log "no repo at $REPO"; exit 1; }
git fetch -q origin main 2>>"$LOG" || exit 0
NEW_SHA=$(git rev-parse origin/main)
[ "$NEW_SHA" = "$(cat "$STATE" 2>/dev/null)" ] && exit 0
git reset -q --hard origin/main
MSG=$(git log -1 --format='%h %an: %s')

if cmp -s pi/teleop_web.py "$LIVE"; then
    echo "$NEW_SHA" > "$STATE"; log "no robot-code change in $MSG"; exit 0
fi
if ! "$PY" -m py_compile pi/teleop_web.py 2>>"$LOG"; then
    echo "$NEW_SHA" > "$STATE"; log "REJECTED (does not compile): $MSG"; exit 0
fi
# Never restart under someone's hands: a restart stops the robot mid-drive.
DRIVER=$(curl -s -m 3 -H "Authorization: Bearer $(cat "$HOME/.alohamini/passcode")" \
    -H 'Content-Type: application/json' -d '{"client":"autodeploy"}' \
    http://127.0.0.1:8080/cmd | "$PY" -c 'import sys,json
try: print(json.load(sys.stdin)["seat"]["driver"] or "")
except Exception: print("")' 2>/dev/null)
if [ -n "$DRIVER" ]; then
    log "waiting: $DRIVER is driving ($MSG)"; exit 0
fi

cp "$LIVE" "$LIVE.prev"
cp pi/teleop_web.py "$LIVE.tmp" && sync && mv "$LIVE.tmp" "$LIVE" && sync   # sync: a power cut once left this file all zeros
sudo systemctl restart "$SVC"
for _ in $(seq 1 30); do
    sleep 2
    if curl -s -m 2 http://127.0.0.1:8080/healthz | grep -q '"ok": true'; then
        echo "$NEW_SHA" > "$STATE"; log "DEPLOYED $MSG"; exit 0
    fi
done
cp "$LIVE.prev" "$LIVE" && sync
sudo systemctl restart "$SVC"
echo "$NEW_SHA" > "$STATE"
log "ROLLED BACK (new version did not start in 60 s; see ~/base_web.log): $MSG"
