#!/bin/sh
# Run on the Frame: start camserver (root) and yoloxr; stop both on exit.
# Extra args go to yoloxr, e.g. ./run.sh --no-xr --dump /tmp/dump
# SIGUSR1 (pkill -USR1 -x run.sh, e.g. after a build) restarts yoloxr; camserver keeps running.
cd "$(dirname "$0")"
[ -f calib.txt ] || python3 calib.py calib.txt
sudo -n pkill -x camserver  # left behind if the last run was SIGKILLed (e.g. by Steam)
sudo -n build/camserver > /tmp/camserver.log 2>&1 &
trap 'kill -INT $app 2>/dev/null; wait $app; sudo -n pkill -x camserver' EXIT INT TERM
trap 'reload=1; kill -INT $app' USR1
while :; do
    reload=
    build/yoloxr "$@" &
    app=$!
    # An open SteamVR dashboard keeps input focus, and without focus OpenXR delivers no eye gaze.
    sleep 3
    (cd /opt/steamvr/bin/linuxarm64 && LD_LIBRARY_PATH=. ./vrcmd --hidedashboard > /dev/null 2>&1)
    wait $app
    wait $app  # again if USR1 cut the first wait short
    [ "$reload" ] || break
done
