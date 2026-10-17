#!/bin/sh
# Run on the Frame: start camserver (root) and yoloxr; stop both on exit.
# Extra args go to yoloxr, e.g. ./run.sh --no-xr --dump /tmp/dump
cd "$(dirname "$0")"
[ -f calib.txt ] || python3 calib.py calib.txt
sudo -n build/camserver > /tmp/camserver.log 2>&1 &
build/yoloxr --model ../models/yolo11n.ncnn "$@" &
app=$!
trap 'kill -INT $app 2>/dev/null; wait $app; sudo -n pkill -x camserver' EXIT INT TERM
# An open SteamVR dashboard keeps input focus, and without focus OpenXR delivers no eye gaze.
sleep 3
(cd /opt/steamvr/bin/linuxarm64 && LD_LIBRARY_PATH=. ./vrcmd --hidedashboard > /dev/null 2>&1)
wait $app
