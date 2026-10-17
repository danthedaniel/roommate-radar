#!/bin/sh
# Run on the Frame: start camserver (root) and yoloxr; stop both on exit.
# Extra args go to yoloxr, e.g. ./run.sh --no-xr --dump /tmp/dump
cd "$(dirname "$0")"
[ -f calib.txt ] || python3 calib.py calib.txt
sudo -n build/camserver > /tmp/camserver.log 2>&1 &
trap 'sudo -n pkill -x camserver' EXIT INT TERM
build/yoloxr --model ../models/yolo11n.ncnn "$@"
