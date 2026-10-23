#!/bin/sh
# Started by SteamVR (guardian.vrmanifest, see guardian --register): start camserver (root) unless it's already
# running (e.g. for yoloxr), then guardian. Extra args go to guardian. Output: /tmp/guardian.log
cd "$(dirname "$0")"
exec >> /tmp/guardian.log 2>&1
# Low GPU priority: the game preempts the detector (patched ncnn, see ansible/ncnn-global-priority.patch)
export NCNN_QUEUE_GLOBAL_PRIORITY=${NCNN_QUEUE_GLOBAL_PRIORITY-low}
[ -f calib.txt ] || python3 calib.py calib.txt
own_cam=
if ! pgrep -x camserver > /dev/null; then
    sudo -n build/camserver > /tmp/camserver.log 2>&1 &
    own_cam=1
fi
trap 'kill -INT $app 2>/dev/null; wait $app; [ "$own_cam" ] && sudo -n pkill -x camserver' EXIT INT TERM
build/guardian "$@" &
app=$!
wait $app
